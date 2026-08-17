import pytest

from main.avoid_direction import (
    AvoidDirectionConfig,
    AvoidDirectionDebouncer,
)
from main.mission_observation import MissionObservation
from main.mission_types import LaneTarget, ObjectLane


def observation(now, *, lane=ObjectLane.LEFT, box=100.0, received_at=None):
    return MissionObservation(
        now=now,
        object_exists=True,
        object_lane=lane,
        object_box_px=box,
        object_received_at=now if received_at is None else received_at,
    )


def feed(guard, samples):
    """(시각, 차선) 목록을 순서대로 넣고 각 단계의 확정 방향을 모은다."""

    return [guard.update(observation(now, lane=lane)).target for now, lane in samples]


def test_three_consecutive_frames_confirm_the_side():
    guard = AvoidDirectionDebouncer()

    targets = feed(
        guard,
        [(1.0, ObjectLane.LEFT), (1.2, ObjectLane.LEFT), (1.4, ObjectLane.LEFT)],
    )

    assert targets == [None, None, LaneTarget.LANE_TWO]


def test_single_flipped_frame_never_confirms_the_wrong_side():
    """rosbag2_2026_08_13-09_30_09 t=42.3~43.4 구간 실측 재생.

    /lane_fit 의 x_line 이 한 프레임 튀어 car_lane 이 1 -> 2 -> 1 로 반전했다.
    예전 코드는 그 한 프레임으로 회피 목표를 장애물이 있는 쪽(1차선)으로
    확정했고 차가 그대로 장애물에 들어갔다.
    """

    guard = AvoidDirectionDebouncer()
    samples = [
        (42.34, ObjectLane.LEFT),
        (42.54, ObjectLane.LEFT),
        (42.76, ObjectLane.RIGHT),   # 단 한 프레임 반전
        (42.95, ObjectLane.LEFT),
        (43.15, ObjectLane.LEFT),
        (43.35, ObjectLane.LEFT),
    ]

    targets = feed(guard, samples)

    # 반전 프레임은 어느 시점에도 방향을 확정하지 못한다.
    assert LaneTarget.LANE_ONE not in targets
    assert targets[-1] is LaneTarget.LANE_TWO


def test_repeated_snapshot_cannot_satisfy_the_debounce():
    """제어 주기(50 Hz)가 카메라(4~6 Hz)보다 빨라도 같은 표본은 한 번만 센다."""

    guard = AvoidDirectionDebouncer()

    targets = [
        guard.update(observation(1.0 + index * 0.02, received_at=1.0)).target
        for index in range(10)
    ]

    assert targets == [None] * 10
    assert guard.streak == 1


def test_republished_identical_result_cannot_confirm_the_side():
    """object_detection 은 같은 YOLO 결과를 50 Hz 타이머로 재발행한다.

    수신 시각은 매번 다르므로 '연속 3표본' 조건은 60 ms 만에 채워진다. 실측
    (bag 재생)에서 /object_info_raw 메시지의 85.7%가 직전과 완전히 동일한
    재발행이었고, t=61.38s 에서 단 한 번의 검출이 3번 재발행되며 차선 변경
    명령 [5, 1] 이 나갔다. min_duration_s 가 이를 막는다.
    """

    guard = AvoidDirectionDebouncer()

    burst = feed(
        guard,
        [
            (61.38, ObjectLane.RIGHT),
            (61.40, ObjectLane.RIGHT),
            (61.41, ObjectLane.RIGHT),
        ],
    )

    assert burst == [None, None, None]
    # 같은 방향 증거가 실제로 시간을 덮으면 그때 확정한다.
    assert guard.update(
        observation(61.70, lane=ObjectLane.RIGHT)
    ).target is LaneTarget.LANE_ONE


def test_missing_box_is_no_evidence_and_keeps_the_confirmed_side():
    guard = AvoidDirectionDebouncer()
    feed(guard, [(1.0, ObjectLane.LEFT), (1.2, ObjectLane.LEFT), (1.4, ObjectLane.LEFT)])

    decision = guard.update(observation(1.6, lane=ObjectLane.UNKNOWN, box=0.0))

    assert decision.target is LaneTarget.LANE_TWO
    assert decision.streak == 0


def test_unlabelled_box_is_no_evidence_and_keeps_the_confirmed_side():
    guard = AvoidDirectionDebouncer()
    feed(guard, [(1.0, ObjectLane.LEFT), (1.2, ObjectLane.LEFT), (1.4, ObjectLane.LEFT)])

    decision = guard.update(observation(1.6, lane=ObjectLane.UNKNOWN, box=2000.0))

    assert decision.target is LaneTarget.LANE_TWO
    assert decision.streak == 0


def test_stale_sample_breaks_the_streak_but_keeps_the_confirmed_side():
    guard = AvoidDirectionDebouncer()
    feed(guard, [(1.0, ObjectLane.LEFT), (1.2, ObjectLane.LEFT), (1.4, ObjectLane.LEFT)])

    stale = guard.update(observation(3.0, lane=ObjectLane.LEFT, received_at=1.6))

    assert stale.target is LaneTarget.LANE_TWO
    assert stale.streak == 0
    assert stale.new_sample is False


def test_opposite_side_is_held_back_inside_the_retarget_hold():
    guard = AvoidDirectionDebouncer(
        AvoidDirectionConfig(
            min_consecutive_frames=2,
            min_duration_s=0.1,
            retarget_hold_s=1.0,
        ),
    )
    feed(guard, [(1.0, ObjectLane.LEFT), (1.1, ObjectLane.LEFT)])

    targets = feed(guard, [(1.2, ObjectLane.RIGHT), (1.3, ObjectLane.RIGHT)])

    assert targets == [LaneTarget.LANE_TWO, LaneTarget.LANE_TWO]


def test_sustained_opposite_evidence_flips_after_the_retarget_hold():
    """구간 안에서도 방향을 고칠 수 있어야 한다.

    반대 증거가 지속되면(연속 프레임 + 유지 시간) 확정 방향을 뒤집는다.
    한 프레임짜리 반전은 여전히 통과하지 못한다.
    """

    guard = AvoidDirectionDebouncer(
        AvoidDirectionConfig(
            min_consecutive_frames=2,
            min_duration_s=0.1,
            retarget_hold_s=0.5,
        ),
    )
    feed(guard, [(1.0, ObjectLane.LEFT), (1.1, ObjectLane.LEFT)])

    flipped = feed(guard, [(1.8, ObjectLane.RIGHT), (1.9, ObjectLane.RIGHT)])

    assert flipped[-1] is LaneTarget.LANE_ONE


def test_reset_clears_the_confirmed_side():
    guard = AvoidDirectionDebouncer()
    feed(guard, [(1.0, ObjectLane.LEFT), (1.2, ObjectLane.LEFT), (1.4, ObjectLane.LEFT)])

    guard.reset()

    assert guard.confirmed is None
    assert guard.streak == 0
    assert guard.last_sample_at is None


@pytest.mark.parametrize(
    "kwargs",
    [
        {"min_consecutive_frames": 0},
        {"min_consecutive_frames": True},
        {"min_consecutive_frames": 2.0},
        {"min_duration_s": -0.1},
        {"max_age_s": -0.1},
        {"max_age_s": float("nan")},
        {"retarget_hold_s": -1.0},
    ],
)
def test_config_rejects_invalid_values(kwargs):
    with pytest.raises(ValueError):
        AvoidDirectionConfig(**kwargs)


@pytest.mark.parametrize("received_at", [None, float("nan"), float("inf")])
def test_invalid_snapshot_timestamp_is_ignored(received_at):
    guard = AvoidDirectionDebouncer()

    decision = guard.update(
        MissionObservation(
            now=1.0,
            object_lane=ObjectLane.LEFT,
            object_box_px=100.0,
            object_received_at=received_at,
        )
    )

    assert decision.target is None
    assert decision.new_sample is False
