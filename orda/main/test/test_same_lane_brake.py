"""Camera-based braking when a fixed obstacle sits in our own lane."""

import math

import pytest

from main.same_lane_brake import (
    SameLaneBrake,
    SameLaneBrakeConfig,
    effective_ego_lane,
)
from main.mission_types import LaneTarget


CFG = SameLaneBrakeConfig()
FAR_BOX = CFG.slow_box_px - 1.0        # 감속 시작 전
MID_BOX = 0.5 * (CFG.slow_box_px + CFG.stop_box_px)
NEAR_BOX = CFG.stop_box_px + 1.0       # 정지 임계 초과


def guard(**kwargs):
    return SameLaneBrake(SameLaneBrakeConfig(**kwargs)) if kwargs else SameLaneBrake()


@pytest.mark.parametrize(
    "field",
    ["slow_box_px", "stop_box_px", "crawl_speed", "hold_s"],
)
def test_config_rejects_negative(field):
    with pytest.raises(ValueError):
        SameLaneBrakeConfig(**{field: -1.0})


def test_config_requires_stop_beyond_slow():
    with pytest.raises(ValueError):
        SameLaneBrakeConfig(slow_box_px=5000.0, stop_box_px=5000.0)


def test_other_lane_never_limits_speed():
    """옆 차선 장애물은 아무리 커도 감속하지 않는다.

    overtake bag 에서 같은 차선 판정이 0 건이었던 상황이 이것이다. 옆으로
    지나가는 차 때문에 서면 경기를 못 한다.
    """

    g = guard()

    decision = g.update(now=1.0, car_lane=2, ego_lane=1, box_px=NEAR_BOX)

    assert decision.same_lane is False
    assert decision.speed_limit == float("inf")


def test_same_lane_but_still_far_does_not_limit():
    g = guard()

    decision = g.update(now=1.0, car_lane=1, ego_lane=1, box_px=FAR_BOX)

    assert decision.same_lane is True
    assert decision.speed_limit == float("inf")


def test_same_lane_and_close_commands_a_stop():
    g = guard()

    decision = g.update(now=1.0, car_lane=1, ego_lane=1, box_px=NEAR_BOX)

    assert decision.same_lane is True
    assert decision.speed_limit == 0.0


def test_speed_limit_decreases_as_the_box_grows():
    limits = []
    for box in (CFG.slow_box_px + 1.0, MID_BOX, CFG.stop_box_px - 1.0):
        g = guard()
        limits.append(g.update(now=1.0, car_lane=1, ego_lane=1, box_px=box).speed_limit)

    assert all(math.isfinite(limit) for limit in limits)
    assert limits == sorted(limits, reverse=True)
    assert limits[0] <= CFG.crawl_speed
    assert limits[-1] > 0.0


@pytest.mark.parametrize("lane", [1, 2])
def test_both_lane_values_are_honoured(lane):
    g = guard()

    assert g.update(
        now=1.0, car_lane=lane, ego_lane=lane, box_px=NEAR_BOX
    ).speed_limit == 0.0


@pytest.mark.parametrize("car_lane", [0, 1, 2])
def test_center_valid_vehicle_box_activates_same_path_brake(car_lane):
    g = guard()
    decision = g.update(
        now=1.0,
        car_lane=car_lane,
        ego_lane=LaneTarget.CENTER.value,
        box_px=NEAR_BOX,
    )
    assert decision.same_lane is True
    assert decision.speed_limit == 0.0


def test_unknown_measured_lane_falls_back_to_center_target():
    lane = effective_ego_lane(
        measured_lane=-1,
        measured_received_at=None,
        lane_target=LaneTarget.CENTER,
        now=2.0,
        max_age_s=1.0,
    )
    assert lane == LaneTarget.CENTER.value
    assert guard().update(
        now=2.0,
        car_lane=1,
        ego_lane=lane,
        box_px=NEAR_BOX,
    ).same_lane is True


def test_stale_measured_lane_falls_back_to_context_target():
    assert effective_ego_lane(
        measured_lane=LaneTarget.LANE_ONE.value,
        measured_received_at=1.0,
        lane_target=LaneTarget.CENTER,
        now=2.1,
        max_age_s=1.0,
    ) == LaneTarget.CENTER.value


@pytest.mark.parametrize(
    ("ego_lane", "car_lane"),
    [(LaneTarget.LANE_ONE.value, 2), (LaneTarget.LANE_TWO.value, 1)],
)
def test_opposite_lane_valid_box_releases_immediately(ego_lane, car_lane):
    g = guard()
    g.update(now=1.0, car_lane=ego_lane, ego_lane=ego_lane, box_px=NEAR_BOX)
    decision = g.update(
        now=1.1,
        car_lane=car_lane,
        ego_lane=ego_lane,
        box_px=NEAR_BOX,
    )
    assert decision.same_lane is False


def test_no_bbox_cannot_create_a_new_brake_latch():
    decision = guard().update(
        now=1.0,
        car_lane=1,
        ego_lane=LaneTarget.CENTER.value,
        box_px=0.0,
    )
    assert decision.same_lane is False


def test_undetermined_frames_hold_the_previous_judgement():
    """확정률이 58%라 미확정 프레임을 '안전'으로 읽으면 안 된다.

    미확정의 원인은 object_detection 의 좌/우 데드밴드(45 px)이고, 실측상
    미확정 프레임의 |dx| 중앙값이 26.6 px 였다. 장애물이 사라진 것이 아니다.
    """

    g = guard()
    g.update(now=1.0, car_lane=1, ego_lane=1, box_px=NEAR_BOX)

    # car_lane 이 미확정(0)으로 떨어진 프레임들
    held = g.update(now=1.3, car_lane=0, ego_lane=1, box_px=NEAR_BOX)

    assert held.same_lane is True
    assert held.holding is True
    assert held.speed_limit == 0.0


def test_hold_expires_after_the_configured_time():
    g = guard(hold_s=0.5)
    g.update(now=1.0, car_lane=1, ego_lane=1, box_px=NEAR_BOX)

    assert g.update(now=1.4, car_lane=0, ego_lane=1, box_px=NEAR_BOX).same_lane is True
    assert g.update(now=1.5, car_lane=0, ego_lane=1, box_px=NEAR_BOX).same_lane is False


def test_contrary_evidence_releases_immediately():
    """회피에 성공해 차선을 옮기면 곧바로 속도를 회복해야 한다.

    hold 를 기다리면 다 피하고 나서도 1초를 기어간다.
    """

    g = guard()
    g.update(now=1.0, car_lane=1, ego_lane=1, box_px=NEAR_BOX)

    released = g.update(now=1.1, car_lane=1, ego_lane=2, box_px=NEAR_BOX)

    assert released.same_lane is False
    assert released.speed_limit == float("inf")


def test_fresh_evidence_rewinds_the_hold():
    g = guard(hold_s=0.5)
    g.update(now=1.0, car_lane=1, ego_lane=1, box_px=NEAR_BOX)
    g.update(now=1.4, car_lane=1, ego_lane=1, box_px=NEAR_BOX)

    # 1.4 에서 다시 확정됐으므로 1.8 까지는 유지된다.
    assert g.update(now=1.7, car_lane=0, ego_lane=1, box_px=NEAR_BOX).same_lane is True


def test_missing_box_keeps_the_last_size():
    """박스가 0인 프레임(검출 실패)을 '멀어졌다'로 읽으면 안 된다.

    object_detection 은 YOLO 결과가 box_max_age_s(0.5초)를 넘으면 면적을 0으로
    내보낸다. 그것을 거리 정보로 쓰면 가장 가까운 순간에 제동이 풀린다.
    """

    g = guard()
    g.update(now=1.0, car_lane=1, ego_lane=1, box_px=NEAR_BOX)

    decision = g.update(now=1.2, car_lane=1, ego_lane=1, box_px=0.0)

    assert decision.speed_limit == 0.0


def test_reset_clears_the_hold():
    g = guard()
    g.update(now=1.0, car_lane=1, ego_lane=1, box_px=NEAR_BOX)

    g.reset()

    assert g.update(now=1.1, car_lane=0, ego_lane=1, box_px=NEAR_BOX).same_lane is False


def test_invalid_timestamp_does_not_limit():
    g = guard()

    assert g.update(
        now=math.nan, car_lane=1, ego_lane=1, box_px=NEAR_BOX
    ).speed_limit == float("inf")


def test_measured_stop_bag_profile_brakes_before_contact():
    """stop_1 실측 분포에서 접촉 전에 0이 되는지 확인한다.

    실측: 같은 차선 프레임의 box_px 가 최소 432 / 중앙 15040 / 최대 22885 였고,
    box 22686 px² 인 프레임의 LiDAR min_dist 가 0.23 m 였다. 즉 2만대는 접촉
    직전이므로, 그보다 앞선 중앙값 부근에서 이미 정지해 있어야 한다.
    """

    g = guard()
    limits = []
    for box in (432.0, 1815.0, 8000.0, 15040.0, 22885.0):
        limits.append(
            g.update(now=1.0, car_lane=1, ego_lane=1, box_px=box).speed_limit
        )

    assert limits[0] == float("inf")        # 432: 아직 멀다
    assert limits[2] > 0.0                  # 8000: 감속 중
    assert limits[3] == 0.0                 # 15040(중앙값): 이미 정지
    assert limits[4] == 0.0                 # 22885(접촉 직전): 당연히 정지
