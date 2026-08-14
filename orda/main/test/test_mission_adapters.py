import math

import pytest

from main.control_selector import CommandCandidate, ControlSource, DriveCommand
from main.mission_types import (
    LaneTarget,
    ObjectLane,
    RouteTrafficSignal,
    opposite_lane_target,
)
from main.mode_info import mode_info_data
from main.race_context import RaceContext
from main.race_fsm import Mode, RaceFSM
from main.runtime_adapter import RaceRuntimeAdapter


def candidate(received_at, angle=2.0, speed=6.0):
    return CommandCandidate(DriveCommand(angle, speed), received_at)


def object_message(*, exists=1.0, distance=1.2, lane=1.0, box=100.0):
    # box(=index 5)는 YOLO 박스 면적이다. exists/distance 는 LiDAR 산출물이고,
    # 회피 방향 판단은 box 와 lane 만 본다.
    return [exists, distance, 0.1, 0.2, 5.0, box, 20.0, 30.0, 4.0, lane]


def no_cluster_message():
    """Exact defaults emitted after object_detection::publishEmpty()."""

    return [0.0, math.inf, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def adapter(mode, *, lane_target=LaneTarget.CENTER):
    return RaceRuntimeAdapter(
        fsm=RaceFSM(initial_state=mode),
        context=RaceContext(state_entered_at=1.0, lane_target=lane_target),
    )


# 회피 방향은 한 프레임으로 확정되지 않는다. AvoidDirectionDebouncer 가
# 같은 방향의 표본 min_consecutive_frames(기본 3)개와, 그 구간이 덮는 시간
# min_duration_s(기본 0.25초)를 함께 요구한다. 시간 조건이 없으면
# object_detection 이 같은 YOLO 결과를 50 Hz 로 재발행하는 것만으로
# 디바운스가 채워진다(실측: 메시지의 85.7%가 직전과 동일한 재발행).
#
# 표본 간격 0.15초는 실측 YOLO 갱신 주기(평균 0.30초, p10 0.15초)를 따른다.
DIRECTION_TIMES = (1.1, 1.25, 1.4)


def feed_direction(runtime, lane, times, *, box=100.0, exists=1.0, distance=1.2):
    """같은 방향 표본을 연속으로 넣고 마지막 사이클을 돌려준다."""

    cycle = None
    for received_at in times:
        assert runtime.record_object_info(
            object_message(
                exists=exists,
                distance=distance,
                lane=lane,
                box=box,
            ),
            received_at,
        ).accepted
        cycle = runtime.step(received_at, lane=candidate(received_at))
    return cycle


def test_object_lane_and_lane_target_domains_are_explicitly_separate():
    assert opposite_lane_target(ObjectLane.LEFT) is LaneTarget.LANE_TWO
    assert opposite_lane_target(ObjectLane.RIGHT) is LaneTarget.LANE_ONE
    assert opposite_lane_target(ObjectLane.UNKNOWN) is None
    assert ObjectLane.LEFT.value == 1
    # car_lane 과 lane_target 은 이제 같은 정수 규약(0=중앙,1=왼쪽,2=오른쪽)을 쓴다.
    assert LaneTarget.CENTER.value == ObjectLane.UNKNOWN.value == 0
    assert LaneTarget.LANE_ONE.value == ObjectLane.LEFT.value == 1
    assert LaneTarget.LANE_TWO.value == ObjectLane.RIGHT.value == 2


@pytest.mark.parametrize(
    "payload",
    [
        object_message()[:-1],
        # 길이 초과는 오류가 아니다. 11번째 is_moving 이 계약에 있다.
        object_message(distance=math.nan),
        object_message(distance=math.inf),
        object_message(lane=1.5),
        object_message(lane=3.0),
        object_message(exists=0.5),
    ],
)
def test_object_info_rejects_wrong_length_nonfinite_and_invalid_enums(payload):
    runtime = adapter(Mode.FIXED_AVOID)

    result = runtime.record_object_info(payload, 1.1)

    assert result.accepted is False
    assert result.warning
    assert runtime.latest_object_snapshot is None


def test_object_info_accepts_publisher_no_cluster_payload_as_absent_snapshot():
    runtime = adapter(Mode.FIXED_AVOID)

    result = runtime.record_object_info(no_cluster_message(), 1.1)
    cycle = runtime.step(1.1, lane=candidate(1.1))

    assert result.accepted is True
    assert cycle.observation.object_exists is False
    assert cycle.observation.object_distance is None
    assert cycle.observation.object_lane is ObjectLane.UNKNOWN
    assert cycle.observation.object_received_at == 1.1
    assert runtime.perception_received_at["object_info"] == 1.1
    assert runtime.lane_action.pending is False
    assert cycle.control.source is ControlSource.LANE


@pytest.mark.parametrize(
    "payload",
    [
        object_message(exists=0.0, distance=math.nan, lane=0.0),
        object_message(exists=0.0, distance=math.inf, lane=3.0),
        [0.0, math.inf, 0.0, 0.0, 0.0, math.inf, 0.0, 0.0, 0.0, 0.0],
    ],
)
def test_absent_object_does_not_hide_malformed_payload(payload):
    runtime = adapter(Mode.FIXED_AVOID)

    result = runtime.record_object_info(payload, 1.1)

    assert result.accepted is False
    assert runtime.latest_object_snapshot is None


def test_no_object_then_valid_object_restores_actionable_snapshot():
    runtime = adapter(Mode.FIXED_AVOID, lane_target=LaneTarget.LANE_ONE)

    assert runtime.record_object_info(no_cluster_message(), 1.1).accepted
    absent = runtime.step(1.1, lane=candidate(1.1))
    detected = feed_direction(runtime, ObjectLane.LEFT.value, (1.2, 1.35, 1.5))

    assert absent.observation.object_distance is None
    assert absent.observation.object_lane is ObjectLane.UNKNOWN
    assert detected.observation.object_distance == pytest.approx(1.2)
    assert detected.observation.object_lane is ObjectLane.LEFT
    assert runtime.lane_action.pending is True
    assert runtime.context.lane_target is LaneTarget.LANE_TWO


def test_valid_object_then_no_object_cannot_complete_pending_action():
    runtime = adapter(Mode.FIXED_AVOID, lane_target=LaneTarget.LANE_ONE)

    feed_direction(runtime, ObjectLane.LEFT.value, DIRECTION_TIMES)
    assert runtime.record_object_info(no_cluster_message(), 1.5).accepted
    absent = runtime.step(1.5, lane=candidate(1.5))

    assert absent.observation.object_exists is False
    assert absent.observation.object_distance is None
    assert absent.observation.object_lane is ObjectLane.UNKNOWN
    assert runtime.lane_action.pending is True
    assert runtime.lane_action.completed is False


def test_stale_object_snapshot_stops_the_avoid_zone():
    """구간 안에서 인지가 실제로 끊기면(예산 초과) 정지한다."""

    runtime = adapter(Mode.FIXED_AVOID, lane_target=LaneTarget.LANE_ONE)

    assert runtime.record_object_info(no_cluster_message(), 1.1).accepted
    # 0.8초 공백. object_max_age_s(0.6)를 넘는다.
    cycle = runtime.step(1.9, lane=candidate(1.9))

    assert cycle.observation.object_exists is False
    assert runtime.lane_action.pending is False
    assert runtime.lane_action.completed is False
    assert runtime.lane_action.safe_to_drive is False
    assert cycle.control.source is ControlSource.STOP


def test_fresh_pre_entry_no_object_holds_the_lane_without_starting_action():
    """구간 진입 직전 표본은 주행을 막지 않지만 회피를 시작하지도 않는다.

    예전에는 '진입 이후 표본'이 아니면 safe_to_drive 를 내려 STOP 으로
    떨어뜨렸다. 그러면 구간에 들어서자마자 다음 /object_info 가 올 때까지
    (실측 최대 0.35초) 장애물 앞에서 멈춰 섰다.
    """

    runtime = adapter(Mode.FIXED_AVOID, lane_target=LaneTarget.LANE_ONE)

    assert runtime.record_object_info(no_cluster_message(), 0.9).accepted
    cycle = runtime.step(1.1, lane=candidate(1.1))

    assert cycle.observation.object_exists is False
    assert runtime.lane_action.pending is False
    assert runtime.lane_action.completed is False
    assert runtime.lane_action.safe_to_drive is True
    assert cycle.control.source is ControlSource.LANE


@pytest.mark.parametrize(
    "payload",
    [[], [0], [0, 0, 0], [2, 0], [0, -1], [True, 0], [0.0, 1.0]],
)
def test_lane_change_state_validates_exact_two_binary_integer_fields(payload):
    runtime = adapter(Mode.FIXED_AVOID)

    result = runtime.record_lane_change_state(payload, 1.1)

    assert result.accepted is False
    assert result.warning


def test_lane_change_success_is_a_single_fresh_rising_edge():
    runtime = adapter(Mode.FIXED_AVOID)

    assert runtime.record_lane_change_state([1, 1], 1.1).accepted
    first = runtime.step(1.1)
    assert runtime.record_lane_change_state([1, 1], 1.2).accepted
    sticky = runtime.step(1.2)
    assert runtime.record_lane_change_state([1, 0], 1.3).accepted
    runtime.step(1.3)
    assert runtime.record_lane_change_state([1, 1], 1.4).accepted
    rearmed = runtime.step(1.4)

    assert first.observation.lane_change_success_edge is True
    assert sticky.observation.lane_change_success_edge is False
    assert rearmed.observation.lane_change_success_edge is True


def test_duplicate_lane_change_receipt_is_rejected():
    runtime = adapter(Mode.FIXED_AVOID)

    first = runtime.record_lane_change_state([0, 0], 1.1)
    duplicate = runtime.record_lane_change_state([0, 1], 1.1)

    assert first.accepted is True
    assert duplicate.accepted is False


def test_left_object_starts_lane_two_action_and_success_does_not_exit_fixed():
    runtime = adapter(Mode.FIXED_AVOID, lane_target=LaneTarget.LANE_ONE)

    started = feed_direction(runtime, ObjectLane.LEFT.value, DIRECTION_TIMES)
    assert runtime.lane_action.pending is True
    assert runtime.context.lane_target is LaneTarget.LANE_TWO
    assert started.control.source is ControlSource.LANE
    assert mode_info_data(
        runtime.fsm.state,
        runtime.context.lane_target.value,
        mission_lane_control_enabled=runtime.lane_action.safe_to_drive,
        lane_change_active=runtime.lane_action.pending,
    ) == [5, 2]

    runtime.record_object_info(object_message(lane=ObjectLane.LEFT.value), 1.5)
    runtime.record_lane_change_state([1, 1], 1.51)
    completed = runtime.step(1.51, lane=candidate(1.51))

    assert completed.transition.changed is False
    assert runtime.fsm.state is Mode.FIXED_AVOID
    assert runtime.lane_action.pending is False
    assert runtime.lane_action.completed is True
    assert mode_info_data(
        runtime.fsm.state,
        runtime.context.lane_target.value,
        mission_lane_control_enabled=runtime.lane_action.safe_to_drive,
        lane_change_active=runtime.lane_action.pending,
    ) == [3, 2]


def test_right_object_maps_to_lane_one():
    runtime = adapter(Mode.FIXED_AVOID, lane_target=LaneTarget.LANE_TWO)

    feed_direction(runtime, ObjectLane.RIGHT.value, DIRECTION_TIMES)

    assert runtime.context.lane_target is LaneTarget.LANE_ONE
    assert runtime.lane_action.target is LaneTarget.LANE_ONE
    assert runtime.lane_action.pending is True

def test_overtake_reuses_lane_action_and_waits_for_explicit_completion():
    runtime = adapter(Mode.OVERTAKE, lane_target=LaneTarget.LANE_ONE)

    started = feed_direction(runtime, ObjectLane.LEFT.value, DIRECTION_TIMES)
    runtime.record_object_info(object_message(lane=ObjectLane.LEFT.value), 1.5)
    runtime.record_lane_change_state([1, 1], 1.51)
    success = runtime.step(1.51, lane=candidate(1.51))

    assert started.control.source is ControlSource.LANE
    assert success.transition.changed is False
    assert runtime.fsm.state is Mode.OVERTAKE
    assert runtime.lane_action.completed is True

    runtime.record_overtake_complete(1.3)
    completed = runtime.step(1.3, lane=candidate(1.3))

    assert completed.transition.target is Mode.LANE_DRIVE


@pytest.mark.parametrize(
    ("payload", "step_time"),
    [
        # 좌/우 미확정: 방향을 못 정하니 회피는 시작하지 않는다.
        (object_message(exists=1.0, lane=ObjectLane.UNKNOWN.value), 1.1),
        # 한 프레임짜리 좌/우: 디바운스를 못 채워 아직 확정이 아니다.
        (object_message(exists=1.0, lane=ObjectLane.LEFT.value), 1.1),
    ],
)
def test_unconfirmed_direction_holds_the_lane_instead_of_stopping(
    payload,
    step_time,
):
    """방향이 아직 확정되지 않았다고 구간 한복판에서 멈추지 않는다.

    지금 차선을 유지한 채 계속 가는 편이, 장애물 앞에서 정지하는 것보다
    안전하다. 카메라가 아무것도 못 보는 프레임(box=0)을 이미 그렇게 다루고
    있었으므로 취급도 일관된다.
    """

    runtime = adapter(Mode.FIXED_AVOID, lane_target=LaneTarget.LANE_ONE)
    runtime.record_object_info(payload, 1.1)

    cycle = runtime.step(step_time, lane=candidate(step_time))

    assert runtime.lane_action.pending is False
    assert runtime.context.lane_target is LaneTarget.LANE_ONE
    assert runtime.lane_action.safe_to_drive is True
    assert cycle.control.source is ControlSource.LANE


def test_object_snapshot_from_before_state_entry_cannot_start_action():
    runtime = adapter(Mode.FIXED_AVOID, lane_target=LaneTarget.LANE_ONE)

    # 방향은 확정될 만큼 들어왔지만 전부 구간 진입(1.0) 이전 표본이다.
    feed_direction(runtime, ObjectLane.LEFT.value, (0.6, 0.75, 0.9))
    cycle = runtime.step(1.1, lane=candidate(1.1))

    assert runtime.avoid_direction.confirmed is LaneTarget.LANE_TWO
    assert runtime.lane_action.pending is False
    assert runtime.context.lane_target is LaneTarget.LANE_ONE
    assert cycle.control.source is ControlSource.LANE


def test_pre_action_success_is_consumed_and_cannot_complete_later_action():
    runtime = adapter(Mode.FIXED_AVOID, lane_target=LaneTarget.LANE_ONE)
    runtime.record_lane_change_state([1, 1], 1.05)
    runtime.step(1.05)
    feed_direction(runtime, ObjectLane.LEFT.value, DIRECTION_TIMES)
    runtime.record_object_info(object_message(lane=ObjectLane.LEFT.value), 1.5)
    runtime.record_lane_change_state([1, 1], 1.51)
    sticky = runtime.step(1.51, lane=candidate(1.51))

    assert sticky.observation.lane_change_success_edge is False
    assert runtime.lane_action.pending is True
    assert runtime.lane_action.completed is False


def test_stale_lane_command_is_zero_even_when_fixed_action_is_authorized():
    runtime = adapter(Mode.FIXED_AVOID)
    # 카메라가 방해차량을 못 보는 프레임(box=0). 회피한 차선을 그대로 유지하므로
    # 구간 자체는 인가된 상태다.
    runtime.record_object_info(
        object_message(exists=0.0, lane=ObjectLane.UNKNOWN.value, box=0.0),
        1.1,
    )

    # 차선 명령은 0.9초 전 것이라 max_lane_age_s(0.8)를 넘었다.
    cycle = runtime.step(1.3, lane=candidate(0.4))

    assert runtime.lane_action.safe_to_drive is True
    assert cycle.control.source is ControlSource.STOP


def test_camera_alone_decides_avoid_direction_without_lidar():
    """LiDAR가 전방 클러스터를 못 만들어도 YOLO 차선만으로 회피 방향을 정한다.

    rosbag2_fixed_obstacles_overtake_1 에서 방해차량이 대부분 2 m 밖이라
    exists 가 끝까지 0이었는데, 그때 lane_target 이 CENTER 에서 움직이지 않아
    차선 변경 명령이 나가지 않았다.
    """

    runtime = adapter(Mode.FIXED_AVOID)
    feed_direction(
        runtime,
        ObjectLane.LEFT.value,
        DIRECTION_TIMES,
        exists=0.0,
        distance=math.inf,
        box=10549.0,
    )

    assert runtime.context.lane_target is LaneTarget.LANE_TWO
    assert runtime.lane_action.target is LaneTarget.LANE_TWO
    assert runtime.lane_action.pending is True
    assert runtime.lane_action.safe_to_drive is True


def test_camera_box_without_lane_label_holds_lane_without_action():
    """박스는 있는데 좌/우가 미확정이면 방향을 정하지 않되, 멈추지도 않는다."""

    runtime = adapter(Mode.FIXED_AVOID)
    feed_direction(
        runtime,
        ObjectLane.UNKNOWN.value,
        DIRECTION_TIMES,
        exists=0.0,
        distance=math.inf,
        box=2000.0,
    )

    assert runtime.context.lane_target is LaneTarget.CENTER
    assert runtime.lane_action.pending is False
    assert runtime.lane_action.safe_to_drive is True


def test_internal_zone_and_complete_seams_drive_only_the_declared_chain():
    runtime = adapter(Mode.LANE_DRIVE)

    runtime.record_fixed_zone_entry(1.1)
    fixed = runtime.step(1.1, lane=candidate(1.1))
    runtime.record_fixed_zone_exit(1.2)
    overtake = runtime.step(1.2, lane=candidate(1.2))
    runtime.record_overtake_complete(1.3)
    lane = runtime.step(1.3, lane=candidate(1.3))

    assert fixed.transition.target is Mode.FIXED_AVOID
    assert overtake.transition.target is Mode.LANE_DRIVE
    assert lane.transition.target is Mode.LANE_DRIVE


def test_bool_start_traffic_is_never_reinterpreted_as_route_or_lap_event():
    runtime = adapter(Mode.LANE_DRIVE)
    runtime.record_traffic(True, 1.1)

    cycle = runtime.step(1.1, lane=candidate(1.1))

    assert cycle.observation.green_detected is True
    assert cycle.observation.route_traffic_signal is RouteTrafficSignal.UNKNOWN
    assert cycle.observation.traffic_encounter_started is False
    assert runtime.context.completed_laps == 0
    assert runtime.fsm.state is Mode.LANE_DRIVE


def test_red_amber_is_recoverable_zero_control_not_terminal_stop():
    runtime = adapter(Mode.LANE_DRIVE)

    runtime.record_route_traffic(RouteTrafficSignal.RED_AMBER, 1.1)
    held = runtime.step(1.1, lane=candidate(1.1))
    still_held = runtime.step(1.2, lane=candidate(1.2))
    runtime.record_route_traffic(RouteTrafficSignal.STRAIGHT, 1.3)
    resumed = runtime.step(1.3, lane=candidate(1.3))

    assert held.transition.changed is False
    assert held.control.source is ControlSource.STOP
    assert still_held.control.source is ControlSource.STOP
    assert runtime.fsm.state is Mode.LANE_DRIVE
    assert resumed.control.source is ControlSource.LANE


def test_route_encounter_seam_is_one_shot_and_updates_lap_context():
    runtime = adapter(Mode.LANE_DRIVE)
    runtime.record_route_traffic(
        RouteTrafficSignal.STRAIGHT,
        1.1,
        encounter_started=True,
    )

    first = runtime.step(1.1, lane=candidate(1.1))
    repeated_timer = runtime.step(1.12, lane=candidate(1.12))

    assert first.observation.traffic_encounter_started is True
    assert repeated_timer.observation.traffic_encounter_started is False
    assert runtime.context.completed_laps == 1


def test_avoidance_stays_in_the_lane_it_moved_to():
    """회피 후 되돌아오지 않는다.

    README: "고정장애물이 2차선에 있으면 1차선으로 회피하고, 그대로 1차선에서
    방해차량 구간을 시작한다. 회피 후 2차선으로 되돌아오지 않는다."
    """
    runtime = adapter(Mode.FIXED_AVOID, lane_target=LaneTarget.CENTER)

    # 오른쪽(Lane2) 고정장애물 → 1차선으로 회피
    feed_direction(runtime, ObjectLane.RIGHT.value, DIRECTION_TIMES)
    assert runtime.context.lane_target is LaneTarget.LANE_ONE

    runtime.record_lane_change_state([1, 1], 1.5)
    runtime.step(1.5, lane=candidate(1.5))

    # 장애물이 사라져도 1차선에 그대로 머문다
    runtime.record_object_info(no_cluster_message(), 1.3)
    runtime.step(1.3, lane=candidate(1.3))
    assert runtime.context.lane_target is LaneTarget.LANE_ONE


def test_single_car_lane_flip_cannot_commit_the_colliding_direction():
    """rosbag2_2026_08_13-09_30_09 t=42.3~43.4 실측 재생 (어댑터 통합).

    car_lane 이 1 -> 2 -> 1 로 한 프레임 반전했을 때, 예전 코드는 그 한
    프레임으로 lane_target 을 1차선(장애물이 있는 쪽)으로 확정하고 되돌리지
    못했다. /mode_info 는 [5, 1] 로 나갔고 박스는 3772 -> 21600 px^2 로
    커졌다(= 장애물로 들어감).
    """

    runtime = adapter(Mode.FIXED_AVOID, lane_target=LaneTarget.CENTER)
    samples = [
        (42.34, ObjectLane.LEFT),
        (42.54, ObjectLane.LEFT),
        (42.76, ObjectLane.RIGHT),   # 단 한 프레임 반전
        (42.95, ObjectLane.LEFT),
        (43.15, ObjectLane.LEFT),
        (43.35, ObjectLane.LEFT),
    ]

    targets = []
    for received_at, lane in samples:
        runtime.record_object_info(
            object_message(lane=lane.value, box=3500.0),
            received_at,
        )
        runtime.step(received_at, lane=candidate(received_at))
        targets.append(runtime.context.lane_target)

    assert LaneTarget.LANE_ONE not in targets
    assert runtime.context.lane_target is LaneTarget.LANE_TWO


def test_center_is_the_default_before_any_avoidance():
    runtime = adapter(Mode.FIXED_AVOID, lane_target=LaneTarget.CENTER)

    runtime.record_object_info(no_cluster_message(), 1.1)
    runtime.step(1.1, lane=candidate(1.1))

    assert runtime.context.lane_target is LaneTarget.CENTER
    assert runtime.lane_action.pending is False


def test_lane_change_completes_when_measured_lane_reaches_the_target():
    """완료 판정은 /lane_position 실측 차선으로 한다.

    /lane_change_state 의 스파이크·안정 조건은 실측 오프셋 거동과 맞지 않아
    완료가 잡히지 않았다 (bag 실측: 스파이크 미달 또는 안정 1프레임).
    """
    runtime = adapter(Mode.FIXED_AVOID, lane_target=LaneTarget.CENTER)

    # 오른쪽(Lane2) 장애물 → 1차선으로 회피 명령
    feed_direction(runtime, ObjectLane.RIGHT.value, DIRECTION_TIMES)
    assert runtime.context.lane_target is LaneTarget.LANE_ONE
    assert runtime.lane_action.pending is True

    # 아직 옮겨가지 못한 동안에는 완료가 아니다
    runtime.measured_lane = 0
    runtime.record_object_info(object_message(lane=ObjectLane.RIGHT.value), 1.5)
    runtime.step(1.5, lane=candidate(1.5))
    assert runtime.lane_action.completed is False

    # 실측 차선이 목표에 도달하면 완료
    runtime.measured_lane = LaneTarget.LANE_ONE.value
    runtime.record_object_info(object_message(lane=ObjectLane.RIGHT.value), 1.3)
    runtime.step(1.3, lane=candidate(1.3))
    assert runtime.lane_action.pending is False
    assert runtime.lane_action.completed is True
