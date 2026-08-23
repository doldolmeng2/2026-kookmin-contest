import inspect
import time
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSCompatibility,
    QoSProfile,
    ReliabilityPolicy,
    qos_check_compatible,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray, Int16, Int32MultiArray

from main.main import (
    LANE_COMMAND_TOPIC,
    LANE_CHANGE_STATE_TOPIC,
    LANE_PATH_PREVIEW_TOPIC,
    MainNode,
    OBJECT_INFO_RAW_TOPIC,
    OBJECT_INFO_TOPIC,
    RUBBERCONE_INFO_TOPIC,
    RUBBERCONE_OFFSET_TOPIC,
    RUBBERCONE_SESSION_ACTIVE_TOPIC,
    SIDE_CLEARANCE_TOPIC,
    RELATIVE_X_ENCOUNTER_TIMEOUT_S,
    parse_initial_mode,
    rubbercone_session_qos,
    sensor_event_qos,
)
from main.overtake import OvertakeGuard
from main.relative_x_fallback import RelativeXObstacleLaneFallback
from main.race_context import RaceContext
from main.race_fsm import Mode, RaceFSM
from main.runtime_adapter import RaceRuntimeAdapter
from main.mission_types import (
    LaneTarget,
    ObjectLane,
    ObjectType,
    RouteTrafficSignal,
)


class CallbackHarness:
    def __init__(self, times, mode=Mode.CONE_DRIVE):
        self._times = iter(times)
        self.runtime = RaceRuntimeAdapter(
            fsm=RaceFSM(initial_state=mode),
            context=RaceContext(state_entered_at=0.5),
        )
        self.warnings = []
        self._traffic_encounter_active = False
        self.fixed_vehicle_lane = 0
        self.moving_vehicle_lane = 0
        self.traffic_signal = 0
        self.overtake = OvertakeGuard()
        self.relative_x_fallback = RelativeXObstacleLaneFallback(
            encounter_timeout_s=RELATIVE_X_ENCOUNTER_TIMEOUT_S,
        )
        self.side_left = float("inf")
        self.side_right = float("inf")

    def _now_seconds(self):
        return next(self._times)

    def _warn_throttled(self, key, message, now):
        self.warnings.append((key, message, now))

    def _record_traffic_signal(self, value, received_at):
        return MainNode._record_traffic_signal(self, value, received_at)


class _EntryLogger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(message)


class ObjectEntryHarness(CallbackHarness):
    """Minimal Main receiver that exercises the real object-entry ordering."""

    def __init__(self, times, *, mode, state_entered_at):
        super().__init__(times, mode=mode)
        self.runtime.context.state_entered_at = state_entered_at
        self.runtime.context.lane_target = LaneTarget.CENTER
        self._zone_state = mode
        self._fixed_entry_sent = False
        self._logger = _EntryLogger()

    def get_logger(self):
        return self._logger


def _object_entry_message(object_type, lane):
    return SimpleNamespace(
        data=[
            0.0,
            float("inf"),
            0.0,
            0.0,
            0.0,
            2090.0,
            265.0,
            199.5,
            -14.541839599609375,
            float(lane.value),
            float(object_type.value),
            0.9,
        ],
    )


def _relative_object_entry_message(object_type, relative_x, index):
    return SimpleNamespace(
        data=[
            0.0,
            float("inf"),
            0.0,
            0.0,
            0.0,
            (1000.0, 1400.0, 2200.0)[index],
            320.0 + relative_x,
            200.0 + index,
            0.0,
            float(ObjectLane.UNKNOWN.value),
            float(object_type.value),
            0.80 + index * 0.01,
        ],
    )


def _record_object_entry(harness, object_type, lane, *, now):
    MainNode.object_info_raw_callback(
        harness,
        _object_entry_message(object_type, lane),
    )
    assert harness.runtime.record_lane_offset(0, now)
    assert harness.runtime.record_scan(now)
    MainNode._drive_mission_zones(harness, now)
    return harness.runtime.step(now)


def test_main_cone_callback_records_ros_clock_receipt_once():
    harness = CallbackHarness([1.0])
    message = SimpleNamespace(data=[7, 1, 0])

    MainNode.rubbercone_callback(harness, message)
    first = harness.runtime.step(1.02)
    second = harness.runtime.step(1.04)

    assert first.observation.cone_message_received_at == 1.0
    assert second.observation.cone_message_received_at is None
    assert harness.warnings == []


def test_main_malformed_cone_callback_warns_without_queuing_event():
    harness = CallbackHarness([2.0])

    MainNode.rubbercone_callback(harness, SimpleNamespace(data=[7, 1]))
    cycle = harness.runtime.step(2.02)

    assert cycle.observation.cone_message_received_at is None
    assert len(harness.warnings) == 1
    assert harness.warnings[0][0] == "malformed_cone"


def test_main_accepts_ppt_rubbercone_offset_contract():
    harness = CallbackHarness([2.5])
    harness.rubbercone_offset = 0
    harness.rubbercone_end_flag = 0

    MainNode.rubbercone_offset_callback(
        harness,
        SimpleNamespace(data=[-12, 1]),
    )

    assert harness.rubbercone_offset == -12
    assert harness.rubbercone_end_flag == 1
    assert harness.warnings == []


def test_main_scan_and_cone_callbacks_use_the_same_clock_helper():
    harness = CallbackHarness([3.0, 3.01])

    MainNode.scan_callback(harness, LaserScan())
    MainNode.rubbercone_callback(
        harness,
        SimpleNamespace(data=[0, 0, 80]),
    )
    cycle = harness.runtime.step(3.02)

    assert cycle.observation.scan_received_at == 3.0
    assert cycle.observation.cone_message_received_at == 3.01
    assert cycle.observation.now == 3.02


def test_main_side_clearance_callback_records_exact_semantic_contract():
    harness = CallbackHarness([3.1])

    MainNode.side_clearance_callback(
        harness,
        SimpleNamespace(data=[0.42, float("inf")]),
    )

    assert harness.runtime.latest_side_left == pytest.approx(0.42)
    assert harness.runtime.latest_side_right == float("inf")
    assert harness.runtime.side_clearance_received_at == 3.1
    assert harness.runtime.perception_received_at["side_clearance"] == 3.1
    assert harness.side_left == pytest.approx(0.42)
    assert harness.side_right == float("inf")
    assert harness.warnings == []


@pytest.mark.parametrize(
    "data",
    ([0.2], [0.2, 0.3, 0.4], [-0.1, 0.3], [float("nan"), 0.3],
     [float("-inf"), 0.3]),
)
def test_main_side_clearance_callback_rejects_malformed_payload(data):
    harness = CallbackHarness([3.2])

    MainNode.side_clearance_callback(harness, SimpleNamespace(data=data))

    assert harness.runtime.side_clearance_received_at is None
    assert harness.runtime.latest_side_left == float("inf")
    assert harness.runtime.latest_side_right == float("inf")
    assert harness.warnings[0][0] == "malformed_side_clearance"


def test_main_object_raw_callback_records_validated_ten_field_snapshot():
    harness = CallbackHarness([4.0])
    message = SimpleNamespace(
        data=[1.0, 1.2, 0.1, 0.2, 5.0, 100.0, 20.0, 30.0, 4.0, 2.0],
    )

    MainNode.object_info_raw_callback(harness, message)
    cycle = harness.runtime.step(4.02)

    assert cycle.observation.object_exists is True
    assert cycle.observation.object_distance == pytest.approx(1.2)
    assert cycle.observation.object_lane is ObjectLane.RIGHT
    assert cycle.observation.object_received_at == 4.0
    assert harness.warnings == []


def test_main_object_info_preserves_signal_fixed_and_moving_simultaneously():
    harness = CallbackHarness([4.5], mode=Mode.WAIT_GREEN)

    MainNode.object_info_callback(
        harness,
        SimpleNamespace(data=[2, 1, 2]),
    )
    cycle = harness.runtime.step(4.5)

    assert harness.traffic_signal == 2
    assert harness.fixed_vehicle_lane == 1
    assert harness.moving_vehicle_lane == 2
    assert cycle.observation.green_detected is True
    assert harness.runtime.perception_received_at["object_info"] == 4.5


def test_main_object_raw_callback_accepts_and_normalizes_no_cluster_heartbeat():
    harness = CallbackHarness([4.0], mode=Mode.FIXED_AVOID)
    message = SimpleNamespace(
        data=[0.0, float("inf"), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    )

    MainNode.object_info_raw_callback(harness, message)
    cycle = harness.runtime.step(4.02)

    assert cycle.observation.object_exists is False
    assert cycle.observation.object_distance is None
    assert cycle.observation.object_lane is ObjectLane.UNKNOWN
    assert cycle.observation.object_received_at == 4.0
    assert harness.runtime.lane_action.pending is False
    assert harness.warnings == []


def test_moving_entry_snapshot_starts_one_lane_action_after_fsm_commit():
    harness = ObjectEntryHarness(
        [1.20],
        mode=Mode.WAIT_GREEN,
        state_entered_at=0.50,
    )
    assert harness.runtime.record_lane_offset(0, 1.0)
    assert harness.runtime.record_scan(1.0)
    assert harness.runtime.record_traffic(True, 1.0)
    green = harness.runtime.step(1.0)
    assert green.transition.target is Mode.LANE_DRIVE
    assert harness.runtime.context.state_entered_at == pytest.approx(1.0)

    cycle = _record_object_entry(
        harness,
        ObjectType.MOVING,
        ObjectLane.RIGHT,
        now=1.21,
    )

    assert cycle.transition.target is Mode.OVERTAKE
    assert harness.runtime.context.state_entered_at == pytest.approx(1.21)
    assert cycle.observation.object_received_at == pytest.approx(1.20)
    assert cycle.observation.object_received_at < harness.runtime.context.state_entered_at
    assert harness.runtime.context.lane_target is LaneTarget.LANE_ONE
    assert harness.runtime.lane_action.target is LaneTarget.LANE_ONE
    assert harness.runtime.lane_action.pending is True
    assert harness.runtime.lane_action.started_at == pytest.approx(1.21)
    assert harness.runtime._pending_object_entry_evidence is None

    started_at = harness.runtime.lane_action.started_at
    harness.runtime.step(1.22)
    assert harness.runtime.lane_action.started_at == started_at


def test_fixed_entry_snapshot_starts_one_lane_action_after_fsm_commit():
    harness = ObjectEntryHarness(
        [1.10],
        mode=Mode.LANE_DRIVE,
        state_entered_at=1.0,
    )

    cycle = _record_object_entry(
        harness,
        ObjectType.FIXED,
        ObjectLane.LEFT,
        now=1.20,
    )

    assert cycle.transition.target is Mode.FIXED_AVOID
    assert harness.runtime.context.lane_target is LaneTarget.LANE_TWO
    assert harness.runtime.lane_action.target is LaneTarget.LANE_TWO
    assert harness.runtime.lane_action.pending is True
    assert harness.runtime.lane_action.started_at == pytest.approx(1.20)
    assert harness.runtime._pending_object_entry_evidence is None


@pytest.mark.parametrize(
    ("ego_lane", "samples"),
    [
        (LaneTarget.LANE_TWO, [-66.0, -63.5, -63.5]),
        (LaneTarget.LANE_ONE, [80.0, 110.0, 139.5]),
    ],
)
@pytest.mark.parametrize("object_type", [ObjectType.FIXED, ObjectType.MOVING])
def test_relative_x_adjacent_obstacle_does_not_create_mission_edge(
    ego_lane,
    samples,
    object_type,
):
    harness = ObjectEntryHarness(
        [1.10, 1.20, 1.30],
        mode=Mode.LANE_DRIVE,
        state_entered_at=1.0,
    )
    harness.runtime.context.lane_target = ego_lane

    for index, relative_x in enumerate(samples):
        MainNode.object_info_raw_callback(
            harness,
            _relative_object_entry_message(object_type, relative_x, index),
        )
        MainNode._drive_mission_zones(harness, 1.10 + index * 0.10)

    assert len(harness.runtime._mission_events["fixed_zone_entry"]) == 0
    assert len(harness.runtime._mission_events["overtake_entry"]) == 0
    assert harness._fixed_entry_sent is False
    assert harness.runtime.fsm.state is Mode.LANE_DRIVE
    assert harness.runtime.context.lane_target is ego_lane


def test_c_far_boxes_wait_for_entry_then_use_rolling_last_three():
    samples = [5.5, 35.0, 80.5, 96.0, 117.0, 131.0]
    areas = [627.0, 756.0, 943.0, 1144.0, 1624.0, 4920.0]
    times = [1.10 + index * 0.10 for index in range(len(samples))]
    harness = ObjectEntryHarness(
        times,
        mode=Mode.LANE_DRIVE,
        state_entered_at=1.0,
    )
    harness.runtime.context.lane_target = LaneTarget.LANE_ONE

    for index, (relative_x, area, now) in enumerate(zip(samples, areas, times)):
        message = _relative_object_entry_message(
            ObjectType.FIXED,
            relative_x,
            min(index, 2),
        )
        message.data[5] = area
        message.data[7] = 200.0 + index
        message.data[11] = 0.80 + index * 0.01
        MainNode.object_info_raw_callback(harness, message)
        MainNode._drive_mission_zones(harness, now)
        if area <= 1900.0:
            assert harness.relative_x_fallback.decided is False

    assert harness.relative_x_fallback.evidence_samples == (96.0, 117.0, 131.0)
    assert harness.relative_x_fallback.median_relative_x == 117.0
    assert harness.relative_x_fallback.latched_lane is ObjectLane.RIGHT
    assert len(harness.runtime._mission_events["fixed_zone_entry"]) == 0
    assert harness._fixed_entry_sent is False
    assert harness.runtime.context.lane_target is LaneTarget.LANE_ONE


@pytest.mark.parametrize(
    ("ego_lane", "samples", "expected_target"),
    [
        (LaneTarget.LANE_ONE, [43.0, 43.0, 45.0], LaneTarget.LANE_TWO),
        (LaneTarget.LANE_TWO, [31.0, 30.0, 33.0], LaneTarget.LANE_ONE),
    ],
)
@pytest.mark.parametrize(
    ("object_type", "expected_state", "event_name"),
    [
        (ObjectType.FIXED, Mode.FIXED_AVOID, "fixed_zone_entry"),
        (ObjectType.MOVING, Mode.OVERTAKE, "overtake_entry"),
    ],
)
def test_relative_x_same_lane_creates_one_edge_and_locks_target(
    ego_lane,
    samples,
    expected_target,
    object_type,
    expected_state,
    event_name,
):
    harness = ObjectEntryHarness(
        [1.10, 1.20, 1.30],
        mode=Mode.LANE_DRIVE,
        state_entered_at=1.0,
    )
    harness.runtime.context.lane_target = ego_lane

    for index, relative_x in enumerate(samples):
        MainNode.object_info_raw_callback(
            harness,
            _relative_object_entry_message(object_type, relative_x, index),
        )
        MainNode._drive_mission_zones(harness, 1.10 + index * 0.10)

    MainNode._drive_mission_zones(harness, 1.31)
    assert len(harness.runtime._mission_events[event_name]) == 1
    assert harness._fixed_entry_sent is True

    assert harness.runtime.record_lane_offset(0, 1.31)
    assert harness.runtime.record_scan(1.31)
    cycle = harness.runtime.step(1.31)

    assert cycle.transition.target is expected_state
    assert harness.runtime.context.lane_target is expected_target
    assert harness.runtime.lane_action.target_locked is True


def test_fresh_relative_x_latch_survives_cone_return_for_same_encounter():
    harness = ObjectEntryHarness(
        [1.10, 1.20, 1.30, 1.40],
        mode=Mode.CONE_DRIVE,
        state_entered_at=0.5,
    )
    harness.runtime.context.lane_target = LaneTarget.LANE_TWO

    for index, relative_x in enumerate([31.0, 30.0, 33.0]):
        MainNode.object_info_raw_callback(
            harness,
            _relative_object_entry_message(ObjectType.FIXED, relative_x, index),
        )
    assert harness.relative_x_fallback.decided is False

    harness.runtime.fsm.state = Mode.LANE_DRIVE
    harness.runtime.context.state_entered_at = 1.35
    MainNode.object_info_raw_callback(
        harness,
        _relative_object_entry_message(ObjectType.FIXED, 32.0, 2),
    )
    MainNode._drive_mission_zones(harness, 1.41)

    assert len(harness.runtime._mission_events["fixed_zone_entry"]) == 1
    assert harness.relative_x_fallback.latched_lane is ObjectLane.RIGHT


@pytest.mark.parametrize(
    ("object_type", "object_lane", "expected_state", "expected_target"),
    [
        (ObjectType.FIXED, ObjectLane.LEFT, Mode.FIXED_AVOID, LaneTarget.LANE_TWO),
        (ObjectType.FIXED, ObjectLane.RIGHT, Mode.FIXED_AVOID, LaneTarget.LANE_ONE),
        (ObjectType.MOVING, ObjectLane.LEFT, Mode.OVERTAKE, LaneTarget.LANE_TWO),
        (ObjectType.MOVING, ObjectLane.RIGHT, Mode.OVERTAKE, LaneTarget.LANE_ONE),
    ],
)
def test_object_entry_locks_fixed_and_moving_lane_mapping(
    object_type,
    object_lane,
    expected_state,
    expected_target,
):
    runtime = RaceRuntimeAdapter(
        fsm=RaceFSM(initial_state=Mode.LANE_DRIVE),
        context=RaceContext(state_entered_at=1.0),
    )
    assert runtime.record_object_info(
        _object_entry_message(object_type, object_lane).data,
        1.10,
    ).accepted
    snapshot = runtime.latest_object_snapshot
    assert snapshot is not None
    assert runtime.record_object_mission_entry(
        expected_state,
        snapshot,
        1.20,
    ).accepted

    cycle = runtime.step(1.20)

    assert cycle.transition.target is expected_state
    assert runtime.context.lane_target is expected_target
    assert runtime.lane_action.target is expected_target
    assert runtime.lane_action.target_locked is True
    assert runtime.lane_action.pending is True


def test_committed_entry_evidence_locks_after_object_freshness_expires():
    runtime = RaceRuntimeAdapter(
        fsm=RaceFSM(initial_state=Mode.LANE_DRIVE),
        context=RaceContext(state_entered_at=1.0),
        object_max_age_s=0.1,
    )
    assert runtime.record_object_info(
        _object_entry_message(ObjectType.FIXED, ObjectLane.LEFT).data,
        1.10,
    ).accepted
    snapshot = runtime.latest_object_snapshot
    assert snapshot is not None
    assert runtime.record_object_mission_entry(
        Mode.FIXED_AVOID,
        snapshot,
        1.20,
    ).accepted

    cycle = runtime.step(1.50)

    assert cycle.transition.target is Mode.FIXED_AVOID
    assert runtime.context.lane_target is LaneTarget.LANE_TWO
    assert runtime.lane_action.target is LaneTarget.LANE_TWO
    assert runtime.lane_action.target_locked is True


@pytest.mark.parametrize(
    ("object_type", "expected_state"),
    [
        (ObjectType.FIXED, Mode.FIXED_AVOID),
        (ObjectType.MOVING, Mode.OVERTAKE),
    ],
)
def test_production_unknown_lane_entry_is_rejected_without_an_edge(
    object_type,
    expected_state,
):
    runtime = RaceRuntimeAdapter(
        fsm=RaceFSM(initial_state=Mode.LANE_DRIVE),
        context=RaceContext(state_entered_at=1.0),
    )
    assert runtime.record_object_info(
        _object_entry_message(object_type, ObjectLane.UNKNOWN).data,
        1.10,
    ).accepted
    snapshot = runtime.latest_object_snapshot
    assert snapshot is not None

    result = runtime.record_object_mission_entry(expected_state, snapshot, 1.20)
    cycle = runtime.step(1.20)

    assert result.accepted is False
    assert "LEFT or RIGHT" in result.warning
    assert cycle.observation.fixed_zone_entered is False
    assert cycle.observation.overtake_entered is False
    assert cycle.transition.changed is False
    assert runtime.fsm.state is Mode.LANE_DRIVE
    assert runtime.context.lane_target is LaneTarget.CENTER
    assert runtime._pending_object_entry_evidence is None


@pytest.mark.parametrize(
    ("receipt_at", "now"),
    [(0.90, 1.10), (1.10, 1.71)],
    ids=["pre-lane-session", "stale"],
)
def test_unqualified_object_snapshot_never_becomes_entry_evidence(
    receipt_at,
    now,
):
    harness = ObjectEntryHarness(
        [receipt_at],
        mode=Mode.LANE_DRIVE,
        state_entered_at=1.0,
    )

    cycle = _record_object_entry(
        harness,
        ObjectType.MOVING,
        ObjectLane.RIGHT,
        now=now,
    )

    assert cycle.transition.changed is False
    assert harness.runtime.fsm.state is Mode.LANE_DRIVE
    assert harness.runtime.context.lane_target is LaneTarget.CENTER
    assert harness.runtime._pending_object_entry_evidence is None


def test_entry_evidence_is_discarded_when_safety_commits_another_state():
    runtime = RaceRuntimeAdapter(
        fsm=RaceFSM(initial_state=Mode.LANE_DRIVE),
        context=RaceContext(
            state_entered_at=1.0,
            lane_target=LaneTarget.CENTER,
        ),
    )
    assert runtime.record_object_info(
        _object_entry_message(ObjectType.FIXED, ObjectLane.LEFT).data,
        1.10,
    ).accepted
    snapshot = runtime.latest_object_snapshot
    assert snapshot is not None
    assert runtime.record_object_mission_entry(
        Mode.FIXED_AVOID,
        snapshot,
        1.20,
    ).accepted

    stopped = runtime.step(1.20, fault_reason="test safety fault")
    assert stopped.transition.target is Mode.LANE_DRIVE
    assert runtime._pending_object_entry_evidence is None

    runtime.fsm.state = Mode.FIXED_AVOID
    runtime.context.state_entered_at = 1.30
    runtime.step(1.40)
    assert runtime.context.lane_target is LaneTarget.CENTER
    assert runtime.lane_action.pending is False
    assert runtime.lane_action.target_locked is False


def test_lane_action_lock_resets_for_a_new_object_mission():
    runtime = RaceRuntimeAdapter(
        fsm=RaceFSM(initial_state=Mode.LANE_DRIVE),
        context=RaceContext(state_entered_at=1.0),
    )
    assert runtime.record_object_info(
        _object_entry_message(ObjectType.FIXED, ObjectLane.RIGHT).data,
        1.10,
    ).accepted
    first_snapshot = runtime.latest_object_snapshot
    assert first_snapshot is not None
    assert runtime.record_object_mission_entry(
        Mode.FIXED_AVOID,
        first_snapshot,
        1.20,
    ).accepted
    runtime.step(1.20)
    assert runtime.context.lane_target is LaneTarget.LANE_ONE
    assert runtime.lane_action.target_locked is True

    assert runtime.record_fixed_zone_exit(1.30).accepted
    exited = runtime.step(1.30)
    assert exited.transition.target is Mode.LANE_DRIVE
    assert runtime.context.lane_target is LaneTarget.LANE_ONE
    assert runtime.lane_action.target_locked is False

    assert runtime.record_object_info(
        _object_entry_message(ObjectType.MOVING, ObjectLane.LEFT).data,
        1.40,
    ).accepted
    second_snapshot = runtime.latest_object_snapshot
    assert second_snapshot is not None
    assert runtime.record_object_mission_entry(
        Mode.OVERTAKE,
        second_snapshot,
        1.50,
    ).accepted
    entered = runtime.step(1.50)

    assert entered.transition.target is Mode.OVERTAKE
    assert runtime.context.lane_target is LaneTarget.LANE_TWO
    assert runtime.lane_action.target is LaneTarget.LANE_TWO
    assert runtime.lane_action.target_locked is True


@pytest.mark.parametrize(
    "data",
    [
        [1.0] * 9,
        [1.0, float("nan"), 0.1, 0.2, 5.0, 100.0, 20.0, 30.0, 4.0, 1.0],
        [1.0, 1.2, 0.1, 0.2, 5.0, 100.0, 20.0, 30.0, 4.0, 3.0],
    ],
)
def test_main_object_raw_callback_warns_on_invalid_payload(data):
    harness = CallbackHarness([4.0])

    MainNode.object_info_raw_callback(harness, SimpleNamespace(data=data))

    assert harness.runtime.latest_object_snapshot is None
    assert harness.warnings[0][0] == "malformed_object_info_raw"


def test_main_lane_change_callback_records_one_fresh_success_edge():
    harness = CallbackHarness([5.0, 5.1], mode=Mode.FIXED_AVOID)

    MainNode.lane_change_state_callback(
        harness,
        SimpleNamespace(data=[1, 1]),
    )
    first = harness.runtime.step(5.02)
    MainNode.lane_change_state_callback(
        harness,
        SimpleNamespace(data=[1, 1]),
    )
    sticky = harness.runtime.step(5.12)

    assert first.observation.lane_change_received_at == 5.0
    assert first.observation.lane_change_success_edge is True
    assert sticky.observation.lane_change_success_edge is False
    assert harness.warnings == []


def test_main_lane_change_callback_warns_on_invalid_payload():
    harness = CallbackHarness([5.0], mode=Mode.FIXED_AVOID)

    MainNode.lane_change_state_callback(
        harness,
        SimpleNamespace(data=[1]),
    )

    assert harness.warnings[0][0] == "malformed_lane_change_state"


def test_callback_and_control_cycle_read_the_same_node_clock_method():
    callback_source = inspect.getsource(MainNode.rubbercone_callback)
    cycle_source = inspect.getsource(MainNode.control_cycle)

    assert "received_at = self._now_seconds()" in callback_source
    assert "now = self._now_seconds()" in cycle_source
    assert "self.runtime.step(" in cycle_source


def test_rubbercone_topics_and_session_qos_match_detector_contract():
    qos = rubbercone_session_qos()

    assert RUBBERCONE_INFO_TOPIC == "/rubbercone_info"
    assert RUBBERCONE_OFFSET_TOPIC == "/rubbercone_offset"
    assert RUBBERCONE_SESSION_ACTIVE_TOPIC == "/rubbercone_session_active"
    assert qos.history is HistoryPolicy.KEEP_LAST
    assert qos.depth == 1
    assert qos.reliability is ReliabilityPolicy.RELIABLE
    assert qos.durability is DurabilityPolicy.TRANSIENT_LOCAL


def test_sensor_qos_is_compatible_with_cone_and_lidar_publishers():
    subscriber_qos = sensor_event_qos()
    rubbercone_publisher_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )

    cone_compatibility, _ = qos_check_compatible(
        rubbercone_publisher_qos,
        subscriber_qos,
    )
    scan_compatibility, _ = qos_check_compatible(
        qos_profile_sensor_data,
        subscriber_qos,
    )

    assert subscriber_qos.reliability is ReliabilityPolicy.BEST_EFFORT
    assert subscriber_qos.depth == 1
    assert cone_compatibility is not QoSCompatibility.ERROR
    assert scan_compatibility is not QoSCompatibility.ERROR


def test_mode_info_publisher_qos_is_compatible_with_lane_detector():
    main_publisher_qos = sensor_event_qos()
    lane_detector_subscriber_qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )

    compatibility, _ = qos_check_compatible(
        main_publisher_qos,
        lane_detector_subscriber_qos,
    )

    assert compatibility is not QoSCompatibility.ERROR


def test_generated_ros_entities_have_exact_topic_type_and_qos():
    context = Context()
    rclpy.init(context=context)
    node = MainNode(context=context)
    try:
        cone_sub = node.rubbercone_info_sub
        assert cone_sub.topic_name == RUBBERCONE_INFO_TOPIC
        assert cone_sub.msg_type is Int32MultiArray
        assert cone_sub.qos_profile.history is HistoryPolicy.KEEP_LAST
        assert cone_sub.qos_profile.depth == 1
        assert cone_sub.qos_profile.reliability is ReliabilityPolicy.BEST_EFFORT
        assert cone_sub.qos_profile.durability is DurabilityPolicy.VOLATILE

        cone_offset_sub = node.rubbercone_offset_sub
        assert cone_offset_sub.topic_name == RUBBERCONE_OFFSET_TOPIC
        assert cone_offset_sub.msg_type is Int32MultiArray

        scan_sub = node.scan_sub
        assert scan_sub.topic_name == "/scan"
        assert scan_sub.msg_type is LaserScan
        assert scan_sub.qos_profile.history is HistoryPolicy.KEEP_LAST
        assert scan_sub.qos_profile.depth == 1
        assert scan_sub.qos_profile.reliability is ReliabilityPolicy.BEST_EFFORT
        assert scan_sub.qos_profile.durability is DurabilityPolicy.VOLATILE

        side_sub = node.side_clearance_sub
        assert side_sub.topic_name == SIDE_CLEARANCE_TOPIC
        assert side_sub.msg_type is Float32MultiArray
        assert side_sub.qos_profile.history is HistoryPolicy.KEEP_LAST
        assert side_sub.qos_profile.depth == 1
        assert side_sub.qos_profile.reliability is ReliabilityPolicy.BEST_EFFORT
        assert side_sub.qos_profile.durability is DurabilityPolicy.VOLATILE

        path_preview_sub = node.lane_path_preview_sub
        assert path_preview_sub.topic_name == LANE_PATH_PREVIEW_TOPIC
        assert path_preview_sub.msg_type is Float32MultiArray
        assert path_preview_sub.qos_profile.history is HistoryPolicy.KEEP_LAST
        assert path_preview_sub.qos_profile.depth == 1
        assert (
            path_preview_sub.qos_profile.reliability
            is ReliabilityPolicy.BEST_EFFORT
        )
        assert (
            path_preview_sub.qos_profile.durability
            is DurabilityPolicy.VOLATILE
        )

        session_pub = node.rubbercone_session_active_pub
        assert session_pub.topic_name == RUBBERCONE_SESSION_ACTIVE_TOPIC
        assert session_pub.msg_type is Bool
        assert session_pub.qos_profile.history is HistoryPolicy.KEEP_LAST
        assert session_pub.qos_profile.depth == 1
        assert session_pub.qos_profile.reliability is ReliabilityPolicy.RELIABLE
        assert session_pub.qos_profile.durability is DurabilityPolicy.TRANSIENT_LOCAL

        lane_change_sub = node.lane_change_state_sub
        assert lane_change_sub.topic_name == LANE_CHANGE_STATE_TOPIC
        assert lane_change_sub.msg_type is Int32MultiArray
        assert lane_change_sub.qos_profile.history is HistoryPolicy.KEEP_LAST
        assert lane_change_sub.qos_profile.depth == 10
        assert lane_change_sub.qos_profile.reliability is ReliabilityPolicy.BEST_EFFORT
        assert lane_change_sub.qos_profile.durability is DurabilityPolicy.VOLATILE

        object_sub = node.object_info_sub
        assert object_sub.topic_name == OBJECT_INFO_TOPIC
        assert object_sub.msg_type is Int32MultiArray
        assert object_sub.qos_profile.reliability is ReliabilityPolicy.BEST_EFFORT

        object_raw_sub = node.object_info_raw_sub
        assert object_raw_sub.topic_name == OBJECT_INFO_RAW_TOPIC
        assert object_raw_sub.msg_type is Float32MultiArray

        mode_pub = node.mode_pub
        assert mode_pub.topic_name == "/mode_info"
        assert mode_pub.msg_type is Int16
        assert mode_pub.qos_profile.reliability is ReliabilityPolicy.BEST_EFFORT

        lane_info_pub = node.lane_info_pub
        assert lane_info_pub.topic_name == "/lane_info"
        assert lane_info_pub.msg_type is Int16

        lane_command_pub = node.lane_command_pub
        assert lane_command_pub.topic_name == LANE_COMMAND_TOPIC
        assert lane_command_pub.msg_type is Int32MultiArray
        assert not hasattr(node, "traffic_sub")
        assert not hasattr(node, "lane_validity_sub")
    finally:
        node.destroy_node()
        rclpy.shutdown(context=context)


def test_main_bootstrap_retains_search_entry_session_state():
    context = Context()
    rclpy.init(context=context)
    main_node = MainNode(context=context)
    observer = Node("cone_session_bootstrap_observer", context=context)
    received = []
    observer.create_subscription(
        Bool,
        RUBBERCONE_SESSION_ACTIVE_TOPIC,
        lambda message: received.append(message.data),
        rubbercone_session_qos(),
    )
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(observer)
    try:
        deadline = time.monotonic() + 2.0
        while not received and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert received == [False]
    finally:
        executor.remove_node(observer)
        executor.shutdown()
        observer.destroy_node()
        main_node.destroy_node()
        rclpy.shutdown(context=context)


def test_lane_change_feedback_uses_existing_topic_without_new_mission_topics():
    assert LANE_CHANGE_STATE_TOPIC == "/lane_change_state"
    source = inspect.getsource(MainNode.__init__)

    assert "LANE_CHANGE_STATE_TOPIC" in source
    assert "fixed_zone_exit" not in source
    assert "overtake_complete" not in source
    assert "shortcut_complete" not in source


def test_main_object_info_maps_straight_to_green_and_route_signal():
    harness = CallbackHarness([6.0], mode=Mode.WAIT_GREEN)

    MainNode.object_info_callback(harness, SimpleNamespace(data=[2, 0, 0]))
    cycle = harness.runtime.step(6.0)

    assert cycle.observation.green_detected is True
    assert cycle.observation.route_traffic_signal is RouteTrafficSignal.STRAIGHT
    assert cycle.observation.traffic_encounter_started is True
    assert harness.warnings == []


def test_main_object_info_red_sets_route_stop_override():
    harness = CallbackHarness([6.0], mode=Mode.LANE_DRIVE)

    MainNode.object_info_callback(harness, SimpleNamespace(data=[1, 0, 0]))
    cycle = harness.runtime.step(6.0)

    assert cycle.observation.green_detected is False
    assert cycle.observation.route_traffic_signal is RouteTrafficSignal.RED_AMBER
    assert harness.runtime.traffic_stop_override is True
    assert harness.warnings == []


def test_main_traffic_encounter_is_one_edge_until_unknown_rearms():
    harness = CallbackHarness(
        [7.0, 7.1, 7.2, 7.3],
        mode=Mode.LANE_DRIVE,
    )

    MainNode.object_info_callback(harness, SimpleNamespace(data=[2, 0, 0]))
    first = harness.runtime.step(7.0)

    MainNode.object_info_callback(harness, SimpleNamespace(data=[2, 0, 0]))
    repeated = harness.runtime.step(7.1)

    MainNode.object_info_callback(harness, SimpleNamespace(data=[0, 0, 0]))
    harness.runtime.step(7.2)

    MainNode.object_info_callback(harness, SimpleNamespace(data=[2, 0, 0]))
    second = harness.runtime.step(7.3)

    assert first.observation.traffic_encounter_started is True
    assert repeated.observation.traffic_encounter_started is False
    assert second.observation.traffic_encounter_started is True
    assert harness.runtime.context.completed_laps == 2


def test_main_object_info_invalid_value_is_rejected():
    harness = CallbackHarness([8.0], mode=Mode.LANE_DRIVE)

    MainNode.object_info_callback(harness, SimpleNamespace(data=[9, 0, 0]))

    assert "object_info" not in harness.runtime.perception_received_at
    assert harness.warnings[0][0] == "malformed_object_info"


@pytest.mark.parametrize(
    "data",
    [
        [0, 0],
        [0, 0, 0, 0],
        [0, 3, 0],
        [0, 0, -1],
        [True, 0, 0],
        [0.0, 0, 0],
    ],
)
def test_main_object_info_rejects_malformed_ppt_payload(data):
    harness = CallbackHarness([8.0], mode=Mode.LANE_DRIVE)

    MainNode.object_info_callback(harness, SimpleNamespace(data=data))

    assert "object_info" not in harness.runtime.perception_received_at
    assert harness.warnings[0][0] == "malformed_object_info"


@pytest.mark.parametrize(
    ("value", "expected"),
    [
        (0, Mode.WAIT_GREEN),
        ("WAIT_TRAFFIC", Mode.WAIT_GREEN),
        (1, Mode.LANE_DRIVE),
        ("2", Mode.CONE_DRIVE),
        (3, Mode.FIXED_AVOID),
        (4, Mode.OVERTAKE),
        (5, Mode.SHORTCUT),
        ("FINISH", Mode.FINISH),
        ("LANE_DRIVE", Mode.LANE_DRIVE),
    ],
)
def test_initial_mode_parser_supports_defined_runtime_states(value, expected):
    assert parse_initial_mode(value) is expected


@pytest.mark.parametrize(
    "value",
    [-1, 6, 9, "INIT", "REJOIN", "BEFORE", "CHANGE_LANE", True],
)
def test_initial_mode_parser_rejects_undefined_legacy_state_guesses(value):
    with pytest.raises(ValueError):
        parse_initial_mode(value)


def test_bag_loop_backjump_resets_the_state_machine():
    """bag이 처음부터 재생되면 상태머신을 시작 모드로 되돌린다.

    시각이 뒤로 튀면 이전 루프의 수신 시각이 전부 '미래'가 되어 신선도 검사가
    음수로 나온다. 끊긴 입력이 신선한 것처럼 보이므로 상태를 버려야 한다.
    """
    import rclpy
    from main.main import MainNode, BAG_LOOP_BACKJUMP_S
    from main.race_fsm import Mode

    rclpy.init(args=['--ros-args', '-p', 'mode:=2'])
    try:
        node = MainNode()
        assert node._initial_mode is Mode.CONE_DRIVE

        node.runtime.fsm.state = Mode.OVERTAKE
        node.box_size = 5000.0
        node.side_right = 0.3
        before = node.runtime

        node._now_seconds = lambda: 100.0
        node.control_cycle()
        # 시각이 크게 역행 → 리셋
        node._now_seconds = lambda: 100.0 - (BAG_LOOP_BACKJUMP_S + 1.0)
        node.control_cycle()

        assert node.runtime is not before          # 어댑터를 새로 만들었다
        assert node.runtime.fsm.state is Mode.CONE_DRIVE
        assert node.box_size == 0.0
        assert node.side_right == float("inf")
        assert node._zone_exit_sent is False
    finally:
        rclpy.shutdown()


def test_small_backward_jitter_does_not_reset():
    """실차 wall clock의 미세 역행(NTP 보정)은 리셋 트리거가 아니다."""
    import rclpy
    from main.main import MainNode
    from main.race_fsm import Mode

    rclpy.init(args=['--ros-args', '-p', 'mode:=2'])
    try:
        node = MainNode()
        node.runtime.fsm.state = Mode.OVERTAKE
        before = node.runtime

        node._now_seconds = lambda: 100.0
        node.control_cycle()
        node._now_seconds = lambda: 99.9      # 0.1초 역행
        node.control_cycle()

        assert node.runtime is before
    finally:
        rclpy.shutdown()


def _node_in_overtake(**overrides):
    import rclpy
    from main.main import MainNode
    from main.race_fsm import Mode
    node = MainNode()
    node.runtime.fsm.state = Mode.OVERTAKE
    node._enter_zone(0.0)
    for k, v in overrides.items():
        setattr(node, k, v)
    return node


def test_is_pass_comp_delegates_to_the_pure_guard():
    """판정 로직은 main.overtake가 갖고, 노드는 배선만 한다."""
    import rclpy
    from main.main import MainNode
    from main.race_fsm import Mode

    rclpy.init(args=['--ros-args', '-p', 'mode:=2'])
    try:
        node = MainNode()
        node.runtime.fsm.state = Mode.OVERTAKE
        node._enter_zone(0.0)
        node.detected_lane = 1            # 1차선(왼쪽) 주행 → 오른쪽을 본다
        # bag 실측 통과 거리(0.26~0.34 m) 안쪽 값. side_detect_m 미만이어야 한다.
        delay = node.overtake.config.pass_delay_s
        assert node.runtime.record_side_clearance(float("inf"), 0.30, 1.0)
        assert node.is_pass_comp(1.0) is False
        assert node.overtake.side_seen_at == 1.0

        # 완료 타이머는 장애물이 감지된 시점이 아니라, 측면에서 사라진
        # 시점부터 시작한다. 같은 근거리 값이 유지되면 고착 센서로 보고
        # fail-closed 상태를 유지해야 한다.
        clear_started_at = 1.1
        assert node.runtime.record_side_clearance(
            float("inf"),
            float("inf"),
            clear_started_at,
        )
        assert node.is_pass_comp(clear_started_at) is False
        assert node.overtake.clear_started_at == clear_started_at
        completed = False
        for now in (1.5, 1.9, 2.3, 2.7, clear_started_at + delay):
            assert node.runtime.record_side_clearance(
                float("inf"),
                float("inf"),
                now,
            )
            completed = node.is_pass_comp(now)
        assert completed is True
    finally:
        rclpy.shutdown()


def test_pass_completion_requires_fresh_post_zone_side_semantics():
    import rclpy
    from main.main import MainNode
    from main.race_fsm import Mode

    rclpy.init(args=['--ros-args', '-p', 'mode:=2'])
    try:
        node = MainNode()
        node.runtime.fsm.state = Mode.OVERTAKE
        node._enter_zone(1.0)
        node.detected_lane = 1

        assert node.is_pass_comp(1.0) is False
        assert node.runtime.record_side_clearance(float("inf"), 0.30, 0.9)
        assert node.is_pass_comp(1.0) is False
        assert node.overtake.side_seen_at is None

        assert node.runtime.record_side_clearance(float("inf"), 0.30, 1.2)
        assert node.is_pass_comp(1.1) is False
        assert node.overtake.side_seen_at is None

        assert node.runtime.record_side_clearance(float("inf"), 0.30, 1.3)
        assert node.is_pass_comp(1.3) is False
        assert node.overtake.side_seen_at == pytest.approx(1.3)

        assert node.runtime.record_side_clearance(
            float("inf"), float("inf"), 1.4
        )
        assert node.is_pass_comp(1.4) is False
        assert node.overtake.clear_started_at == pytest.approx(1.4)

        assert node.is_pass_comp(2.0) is False
        assert node.overtake.clear_started_at is None
        assert node.overtake.side_seen_at == pytest.approx(1.3)

        assert node.runtime.record_side_clearance(
            float("inf"), float("inf"), 2.1
        )
        assert node.is_pass_comp(2.1) is False
        assert node.overtake.clear_started_at == pytest.approx(2.1)
    finally:
        rclpy.shutdown()


def test_is_change_end_follows_lane_change_state_feedback():
    import rclpy

    rclpy.init(args=['--ros-args', '-p', 'mode:=2'])
    try:
        node = _node_in_overtake()
        assert node.is_change_end() is False
        node.runtime.lane_action.completed = True
        assert node.is_change_end() is True
    finally:
        rclpy.shutdown()


class ShapeHarness:
    """_shape_selected_control 만 떼어 보기 위한 최소 상태."""

    def __init__(self, *, now_speed=0.0, now_angle=0.0, race_started_at=None):
        self.runtime = SimpleNamespace(
            context=RaceContext(race_started_at=race_started_at),
        )
        self.now_speed = now_speed
        self.now_angle = now_angle


def test_hold_source_ramps_speed_down_and_holds_the_steering_angle():
    """인지 한 프레임 공백이 완전 정지로 번지지 않게 램프로 감속한다.

    예전에는 ControlSource.HOLD 이면 속도를 즉시 0으로, 조향을 0으로 꺾었다.
    재가속이 사이클당 +0.1(=5/초)이라 21.5까지 4초 넘게 걸렸고, 그 전에 다음
    stale 이 와서 차가 앞으로 못 나갔다 (실측 평균 속도 2.24).
    """
    from main.control_selector import ControlSource, DriveCommand
    from main.main import MainNode, STOP_DECEL_STEP

    harness = ShapeHarness(now_speed=21.5, now_angle=12.0)

    angle, speed = MainNode._shape_selected_control(
        harness,
        ControlSource.HOLD,
        DriveCommand(0.0, 0.0),
        1.0,
    )

    assert speed == pytest.approx(21.5 - STOP_DECEL_STEP)
    # 곡선 주행 중 앞바퀴를 펴면 감속하는 동안 코스 밖으로 밀려난다.
    assert angle == pytest.approx(12.0)


def test_hold_source_still_reaches_a_full_stop_quickly():
    from main.control_selector import ControlSource, DriveCommand
    from main.main import MainNode

    harness = ShapeHarness(now_speed=21.5)

    speed = 21.5
    cycles = 0
    while speed > 0.0 and cycles < 200:
        _, speed = MainNode._shape_selected_control(
            harness,
            ControlSource.HOLD,
            DriveCommand(0.0, 0.0),
            1.0,
        )
        cycles += 1

    assert speed == 0.0
    assert cycles <= 40          # 50 Hz 기준 0.8초 이내


def test_lane_source_recovers_cruise_speed_in_about_one_second():
    from main.control_selector import ControlSource, DriveCommand
    from main.main import MainNode

    harness = ShapeHarness()

    speed = 0.0
    cycles = 0
    while speed < 21.5 and cycles < 500:
        _, speed = MainNode._shape_selected_control(
            harness,
            ControlSource.LANE,
            DriveCommand(3.0, 21.5),
            1.0,
        )
        cycles += 1

    assert speed == pytest.approx(21.5)
    assert cycles <= 60          # 50 Hz 기준 1.2초 이내


@pytest.mark.parametrize(
    ("code", "green"),
    [(0, False), (1, False), (2, True), (3, True)],
)
def test_object_info_maps_signal_codes_to_green(code, green):
    """0=미검출, 1=적색/주황, 2=녹색, 3=좌회전 녹색."""
    from main.main import MainNode

    harness = CallbackHarness([7.0], mode=Mode.WAIT_GREEN)

    MainNode.object_info_callback(harness, SimpleNamespace(data=[code, 0, 0]))
    cycle = harness.runtime.step(7.01)

    assert harness.traffic_signal == code
    assert cycle.observation.green_detected is green
    assert cycle.observation.traffic_message_received_at == 7.0


# ─────────────────────────────────────────────────────────────────────────────
# 가드레일 이득 ROS 파라미터
#
# 실차에서 이득을 찾는 동안 재빌드/재실행을 없애려고 연 통로다. 파라미터가
# 컨트롤러까지 실제로 닿는지, 잘못된 값이 거절되는지, bag --loop 되감기로
# 컨트롤러가 새로 만들어져도 살아남는지를 확인한다.
# ─────────────────────────────────────────────────────────────────────────────

def test_guardrail_parameters_reach_the_controller_at_launch():
    from rclpy.parameter import Parameter
    from main.main import MainNode

    rclpy.init()
    try:
        node = MainNode()
        node.set_parameters([Parameter('guardrail_gain_deg', value=12.5)])
        assert node.lane_controller.guardrail_params['gain_deg'] == 12.5
        assert node.cone_controller.guardrail_params['gain_deg'] == 12.5
    finally:
        rclpy.shutdown()


def test_guardrail_parameter_launch_override_is_applied():
    from main.main import MainNode

    rclpy.init(args=['--ros-args', '-p', 'guardrail_gain_deg:=7.0'])
    try:
        node = MainNode()
        assert node.lane_controller.guardrail_params['gain_deg'] == 7.0
    finally:
        rclpy.shutdown()


def test_guardrail_parameter_rejects_bad_value_and_keeps_the_old_one():
    from rclpy.parameter import Parameter
    from main.main import MainNode

    rclpy.init()
    try:
        node = MainNode()
        before = node.lane_controller.guardrail_params['gain_deg']
        result = node.set_parameters([Parameter('guardrail_gain_deg', value=-5.0)])[0]
        assert not result.successful
        assert 'gain_deg' in result.reason
        assert node.lane_controller.guardrail_params['gain_deg'] == before
    finally:
        rclpy.shutdown()


def test_guardrail_atomic_set_is_all_or_nothing():
    """한 콜백에 여러 항목이 같이 오면, 하나가 거절될 때 나머지도 적용되면 안 된다.

    set_parameters(비원자)는 항목마다 콜백을 따로 부르므로 요청끼리 독립이다
    (ros2 param set 도 한 번에 하나다). 원자 경로에서만 이 계약이 의미가 있다.
    """
    from rclpy.parameter import Parameter
    from main.main import MainNode

    rclpy.init()
    try:
        node = MainNode()
        before = dict(node.lane_controller.guardrail_params)
        result = node.set_parameters_atomically([
            Parameter('guardrail_gain_deg', value=11.0),
            Parameter('guardrail_rate_deg', value=-1.0),
        ])
        assert not result.successful
        assert node.lane_controller.guardrail_params == before
    finally:
        rclpy.shutdown()


def test_lane_guardrail_toggle_takes_effect_at_runtime():
    from rclpy.parameter import Parameter
    from main.main import MainNode

    rclpy.init()
    try:
        node = MainNode()
        assert node.lane_guardrail_enabled
        node.set_parameters([Parameter('lane_guardrail', value=False)])
        assert not node.lane_guardrail_enabled
    finally:
        rclpy.shutdown()


def test_curve_preview_toggle_rolls_back_at_runtime():
    from rclpy.parameter import Parameter
    from main.main import MainNode

    rclpy.init()
    try:
        node = MainNode()
        now = node._now_seconds()
        assert node.runtime.record_lane_path_preview(
            80.0, 0.2, 0.9, 0.25, 10.0, now
        )
        preview = node._curve_preview_for(Mode.LANE_DRIVE, now, 10.0)
        assert preview[:2] == (80.0, 0.2)
        assert preview[2] == pytest.approx(
            (0.9 - node.curve_preview_min_confidence)
            / (1.0 - node.curve_preview_min_confidence)
        )

        result = node.set_parameters([
            Parameter('curve_preview_enabled', value=False)
        ])[0]
        assert result.successful
        assert not node.curve_preview_enabled
        assert node._curve_preview_for(Mode.LANE_DRIVE, now, 10.0) is None
    finally:
        rclpy.shutdown()


def test_curve_preview_automatically_falls_back_on_low_confidence_or_skew():
    from main.main import LANE_PATH_PREVIEW_MAX_SKEW_S, MainNode

    rclpy.init()
    try:
        node = MainNode()
        now = node._now_seconds()
        low = node.curve_preview_min_confidence - 0.01
        assert node.runtime.record_lane_path_preview(
            80.0, 0.2, low, 0.25, 10.0, now
        )
        assert node._curve_preview_for(Mode.LANE_DRIVE, now, 10.0) is None

        later = now + 1.0
        assert node.runtime.record_lane_path_preview(
            80.0, 0.2, 0.9, 0.25, 10.0, later
        )
        old_lane_receipt = later - LANE_PATH_PREVIEW_MAX_SKEW_S - 0.01
        assert node._curve_preview_for(
            Mode.LANE_DRIVE, old_lane_receipt, 10.0
        ) is None
        assert node._curve_preview_for(
            Mode.LANE_DRIVE, later, 11.0
        ) is None
        assert node._curve_preview_for(
            Mode.FIXED_AVOID, later, 10.0
        ) is None
    finally:
        rclpy.shutdown()


def test_curve_preview_is_disabled_by_authoritative_lane_action_pending():
    from main.main import MainNode

    rclpy.init()
    try:
        node = MainNode()
        now = node._now_seconds()
        assert node.runtime.record_lane_path_preview(
            80.0, 0.2, 0.9, 0.25, 10.0, now
        )
        node.runtime.lane_action.pending = True

        assert node._curve_preview_for(
            Mode.LANE_DRIVE, now, 10.0
        ) is None
    finally:
        rclpy.shutdown()


def test_curve_preview_confidence_parameter_rejects_out_of_range_value():
    from rclpy.parameter import Parameter
    from main.main import MainNode

    rclpy.init()
    try:
        node = MainNode()
        before = node.curve_preview_min_confidence
        result = node.set_parameters([
            Parameter('curve_preview_min_confidence', value=1.5)
        ])[0]
        assert not result.successful
        assert node.curve_preview_min_confidence == before
    finally:
        rclpy.shutdown()


def test_guardrail_parameters_survive_runtime_reset():
    """bag --loop 되감기는 Controller를 새로 만든다. 맞춰 둔 이득이 날아가면 안 된다."""
    from rclpy.parameter import Parameter
    from main.main import MainNode

    rclpy.init()
    try:
        node = MainNode()
        node.set_parameters([Parameter('guardrail_gain_deg', value=9.0)])
        node._reset_for_bag_loop()
        assert node.lane_controller.guardrail_params['gain_deg'] == 9.0
    finally:
        rclpy.shutdown()
