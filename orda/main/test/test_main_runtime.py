import inspect
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.context import Context
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
from std_msgs.msg import Bool, Empty, Float32MultiArray, Int32, Int32MultiArray

from main.main import (
    LANE_VALIDITY_REQUIRED_RATE_HZ,
    LANE_VALIDITY_TOPIC,
    LANE_CHANGE_STATE_TOPIC,
    MainNode,
    RUBBERCONE_INFO_TOPIC,
    RUBBERCONE_RESET_TOPIC,
    lane_validity_qos,
    parse_initial_mode,
    rubbercone_reset_qos,
    sensor_event_qos,
)
from main.race_context import RaceContext
from main.race_fsm import Mode, RaceFSM
from main.runtime_adapter import RaceRuntimeAdapter
from main.mission_types import ObjectLane, RouteTrafficSignal


class CallbackHarness:
    def __init__(self, times, mode=Mode.CONE_DRIVE):
        self._times = iter(times)
        self.runtime = RaceRuntimeAdapter(
            fsm=RaceFSM(initial_state=mode),
            context=RaceContext(state_entered_at=0.5),
        )
        self.warnings = []
        self._traffic_encounter_active = False

    def _now_seconds(self):
        return next(self._times)

    def _warn_throttled(self, key, message, now):
        self.warnings.append((key, message, now))


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


def test_main_scan_and_cone_callbacks_use_the_same_clock_helper():
    harness = CallbackHarness([3.0, 3.01])

    MainNode.scan_callback(harness, SimpleNamespace())
    MainNode.rubbercone_callback(
        harness,
        SimpleNamespace(data=[0, 0, 80]),
    )
    cycle = harness.runtime.step(3.02)

    assert cycle.observation.scan_received_at == 3.0
    assert cycle.observation.cone_message_received_at == 3.01
    assert cycle.observation.now == 3.02


def test_main_object_callback_records_validated_ten_field_snapshot():
    harness = CallbackHarness([4.0])
    message = SimpleNamespace(
        data=[1.0, 1.2, 0.1, 0.2, 5.0, 100.0, 20.0, 30.0, 4.0, 2.0],
    )

    MainNode.object_info_callback(harness, message)
    cycle = harness.runtime.step(4.02)

    assert cycle.observation.object_exists is True
    assert cycle.observation.object_distance == pytest.approx(1.2)
    assert cycle.observation.object_lane is ObjectLane.RIGHT
    assert cycle.observation.object_received_at == 4.0
    assert harness.warnings == []


def test_main_object_callback_accepts_and_normalizes_no_cluster_heartbeat():
    harness = CallbackHarness([4.0], mode=Mode.FIXED_AVOID)
    message = SimpleNamespace(
        data=[0.0, float("inf"), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    )

    MainNode.object_info_callback(harness, message)
    cycle = harness.runtime.step(4.02)

    assert cycle.observation.object_exists is False
    assert cycle.observation.object_distance is None
    assert cycle.observation.object_lane is ObjectLane.UNKNOWN
    assert cycle.observation.object_received_at == 4.0
    assert harness.runtime.lane_action.pending is False
    assert harness.warnings == []


@pytest.mark.parametrize(
    "data",
    [
        [1.0] * 9,
        [1.0, float("nan"), 0.1, 0.2, 5.0, 100.0, 20.0, 30.0, 4.0, 1.0],
        [1.0, 1.2, 0.1, 0.2, 5.0, 100.0, 20.0, 30.0, 4.0, 3.0],
    ],
)
def test_main_object_callback_warns_on_invalid_payload(data):
    harness = CallbackHarness([4.0])

    MainNode.object_info_callback(harness, SimpleNamespace(data=data))

    assert harness.runtime.latest_object_snapshot is None
    assert harness.warnings[0][0] == "malformed_object_info"


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


def test_rubbercone_topics_and_reset_qos_match_detector_contract():
    qos = rubbercone_reset_qos()

    assert RUBBERCONE_INFO_TOPIC == "/rubbercone_info"
    assert RUBBERCONE_RESET_TOPIC == "/rubbercone_reset"
    assert qos.history is HistoryPolicy.KEEP_LAST
    assert qos.depth == 10
    assert qos.reliability is ReliabilityPolicy.RELIABLE
    assert qos.durability is DurabilityPolicy.VOLATILE


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

        scan_sub = node.scan_sub
        assert scan_sub.topic_name == "/scan"
        assert scan_sub.msg_type is LaserScan
        assert scan_sub.qos_profile.history is HistoryPolicy.KEEP_LAST
        assert scan_sub.qos_profile.depth == 1
        assert scan_sub.qos_profile.reliability is ReliabilityPolicy.BEST_EFFORT
        assert scan_sub.qos_profile.durability is DurabilityPolicy.VOLATILE

        reset_pub = node.rubbercone_reset_pub
        assert reset_pub.topic_name == RUBBERCONE_RESET_TOPIC
        assert reset_pub.msg_type is Empty
        assert reset_pub.qos_profile.history is HistoryPolicy.KEEP_LAST
        assert reset_pub.qos_profile.depth == 10
        assert reset_pub.qos_profile.reliability is ReliabilityPolicy.RELIABLE
        assert reset_pub.qos_profile.durability is DurabilityPolicy.VOLATILE

        validity_sub = node.lane_validity_sub
        assert validity_sub.topic_name == LANE_VALIDITY_TOPIC
        assert validity_sub.msg_type is Bool
        assert validity_sub.qos_profile == lane_validity_qos()

        lane_change_sub = node.lane_change_state_sub
        assert lane_change_sub.topic_name == LANE_CHANGE_STATE_TOPIC
        assert lane_change_sub.msg_type is Int32MultiArray
        assert lane_change_sub.qos_profile.history is HistoryPolicy.KEEP_LAST
        assert lane_change_sub.qos_profile.depth == 10
        assert lane_change_sub.qos_profile.reliability is ReliabilityPolicy.BEST_EFFORT
        assert lane_change_sub.qos_profile.durability is DurabilityPolicy.VOLATILE

        object_sub = node.object_info_sub
        assert object_sub.topic_name == "/object_info"
        assert object_sub.msg_type is Float32MultiArray
        assert object_sub.qos_profile.reliability is ReliabilityPolicy.BEST_EFFORT

        traffic_sub = node.traffic_sub
        assert traffic_sub.topic_name == "/traffic_detection"
        assert traffic_sub.msg_type is Int32
        assert traffic_sub.qos_profile.reliability is ReliabilityPolicy.BEST_EFFORT

        mode_pub = node.mode_pub
        assert mode_pub.topic_name == "/mode_info"
        assert mode_pub.msg_type is Int32MultiArray
        assert mode_pub.qos_profile.reliability is ReliabilityPolicy.BEST_EFFORT
    finally:
        node.destroy_node()
        rclpy.shutdown(context=context)


def test_lane_validity_publisher_contract_is_explicit():
    qos = lane_validity_qos()

    assert LANE_VALIDITY_TOPIC == "/lane_valid"
    assert LANE_VALIDITY_REQUIRED_RATE_HZ == 10.0
    assert qos.history is HistoryPolicy.KEEP_LAST
    assert qos.depth == 10
    assert qos.reliability is ReliabilityPolicy.BEST_EFFORT
    assert qos.durability is DurabilityPolicy.VOLATILE


def test_lane_change_feedback_uses_existing_topic_without_new_mission_topics():
    assert LANE_CHANGE_STATE_TOPIC == "/lane_change_state"
    source = inspect.getsource(MainNode.__init__)

    assert "LANE_CHANGE_STATE_TOPIC" in source
    assert "fixed_avoid_complete" not in source
    assert "overtake_complete" not in source
    assert "shortcut_complete" not in source


def test_main_traffic_int32_maps_straight_to_green_and_route_signal():
    harness = CallbackHarness([6.0], mode=Mode.WAIT_GREEN)

    MainNode.traffic_callback(harness, SimpleNamespace(data=2))
    cycle = harness.runtime.step(6.0)

    assert cycle.observation.green_detected is True
    assert cycle.observation.route_traffic_signal is RouteTrafficSignal.STRAIGHT
    assert cycle.observation.traffic_encounter_started is True
    assert harness.warnings == []


def test_main_traffic_int32_red_sets_route_stop_override():
    harness = CallbackHarness([6.0], mode=Mode.LANE_DRIVE)

    MainNode.traffic_callback(harness, SimpleNamespace(data=1))
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

    MainNode.traffic_callback(harness, SimpleNamespace(data=2))
    first = harness.runtime.step(7.0)

    MainNode.traffic_callback(harness, SimpleNamespace(data=2))
    repeated = harness.runtime.step(7.1)

    MainNode.traffic_callback(harness, SimpleNamespace(data=0))
    harness.runtime.step(7.2)

    MainNode.traffic_callback(harness, SimpleNamespace(data=2))
    second = harness.runtime.step(7.3)

    assert first.observation.traffic_encounter_started is True
    assert repeated.observation.traffic_encounter_started is False
    assert second.observation.traffic_encounter_started is True
    assert harness.runtime.context.completed_laps == 2


def test_main_traffic_invalid_value_is_rejected():
    harness = CallbackHarness([8.0], mode=Mode.LANE_DRIVE)

    MainNode.traffic_callback(harness, SimpleNamespace(data=9))

    assert "traffic_detection" not in harness.runtime.perception_received_at
    assert harness.warnings[0][0] == "malformed_traffic"


@pytest.mark.parametrize(
    ("value", "expected"),
    [
        (0, Mode.INIT),
        ("INIT", Mode.INIT),
        (1, Mode.CONE_DRIVE),
        ("2", Mode.REJOIN),
        (3, Mode.LANE_DRIVE),
        ("LANE_DRIVE", Mode.LANE_DRIVE),
    ],
)
def test_initial_mode_parser_supports_defined_runtime_states(value, expected):
    assert parse_initial_mode(value) is expected


@pytest.mark.parametrize("value", [4, 5, "BEFORE", "CHANGE_LANE", True])
def test_initial_mode_parser_rejects_undefined_legacy_state_guesses(value):
    with pytest.raises(ValueError):
        parse_initial_mode(value)
