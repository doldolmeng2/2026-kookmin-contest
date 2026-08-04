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
from std_msgs.msg import Bool, Empty, Int32MultiArray

from main.main import (
    LANE_VALIDITY_REQUIRED_RATE_HZ,
    LANE_VALIDITY_TOPIC,
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


class CallbackHarness:
    def __init__(self, times, mode=Mode.CONE_DRIVE):
        self._times = iter(times)
        self.runtime = RaceRuntimeAdapter(
            fsm=RaceFSM(initial_state=mode),
            context=RaceContext(state_entered_at=0.5),
        )
        self.warnings = []

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

        mode_pub = node.mode_pub
        assert mode_pub.topic_name == "/mode_info"
        assert mode_pub.msg_type is Int32MultiArray
        assert mode_pub.qos_profile.reliability is ReliabilityPolicy.BEST_EFFORT
    finally:
        node.destroy_node()
        rclpy.shutdown(context=context)


def test_missing_lane_validity_publisher_contract_is_explicit():
    qos = lane_validity_qos()

    assert LANE_VALIDITY_TOPIC == "/lane_valid"
    assert LANE_VALIDITY_REQUIRED_RATE_HZ == 10.0
    assert qos.history is HistoryPolicy.KEEP_LAST
    assert qos.depth == 10
    assert qos.reliability is ReliabilityPolicy.BEST_EFFORT
    assert qos.durability is DurabilityPolicy.VOLATILE


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
