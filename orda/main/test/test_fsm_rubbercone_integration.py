import ast
from pathlib import Path

import pytest

from main.cone_entry import ConeEntryConfig
from main.control import Controller
from main.control_selector import (
    CommandCandidate,
    ControlSource,
    DriveCommand,
)
from main.race_context import RaceContext
from main.race_fsm import Mode, RaceFSM
from main.runtime_adapter import (
    RaceRuntimeAdapter,
    dispatch_cone_reset,
    runtime_safety_monitor,
)


ORDA_ROOT = Path(__file__).resolve().parents[2]
RUBBERCONE_SOURCE = (
    ORDA_ROOT / "driving" / "rubbercone" / "src" / "rubbercone.cpp"
)
OBJECT_SOURCE = (
    ORDA_ROOT / "perception" / "object_detection" / "src"
    / "object_detection.cpp"
)
OBJECT_CONFIG = (
    ORDA_ROOT / "perception" / "object_detection" / "config"
    / "object_detection.yaml"
)
OBJECT_CMAKE = ORDA_ROOT / "perception" / "object_detection" / "CMakeLists.txt"
PRODUCTION_LAUNCH = ORDA_ROOT / "main" / "launch" / "module_drive.py"
BAG_TEST_LAUNCH = ORDA_ROOT / "main" / "launch" / "module_drive_bag_test.py"
BAG_TEST_RUNNER = ORDA_ROOT / "main" / "tools" / "run_rubbercone_bag_test.sh"


def candidate(angle, speed, received_at):
    return CommandCandidate(DriveCommand(angle, speed), received_at)


def one_message_entry_runtime():
    return RaceRuntimeAdapter(
        fsm=RaceFSM(
            initial_state=Mode.LANE_DRIVE,
            cone_entry_config=ConeEntryConfig(
                min_messages=1,
                min_duration_s=0.0,
            ),
        ),
        context=RaceContext(state_entered_at=0.5),
    )


def test_integrated_cone_session_resets_once_then_accepts_fixed_entry():
    adapter = one_message_entry_runtime()
    resets = []
    adapter.record_scan(1.0)
    adapter.record_cone_message([4, 0, 90], 1.0)
    adapter.record_cone_message([99, 1, 0], 1.01)

    entered = adapter.step(
        1.02,
        lane=candidate(1.0, 5.0, 1.0),
        cone=candidate(-2.0, 8.0, 1.0),
    )
    dispatch_cone_reset(entered, lambda: resets.append("reset"))
    stayed = adapter.step(1.04)
    dispatch_cone_reset(stayed, lambda: resets.append("duplicate"))

    assert entered.transition.target is Mode.CONE_DRIVE
    assert entered.publish_cone_reset is True
    assert entered.discarded_pre_reset_events == 1
    assert adapter.latest_cone_event is None
    assert adapter.pending_cone_event_count == 0
    assert adapter.fsm.cone_exit_armed is False
    assert resets == ["reset"]

    adapter.record_cone_message([0, 0, 80], 1.1)
    armed = adapter.step(1.1, cone=candidate(0.0, 8.0, 1.1))
    adapter.record_cone_message([0, 1, 0], 1.2)
    returned = adapter.step(1.2)

    assert armed.transition.reason == "cone exit session armed"
    assert returned.transition.target is Mode.LANE_DRIVE
    assert returned.control.source is ControlSource.STOP

    for timestamp in (1.3, 1.4):
        waiting = adapter.step(
            timestamp,
            lane=candidate(1.0, 5.0, timestamp),
        )
        assert waiting.transition.changed is False
        assert waiting.control.source is ControlSource.LANE

    adapter.record_fixed_zone_entry(1.51)
    fixed = adapter.step(1.51, lane=candidate(1.0, 5.0, 1.51))

    assert fixed.transition.source is Mode.LANE_DRIVE
    assert fixed.transition.target is Mode.FIXED_AVOID
    assert fixed.control.source is ControlSource.STOP


def test_safety_stop_wins_over_fresh_cone_exit_in_integrated_cycle():
    adapter = RaceRuntimeAdapter(
        fsm=RaceFSM(initial_state=Mode.CONE_DRIVE),
        context=RaceContext(state_entered_at=1.0, cone_entered_at=1.0),
        safety_monitor=runtime_safety_monitor(),
    )
    adapter.record_scan(1.1)
    adapter.record_cone_message([0, 0, 90], 1.1)
    adapter.step(1.1, cone=candidate(0.0, 8.0, 1.1))
    adapter.record_scan(1.2)
    adapter.record_cone_message([0, 1, 0], 1.2)

    stopped = adapter.step(
        1.2,
        cone=candidate(0.0, 8.0, 1.2),
        fault_reason="integration fault",
    )

    assert stopped.transition.target is Mode.STOP
    assert stopped.transition.reason == "external fault: integration fault"
    assert stopped.control.source is ControlSource.STOP
    assert stopped.control.command == DriveCommand(0.0, 0.0)


def test_stale_rubbercone_command_stops_without_lane_fallback():
    adapter = RaceRuntimeAdapter(
        fsm=RaceFSM(initial_state=Mode.CONE_DRIVE),
        context=RaceContext(state_entered_at=4.0, cone_entered_at=4.0),
        safety_monitor=runtime_safety_monitor(),
    )
    adapter.record_scan(5.0)

    cycle = adapter.step(
        5.0,
        lane=candidate(9.0, 9.0, 5.0),
        cone=candidate(-3.0, 8.0, 4.0),
    )

    assert cycle.transition.changed is False
    assert adapter.fsm.state is Mode.CONE_DRIVE
    assert cycle.control.source is ControlSource.STOP
    assert cycle.control.reason == "cone command stale"


def test_controller_accepts_only_current_fsm_drive_modes_and_keeps_tuning():
    controller = Controller()

    controller.update(Mode.CONE_DRIVE, 45, float("inf"), 100)
    assert controller.get_angle() == pytest.approx(45.0)
    assert controller.get_speed() == pytest.approx(16.0)

    controller.update(Mode.WAIT_GREEN, 45, float("inf"), 100)
    assert controller.get_angle() == 0.0
    assert controller.get_speed() == 0.0

    controller.update(Mode.LANE_DRIVE, 0, float("inf"), 100)
    assert controller.get_speed() == pytest.approx(43.0)


def test_detector_reset_clears_detection_debounce_filter_and_debug_state():
    source = RUBBERCONE_SOURCE.read_text(encoding="utf-8")
    reset_body = source.split("void resetSessionState()", 1)[1].split(
        "void resetCallback", 1
    )[0]

    expected_resets = (
        "valid_frame_count_ = 0;",
        "missing_frame_count_ = 0;",
        "cone_section_armed_ = false;",
        "end_latched_ = false;",
        "adaptive_half_width_ = nominal_half_width_;",
        "filtered_target_y_ = 0.0f;",
        "has_filtered_target_y_ = false;",
        "filtered_offset_ = 0.0f;",
        "has_filtered_offset_ = false;",
        "rubber_offset_value_ = 0;",
        "rubber_end_value_ = 0;",
        "rubber_confidence_value_ = 0;",
        "debug_ = DebugSnapshot{};",
    )
    for reset in expected_resets:
        assert reset in reset_body

    publish_body = source.split("void publishInfo()", 1)[1].split(
        "extractConeCenters", 1
    )[0]
    assert "offset_msg.data = {rubber_offset_value_, rubber_end_value_};" in publish_body
    assert "offset_pub_->publish(offset_msg);" in publish_body
    assert "rubber_confidence_value_," in publish_body
    assert "info_pub_->publish(info_msg);" in publish_body


def test_object_detector_dual_publishes_official_and_internal_contracts():
    source = OBJECT_SOURCE.read_text(encoding="utf-8")
    publish_body = source.split("void onPublishTick()", 1)[1].split(
        "void onImage", 1
    )[0]

    assert '"/traffic_detection", qos_fast' in source
    assert '"/object_info", qos_fast' in source
    assert '"/object_info_raw", qos_fast' in source
    assert (
        "info.data = {traffic_signal, fixed_lane_label, moving_lane_label};"
        in publish_body
    )
    assert "pub_obj_->publish(info);" in publish_body
    assert "pub_obj_raw_->publish(raw);" in publish_body
    assert "msg->data.size() != 10 && msg->data.size() != 20" in source
    assert "parse_slot(0, 0, fixed)" in source
    assert "parse_slot(10, 1, moving)" in source


def test_object_class_mapping_is_loaded_from_installed_yaml():
    config = OBJECT_CONFIG.read_text(encoding="utf-8")
    cmake = OBJECT_CMAKE.read_text(encoding="utf-8")
    production = PRODUCTION_LAUNCH.read_text(encoding="utf-8")
    bag_test = BAG_TEST_LAUNCH.read_text(encoding="utf-8")

    assert "object_yolo_node:" in config
    assert "fixed_class_ids: [0]" in config
    assert "# 예: moving_class_ids: [1]" in config
    assert "install(DIRECTORY config" in cmake
    assert "parameters=[object_detection_config]" in production
    assert "parameters=[object_detection_config, {'use_sim_time': True}]" in bag_test


def test_bag_launch_isolates_motor_output_and_contains_no_hardware_nodes():
    tree = ast.parse(BAG_TEST_LAUNCH.read_text(encoding="utf-8"))
    node_calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "Node"
    ]

    packages = {
        ast.literal_eval(keyword.value)
        for call in node_calls
        for keyword in call.keywords
        if keyword.arg == "package"
    }
    assert packages == {
        "main",
        "traffic_light",
        "rubbercone",
        "image_resize",
        "lane_detection",
        "object_detection",
    }

    main_call = next(
        call
        for call in node_calls
        if any(
            keyword.arg == "package"
            and ast.literal_eval(keyword.value) == "main"
            for keyword in call.keywords
        )
    )
    remappings = next(
        ast.literal_eval(keyword.value)
        for keyword in main_call.keywords
        if keyword.arg == "remappings"
    )
    assert remappings == [("xycar_motor", "/bag_test/xycar_motor")]

    assert not any(
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "IncludeLaunchDescription"
        for node in ast.walk(tree)
    )


def test_rubbercone_bag_runner_enforces_safe_scan_only_playback():
    source = BAG_TEST_RUNNER.read_text(encoding="utf-8")
    scan_mock_stop = '\nstop_process "${SCAN_MOCK_PID}"\nSCAN_MOCK_PID=""'
    clock_mock_stop = (
        '\nstop_process "${CLOCK_MOCK_PID}"\nCLOCK_MOCK_PID=""'
    )
    scan_only_play = (
        'ros2 bag play "${BAG_PATH}" --disable-keyboard-controls '
        '--clock --topics /scan'
    )
    cone_to_lane = (
        'wait_for_log "FSM CONE_DRIVE -> LANE_DRIVE: fresh cone end flag" 20'
    )

    assert "mode:=0" in source
    assert "kmu_test_scan_mock" in source
    assert "kmu_test_clock_mock" in source
    assert 'publisher = node.create_publisher(Clock, "/clock", 10)' in source
    assert "stamp = 1.0 + time.monotonic() - started" in source
    assert scan_mock_stop in source
    assert source.index(scan_mock_stop) < source.index(scan_only_play)
    assert clock_mock_stop in source
    assert source.index(clock_mock_stop) < source.index(scan_only_play)
    assert source.count('assert_no_real_motor_publishers "') == 3
    assert scan_only_play in source
    assert source.count("--topics /scan") == 2
    assert "kmu_test_object_yolo_mock" in source
    assert "/object_yolo std_msgs/msg/Float32MultiArray" in source
    assert cone_to_lane in source
    assert "post_cone_lane_motor_sample.log" in source
    assert "--topics /scan /xycar_motor" not in source
    assert "--topics /scan /rubbercone_info" not in source
    assert "/bag_test/xycar_motor" in source
    assert "FSM LANE_DRIVE -> CONE_DRIVE: cone entry confirmed" in source
    assert "xycar_camera" not in source
    assert "xycar_lidar" not in source
