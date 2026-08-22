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
    dispatch_cone_session_state,
    runtime_safety_monitor,
)


ORDA_ROOT = Path(__file__).resolve().parents[2]
RUBBERCONE_SOURCE = (
    ORDA_ROOT / "driving" / "rubbercone" / "src" / "rubbercone.cpp"
)
RUBBERCONE_SESSION_HEADER = (
    ORDA_ROOT / "driving" / "rubbercone" / "include" / "rubbercone"
    / "session_lifecycle.hpp"
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


def test_integrated_cone_session_activates_once_then_accepts_fixed_entry():
    adapter = one_message_entry_runtime()
    phases = []
    adapter.record_scan(1.0)
    adapter.record_cone_message([4, 0, 90], 1.0)
    adapter.record_cone_message([99, 1, 0], 1.01)

    entered = adapter.step(
        1.02,
        lane=candidate(1.0, 5.0, 1.0),
        cone=candidate(-2.0, 8.0, 1.0),
    )
    dispatch_cone_session_state(entered, phases.append)
    stayed = adapter.step(1.04)
    dispatch_cone_session_state(stayed, phases.append)

    assert entered.transition.target is Mode.CONE_DRIVE
    assert entered.cone_session_active_command is True
    assert entered.discarded_pre_phase_events == 1
    assert adapter.latest_cone_event is None
    assert adapter.pending_cone_event_count == 0
    assert adapter.fsm.cone_exit_armed is False
    assert phases == [True]

    adapter.record_cone_message([0, 0, 80], 1.1)
    armed = adapter.step(1.1, cone=candidate(0.0, 8.0, 1.1))
    adapter.record_cone_message([0, 1, 0], 1.2)
    returned = adapter.step(1.2)

    assert armed.transition.reason == "cone exit session armed"
    assert returned.transition.target is Mode.LANE_DRIVE
    assert returned.control.source is ControlSource.HOLD

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
    assert fixed.control.source is ControlSource.HOLD


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

    assert stopped.transition.target is Mode.CONE_DRIVE
    assert stopped.transition.reason == "external fault: integration fault"
    assert stopped.control.source is ControlSource.HOLD
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
    assert cycle.control.source is ControlSource.HOLD
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
    lifecycle = RUBBERCONE_SESSION_HEADER.read_text(encoding="utf-8")
    reset_body = source.split("void resetSessionState()", 1)[1].split(
        "void sessionActiveCallback", 1
    )[0]
    tracking_body = source.split("void resetTrackingState()", 1)[1].split(
        "void resetSessionState", 1
    )[0]

    expected_resets = (
        "adaptive_half_width_ = nominal_half_width_;",
        "filtered_target_y_ = 0.0f;",
        "has_filtered_target_y_ = false;",
        "filtered_offset_ = 0.0f;",
        "has_filtered_offset_ = false;",
        "rubber_offset_value_ = 0;",
        "rubber_end_value_ = 0;",
        "rubber_confidence_value_ = 0;",
        "rubber_entry_ready_value_ = 0;",
        "debug_ = DebugSnapshot{};",
    )
    for reset in expected_resets:
        assert reset in tracking_body
    assert "session_lifecycle_.manualResetToSearch();" in reset_body
    assert "resetTrackingState();" in reset_body
    for reset in (
        "valid_frame_count_ = 0;",
        "missing_frame_count_ = 0;",
        "exit_armed_ = false;",
        "end_latched_ = false;",
        "entry_readiness_.reset();",
    ):
        assert reset in lifecycle

    publish_body = source.split("void publishInfo()", 1)[1].split(
        "extractConeCenters", 1
    )[0]
    assert "offset_msg.data = {rubber_offset_value_, rubber_end_value_};" in publish_body
    assert "offset_pub_->publish(offset_msg);" in publish_body
    assert "rubber_confidence_value_," in publish_body
    assert "rubber_entry_ready_value_," in publish_body
    assert "info_pub_->publish(info_msg);" in publish_body


def test_object_detector_dual_publishes_official_and_internal_contracts():
    source = OBJECT_SOURCE.read_text(encoding="utf-8")
    publish_body = source.split("void onPublishTick()", 1)[1].split(
        "void onImage", 1
    )[0]

    assert '"/object_info", qos_fast' in source
    assert '"/object_info_raw", qos_fast' in source
    # 신호등은 별도 /traffic_detection 토픽이 아니라 /object_info 의 첫 필드로
    # 나간다 (크롭 분류기 경로와 함께 2026-08-22 에 정리했다).
    assert '"/traffic_detection", qos_fast' not in source
    assert "out.data = { traffic_state," in publish_body
    assert "pub_obj_->publish(out);" in publish_body
    assert "pub_obj_raw_->publish(raw);" in publish_body
    assert "msg->data.size() != 10 && msg->data.size() != 20" in source
    assert "parse_slot(0, 0, fixed)" in source
    assert "parse_slot(10, 1, moving)" in source
    assert "fixed_lane_stabilizer_.update" in source
    assert "moving_lane_stabilizer_.update" in source


@pytest.mark.skip(reason="클래스 id 매핑을 config/object_detection.yaml 의 object_yolo_node 블록에서 읽던 설계를 검사한다. 2026-08-22 통합에서 그 블록을 없앴다 — 옛 블록의 traffic_output_topic 이 /traffic_detection 이라 새 노드에 그대로 먹으면 신호등 박스가 C++ 구독자에 안 닿았다. 지금 매핑은 object_yolo_node.py 의 파라미터 기본값이고 test_object_yolo_contract.py 가 고정한다. yaml 로 다시 옮기면 이 테스트를 되살려야 한다.")
def test_object_class_mapping_is_loaded_from_installed_yaml():
    config = OBJECT_CONFIG.read_text(encoding="utf-8")
    cmake = OBJECT_CMAKE.read_text(encoding="utf-8")
    production = PRODUCTION_LAUNCH.read_text(encoding="utf-8")
    bag_test = BAG_TEST_LAUNCH.read_text(encoding="utf-8")

    assert "object_yolo_node:" in config
    assert "fixed_class_ids: [0]" in config
    assert "moving_class_ids: [1]" in config
    assert "traffic_classifier_model_path" in config
    assert "install(DIRECTORY config" in cmake
    assert "parameters=[object_detection_config, {" in production
    assert "parameters=[object_detection_config, {" in bag_test
    assert "'traffic_classifier_model_path': traffic_classifier_model_path" in production
    assert "'traffic_classifier_model_path': traffic_classifier_model_path" in bag_test


def test_bag_launch_isolates_motor_output_and_contains_no_hardware_nodes():
    source = BAG_TEST_LAUNCH.read_text(encoding="utf-8")
    tree = ast.parse(source)
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
        "rubbercone",
        "lane_detection",
        "object_detection",
        "segmentation_tools",
        "image_resize",
    }
    assert "'input_topic': '/resized_image'" in source
    # 하드웨어 드라이버만 빠진다. resize_node 는 처리 노드라 남는다.
    assert not {"xycar_cam", "xycar_lidar"}.intersection(packages)
    assert "default_value='false'" in source
    assert "'udp_motor_bridge', default_value='false'" in source
    assert "' == 'true' and '" in source

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
    assert remappings == [
        ("xycar_motor", "/kmu_main_offline/xycar_motor")
    ]
    assert "'/xycar_motor'" not in source

    assert not any(
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "IncludeLaunchDescription"
        for node in ast.walk(tree)
    )


def test_bag_launch_has_exactly_one_resized_image_supplier():
    """/resized_image 공급자가 0개도 2개도 되지 않게 막는다.

    resize_node 가 빠져 있던 동안, /image_raw 만 재생하는 bag 에서는
    /resized_image 발행자가 0이었다. 구독자(object_yolo_node, lane_node,
    pidnet)는 조용히 대기만 하고 경고 하나 안 났다. 반대로 bag 이
    /resized_image 를 재생하는데 resize_node 까지 띄우면 같은 토픽에 두 벌이
    섞여 들어간다.
    """
    source = BAG_TEST_LAUNCH.read_text(encoding="utf-8")
    tree = ast.parse(source)

    resize_calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "Node"
        and any(
            keyword.arg == "package"
            and ast.literal_eval(keyword.value) == "image_resize"
            for keyword in node.keywords
        )
    ]
    assert len(resize_calls) == 1

    # bag 이 /resized_image 를 재생할 때만 resize_node 를 끈다.
    condition = next(
        keyword.value
        for keyword in resize_calls[0].keywords
        if keyword.arg == "condition"
    )
    assert isinstance(condition, ast.Call)
    assert condition.func.id == "UnlessCondition"
    assert condition.args[0].id == "replay_resized_image"
    assert "'replay_resized_image', default_value='false'" in source


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
    assert "/kmu_main_offline/xycar_motor" in source
    assert "FSM LANE_DRIVE -> CONE_DRIVE: cone entry confirmed" in source
    assert "xycar_camera" not in source
    assert "xycar_lidar" not in source
