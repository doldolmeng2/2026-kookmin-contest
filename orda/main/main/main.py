"""ROS runtime wiring for the authoritative 2026 finals race FSM."""

import threading
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from sensor_msgs.msg import LaserScan
from std_msgs.msg import (
    Bool,
    Float32MultiArray,
    Int16,
    Int32,
    Int32MultiArray,
)

from main.control import (
    GUARDRAIL_PARAMS,
    GUARDRAIL_TUNABLES,
    STEERING_FILTER_PARAMS,
    STEERING_FILTER_TUNABLES,
    Controller,
    validate_guardrail_params,
    validate_steering_filter_params,
)
from main.control_selector import (
    CommandCandidate,
    ControlSource,
    DriveCommand,
)
from main.mission_types import (
    LaneTarget,
    ObjectLane,
    ObjectType,
    RouteTrafficSignal,
    opposite_lane_target,
)
from main.mode_info import (
    external_mode_code,
    lane_command_data,
    lane_info_value,
)
from main.overtake import OvertakeGuard
from main.object_mission_episode import ObjectMissionEpisodeGate
from main.relative_x_fallback import (
    RelativeXObstacleLaneFallback,
    effective_object_lane,
    object_mission_entry_allowed,
)
from main.runtime_diagnostics import (
    RuntimeDiagnosticReporter,
    RuntimeDiagnosticSnapshot,
)
from main.same_lane_brake import (
    SameLaneBrake,
    SameLaneBrakeDecision,
    effective_ego_lane,
)
from main.shortcut_exit import ShortcutExitGuard
from main.traffic_encounter import TrafficEncounterGate
from main.race_fsm import Mode, RaceFSM
from main.runtime_adapter import (
    SIDE_CLEARANCE_MAX_AGE_S,
    MissionTestProfile,
    RaceRuntimeAdapter,
    dispatch_cone_session_state,
    parse_test_profile,
    runtime_safety_monitor,
)


RUBBERCONE_INFO_TOPIC = "/rubbercone_info"
RUBBERCONE_OFFSET_TOPIC = "/rubbercone_offset"
RUBBERCONE_SESSION_ACTIVE_TOPIC = "/rubbercone_session_active"
LANE_PATH_PREVIEW_TOPIC = "/lane_path_preview"
OBJECT_INFO_TOPIC = "/object_info"
OBJECT_INFO_RAW_TOPIC = "/object_info_raw"
SIDE_CLEARANCE_TOPIC = "/side_clearance"
LANE_COMMAND_TOPIC = "/internal/lane_command"
LANE_CHANGE_STATE_TOPIC = "/lane_change_state"
CONE_EVENT_WARNING_PERIOD_S = 5.0

# /object_info 첫 필드의 신호등 값 규약:
#   0 = 미검출, 1 = 적색/주황, 2 = 녹색, 3 = 좌회전 녹색
TRAFFIC_GREEN_CODES = (2, 3)

# The controller gains remain the proven legacy implementation. These limits
# are output shaping only; they do not participate in mission transitions.
# 0.10 (5.0/s) took too long to reach the tuned 6.0 cruise speed in short
# rubber-cone sections. The tuned steps reach it quickly while keeping
# deceleration faster than acceleration.
RUBBERCONE_ACCEL_STEP = 0.25
RUBBERCONE_DECEL_STEP = 0.35
RUBBERCONE_STEERING_STEP = 3.0

# 차선 주행 가감속 (50 Hz 제어 주기 기준, 사이클당 증분).
#
# LANE_ACCEL_STEP: 예전 값 0.1 은 초당 5 여서 목표 속도 21.5 까지 4.3초가
#   걸렸다. 인지가 잠깐 끊길 때마다 속도가 0으로 리셋됐으므로, 재가속이 끝나기
#   전에 다음 끊김이 왔고 차는 사실상 앞으로 못 나갔다. 0.4 는 초당 20으로
#   약 1.1초 만에 순항 속도를 회복한다.
# STOP_DECEL_STEP: 초당 30. 21.5 에서 0까지 약 0.7초다. 정지는 여전히 확실히
#   이뤄지되, 한두 프레임짜리 인지 공백이 완전 정지로 번지지 않는다.
LANE_ACCEL_STEP = 0.4
STOP_DECEL_STEP = 0.6

# /lane_offset 과 /lane_guardrail 은 lane_node 의 같은 콜백에서 나가므로 정상이면
# 수신 시각이 거의 같다. 이보다 벌어졌다면 둘 중 하나가 밀린 것이고, 그때는
# 가드레일을 쓰지 않는다 — 오래된 여유로 조향을 더하면 이미 지나간 코너를
# 향해 꺾게 된다. 24 Hz 기준 한 프레임이 42ms 라 두 프레임 남짓으로 잡았다.
GUARDRAIL_MAX_SKEW_S = 0.10

# /lane_path_preview도 /lane_offset과 같은 lane_node 영상 콜백에서 발행된다.
# 두 토픽이 이 범위를 벗어나면 서로 다른 프레임의 경로를 섞지 않고 즉시 기존
# offset-only Pure Pursuit로 폴백한다. 24 Hz 기준 한 프레임(약 42 ms)보다
# 짧게 잡고, 메시지 안의 source offset도 함께 대조한다.
LANE_PATH_PREVIEW_MAX_SKEW_S = 0.03
LANE_PATH_PREVIEW_DEFAULT_MIN_CONFIDENCE = 0.55

# 이보다 크게 시각이 역행하면 "bag이 처음부터 다시 재생됨"으로 본다.
# 실차 wall clock도 NTP 보정으로 아주 드물게 살짝 뒤로 갈 수 있어, 그런 미세
# 역행까지 리셋으로 잡지 않도록 넉넉한 임계값을 둔다.
BAG_LOOP_BACKJUMP_S = 2.0

# 이 면적을 넘는 박스를 object_type에 따라 FIXED_AVOID 또는 OVERTAKE로 보낸다.
FIXED_ENTRY_BOX_PX = 1900.0
# 반복 regression에서 entry 전 동일 물체 independent YOLO 최대 gap은 1.800초였다.
# perception freshness(0.6초)와 encounter grouping을 분리하고 0.10초 margin만 둔다.
RELATIVE_X_ENCOUNTER_TIMEOUT_S = 1.90
# Baseline 70 s fixture had a 0.041 s UNKNOWN display pulse (0.545 s from
# the last YOLO evidence to the redisplayed green), while the next physical
# fixture was 62.8 s later. One continuous neutral second separates episodes
# without approaching the inter-fixture interval.
TRAFFIC_ENCOUNTER_RELEASE_S = 1.0

# 추월 완료 판정 임계값은 main.overtake.OvertakeConfig가 갖는다.

# Keep the launch file's existing integer ``mode`` parameter usable for the
# states that have a defined 2026 counterpart. Unsupported legacy mission
# states are rejected instead of being guessed.
_NUMERIC_INITIAL_MODE = {
    0: Mode.WAIT_GREEN,
    1: Mode.LANE_DRIVE,
    2: Mode.CONE_DRIVE,
    3: Mode.FIXED_AVOID,
    4: Mode.OVERTAKE,
    5: Mode.SHORTCUT,
}


def parse_initial_mode(value) -> Mode:
    """Parse a RaceFSM name or a supported legacy launch value."""

    if isinstance(value, Mode):
        return value
    if isinstance(value, int) and not isinstance(value, bool):
        if value in _NUMERIC_INITIAL_MODE:
            return _NUMERIC_INITIAL_MODE[value]
        raise ValueError(f"unsupported numeric initial mode: {value}")
    if isinstance(value, str):
        stripped = value.strip()
        if stripped.lstrip("-").isdigit():
            return parse_initial_mode(int(stripped))
        named_mode = Mode.__members__.get(stripped)
        if named_mode is not None:
            return named_mode
        try:
            return Mode(stripped)
        except ValueError as exc:
            raise ValueError(f"unknown RaceFSM initial mode: {value}") from exc
    raise ValueError(f"invalid initial mode value: {value!r}")


def validate_curve_preview_min_confidence(value) -> float:
    """Return a finite confidence threshold in the closed interval [0, 1]."""

    if isinstance(value, bool):
        raise ValueError("curve_preview_min_confidence must be a number in [0, 1]")
    try:
        converted = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            "curve_preview_min_confidence must be a number in [0, 1]"
        ) from exc
    if not np.isfinite(converted) or not 0.0 <= converted <= 1.0:
        raise ValueError("curve_preview_min_confidence must be in [0, 1]")
    return converted


def rubbercone_session_qos() -> QoSProfile:
    """Return the detector's stateful lifecycle-command QoS contract."""

    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def sensor_event_qos(*, depth: int = 1) -> QoSProfile:
    """Return the BestEffort/Volatile contract used by sensor publishers."""

    return QoSProfile(
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class MainNode(Node):
    """Collect ROS inputs, step one RaceFSM, select, and publish control."""

    def __init__(self, *, context: Optional[Context] = None):
        super().__init__("main_node", context=context)

        self.declare_parameter("mode", 0)
        self.declare_parameter("lane_target", LaneTarget.CENTER.value)
        self.declare_parameter(
            "test_profile",
            MissionTestProfile.RACE.value,
            ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter("show_debug", False)
        # 터미널에서 Enter 를 눌러 모터 출력만 껐다 켠다. 인지·FSM·조향은 계속
        # 돌고 바퀴로 나가는 속도만 막히므로, 차를 세운 채로 차선 인식을 보며
        # 튜닝할 수 있다. 기본은 꺼짐이고, 튜닝 프로파일(module_lane_only)에서만
        # 켠다 — 기본값을 켜 두면 테스트가 제어 터미널 입력을 가로챌 수 있다.
        self.declare_parameter("enter_motor_toggle", False)
        self.declare_parameter("lane_guardrail", True)
        # 새 곡선 미리보기는 기존 /lane_offset과 병렬 입력이다. false로 바꾸면
        # lane_node를 재시작하지 않고도 다음 제어 사이클부터 기존 Pure Pursuit로
        # 즉시 돌아간다.
        self.declare_parameter("curve_preview_enabled", True)
        self.declare_parameter(
            "curve_preview_min_confidence",
            LANE_PATH_PREVIEW_DEFAULT_MIN_CONFIDENCE,
        )
        # 가드레일 이득은 실차에서 반복해 찾는 값이라 ROS 파라미터로 열어 둔다.
        # 기본값을 control.GUARDRAIL_PARAMS 에서 그대로 읽으므로 소스 기본값과
        # 파라미터 기본값이 갈라질 수 없다. 이름은 'guardrail_' + 항목명이다.
        for name in GUARDRAIL_TUNABLES:
            self.declare_parameter(f"guardrail_{name}", GUARDRAIL_PARAMS[name])
        # 지터 억제 값도 실차 계측으로 찾는 값이라 같은 방식으로 열어 둔다.
        # 이름은 control.STEERING_FILTER_PARAMS 의 항목명 그대로다 — 이미
        # 충분히 구체적이라 접두어를 붙이면 길기만 하다.
        for name in STEERING_FILTER_TUNABLES:
            self.declare_parameter(name, STEERING_FILTER_PARAMS[name])
        test_profile = parse_test_profile(
            self.get_parameter("test_profile").value
        )
        self._initial_mode = parse_initial_mode(self.get_parameter("mode").value)
        self.show_debug = bool(self.get_parameter("show_debug").value)
        self.lane_guardrail_enabled = bool(
            self.get_parameter("lane_guardrail").value
        )
        self.curve_preview_enabled = bool(
            self.get_parameter("curve_preview_enabled").value
        )
        self.curve_preview_min_confidence = (
            validate_curve_preview_min_confidence(
                self.get_parameter("curve_preview_min_confidence").value
            )
        )
        # launch 인자로 들어온 값도 ros2 param set 과 같은 검증을 거친다.
        # 여기서 던지면 노드가 뜨지 않는데, 잘못된 이득으로 주행이 시작되는
        # 것보다 낫다.
        self._guardrail_overrides = validate_guardrail_params({
            name: self.get_parameter(f"guardrail_{name}").value
            for name in GUARDRAIL_TUNABLES
        })
        self._steering_filter_overrides = validate_steering_filter_params({
            name: self.get_parameter(name).value
            for name in STEERING_FILTER_TUNABLES
        })

        # bag --loop 재생 감지용. use_sim_time=True 로 띄우고 bag을 --clock 과
        # 함께 재생하면, 루프가 돌 때 /clock 시각이 뒤로 튄다. 그 역행을 잡아
        # 상태머신을 시작 모드로 되돌린다.
        # 실차(wall clock)에서는 시간이 뒤로 가지 않으므로 발동하지 않는다.
        self._last_now: Optional[float] = None

        self.runtime = RaceRuntimeAdapter(
            fsm=RaceFSM(initial_state=self._initial_mode),
            safety_monitor=runtime_safety_monitor(),
        )
        lane_target_value = int(self.get_parameter("lane_target").value)
        self._initial_lane_target = LaneTarget(lane_target_value)
        self.runtime.context.lane_target = self._initial_lane_target
        if test_profile is not MissionTestProfile.RACE:
            self.runtime.bootstrap_test_profile(
                test_profile,
                self._now_seconds(),
            )
        self.lane_controller = Controller()
        self.cone_controller = Controller()
        self._apply_guardrail_params()
        self._apply_steering_filter_params()
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.lane = 1
        self.object_dist = float("inf")
        self.obj_exists = 0.0
        self.obj_angle = 0.0
        self.obj_span = 0.0
        self.obj_cluster = 0.0
        self.box_size = 0.0
        self.box_cx = float("nan")
        self.box_cy = float("nan")
        self.box_dx = float("nan")
        self.car_lane = -1
        self.object_type = ObjectType.UNKNOWN
        self.object_confidence = 0.0
        self.fixed_vehicle_lane = 0
        self.moving_vehicle_lane = 0
        self.rubbercone_offset = 0
        self.rubbercone_end_flag = 0
        self.detected_lane = -1         # 실측 현재 차선 (-1=미확정, 0=중앙, 1=왼쪽, 2=오른쪽)
        self.side_left = float("inf")   # perception이 승인한 좌측면 최소 거리 (m)
        self.side_right = float("inf")  # perception이 승인한 우측면 최소 거리 (m)
        self.left = float("inf")
        self.right = float("inf")
        self.traffic_signal = 0        # /object_info 최신 신호등 코드 (표시용)

        # 구간(FIXED_AVOID / OVERTAKE) 종료 판정. 순수 로직은 main.overtake가 갖는다.
        self.overtake = OvertakeGuard()
        # 같은 차선 고정장애물 감속. 순수 로직은 main.same_lane_brake 가 갖는다.
        self.same_lane_brake = SameLaneBrake()
        self.relative_x_fallback = RelativeXObstacleLaneFallback(
            encounter_timeout_s=RELATIVE_X_ENCOUNTER_TIMEOUT_S,
        )
        self.object_mission_episode = ObjectMissionEpisodeGate(
            release_after_s=RELATIVE_X_ENCOUNTER_TIMEOUT_S,
        )
        self.last_same_lane_brake = SameLaneBrakeDecision(
            speed_limit=float("inf"), same_lane=False, reason="not evaluated yet"
        )
        self.runtime_diagnostic_reporter = RuntimeDiagnosticReporter()
        self.last_runtime_diagnostic: Optional[RuntimeDiagnosticSnapshot] = None
        self._zone_state: Optional[Mode] = None   # 직전 사이클의 FSM 상태
        self._zone_exit_sent = False              # 현재 구간의 종료 엣지를 이미 냈는지
        self._fixed_entry_sent = False            # 이번 LANE_DRIVE 세션에서 진입 엣지를 냈는지
        self.shortcut_exit = ShortcutExitGuard()
        self._shortcut_exit_sent = False

        self.now_speed = 0.0
        self.now_angle = 0.0
        self.last_debug_time: Optional[float] = None
        self._warning_times: dict[str, float] = {}
        self.traffic_encounter = TrafficEncounterGate(
            release_after_s=TRAFFIC_ENCOUNTER_RELEASE_S,
        )

        qos_fast = sensor_event_qos()
        qos_command = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        session_qos = rubbercone_session_qos()

        self.motor_pub = self.create_publisher(
            Float32MultiArray,
            "xycar_motor",
            qos_command,
        )
        self.mode_pub = self.create_publisher(
            Int16,
            "mode_info",
            qos_fast,
        )
        self.lane_info_pub = self.create_publisher(
            Int16,
            "lane_info",
            qos_fast,
        )
        self.lane_command_pub = self.create_publisher(
            Int32MultiArray,
            LANE_COMMAND_TOPIC,
            qos_fast,
        )
        self.rubbercone_session_active_pub = self.create_publisher(
            Bool,
            RUBBERCONE_SESSION_ACTIVE_TOPIC,
            session_qos,
        )
        self._publish_cone_session_state(
            self.runtime.fsm.state is Mode.CONE_DRIVE
        )

        self.rubbercone_info_sub = self.create_subscription(
            Int32MultiArray,
            RUBBERCONE_INFO_TOPIC,
            self.rubbercone_callback,
            qos_fast,
        )
        self.rubbercone_offset_sub = self.create_subscription(
            Int32MultiArray,
            RUBBERCONE_OFFSET_TOPIC,
            self.rubbercone_offset_callback,
            qos_fast,
        )
        self.lane_offset_sub = self.create_subscription(
            Int16,
            "lane_offset",
            self.lane_offset_callback,
            qos_fast,
        )
        self.lane_path_preview_sub = self.create_subscription(
            Float32MultiArray,
            LANE_PATH_PREVIEW_TOPIC,
            self.lane_path_preview_callback,
            qos_fast,
        )
        self.lane_guardrail_sub = self.create_subscription(
            Float32MultiArray,
            "lane_guardrail",
            self.lane_guardrail_callback,
            qos_fast,
        )
        self.object_info_sub = self.create_subscription(
            Int32MultiArray,
            OBJECT_INFO_TOPIC,
            self.object_info_callback,
            qos_fast,
        )
        self.object_info_raw_sub = self.create_subscription(
            Float32MultiArray,
            OBJECT_INFO_RAW_TOPIC,
            self.object_info_raw_callback,
            qos_fast,
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos_fast,
        )
        self.side_clearance_sub = self.create_subscription(
            Float32MultiArray,
            SIDE_CLEARANCE_TOPIC,
            self.side_clearance_callback,
            qos_fast,
        )
        self.lane_change_state_sub = self.create_subscription(
            Int32MultiArray,
            LANE_CHANGE_STATE_TOPIC,
            self.lane_change_state_callback,
            sensor_event_qos(depth=10),
        )
        self.lane_position_sub = self.create_subscription(
            Int16,
            "lane_position",
            self.lane_position_callback,
            qos_fast,
        )
        self.road_surface_sub = self.create_subscription(
            Int32,
            "/road_surface",
            self.road_surface_callback,
            qos_fast,
        )
        self.ultrasonic_sub = self.create_subscription(
            Int32MultiArray,
            "xycar_ultrasonic",
            self.ultrasonic_callback,
            10,
        )

        # ── 상태 창 표시 전용 스레드 ─────────────────────────────────────
        # cv2.imshow/waitKey를 rclpy 타이머 콜백 안에서 직접 부르면 GTK/Wayland
        # 이벤트 루프와 충돌해 창이 아예 안 뜨는 환경이 있다(WSLg 등).
        # control_cycle은 최신 프레임만 공유 변수에 넘기고, 실제 표시는 이
        # 독립 스레드가 전담한다.
        self._status_img_lock = threading.Lock()
        self._status_img = None

        # ── 모터 출력 게이트 ─────────────────────────────────────────────
        # set() 이면 주행, clear() 면 정지다. Event 자체가 스레드 안전하므로
        # 터미널 스레드와 제어 타이머가 락 없이 같은 값을 본다.
        self._motor_gate = threading.Event()
        self._motor_gate.set()
        if bool(self.get_parameter("enter_motor_toggle").value):
            threading.Thread(
                target=self._run_motor_toggle_loop,
                name="enter_motor_toggle",
                daemon=True,
            ).start()

        self.create_timer(0.02, self.control_cycle)

    def run_display_loop(self) -> None:
        """최신 상태 이미지를 표시한다. **메인 스레드에서** 호출해야 한다.

        control_cycle은 이미지를 공유 변수에 넣기만 하고, 실제 imshow/waitKey는
        여기서 부른다. rclpy 콜백 안에서 직접 부르면 인지 주기를 늘어뜨리고,
        보조 스레드에서 부르면 GTK 백엔드가 창을 화면에 띄우지 못한다.
        """

        while rclpy.ok():
            with self._status_img_lock:
                image = self._status_img
            if image is not None:
                cv2.imshow("Status", image)
            cv2.waitKey(30)

    def _set_motor_enabled(self, enabled: bool) -> None:
        """모터 출력 게이트를 바꾸고 바뀐 상태만 로그로 남긴다."""

        if enabled == self._motor_gate.is_set():
            return
        if enabled:
            self._motor_gate.set()
            self.get_logger().info("MOTOR ON  — 주행. Enter 로 정지.")
        else:
            self._motor_gate.clear()
            self.get_logger().warning("MOTOR OFF — 정지. Enter 로 주행.")

    def _run_motor_toggle_loop(self) -> None:
        """터미널에서 Enter 를 받을 때마다 모터 출력을 껐다 켠다.

        sys.stdin 이 아니라 /dev/tty 를 직접 연다. `ros2 launch` 는 자식 노드의
        stdin 을 파이프로 바꿔 버리므로(asyncio subprocess_exec 의 기본값이
        stdin=PIPE 다), sys.stdin 을 읽으면 런치 시스템이 그 파이프에 무언가
        써 주기 전까지 영원히 막힌다 — 터미널에서 Enter 를 아무리 눌러도 오지
        않는다. 제어 터미널은 그 파이프와 무관하게 /dev/tty 로 열 수 있다.

        데몬 스레드에서 돈다. readline 은 막히는 호출이라, 종료할 때 이 스레드를
        기다리면 노드가 내려가지 못한다.
        """

        try:
            terminal = open("/dev/tty", "r")
        except OSError as exc:
            # 제어 터미널이 없는 실행(리다이렉트, 서비스, CI)에서는 토글만
            # 포기하고 주행은 그대로 둔다.
            self.get_logger().warning(
                f"Enter 모터 토글 비활성: 제어 터미널을 열 수 없습니다 ({exc})"
            )
            return

        self.get_logger().info(
            "Enter 모터 토글 준비됨: 이 터미널에서 Enter 를 누를 때마다 "
            "모터 출력이 정지/주행 으로 바뀝니다."
        )
        with terminal:
            while self.context.ok():
                if terminal.readline() == "":
                    # 터미널이 닫혔다. 계속 돌면 빈 줄을 무한히 읽는다.
                    return
                self._set_motor_enabled(not self._motor_gate.is_set())

    def _now_seconds(self) -> float:
        """Read the node ROS clock used by both callbacks and FSM steps."""

        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _publish_cone_session_state(self, active: bool) -> None:
        message = Bool()
        message.data = bool(active)
        self.rubbercone_session_active_pub.publish(message)

    def rubbercone_callback(self, msg: Int32MultiArray) -> None:
        """Consume the detailed internal cone contract used by the FSM."""

        received_at = self._now_seconds()
        result = self.runtime.record_cone_message(msg.data, received_at)
        if result.warning is not None:
            key = "cone_queue_overflow" if result.dropped_oldest else "malformed_cone"
            self._warn_throttled(key, result.warning, received_at)

    def rubbercone_offset_callback(self, msg: Int32MultiArray) -> None:
        """Validate and expose the PPT ``[offset, end_flag]`` contract."""

        received_at = self._now_seconds()
        data = list(msg.data)
        if (
            len(data) != 2
            or isinstance(data[0], bool)
            or isinstance(data[1], bool)
            or not isinstance(data[0], int)
            or data[1] not in (0, 1)
        ):
            self._warn_throttled(
                "malformed_rubbercone_offset",
                "rubbercone_offset requires two integers [offset, end_flag]",
                received_at,
            )
            return
        self.rubbercone_offset = int(data[0])
        self.rubbercone_end_flag = int(data[1])

    def lane_offset_callback(self, msg: Int16) -> None:
        received_at = self._now_seconds()
        if not self.runtime.record_lane_offset(msg.data, received_at):
            self._warn_throttled(
                "malformed_lane",
                "invalid lane_offset message ignored",
                received_at,
            )

    def lane_path_preview_callback(self, msg: Float32MultiArray) -> None:
        """Validate the optional far-path preview published beside lane_offset."""

        received_at = self._now_seconds()
        data = list(msg.data)
        if len(data) != 5:
            self._warn_throttled(
                "malformed_lane_path_preview",
                "lane_path_preview requires "
                "[target_offset_px, curvature_norm, confidence, target_y_ratio, "
                "source_lane_offset_px]",
                received_at,
            )
            return
        if not self.runtime.record_lane_path_preview(*data, received_at):
            self._warn_throttled(
                "malformed_lane_path_preview",
                "invalid lane_path_preview message ignored",
                received_at,
            )

    def lane_guardrail_callback(self, msg: Float32MultiArray) -> None:
        received_at = self._now_seconds()
        data = list(msg.data)
        if len(data) < 2:
            self._warn_throttled(
                "malformed_guardrail",
                "invalid lane_guardrail message ignored",
                received_at,
            )
            return
        if not self.runtime.record_lane_guardrail(
                data[0], data[1], received_at):
            self._warn_throttled(
                "malformed_guardrail",
                "invalid lane_guardrail message ignored",
                received_at,
            )

    def _record_traffic_signal(self, value: int, received_at: float) -> None:
        try:
            signal = RouteTrafficSignal(value)
        except (TypeError, ValueError):
            self._warn_throttled(
                "malformed_traffic",
                f"invalid object_info traffic value ignored: {value}",
                received_at,
            )
            return

        # Preserve the start-only WAIT_GREEN contract.
        is_green = signal in (
            RouteTrafficSignal.STRAIGHT,
            RouteTrafficSignal.LEFT,
        )
        self.runtime.record_traffic(is_green, received_at)

        # Stable display state and physical fixture ownership are separate.
        # The WAIT_GREEN fixture starts the race but is never a completed lap.
        encounter_started = self.traffic_encounter.update(signal, received_at)
        if self.runtime.fsm.state is Mode.WAIT_GREEN:
            encounter_started = False

        result = self.runtime.record_route_traffic(
            signal,
            received_at,
            encounter_started=encounter_started,
        )
        if not result.accepted:
            self._warn_throttled(
                "malformed_route_traffic",
                result.warning or "invalid route traffic input ignored",
                received_at,
            )
        self.traffic_signal = int(signal)

    def object_info_callback(self, msg: Int32MultiArray) -> None:
        """Consume the PPT ``[traffic, fixed_lane, moving_lane]`` contract."""

        received_at = self._now_seconds()
        data = list(msg.data)
        if (
            len(data) != 3
            or any(
                isinstance(value, bool) or not isinstance(value, int)
                for value in data
            )
            or data[0] not in (0, 1, 2, 3)
            or data[1] not in (0, 1, 2)
            or data[2] not in (0, 1, 2)
        ):
            self._warn_throttled(
                "malformed_object_info",
                "object_info requires [traffic(0..3), fixed_lane(0..2), "
                "moving_lane(0..2)]",
                received_at,
            )
            return
        self.fixed_vehicle_lane = data[1]
        self.moving_vehicle_lane = data[2]
        self._record_traffic_signal(data[0], received_at)

    def scan_callback(self, _msg: LaserScan) -> None:
        self.runtime.record_scan(self._now_seconds())

    def side_clearance_callback(self, msg: Float32MultiArray) -> None:
        """Consume perception-owned ``[left_m, right_m]`` side distances."""

        received_at = self._now_seconds()
        data = list(msg.data)
        if len(data) != 2 or not self.runtime.record_side_clearance(
            data[0],
            data[1],
            received_at,
        ):
            self._warn_throttled(
                "malformed_side_clearance",
                "side_clearance requires two non-negative finite values or +Infinity",
                received_at,
            )
            return
        self.side_left = self.runtime.latest_side_left
        self.side_right = self.runtime.latest_side_right

    def lane_position_callback(self, msg: Int16) -> None:
        """lane_detection이 노란 중앙선 실제 위치로 역산한 실측 현재 차선.

        -1=미확정, 0=중앙, 1=왼쪽(1차선), 2=오른쪽(2차선)
        """

        value = int(msg.data)
        detected_lane = value if value in (0, 1, 2) else -1
        # Real callbacks always use the node clock.  Lightweight method
        # harnesses have no ROS clock, so conservatively timestamp their cached
        # input at the current state boundary.
        received_at = (
            self._now_seconds()
            if isinstance(self, Node)
            else self.runtime.context.state_entered_at
        )
        if received_at is None:
            return
        if self.runtime.record_lane_position(detected_lane, received_at):
            self.detected_lane = detected_lane

    def road_surface_callback(self, msg: Int32) -> None:
        """Consume CNN road labels: 0 unknown, 1 black road, 2 white shortcut."""

        if self.runtime.fsm.state is not Mode.SHORTCUT:
            return
        now = self._now_seconds()
        if (
            not self._shortcut_exit_sent
            and self.shortcut_exit.update(msg.data, now)
        ):
            self._shortcut_exit_sent = True
            self.runtime.record_shortcut_complete(now)
            self.get_logger().info(
                "지름길 흰 차선 이후 검은 도로 연속 인식 -> 차선 주행 복귀"
            )

    def object_info_raw_callback(self, msg: Float32MultiArray) -> None:
        received_at = self._now_seconds()
        data = msg.data
        result = self.runtime.record_object_info(data, received_at)
        if not result.accepted:
            self._warn_throttled(
                "malformed_object_info_raw",
                result.warning or "invalid object_info_raw message ignored",
                received_at,
            )
            return
        self.obj_exists = float(data[0])
        self.object_dist = float(data[1])
        self.obj_angle = float(data[2])
        self.obj_span = float(data[3])
        self.obj_cluster = float(data[4])
        self.box_size = float(data[5])
        self.box_cx = float(data[6])
        self.box_cy = float(data[7])
        self.box_dx = float(data[8])
        self.car_lane = int(data[9])
        snapshot = self.runtime.latest_object_snapshot
        self.object_type = (
            snapshot.object_type if snapshot is not None else ObjectType.UNKNOWN
        )
        self.object_confidence = (
            snapshot.confidence if snapshot is not None else 0.0
        )
        if (
            snapshot is not None
            and snapshot.box_px > 0.0
            and snapshot.object_type in (ObjectType.FIXED, ObjectType.MOVING)
        ):
            self.object_mission_episode.observe_valid_detection(received_at)
        # Object mission을 이미 commit한 뒤에는 entry lane을 다시 분류하지
        # 않는다. CONE_DRIVE 중에는 edge를 만들지 않지만, 같은 카메라
        # encounter의 초기 YOLO 상태는 보존해야 LANE_DRIVE 복귀 직후 판단할
        # 수 있다.
        if self.runtime.fsm.state in (Mode.LANE_DRIVE, Mode.CONE_DRIVE):
            self.relative_x_fallback.observe(
                object_type=self.object_type,
                box_size=self.box_size,
                box_cx=self.box_cx,
                box_cy=self.box_cy,
                confidence=self.object_confidence,
                received_at=received_at,
            )

    def lane_change_state_callback(self, msg: Int32MultiArray) -> None:
        received_at = self._now_seconds()
        result = self.runtime.record_lane_change_state(msg.data, received_at)
        if not result.accepted:
            self._warn_throttled(
                "malformed_lane_change_state",
                result.warning or "invalid lane_change_state message ignored",
                received_at,
            )

    def ultrasonic_callback(self, msg: Int32MultiArray) -> None:
        data = msg.data
        if len(data) > 4:
            self.left = data[0]
            self.right = data[4]

    def control_cycle(self) -> None:
        now = self._now_seconds()

        if self._last_now is not None and self._last_now - now > BAG_LOOP_BACKJUMP_S:
            self.get_logger().info(
                f"bag 루프 감지 ({self._last_now - now:.1f}초 역행) -> 상태머신 리셋"
            )
            self._reset_for_bag_loop()
        self._last_now = now

        # 전이 없이 시작한 모드(mode:= 로 지정)는 state_entered_at 이 비어 있다.
        # 미션 엣지는 이 값을 기준으로 유효성을 따지므로 첫 주기에 채워준다.
        if self.runtime.context.state_entered_at is None:
            self.runtime.context.state_entered_at = now

        self._drive_mission_zones(now)
        # 같은 차선 감속 판정은 제어값을 만들기 전에 갱신한다.
        brake_ego_lane = effective_ego_lane(
            measured_lane=self.detected_lane,
            measured_received_at=self.runtime.measured_lane_received_at,
            lane_target=self.runtime.context.lane_target,
            now=now,
            max_age_s=self.runtime.lane_position_max_age_s,
        )
        self.last_same_lane_brake = self.same_lane_brake.update(
            now=now,
            car_lane=self.car_lane,
            ego_lane=brake_ego_lane,
            box_px=self.box_size,
        )
        lane_candidate, cone_candidate = self._command_candidates()

        cycle = self.runtime.step(
            now,
            lane=lane_candidate,
            cone=cone_candidate,
        )
        if cycle.transition.changed:
            self.get_logger().info(
                f"FSM {cycle.transition.source.value} -> "
                f"{cycle.transition.target.value}: {cycle.transition.reason}"
            )
        self._commit_object_mission_episode(cycle)
        self._emit_runtime_diagnostic(cycle, now)

        if cycle.cone_session_active_command is not None:
            try:
                dispatch_cone_session_state(
                    cycle,
                    self._publish_cone_session_state,
                )
            except Exception as exc:  # rclpy publisher errors are fail-safe here.
                self.get_logger().error(
                    f"rubbercone session-state publish failed: {exc}"
                )

        # FINISH is a committed terminal state, not a transient safety HOLD.
        # Bypass the shared HOLD deceleration ramp so the very first terminal
        # command and its shaping history are both exactly zero.
        if self.runtime.fsm.state is Mode.FINISH:
            self.now_angle = 0.0
            self.now_speed = 0.0
            angle, speed = 0.0, 0.0
        else:
            angle, speed = self._shape_selected_control(
                cycle.control.source,
                cycle.control.command,
                now,
            )
        # Enter 로 내린 모터 차단. 조향은 제어가 계산한 값을 그대로 내보내고
        # 속도만 0 으로 막는다 — 코너에서 멈출 때 앞바퀴를 0 으로 펴면 다시
        # 출발할 때 차가 코스 밖을 향한다.
        #
        # now_speed 도 함께 0 으로 되돌린다. 이 값은 램프의 현재 상태라, 그냥
        # 두면 다시 켠 순간 멈추기 직전 속도로 튄다. 0 에서 시작하면 평소
        # 가속과 같은 LANE_ACCEL_STEP 램프를 탄다.
        if not self._motor_gate.is_set():
            self.now_speed = 0.0
            speed = 0.0
        motor_msg = Float32MultiArray()
        motor_msg.data = [float(angle), float(speed)]
        self.motor_pub.publish(motor_msg)

        self.lane = self.runtime.context.lane_target.value
        # 차선 변경 명령은 일단 나가면 래치한다.
        #
        # lane_detection 의 추적기는 mode!=5 를 보는 순간 진행도를 버린다
        # (handleCommand → resetProgress). 그런데 safe_to_drive 는 /object_info_raw
        # 신선도(0.25s)를 따라 매 주기 뒤집히고, YOLO 검출은 간헐적으로 끊긴다.
        # 그 결과 내부 명령이 5↔3↔0 으로 요동쳐 추적기가 완료(8프레임 안정)를
        # 모으지 못했다. 실측: 변경 시작 279회 / 성공 0회.
        #
        # 이미 시작한 회피는 끝내는 편이 중간에 포기하는 것보다 안전하므로,
        # pending 인 동안에는 인지가 잠깐 끊겨도 명령을 유지한다.
        action = self.runtime.lane_action
        lane_command_msg = Int32MultiArray()
        lane_command_msg.data = lane_command_data(
            self.runtime.fsm.state,
            self.lane,
            mission_lane_control_enabled=action.safe_to_drive or action.pending,
            lane_change_active=action.pending,
        )
        self.lane_command_pub.publish(lane_command_msg)

        # PPT 공식 외부 인터페이스는 모드와 차선을 분리한다. FINISH/STOP은
        # 아직 팀 코드가 배정되지 않았으므로 임의의 숫자를 발행하지 않는다.
        mode_code = external_mode_code(self.runtime.fsm.state)
        if mode_code is not None:
            mode_msg = Int16()
            mode_msg.data = int(mode_code)
            self.mode_pub.publish(mode_msg)

        lane_info_msg = Int16()
        lane_info_msg.data = lane_info_value(self.lane)
        self.lane_info_pub.publish(lane_info_msg)

        self._update_debug_window(now, angle, speed)

    def _enter_zone(self, now: float) -> None:
        """구간 진입 시 종료 판정 상태를 초기화한다."""

        self.overtake.enter_zone(now)
        self._zone_exit_sent = False
        self.shortcut_exit.reset()
        self._shortcut_exit_sent = False

    def is_change_end(self) -> bool:
        """차선 변경이 끝났는지.

        /lane_change_state 의 성공 엣지를 runtime_adapter 가 추적하므로 그
        결과를 그대로 쓴다.
        """

        return self.runtime.lane_action.completed

    def is_pass_comp(self, now: float) -> bool:
        """추월 완료 여부. 판정은 main.overtake.OvertakeGuard가 한다."""

        clearance_received_at = self.runtime.side_clearance_received_at
        zone_entered_at = self.overtake.zone_entered_at
        clearance_is_fresh = (
            isinstance(clearance_received_at, (int, float))
            and not isinstance(clearance_received_at, bool)
            and np.isfinite(clearance_received_at)
            and isinstance(now, (int, float))
            and not isinstance(now, bool)
            and np.isfinite(now)
            and 0.0 <= now - clearance_received_at <= SIDE_CLEARANCE_MAX_AGE_S
            and isinstance(zone_entered_at, (int, float))
            and not isinstance(zone_entered_at, bool)
            and np.isfinite(zone_entered_at)
            and clearance_received_at > zone_entered_at
        )
        if not clearance_is_fresh:
            self.overtake.invalidate_clearance()
            return False

        decision = self.overtake.update_zone(
            now=now,
            lane_target=self.runtime.context.lane_target.value,
            side_left=self.runtime.latest_side_left,
            side_right=self.runtime.latest_side_right,
            ego_lane=self.detected_lane,
        )
        if decision.side_just_seen:
            self.get_logger().info(
                f"측면 방해차량 인식 ({decision.side_distance:.2f}m), "
                "사라질 때까지 추적")
        if decision.side_just_cleared:
            self.get_logger().info(
                f"측면 방해차량 사라짐, {self.overtake.config.pass_delay_s:.1f}초 "
                "연속 clear 확인 시작")
        if decision.timed_out:
            self.get_logger().warning(
                "방해차량 구간 시간 상한 초과: LiDAR clear가 확인되지 않아 "
                "미션 상태 유지")
        return decision.complete

    @staticmethod
    def _lane_label(lane: int) -> str:
        # cv2.putText는 한글을 못 그리므로 ASCII로 표기한다.
        # 규약: 0=중앙, 1=왼쪽(1차선), 2=오른쪽(2차선)
        return {0: "Center", 1: "Lane 1", 2: "Lane 2"}.get(lane, "Unknown")

    @staticmethod
    def _fmt_side(distance: float) -> str:
        """측면 LiDAR 거리를 디버그 창용으로 만든다.

        perception은 유효한 측면 반사가 없으면 inf를 준다. 'inf'를 그대로 찍으면
        값이 있을 때와 폭이 달라져 눈으로 훑기 나쁘므로 N/A로 통일한다.
        """
        if not np.isfinite(distance):
            return "N/A"
        return f"{distance:.2f} m"

    def _avoid_direction_text(self) -> str:
        """Display the avoidance target approved by object perception."""

        snapshot = self.runtime.latest_object_snapshot
        if snapshot is None:
            return "N/A"
        target = opposite_lane_target(snapshot.lane)
        if target is None:
            return "unconfirmed"
        return f"{self._lane_label(target.value)} (perception stable)"

    def _same_lane_brake_text(self) -> str:
        """같은 차선 감속 상태를 표시한다.

        상한이 무한대면 개입하지 않는 상태이므로 그대로 'off' 로 적는다.
        hold 로 유지 중인지도 함께 보여야, 카메라가 잠깐 놓친 것인지 정말
        차선을 벗어난 것인지 화면에서 구분할 수 있다.
        """

        decision = self.last_same_lane_brake
        if not decision.same_lane:
            return "off"
        limit = decision.speed_limit
        cap = "no cap" if not np.isfinite(limit) else f"<= {limit:.1f}"
        return f"SAME LANE {cap}{' (hold)' if decision.holding else ''}"

    def _lane_command_text(self) -> str:
        """실제로 나가 있는 차선 명령을 표시한다.

        회피와 복귀 모두 runtime_adapter._update_lane_action() 이 단독으로
        내므로, 그 결과(lane_target / lane_action)를 그대로 읽는다.

        구간 밖인지 여부는 바로 위 Mode 줄에 이미 나오므로 여기서 되풀이하지
        않는다. 변경 중일 때만 화살표로 구분한다.

        유지 중이어도 래치된 목표 차선을 반드시 함께 찍는다. 예전에는 그냥
        "Holding" 이라고만 써서, 차선 변경이 끝난 뒤 lane_target 이 1차선에
        물려 있는 상태(= lane_detection 이 기준선을 1차선으로 계속 유지하는
        상태)를 화면 어디서도 볼 수 없었다. 그래서 "Holding 인데 차가 왼쪽으로
        꺾인다"처럼 보였다.
        """
        target = self._lane_label(self.runtime.context.lane_target.value)
        if self.runtime.lane_action.pending:
            return f"-> {target} (changing)"
        return f"Holding @ {target}"

    def _reset_for_bag_loop(self) -> None:
        """bag이 처음부터 다시 재생될 때 내부 상태를 시작 시점으로 되돌린다.

        런타임 어댑터를 통째로 새로 만드는 이유는 수신 시각 때문이다. 시각이
        뒤로 튀면 이전 루프에서 기록한 receipt 시각이 전부 '미래'가 되어,
        신선도 검사(now - received_at)가 음수로 나온다. 그러면 끊긴 입력이
        신선한 것처럼 보이고 미션 엣지 판정도 어긋난다. 시각 기록을 남겨두는
        것보다 버리는 편이 안전하다.
        """

        self.runtime = RaceRuntimeAdapter(
            fsm=RaceFSM(initial_state=self._initial_mode),
            safety_monitor=runtime_safety_monitor(),
        )
        self.runtime.context.lane_target = self._initial_lane_target
        self.lane_controller = Controller()
        self.cone_controller = Controller()
        self._apply_guardrail_params()
        self._apply_steering_filter_params()

        self.overtake.reset()
        self.same_lane_brake.reset()
        self.relative_x_fallback.reset()
        self.object_mission_episode.reset()
        self.traffic_encounter.reset()
        self.last_same_lane_brake = SameLaneBrakeDecision(
            speed_limit=float("inf"), same_lane=False, reason="reset"
        )
        self.runtime_diagnostic_reporter.reset()
        self.last_runtime_diagnostic = None
        self._zone_state = None
        self._zone_exit_sent = False
        self.shortcut_exit.reset()
        self._shortcut_exit_sent = False

        # 인지 캐시도 비운다. 이전 루프의 마지막 프레임이 새 루프 첫 판단에
        # 섞이지 않게 한다.
        self.object_dist = float("inf")
        self.obj_exists = 0.0
        self.box_size = 0.0
        self.car_lane = -1
        self.object_type = ObjectType.UNKNOWN
        self.object_confidence = 0.0
        self.fixed_vehicle_lane = 0
        self.moving_vehicle_lane = 0
        self.rubbercone_offset = 0
        self.rubbercone_end_flag = 0
        self.detected_lane = -1
        self.side_left = float("inf")
        self.side_right = float("inf")
        self.now_speed = 0.0
        self.now_angle = 0.0
        self._publish_cone_session_state(
            self.runtime.fsm.state is Mode.CONE_DRIVE
        )

    def _drive_mission_zones(self, now: float) -> None:
        """Create typed obstacle-zone edges and CNN shortcut-exit sessions.

        ``object_type`` separates fixed and moving obstacles at entry. Both
        zones finish only after the side LiDAR has seen the obstacle, then
        continuously seen clearance for ``pass_delay_s``. The adapter alone
        owns avoidance direction and ``lane_target`` so the avoided lane is
        retained after returning to LANE_DRIVE.
        """

        state = self.runtime.fsm.state
        entered = state is not self._zone_state
        self._zone_state = state

        if self.object_mission_episode.expire(now):
            # Episode ownership and Relative-X evidence share the existing
            # evidence-backed 1.90 s encounter boundary.
            self.relative_x_fallback.reset()

        if state is Mode.LANE_DRIVE:
            if entered:
                self._fixed_entry_sent = False
            self.relative_x_fallback.expire(now)
            entered_at = self.runtime.context.state_entered_at
            snapshot = self.runtime.latest_object_snapshot
            # 엣지는 state_entered_at 보다 "뒤"여야 수락되고 한 번만 소비된다.
            if (
                not self._fixed_entry_sent
                and self.object_mission_episode.entry_allowed
                and snapshot is not None
                and entered_at is not None
                and snapshot.received_at > entered_at
                and self.runtime._event_is_fresh(
                    snapshot.received_at,
                    now,
                    self.runtime.object_max_age_s,
                )
                and snapshot.box_px > FIXED_ENTRY_BOX_PX
                and snapshot.object_type in (ObjectType.FIXED, ObjectType.MOVING)
            ):
                if (
                    snapshot.lane is ObjectLane.UNKNOWN
                    and not self.relative_x_fallback.decided
                    and self.relative_x_fallback.latch_for_entry(
                        self.runtime.context.lane_target
                    )
                ):
                    median_x = self.relative_x_fallback.median_relative_x
                    lane = self.relative_x_fallback.latched_lane
                    samples = self.relative_x_fallback.evidence_samples
                    self.get_logger().info(
                        "relative-X obstacle lane latched at entry: "
                        f"type={snapshot.object_type.name} lane={lane.value} "
                        f"median={median_x:.2f}px samples={samples}"
                    )
                effective_lane = effective_object_lane(
                    snapshot.lane,
                    self.relative_x_fallback.latched_lane,
                )
                if not object_mission_entry_allowed(
                    self.runtime.context.lane_target,
                    effective_lane,
                ):
                    return
                if snapshot.object_type is ObjectType.MOVING:
                    expected_state = Mode.OVERTAKE
                else:
                    expected_state = Mode.FIXED_AVOID
                result = self.runtime.record_object_mission_entry(
                    expected_state,
                    snapshot,
                    now,
                    effective_lane,
                )
                if not result.accepted:
                    return
                # Prevent duplicate queueing before this control cycle runs.
                # If safety rejects the edge, _commit_object_mission_episode()
                # rearms this one-shot without consuming detector ownership.
                self._fixed_entry_sent = True
            return

        if state is Mode.FIXED_AVOID:
            if entered:
                self._enter_zone(now)
            # 고정장애물을 추월할 때까지 이 구간에 머문다. 시간으로 먼저
            # 빠져나가면 회피가 끝나기 전에 모드가 바뀌어 차선 변경 명령이
            # 취소되고, /internal/lane_command가 5↔3↔0 으로 요동친다.
            if not self._zone_exit_sent and self.is_pass_comp(now):
                self._zone_exit_sent = True
                self.runtime.record_fixed_zone_exit(now)
                self.get_logger().info("고정장애물 추월 확인 -> 차선 주행 복귀")
            return

        if state is Mode.OVERTAKE:
            if entered:
                self._enter_zone(now)
            if not self._zone_exit_sent and self.is_pass_comp(now):
                self._zone_exit_sent = True
                self.runtime.record_overtake_complete(now)
                self.get_logger().info("방해차량 추월 확인 -> 현재 차선 유지")
            return

        if state is Mode.SHORTCUT:
            if entered:
                self.shortcut_exit.reset()
                self._shortcut_exit_sent = False
            return

        # 그 밖의 상태(WAIT_GREEN/FINISH)에서는 구간 판정을 쉰다.
        if entered:
            self._enter_zone(now)

    def _apply_guardrail_params(self) -> None:
        """현재 파라미터 값을 두 컨트롤러에 밀어 넣는다.

        _reset_runtime 이 Controller 를 새로 만들기 때문에(bag --loop 되감기)
        생성 직후마다 다시 불러야 한다. 그러지 않으면 루프가 한 바퀴 돌 때
        런타임에 맞춰 둔 이득이 소스 기본값으로 조용히 되돌아간다.
        """

        for controller in (self.lane_controller, self.cone_controller):
            controller.apply_guardrail_params(self._guardrail_overrides)

    def _apply_steering_filter_params(self) -> None:
        """지터 억제 값을 두 컨트롤러에 밀어 넣는다.

        _apply_guardrail_params 와 같은 이유로, Controller 를 새로 만들 때마다
        다시 불러야 소스 기본값으로 되돌아가지 않는다.
        """

        for controller in (self.lane_controller, self.cone_controller):
            controller.apply_steering_filter_params(
                self._steering_filter_overrides)

    def _on_set_parameters(self, parameters) -> SetParametersResult:
        """주행 중 ros2 param set 을 그 자리에서 반영한다.

        rclpy 는 이 콜백을 값이 커밋되기 **전에** 부른다. 그래서 먼저 전부
        검증하고, 통과했을 때만 컨트롤러에 밀어 넣는다. 한 번의 호출에 여러
        항목이 실려 오면 하나만 잘못돼도 전부 거절한다 — 절반만 적용된 이득으로
        주행하는 상태를 만들지 않기 위해서다. (set_parameters 는 항목마다 콜백을
        따로 부르므로 요청끼리는 어차피 독립이고, ros2 param set 도 한 번에
        하나다. 이 규칙은 set_parameters_atomically 와 파라미터 파일 로드처럼
        여러 항목이 한 콜백에 실려 오는 경로에서 의미가 있다.)

        여기 없는 파라미터(mode, lane_target 등)는 손대지 않는다. 그것들은
        시작 시점에 한 번 읽고 끝이라, 런타임에 바꿔도 반영되지 않는다는
        기존 동작을 그대로 둔다.
        """

        overrides = {}
        filter_overrides = {}
        guardrail_enabled = None
        curve_preview_enabled = None
        curve_preview_min_confidence = None
        for parameter in parameters:
            if parameter.name == "lane_guardrail":
                guardrail_enabled = bool(parameter.value)
            elif parameter.name == "curve_preview_enabled":
                curve_preview_enabled = bool(parameter.value)
            elif parameter.name == "curve_preview_min_confidence":
                curve_preview_min_confidence = parameter.value
            elif parameter.name.startswith("guardrail_"):
                overrides[parameter.name[len("guardrail_"):]] = parameter.value
            elif parameter.name in STEERING_FILTER_TUNABLES:
                filter_overrides[parameter.name] = parameter.value
        if (
            not overrides
            and not filter_overrides
            and guardrail_enabled is None
            and curve_preview_enabled is None
            and curve_preview_min_confidence is None
        ):
            return SetParametersResult(successful=True)

        try:
            checked_filter = validate_steering_filter_params(filter_overrides)
            checked = validate_guardrail_params(overrides)
            checked_curve_confidence = (
                validate_curve_preview_min_confidence(
                    curve_preview_min_confidence
                )
                if curve_preview_min_confidence is not None
                else None
            )
        except ValueError as exc:
            self.get_logger().warn(f"control parameter rejected: {exc}")
            return SetParametersResult(successful=False, reason=str(exc))

        if guardrail_enabled is not None:
            self.lane_guardrail_enabled = guardrail_enabled
            self.get_logger().info(f"lane_guardrail -> {guardrail_enabled}")
        if checked:
            self._guardrail_overrides.update(checked)
            self._apply_guardrail_params()
            self.get_logger().info(
                "guardrail params -> "
                + ", ".join(f"{k}={v}" for k, v in sorted(checked.items()))
            )
        if checked_filter:
            self._steering_filter_overrides.update(checked_filter)
            self._apply_steering_filter_params()
            self.get_logger().info(
                "steering filter params -> "
                + ", ".join(
                    f"{k}={v}" for k, v in sorted(checked_filter.items()))
            )
        if curve_preview_enabled is not None:
            self.curve_preview_enabled = curve_preview_enabled
            self.get_logger().info(
                f"curve_preview_enabled -> {curve_preview_enabled}"
            )
        if checked_curve_confidence is not None:
            self.curve_preview_min_confidence = checked_curve_confidence
            self.get_logger().info(
                "curve_preview_min_confidence -> "
                f"{checked_curve_confidence:.2f}"
            )
        return SetParametersResult(successful=True)

    def _guardrail_for(self, control_mode, lane_received_at: float):
        """Return the guardrail margins only where the term is allowed to act.

        Returning None disables it in the controller, which then behaves
        exactly as Pure Pursuit alone.
        """

        if not self.lane_guardrail_enabled:
            return None
        if control_mode is not Mode.LANE_DRIVE:
            # FIXED_AVOID crosses lines on purpose; CONE_DRIVE has no lines.
            return None
        margins = self.runtime.latest_lane_guardrail
        received_at = self.runtime.lane_guardrail_received_at
        if margins is None or received_at is None:
            return None
        # lane_node publishes both topics in one callback, so anything more
        # than a frame apart means one of them stalled.
        if abs(received_at - lane_received_at) > GUARDRAIL_MAX_SKEW_S:
            return None
        if self.runtime.lane_change_in_progress(self._now_seconds()):
            return None
        return margins

    def _curve_preview_for(
        self,
        control_mode,
        lane_received_at: float,
        lane_offset: float,
    ):
        """Return a fresh, confident preview or ``None`` for exact legacy control.

        The feature gate lives here rather than in lane_node, so a single runtime
        parameter switch changes the next motor command without restarting the
        perception pipeline. Lane changes deliberately keep the established
        offset/ref-transition controller; a far preview from the moving reference
        would otherwise add a second lane-change command.
        """

        if not self.curve_preview_enabled or control_mode is not Mode.LANE_DRIVE:
            return None
        # lane_action.pending is the authoritative command latch. It becomes
        # true before lane_node feedback can arrive, so relying on the feedback
        # topic alone leaves a window where preview steering fights the change.
        if self.runtime.lane_action.pending:
            return None
        preview = self.runtime.latest_lane_path_preview
        received_at = self.runtime.lane_path_preview_received_at
        if preview is None or received_at is None:
            return None
        if abs(received_at - lane_received_at) > LANE_PATH_PREVIEW_MAX_SKEW_S:
            return None
        (
            target_offset,
            curvature,
            confidence,
            _target_y_ratio,
            source_lane_offset,
        ) = preview
        # Separate BestEffort topics have no shared Header. Pairing their receipt
        # times is not enough when one topic drops, so also require the offset
        # copied into the preview message to match the selected lane input.
        if abs(source_lane_offset - float(lane_offset)) > 0.5:
            return None
        if confidence < self.curve_preview_min_confidence:
            return None
        # Start continuously at zero contribution at the confidence threshold;
        # otherwise crossing 0.55 by one sample causes a visible steering step.
        if self.curve_preview_min_confidence >= 1.0:
            effective_confidence = 1.0
        else:
            effective_confidence = (
                confidence - self.curve_preview_min_confidence
            ) / (1.0 - self.curve_preview_min_confidence)
        if effective_confidence <= 0.0:
            return None
        if self.runtime.lane_change_in_progress(self._now_seconds()):
            return None
        return target_offset, curvature, effective_confidence

    def _command_candidates(self):
        lane_candidate = None
        lane_received_at = self.runtime.latest_lane_received_at
        lane_offset = self.runtime.latest_lane_offset
        if lane_received_at is not None and lane_offset is not None:
            # FIXED_AVOID 구간에서는 회피 전용 PD/속도 프로필을 쓴다.
            control_mode = (
                Mode.FIXED_AVOID
                if self.runtime.fsm.state is Mode.FIXED_AVOID
                else Mode.LANE_DRIVE
            )
            self.lane_controller.update(
                control_mode,
                lane_offset,
                self.object_dist,
                100,
                self._guardrail_for(control_mode, lane_received_at),
                self._curve_preview_for(
                    control_mode, lane_received_at, lane_offset
                ),
            )
            speed = float(self.lane_controller.get_speed()) #/2.0 속도 절반
            # 고정장애물이 우리 차선에 있으면 접근할수록 속도를 낮춘다.
            # 구간(FIXED_AVOID) 안팎을 가리지 않는다 — 진입 전에 미리 줄여 두는
            # 편이 회피할 시간을 벌기 때문이다. 회피에 성공해 차선을 옮기면
            # car_lane != ego_lane 이 되어 상한이 곧바로 풀린다.
            speed = min(speed, self.last_same_lane_brake.speed_limit)
            lane_candidate = CommandCandidate(
                DriveCommand(
                    angle=float(self.lane_controller.get_angle()),
                    speed=speed,
                ),
                received_at=lane_received_at,
            )

        cone_candidate = None
        cone_event = self.runtime.latest_cone_event
        if cone_event is not None and not cone_event.end_flag:
            self.cone_controller.update(
                Mode.CONE_DRIVE,
                cone_event.offset,
                self.object_dist,
                cone_event.confidence,
            )
            cone_candidate = CommandCandidate(
                DriveCommand(
                    angle=float(self.cone_controller.get_angle()),
                    speed=float(self.cone_controller.get_speed() / 2.0),
                ),
                received_at=cone_event.received_at,
            )
        return lane_candidate, cone_candidate

    def _shape_selected_control(
        self,
        source: ControlSource,
        command: DriveCommand,
        now: float,
    ) -> tuple[float, float]:
        target_angle = command.angle
        target_speed = command.speed

        race_started_at = self.runtime.context.race_started_at
        if (
            source is ControlSource.LANE
            and race_started_at is not None
            and 0.0 <= now - race_started_at < 3.0
        ):
            target_speed = min(target_speed, 5.0)

        if source is ControlSource.HOLD:
            # 감속은 램프로 한다. 인지가 한 프레임 늦었다고 속도를 즉시 0으로
            # 꺾으면 재가속에 수 초가 걸려, 다음 끊김 전에 순항 속도를 회복하지
            # 못한다. 실측(bag): 속도 0인 사이클 24.2%, 5 미만 83.4%, 평균 2.24
            # (LANE_DRIVE 목표는 21.5). 램프면 0.3초짜리 공백은 속도를 조금
            # 깎을 뿐이고, 진짜 정지 사유는 0.7초 안에 완전히 멈춘다.
            #
            # 조향은 마지막 값을 유지한다. 곡선 주행 중 감속하면서 앞바퀴를
            # 0으로 펴면 차가 코스 밖으로 밀려난다.
            self.now_speed = max(0.0, self.now_speed - STOP_DECEL_STEP)
        elif source is ControlSource.CONE:
            angle_delta = target_angle - self.now_angle
            self.now_angle += max(
                -RUBBERCONE_STEERING_STEP,
                min(RUBBERCONE_STEERING_STEP, angle_delta),
            )
            if self.now_speed < target_speed:
                self.now_speed = min(
                    target_speed,
                    self.now_speed + RUBBERCONE_ACCEL_STEP,
                )
            else:
                self.now_speed = max(
                    target_speed,
                    self.now_speed - RUBBERCONE_DECEL_STEP,
                )
        else:
            self.now_angle = target_angle
            if self.now_speed < target_speed:
                self.now_speed = min(
                    target_speed,
                    self.now_speed + LANE_ACCEL_STEP,
                )
            else:
                self.now_speed = target_speed

        return self.now_angle, self.now_speed

    def _warn_throttled(
        self,
        key: str,
        message: str,
        now: float,
    ) -> None:
        last = self._warning_times.get(key)
        if (
            last is None
            or now < last
            or now - last >= CONE_EVENT_WARNING_PERIOD_S
        ):
            self._warning_times[key] = now
            self.get_logger().warning(message)

    def _runtime_diagnostic_snapshot(
        self,
        cycle,
        now: float,
    ) -> RuntimeDiagnosticSnapshot:
        state = self.runtime.fsm.state
        # On the exact transition cycle _drive_mission_zones() still belongs
        # to the source state. Do not display the previous zone's timer as the
        # newly entered mission's elapsed time.
        zone_active = (
            state in (Mode.FIXED_AVOID, Mode.OVERTAKE)
            and self._zone_state is state
        )
        zone_elapsed = self.overtake.zone_elapsed(now) if zone_active else None
        clear_elapsed = None
        if zone_active and self.overtake.clear_started_at is not None:
            clear_elapsed = now - self.overtake.clear_started_at
        action = self.runtime.lane_action
        return RuntimeDiagnosticSnapshot(
            mode=state.value,
            control_source=cycle.control.source.value,
            control_reason=cycle.control.reason,
            safety_reason=cycle.safety.reason,
            missing_inputs=cycle.safety.missing_inputs,
            stale_inputs=cycle.safety.stale_inputs,
            traffic_stop_override=self.runtime.traffic_stop_override,
            lane_action_safe_to_drive=action.safe_to_drive,
            lane_action_pending=action.pending,
            lane_action_completed=action.completed,
            zone_elapsed_s=zone_elapsed,
            side_obstacle_seen=self.overtake.side_seen_at is not None,
            clear_timer_elapsed_s=clear_elapsed,
            same_lane_brake_reason=self.last_same_lane_brake.reason,
        )

    def _commit_object_mission_episode(self, cycle) -> None:
        """Consume object ownership only after the FSM accepts the entry."""

        transition = cycle.transition
        if (
            not transition.changed
            or transition.source is not Mode.LANE_DRIVE
            or transition.target not in (Mode.FIXED_AVOID, Mode.OVERTAKE)
        ):
            if self.runtime.fsm.state is Mode.LANE_DRIVE:
                self._fixed_entry_sent = False
            return
        self.object_mission_episode.consume()
        self._fixed_entry_sent = True
        label = "고정장애물" if transition.target is Mode.FIXED_AVOID else "방해차량"
        self.get_logger().info(
            f"{label} 미션 entry commit -> detector episode consumed"
        )

    def _emit_runtime_diagnostic(self, cycle, now: float) -> None:
        snapshot = self._runtime_diagnostic_snapshot(cycle, now)
        self.last_runtime_diagnostic = snapshot
        message = self.runtime_diagnostic_reporter.update(snapshot, now)
        if message is not None:
            self.get_logger().info(message)

    def _update_debug_window(
        self,
        now: float,
        angle: float,
        speed: float,
    ) -> None:
        if not self.show_debug:
            return
        if self.last_debug_time is not None:
            elapsed = now - self.last_debug_time
            if 0.0 <= elapsed < 0.1:
                return
        self.last_debug_time = now

        cone_event = self.runtime.latest_cone_event
        cone_confidence = cone_event.confidence if cone_event is not None else 0
        cone_end = cone_event.end_flag if cone_event is not None else None
        offset = (
            cone_event.offset
            if self.runtime.fsm.state is Mode.CONE_DRIVE and cone_event is not None
            else self.runtime.latest_lane_offset
        )
        lines = [
            f"Mode: {self.runtime.fsm.state.value}",
            f"Angle: {angle:.1f}",
            f"Speed: {speed:.1f}",
            f"Offset: {offset if offset is not None else 'N/A'}",
            f"Rubber confidence: {cone_confidence}%",
            f"Rubber end: {cone_end if cone_end is not None else 'N/A'}",
            f"Object dist: {self.object_dist:.2f} m",
            f"Side L/R: {self._fmt_side(self.side_left)}"
            f" / {self._fmt_side(self.side_right)}",
            f"Box: {self.box_size:.0f}px^2  car_lane={self.car_lane}",
            f"Object: {self.object_type.name} conf={self.object_confidence:.2f}",
            f"Avoid dir: {self._avoid_direction_text()}",
            f"Current lane: {self._lane_label(self.detected_lane)}",
            f"Lane cmd: {self._lane_command_text()}",
            f"Same-lane brake: {self._same_lane_brake_text()}",
        ]
        diagnostic = self.last_runtime_diagnostic
        if diagnostic is not None:
            lines.extend(
                [
                    "Control: "
                    f"{diagnostic.control_source} ({diagnostic.control_reason})",
                    f"Safety: {diagnostic.safety_reason or 'ready'}",
                    "Missing: "
                    f"{', '.join(diagnostic.missing_inputs) or 'none'}",
                    f"Stale: {', '.join(diagnostic.stale_inputs) or 'none'}",
                    "Traffic hold: "
                    f"{diagnostic.traffic_stop_override}",
                    "Lane action: "
                    f"safe={diagnostic.lane_action_safe_to_drive} "
                    f"pending={diagnostic.lane_action_pending} "
                    f"complete={diagnostic.lane_action_completed}",
                    "Zone: "
                    f"elapsed={diagnostic.zone_elapsed_s} "
                    f"side_seen={diagnostic.side_obstacle_seen} "
                    f"clear={diagnostic.clear_timer_elapsed_s}",
                    "Brake reason: "
                    f"{diagnostic.same_lane_brake_reason}",
                ]
            )
        # 줄 수에 맞춰 캔버스를 잡는다. 높이를 고정해 두면 줄을 추가했을 때
        # 마지막 줄이 조용히 화면 밖으로 밀려난다.
        image = np.zeros((30 + len(lines) * 32, 640, 3), dtype=np.uint8)
        for index, text in enumerate(lines):
            cv2.putText(
                image,
                text,
                (10, 30 + index * 32),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )
        # imshow/waitKey는 여기서 직접 부르지 않고 표시 스레드에 넘긴다.
        with self._status_img_lock:
            self._status_img = image


def main(args=None):
    rclpy.init(args=args)
    node = MainNode()
    try:
        if node.show_debug:
            # OpenCV(GTK 백엔드)는 imshow/waitKey를 **메인 스레드**에서 불러야
            # 창이 화면에 매핑된다. 백그라운드 스레드에서 부르면 창 객체는
            # 만들어지지만(AUTOSIZE 조회는 성공) 실제로 보이지 않는다.
            # 그래서 rclpy를 보조 스레드로 돌리고 표시를 메인 스레드가 맡는다.
            spin_thread = threading.Thread(
                target=rclpy.spin,
                args=(node,),
                daemon=True,
            )
            spin_thread.start()
            node.run_display_loop()
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
