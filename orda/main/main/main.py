"""ROS runtime wiring for the authoritative 2026 finals race FSM."""

import threading
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import LaserScan
from std_msgs.msg import (
    Bool,
    Float32MultiArray,
    Int16,
    Int32,
    Int32MultiArray,
)

from main.control import Controller
from main.control_selector import (
    CommandCandidate,
    ControlSource,
    DriveCommand,
)
from main.mission_types import (
    LaneTarget,
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
from main.same_lane_brake import SameLaneBrake, SameLaneBrakeDecision
from main.shortcut_exit import ShortcutExitGuard
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
# 0.10 (5.0/s) took 2.2 s to reach the tuned 11.0 cruise speed in short
# rubber-cone sections. The tuned steps reach it in about 0.9 s while keeping
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

# 이보다 크게 시각이 역행하면 "bag이 처음부터 다시 재생됨"으로 본다.
# 실차 wall clock도 NTP 보정으로 아주 드물게 살짝 뒤로 갈 수 있어, 그런 미세
# 역행까지 리셋으로 잡지 않도록 넉넉한 임계값을 둔다.
BAG_LOOP_BACKJUMP_S = 2.0

# 이 면적을 넘는 박스를 object_type에 따라 FIXED_AVOID 또는 OVERTAKE로 보낸다.
FIXED_ENTRY_BOX_PX = 1900.0

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
        test_profile = parse_test_profile(
            self.get_parameter("test_profile").value
        )
        self._initial_mode = parse_initial_mode(self.get_parameter("mode").value)
        self.show_debug = bool(self.get_parameter("show_debug").value)

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
        self.last_same_lane_brake = SameLaneBrakeDecision(
            speed_limit=float("inf"), same_lane=False, reason="not evaluated yet"
        )
        self._zone_state: Optional[Mode] = None   # 직전 사이클의 FSM 상태
        self._zone_exit_sent = False              # 현재 구간의 종료 엣지를 이미 냈는지
        self._fixed_entry_sent = False            # 이번 LANE_DRIVE 세션에서 진입 엣지를 냈는지
        self.shortcut_exit = ShortcutExitGuard()
        self._shortcut_exit_sent = False

        self.now_speed = 0.0
        self.now_angle = 0.0
        self.last_debug_time: Optional[float] = None
        self._warning_times: dict[str, float] = {}
        self._traffic_encounter_active = False

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

        # One lap encounter edge per visible traffic-light episode.
        # RED_AMBER does not start an encounter. UNKNOWN rearms the edge.
        encounter_started = False
        if signal is RouteTrafficSignal.UNKNOWN:
            self._traffic_encounter_active = False
        elif signal in (
            RouteTrafficSignal.STRAIGHT,
            RouteTrafficSignal.LEFT,
        ):
            encounter_started = not self._traffic_encounter_active
            self._traffic_encounter_active = True

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
        self.last_same_lane_brake = self.same_lane_brake.update(
            now=now,
            car_lane=self.car_lane,
            ego_lane=self.detected_lane,
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

        angle, speed = self._shape_selected_control(
            cycle.control.source,
            cycle.control.command,
            now,
        )
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

        self.overtake.reset()
        self.same_lane_brake.reset()
        self.last_same_lane_brake = SameLaneBrakeDecision(
            speed_limit=float("inf"), same_lane=False, reason="reset"
        )
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

        if state is Mode.LANE_DRIVE:
            if entered:
                self._fixed_entry_sent = False
            entered_at = self.runtime.context.state_entered_at
            snapshot = self.runtime.latest_object_snapshot
            # 엣지는 state_entered_at 보다 "뒤"여야 수락되고 한 번만 소비된다.
            if (
                not self._fixed_entry_sent
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
                if snapshot.object_type is ObjectType.MOVING:
                    expected_state = Mode.OVERTAKE
                    label = "방해차량"
                else:
                    expected_state = Mode.FIXED_AVOID
                    label = "고정장애물"
                result = self.runtime.record_object_mission_entry(
                    expected_state,
                    snapshot,
                    now,
                )
                if not result.accepted:
                    return
                self._fixed_entry_sent = True
                self.get_logger().info(
                    f"{label} 검출 ({snapshot.box_px:.0f}px^2) -> 미션 구간 진입")
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
            )
            speed = float(self.lane_controller.get_speed() / 2.0)
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
