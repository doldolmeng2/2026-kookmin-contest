"""ROS runtime wiring for the authoritative 2026 finals race FSM."""

import threading
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import (
    Bool,
    Empty,
    Float32MultiArray,
    Int16,
    Int32MultiArray,
)

from main.control import Controller
from main.control_selector import (
    CommandCandidate,
    ControlSource,
    DriveCommand,
)
from main.mode_info import mode_info_data
from main.race_fsm import Mode, RaceFSM
from main.runtime_adapter import (
    RaceRuntimeAdapter,
    dispatch_cone_reset,
    runtime_safety_monitor,
)


RUBBERCONE_INFO_TOPIC = "/rubbercone_info"
RUBBERCONE_RESET_TOPIC = "/rubbercone_reset"
LANE_VALIDITY_TOPIC = "/lane_valid"
LANE_POSITION_TOPIC = "/lane_position"
LANE_VALIDITY_REQUIRED_RATE_HZ = 10.0
CONE_EVENT_WARNING_PERIOD_S = 5.0

# bag을 --loop --clock 으로 재생하면 루프 시작 시 ROS 시각이 뒤로 튄다.
# 이보다 크게 역행하면 "bag이 처음부터 다시 재생됐다"고 보고 표시 상태를 초기화한다.
BAG_LOOP_BACKJUMP_S = 2.0

# The controller gains remain the proven legacy implementation. These limits
# are output shaping only; they do not participate in mission transitions.
# 0.10 (5.0/s) took 2.2 s to reach the tuned 11.0 cruise speed in short
# rubber-cone sections. The tuned steps reach it in about 0.9 s while keeping
# deceleration faster than acceleration.
RUBBERCONE_ACCEL_STEP = 0.25
RUBBERCONE_DECEL_STEP = 0.35
RUBBERCONE_STEERING_STEP = 3.0

# Keep the launch file's existing integer ``mode`` parameter usable for the
# states that have a defined 2026 counterpart. Unsupported legacy mission
# states are rejected instead of being guessed.
_LEGACY_INITIAL_MODE = {
    0: Mode.INIT,
    1: Mode.CONE_DRIVE,
    2: Mode.REJOIN,
    3: Mode.LANE_DRIVE,
}


def parse_initial_mode(value) -> Mode:
    """Parse a RaceFSM name or a supported legacy launch value."""

    if isinstance(value, Mode):
        return value
    if isinstance(value, int) and not isinstance(value, bool):
        if value in _LEGACY_INITIAL_MODE:
            return _LEGACY_INITIAL_MODE[value]
        raise ValueError(f"unsupported legacy initial mode: {value}")
    if isinstance(value, str):
        stripped = value.strip()
        if stripped.lstrip("-").isdigit():
            return parse_initial_mode(int(stripped))
        try:
            return Mode(stripped)
        except ValueError as exc:
            raise ValueError(f"unknown RaceFSM initial mode: {value}") from exc
    raise ValueError(f"invalid initial mode value: {value!r}")


def rubbercone_reset_qos() -> QoSProfile:
    """Return the detector's reliable edge-command QoS contract."""

    return QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def sensor_event_qos(*, depth: int = 1) -> QoSProfile:
    """Return the BestEffort/Volatile contract used by sensor publishers."""

    return QoSProfile(
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def lane_validity_qos() -> QoSProfile:
    """Return the required QoS for the currently missing validity publisher."""

    return sensor_event_qos(depth=10)


class MainNode(Node):
    """Collect ROS inputs, step one RaceFSM, select, and publish control."""

    def __init__(self, *, context: Optional[Context] = None):
        super().__init__("main_node", context=context)

        self.declare_parameter("mode", 0)
        self.declare_parameter("show_debug", False)
        initial_mode = parse_initial_mode(self.get_parameter("mode").value)
        self.show_debug = bool(self.get_parameter("show_debug").value)

        self.runtime = RaceRuntimeAdapter(
            fsm=RaceFSM(initial_state=initial_mode),
            safety_monitor=runtime_safety_monitor(),
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
        self.left = float("inf")
        self.right = float("inf")

        # /lane_position 으로 받는 실측 현재 차선 (-1=미확정, 0=1차선, 1=2차선).
        # self.lane 은 "가려는" 차선(명령)이라 차선 변경 도중에는 목표를 가리키므로
        # 방해차량이 내 차선에 있는지 판정하려면 이 실측값이 따로 필요하다.
        self.detected_lane = -1
        # 박스 면적 변화율(px/s, EMA). 양수=작아지는 중(멀어짐/추월), 음수=접근 중.
        self.box_shrink_rate = 0.0
        self._prev_box_size: Optional[float] = None
        self._prev_box_time: Optional[float] = None

        self.now_speed = 0.0
        self.now_angle = 0.0
        self.last_debug_time: Optional[float] = None
        self._warning_times: dict[str, float] = {}

        # imshow/waitKey는 rclpy 콜백에서 직접 부르지 않고 전용 스레드에 넘긴다.
        # WSLg 환경에서 콜백 안 imshow가 창을 못 띄우거나 인지 주기를 늘어뜨렸다.
        self._status_img_lock = threading.Lock()
        self._latest_status_img: Optional[np.ndarray] = None
        self._display_stop = threading.Event()
        self._display_thread: Optional[threading.Thread] = None
        if self.show_debug:
            self._display_thread = threading.Thread(
                target=self._display_loop, daemon=True
            )
            self._display_thread.start()

        qos_fast = sensor_event_qos()
        qos_command = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        reset_qos = rubbercone_reset_qos()

        self.motor_pub = self.create_publisher(
            Float32MultiArray,
            "xycar_motor",
            qos_command,
        )
        self.mode_pub = self.create_publisher(
            Int32MultiArray,
            "mode_info",
            qos_fast,
        )
        self.rubbercone_reset_pub = self.create_publisher(
            Empty,
            RUBBERCONE_RESET_TOPIC,
            reset_qos,
        )

        self.rubbercone_info_sub = self.create_subscription(
            Int32MultiArray,
            RUBBERCONE_INFO_TOPIC,
            self.rubbercone_callback,
            qos_fast,
        )
        self.lane_offset_sub = self.create_subscription(
            Int16,
            "lane_offset",
            self.lane_offset_callback,
            qos_fast,
        )
        self.lane_position_sub = self.create_subscription(
            Int16,
            LANE_POSITION_TOPIC,
            self.lane_position_callback,
            qos_fast,
        )
        self.object_info_sub = self.create_subscription(
            Float32MultiArray,
            "object_info",
            self.object_info_callback,
            qos_fast,
        )
        self.traffic_sub = self.create_subscription(
            Bool,
            "traffic_detection",
            self.traffic_callback,
            qos_fast,
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos_fast,
        )
        self.lane_validity_sub = self.create_subscription(
            Bool,
            LANE_VALIDITY_TOPIC,
            self.lane_validity_callback,
            lane_validity_qos(),
        )
        self.ultrasonic_sub = self.create_subscription(
            Int32MultiArray,
            "xycar_ultrasonic",
            self.ultrasonic_callback,
            10,
        )

        self.create_timer(0.02, self.control_cycle)

    def _now_seconds(self) -> float:
        """Read the node ROS clock used by both callbacks and FSM steps."""

        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def rubbercone_callback(self, msg: Int32MultiArray) -> None:
        received_at = self._now_seconds()
        result = self.runtime.record_cone_message(msg.data, received_at)
        if result.warning is not None:
            key = "cone_queue_overflow" if result.dropped_oldest else "malformed_cone"
            self._warn_throttled(key, result.warning, received_at)

    def lane_offset_callback(self, msg: Int16) -> None:
        received_at = self._now_seconds()
        if not self.runtime.record_lane_offset(msg.data, received_at):
            self._warn_throttled(
                "malformed_lane",
                "invalid lane_offset message ignored",
                received_at,
            )

    def traffic_callback(self, msg: Bool) -> None:
        received_at = self._now_seconds()
        self.runtime.record_traffic(msg.data, received_at)

    def scan_callback(self, _msg: LaserScan) -> None:
        self.runtime.record_scan(self._now_seconds())

    def lane_validity_callback(self, msg: Bool) -> None:
        """Record only an explicit future lane-validity publisher edge."""

        received_at = self._now_seconds()
        if not self.runtime.record_lane_validity(msg.data, received_at):
            self._warn_throttled(
                "malformed_lane_validity",
                "invalid lane-validity message ignored",
                received_at,
            )

    def lane_position_callback(self, msg: Int16) -> None:
        """lane_node가 노란 중앙선 위치로 판정한 실측 현재 차선."""
        value = int(msg.data)
        self.detected_lane = value if value in (0, 1) else -1

    def object_info_callback(self, msg: Float32MultiArray) -> None:
        data = msg.data
        if len(data) < 10:
            return
        self._update_box_shrink_rate(float(data[5]))
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

    def _update_box_shrink_rate(self, box_size: float) -> None:
        """박스 면적 변화율을 EMA로 갱신한다 (추월 진행 상황 판단용)."""
        now = self._now_seconds()
        prev_size, prev_time = self._prev_box_size, self._prev_box_time
        self._prev_box_size, self._prev_box_time = box_size, now
        if prev_size is None or prev_time is None:
            return
        dt = now - prev_time
        if dt <= 0.0:  # bag 루프 등으로 시각이 역행하면 이번 샘플은 버린다
            return
        raw_rate = (prev_size - box_size) / dt
        alpha = 0.2
        self.box_shrink_rate = alpha * raw_rate + (1.0 - alpha) * self.box_shrink_rate

    def is_obstacle_in_ego_lane(self) -> bool:
        """
        검출된 장애물이 우리 차량과 같은 차선에 있는지 판정한다.

        car_lane 이 0(중앙)이거나 미확정이면 보수적으로 True 를 반환한다.
        회피를 놓치는 쪽이 불필요하게 피하는 쪽보다 비용이 크기 때문이다.
        실측 차선(/lane_position)이 아직 안 들어왔으면 명령 차선으로 폴백한다.
        """
        if self.car_lane not in (1, 2):
            return True
        obstacle_lane = 0 if self.car_lane == 1 else 1
        ego_lane = self.detected_lane if self.detected_lane in (0, 1) else self.lane
        return obstacle_lane == ego_lane

    def ultrasonic_callback(self, msg: Int32MultiArray) -> None:
        data = msg.data
        if len(data) > 4:
            self.left = data[0]
            self.right = data[4]

    def control_cycle(self) -> None:
        now = self._now_seconds()
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

        if cycle.publish_cone_reset:
            try:
                dispatch_cone_reset(
                    cycle,
                    lambda: self.rubbercone_reset_pub.publish(Empty()),
                )
            except Exception as exc:  # rclpy publisher errors are fail-safe here.
                self.get_logger().error(f"rubbercone reset publish failed: {exc}")

        angle, speed = self._shape_selected_control(
            cycle.control.source,
            cycle.control.command,
            now,
        )
        motor_msg = Float32MultiArray()
        motor_msg.data = [float(angle), float(speed)]
        self.motor_pub.publish(motor_msg)

        mode_msg = Int32MultiArray()
        mode_msg.data = mode_info_data(self.runtime.fsm.state, self.lane)
        self.mode_pub.publish(mode_msg)

        self._update_debug_window(now, angle, speed)

    def _command_candidates(self):
        lane_candidate = None
        lane_received_at = self.runtime.latest_lane_received_at
        lane_offset = self.runtime.latest_lane_offset
        if lane_received_at is not None and lane_offset is not None:
            self.lane_controller.update(
                Mode.LANE_DRIVE,
                lane_offset,
                self.object_dist,
                100,
            )
            lane_candidate = CommandCandidate(
                DriveCommand(
                    angle=float(self.lane_controller.get_angle()),
                    speed=float(self.lane_controller.get_speed() / 2.0),
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

        if source is ControlSource.STOP:
            self.now_angle = 0.0
            self.now_speed = 0.0
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
                self.now_speed = min(target_speed, self.now_speed + 0.1)
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
            if elapsed < -BAG_LOOP_BACKJUMP_S:
                # bag이 처음부터 다시 재생됨. 변화율 추정을 새로 시작한다.
                self._prev_box_size = None
                self._prev_box_time = None
                self.box_shrink_rate = 0.0
            elif 0.0 <= elapsed < 0.1:
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
            f"Box: {self.box_size:.0f}px^2  car_lane={self.car_lane}",
            f"Box shrink rate: {self.box_shrink_rate:+.1f} px/s",
            f"Obstacle in ego lane: {self.is_obstacle_in_ego_lane()}",
        ]
        image = np.zeros((360, 640, 3), dtype=np.uint8)
        # 실측 현재 차선은 판단의 핵심이라 큰 글씨로 따로 그린다.
        detected_label = {-1: "Unknown", 0: "Lane 1", 1: "Lane 2"}[self.detected_lane]
        cv2.putText(
            image,
            f"Current Lane: {detected_label}",
            (10, 36),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.1,
            (0, 255, 255),
            3,
        )
        for index, text in enumerate(lines):
            cv2.putText(
                image,
                text,
                (10, 76 + index * 32),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )
        with self._status_img_lock:
            self._latest_status_img = image

    def _display_loop(self) -> None:
        """상태 창 전용 표시 스레드 (rclpy 콜백에서 imshow를 부르지 않기 위함)."""
        while not self._display_stop.is_set():
            with self._status_img_lock:
                image = self._latest_status_img
            if image is not None:
                cv2.imshow("Status", image)
            cv2.waitKey(30)
        cv2.destroyAllWindows()

    def stop_display(self) -> None:
        """표시 스레드를 정리한다 (노드 종료 시 호출)."""
        self._display_stop.set()
        if self._display_thread is not None:
            self._display_thread.join(timeout=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = MainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_display()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
