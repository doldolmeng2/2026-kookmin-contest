#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────────────────
# lane_drive_only.py
#
# 역할: main_node 없이 차선 주행만 시키는 독립 실행 노드
#
# main_node와의 차이:
#   - RaceFSM, SafetyMonitor, ControlSelector를 쓰지 않는다.
#     따라서 입력이 잠깐 끊겨도 터미널 STOP으로 래치되지 않는다.
#   - 신호등, 라바콘, 장애물, 차선변경을 전혀 보지 않는다.
#
# 제어식은 main 패키지 control.py의 LANE_DRIVE 프로필과 동일하다.
#   조향: angle = kp * offset + kd * (offset - prev_offset)
#   속도: speed = max_speed - |angle| * scale_factor, 하한 min_speed
#   발행: 위 speed에 speed_scale(기본 0.5)을 곱한 값
#
# 입력이 stale_after_s 이상 끊기면 정지 명령(0, 0)을 내보내고,
# 다시 들어오면 그대로 주행을 재개한다.
#
# 실행:
#   python3 lane_drive_only.py
#   python3 lane_drive_only.py --ros-args -p max_speed:=30.0
# ─────────────────────────────────────────────────────────────────────────────

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, Int16


class LaneDriveOnly(Node):
    def __init__(self):
        super().__init__('lane_drive_only')

        # ── 튜닝 파라미터 (control.py의 LANE_DRIVE 값과 동일한 기본값) ──────
        self.kp = self.declare_parameter('kp', 0.145).value
        self.kd = self.declare_parameter('kd', 0.3).value
        self.max_speed = self.declare_parameter('max_speed', 43.0).value
        self.min_speed = self.declare_parameter('min_speed', 16.0).value
        self.scale_factor = self.declare_parameter('scale_factor', 0.5).value
        # main.py가 최종 발행 직전에 적용하는 1/2 스케일
        self.speed_scale = self.declare_parameter('speed_scale', 0.5).value
        # 가속 스텝 (감속은 즉시). main.py와 동일하게 사이클당 0.1
        self.accel_step = self.declare_parameter('accel_step', 0.1).value
        # 이 시간 이상 /lane_offset이 없으면 정지 명령을 낸다
        self.stale_after_s = self.declare_parameter('stale_after_s', 0.5).value

        # ── 제어 상태 ────────────────────────────────────────────────────
        self.prev_offset = 0.0
        self.now_speed = 0.0
        self.last_offset = None
        self.last_received_at = None

        # lane_node의 /lane_offset 발행 QoS와 일치시킨다
        offset_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        command_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.motor_pub = self.create_publisher(
            Float32MultiArray, 'xycar_motor', command_qos
        )
        self.offset_sub = self.create_subscription(
            Int16, 'lane_offset', self.lane_offset_callback, offset_qos
        )

        self.create_timer(0.02, self.control_cycle)
        self.get_logger().info(
            'lane_drive_only 시작: /lane_offset -> /xycar_motor '
            f'(kp={self.kp}, kd={self.kd}, '
            f'speed={self.min_speed * self.speed_scale:.1f}'
            f'~{self.max_speed * self.speed_scale:.1f})'
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def lane_offset_callback(self, msg: Int16) -> None:
        self.last_offset = float(msg.data)
        self.last_received_at = self._now()

    def control_cycle(self) -> None:
        now = self._now()

        # 입력이 없거나 오래됐으면 정지. STOP으로 래치하지 않고
        # 새 오프셋이 들어오는 즉시 다시 주행한다.
        if (
            self.last_offset is None
            or self.last_received_at is None
            or not (0.0 <= now - self.last_received_at <= self.stale_after_s)
        ):
            self.now_speed = 0.0
            self.prev_offset = 0.0
            self.publish(0.0, 0.0)
            return

        error = self.last_offset
        diff = error - self.prev_offset
        self.prev_offset = error
        angle = self.kp * error + self.kd * diff

        target_speed = max(
            self.min_speed, self.max_speed - abs(angle) * self.scale_factor
        ) * self.speed_scale

        # 가속은 스텝 제한, 감속은 즉시 (main.py와 동일)
        if self.now_speed < target_speed:
            self.now_speed = min(target_speed, self.now_speed + self.accel_step)
        else:
            self.now_speed = target_speed

        self.publish(angle, self.now_speed)

    def publish(self, angle: float, speed: float) -> None:
        msg = Float32MultiArray()
        msg.data = [float(angle), float(speed)]
        self.motor_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaneDriveOnly()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # 컨텍스트가 아직 살아 있을 때만 마지막 정지 명령을 낸다.
        # 외부 SIGTERM으로 이미 종료된 뒤에 발행하면 RCLError가 난다.
        if rclpy.ok():
            node.publish(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
