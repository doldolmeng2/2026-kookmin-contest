#!/usr/bin/env python3
"""Drive the car from rubbercone perception only.

This node is intentionally small: it lets the rubbercone detector be tested
without the unfinished race FSM. It publishes the same /xycar_motor payload
shape used by main.py: std_msgs/Float32MultiArray [angle, speed].
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32MultiArray, Int32MultiArray


class RubberconeDriveOnlyNode(Node):
    def __init__(self) -> None:
        super().__init__("rubbercone_drive_only_node")

        self.declare_parameter("kp", 1.0)
        self.declare_parameter("kd", 0.0)
        self.declare_parameter("max_angle", 45.0)
        self.declare_parameter("cruise_speed", 11.0)
        self.declare_parameter("curve_speed", 8.0)
        self.declare_parameter("recovery_speed", 5.5)
        self.declare_parameter("min_drive_confidence", 20)
        self.declare_parameter("full_speed_confidence", 85)
        self.declare_parameter("max_cone_age_s", 0.25)
        self.declare_parameter("control_hz", 30.0)
        self.declare_parameter("stop_on_end_flag", True)
        self.declare_parameter("publish_session_active", True)

        self.kp = float(self.get_parameter("kp").value)
        self.kd = float(self.get_parameter("kd").value)
        self.max_angle = abs(float(self.get_parameter("max_angle").value))
        self.cruise_speed = max(0.0, float(self.get_parameter("cruise_speed").value))
        self.curve_speed = max(0.0, float(self.get_parameter("curve_speed").value))
        self.recovery_speed = max(0.0, float(self.get_parameter("recovery_speed").value))
        self.min_drive_confidence = int(self.get_parameter("min_drive_confidence").value)
        self.full_speed_confidence = int(self.get_parameter("full_speed_confidence").value)
        self.max_cone_age_s = max(0.0, float(self.get_parameter("max_cone_age_s").value))
        control_hz = max(1.0, float(self.get_parameter("control_hz").value))
        self.stop_on_end_flag = bool(self.get_parameter("stop_on_end_flag").value)
        self.publish_session_active = bool(
            self.get_parameter("publish_session_active").value
        )

        qos_fast = QoSProfile(depth=1)
        qos_fast.reliability = ReliabilityPolicy.BEST_EFFORT
        qos_fast.durability = DurabilityPolicy.VOLATILE

        session_qos = QoSProfile(depth=1)
        session_qos.reliability = ReliabilityPolicy.RELIABLE
        session_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.motor_pub = self.create_publisher(Float32MultiArray, "/xycar_motor", 10)
        self.session_pub = self.create_publisher(
            Bool, "/rubbercone_session_active", session_qos
        )
        self.create_subscription(
            Int32MultiArray, "/rubbercone_info", self._cone_callback, qos_fast
        )

        self.last_offset = 0
        self.last_end_flag = 0
        self.last_confidence = 0
        self.last_received_at: Optional[float] = None
        self.prev_offset = 0.0

        self.timer = self.create_timer(1.0 / control_hz, self._control_cycle)
        self.session_timer = self.create_timer(0.5, self._publish_session_active)
        self._publish_session_active()

        self.get_logger().info(
            "rubbercone_drive_only_node started: /rubbercone_info -> /xycar_motor"
        )

    def _cone_callback(self, msg: Int32MultiArray) -> None:
        if len(msg.data) < 3:
            self.get_logger().warn("ignored malformed /rubbercone_info")
            return
        offset, end_flag, confidence = msg.data[:3]
        if end_flag not in (0, 1):
            self.get_logger().warn("ignored rubbercone_info with invalid end_flag")
            return
        self.last_offset = int(offset)
        self.last_end_flag = int(end_flag)
        self.last_confidence = max(0, min(100, int(confidence)))
        self.last_received_at = self.get_clock().now().nanoseconds * 1e-9

    def _publish_session_active(self) -> None:
        if not self.publish_session_active:
            return
        msg = Bool()
        msg.data = True
        self.session_pub.publish(msg)

    def _control_cycle(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        stale = (
            self.last_received_at is None
            or now - self.last_received_at > self.max_cone_age_s
        )
        if stale or (self.stop_on_end_flag and self.last_end_flag == 1):
            self._publish_motor(0.0, 0.0)
            return

        confidence = self.last_confidence
        if confidence < self.min_drive_confidence:
            self._publish_motor(0.0, 0.0)
            return

        offset = float(self.last_offset)
        angle = self.kp * offset + self.kd * (offset - self.prev_offset)
        self.prev_offset = offset
        angle = max(-self.max_angle, min(self.max_angle, angle))

        speed = self._speed_for(angle, confidence)
        self._publish_motor(angle, speed)

    def _speed_for(self, angle: float, confidence: int) -> float:
        full = max(self.min_drive_confidence + 1, self.full_speed_confidence)
        confidence_ratio = (confidence - self.min_drive_confidence) / (
            full - self.min_drive_confidence
        )
        confidence_ratio = max(0.0, min(1.0, confidence_ratio))
        confidence_speed = (
            self.recovery_speed
            + (self.cruise_speed - self.recovery_speed) * confidence_ratio
        )

        turn_ratio = min(1.0, abs(angle) / max(self.max_angle, 1e-6))
        turn_speed = self.cruise_speed - (
            self.cruise_speed - self.curve_speed
        ) * turn_ratio
        return max(0.0, min(confidence_speed, turn_speed))

    def _publish_motor(self, angle: float, speed: float) -> None:
        if not math.isfinite(angle) or not math.isfinite(speed):
            angle, speed = 0.0, 0.0
        msg = Float32MultiArray()
        msg.data = [float(angle), float(speed)]
        self.motor_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RubberconeDriveOnlyNode()
    try:
        rclpy.spin(node)
    finally:
        node._publish_motor(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
