"""One-shot ROS graph preflight for production and isolated launches."""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node


class PreflightNode(Node):
    def __init__(self) -> None:
        super().__init__("kmu_preflight")
        self.declare_parameter("required_topics", ["/lane_offset", "/scan"])
        self.declare_parameter("motor_output_topic", "/xycar_motor")
        self.declare_parameter("require_motor_subscriber", False)
        self.declare_parameter("startup_timeout_s", 2.0)

    def inspect(self) -> tuple[bool, str]:
        required = [
            str(topic) for topic in self.get_parameter("required_topics").value
        ]
        missing = [
            topic
            for topic in required
            if self.count_publishers(topic) == 0
        ]
        motor_topic = str(self.get_parameter("motor_output_topic").value)
        motor_publishers = self.count_publishers(motor_topic)
        motor_subscribers = self.count_subscribers(motor_topic)
        subscriber_required = bool(
            self.get_parameter("require_motor_subscriber").value
        )
        passed = not missing and (
            not subscriber_required or motor_subscribers > 0
        )
        status = "PASS" if passed else "FAIL"
        line = (
            f"PREFLIGHT {status} missing={missing or 'none'} "
            f"motor_topic={motor_topic} publishers={motor_publishers} "
            f"subscribers={motor_subscribers}"
        )
        return passed, line


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PreflightNode()
    try:
        timeout_s = float(node.get_parameter("startup_timeout_s").value)
        deadline = time.monotonic() + max(0.0, timeout_s)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        passed, line = node.inspect()
        (node.get_logger().info if passed else node.get_logger().error)(line)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
