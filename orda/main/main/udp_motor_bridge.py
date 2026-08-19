"""Forward the ROS 2 motor contract to the local ROS 1 UDP receiver."""

import math
import socket
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray


MOTOR_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


def pack_motor_command(values: list[float]) -> bytes:
    """Validate and encode the fixed ROS1 UDP ``!ff`` motor contract."""
    if len(values) < 2:
        raise ValueError("motor command requires angle and speed")
    angle, speed = values[:2]
    if not all(
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
        for value in (angle, speed)
    ):
        raise ValueError("motor command fields must be finite numbers")
    return struct.pack("!ff", float(angle), float(speed))


class UdpMotorSender:
    """Small socket seam kept independent from ROS for contract tests."""

    def __init__(self, host: str, port: int) -> None:
        self.address = (host, int(port))
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, values: list[float]) -> None:
        self.socket.sendto(pack_motor_command(values), self.address)

    def close_with_zero(self, count: int = 3) -> None:
        for _ in range(count):
            self.send([0.0, 0.0])
        self.socket.close()


class UdpMotorBridge(Node):
    def __init__(self) -> None:
        super().__init__("udp_motor_bridge")
        host = self.declare_parameter("udp_host", "127.0.0.1").value
        port = self.declare_parameter("udp_port", 39001).value
        self.sender = UdpMotorSender(str(host), int(port))
        self.subscription = self.create_subscription(
            Float32MultiArray, "xycar_motor", self.motor_callback, MOTOR_QOS
        )
        self.get_logger().info(
            f"UDP motor bridge ready: /xycar_motor -> {host}:{port}"
        )

    def motor_callback(self, message: Float32MultiArray) -> None:
        try:
            self.sender.send(list(message.data))
        except ValueError as error:
            self.get_logger().warning(f"rejected motor command: {error}")
        except OSError as error:
            self.get_logger().error(f"UDP motor send failed: {error}")

    def destroy_node(self) -> bool:
        self.sender.close_with_zero()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UdpMotorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
