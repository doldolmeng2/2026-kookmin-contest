"""Deadman-gated Xbox Joy to Xycar motor command adapter."""

from dataclasses import dataclass
import math
from typing import Sequence

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray


@dataclass(frozen=True)
class ManualDriveCommand:
    steering: float = 0.0
    speed: float = 0.0


STOP_COMMAND = ManualDriveCommand()


@dataclass(frozen=True)
class ManualDriveConfig:
    steering_axis: int = 0
    steering_scale: float = -100.0
    speed_axis: int = 4
    speed_scale: float = 50.0
    deadman_button: int = -1
    max_abs_steering: float = 45.0
    max_abs_speed: float = 7.5
    joy_timeout_s: float = 0.25
    publish_rate_hz: float = 20.0

    def __post_init__(self) -> None:
        for name, value in (
            ('steering_axis', self.steering_axis),
            ('speed_axis', self.speed_axis),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(f'{name} must be a non-negative integer')
        if (
            isinstance(self.deadman_button, bool)
            or not isinstance(self.deadman_button, int)
            or self.deadman_button < -1
        ):
            raise ValueError('deadman_button must be -1 or a non-negative integer')
        for name, value in (
            ('steering_scale', self.steering_scale),
            ('speed_scale', self.speed_scale),
        ):
            if not _finite_number(value):
                raise ValueError(f'{name} must be finite')
        for name, value in (
            ('max_abs_steering', self.max_abs_steering),
            ('max_abs_speed', self.max_abs_speed),
            ('joy_timeout_s', self.joy_timeout_s),
            ('publish_rate_hz', self.publish_rate_hz),
        ):
            if not _finite_number(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')


class ManualDriveGate:
    """Pure deadman, shape validation, clamp, and receipt-time watchdog."""

    def __init__(self, config: ManualDriveConfig):
        self.config = config
        self.last_received_at = None
        self.latest_command = STOP_COMMAND

    def record_joy(
        self,
        axes: Sequence[float],
        buttons: Sequence[int],
        received_at: float,
    ) -> ManualDriveCommand:
        if not _finite_number(received_at):
            self.latest_command = STOP_COMMAND
            return STOP_COMMAND

        self.last_received_at = float(received_at)
        config = self.config
        required_axis = max(config.steering_axis, config.speed_axis)
        if (
            config.deadman_button < 0
            or len(axes) <= required_axis
            or len(buttons) <= config.deadman_button
            or buttons[config.deadman_button] != 1
        ):
            self.latest_command = STOP_COMMAND
            return STOP_COMMAND

        steering_input = axes[config.steering_axis]
        speed_input = axes[config.speed_axis]
        if not (
            _finite_number(steering_input)
            and _finite_number(speed_input)
        ):
            self.latest_command = STOP_COMMAND
            return STOP_COMMAND

        self.latest_command = ManualDriveCommand(
            steering=_clamp(
                float(steering_input) * config.steering_scale,
                config.max_abs_steering,
            ),
            speed=_clamp(
                float(speed_input) * config.speed_scale,
                config.max_abs_speed,
            ),
        )
        return self.latest_command

    def command_at(self, now: float) -> ManualDriveCommand:
        if not (
            _finite_number(now)
            and self.last_received_at is not None
        ):
            return STOP_COMMAND
        age_s = float(now) - self.last_received_at
        if not 0.0 <= age_s <= self.config.joy_timeout_s:
            self.latest_command = STOP_COMMAND
        return self.latest_command


class JoyToMotor(Node):
    """Publish manual commands only while fresh Joy deadman input is held."""

    def __init__(self):
        super().__init__('manual_drive_node')

        self.config = ManualDriveConfig(
            steering_axis=self.declare_parameter('steering_axis', 0).value,
            steering_scale=self.declare_parameter(
                'steering_scale', -100.0
            ).value,
            speed_axis=self.declare_parameter('speed_axis', 4).value,
            speed_scale=self.declare_parameter('speed_scale', 50.0).value,
            deadman_button=self.declare_parameter('deadman_button', -1).value,
            max_abs_steering=self.declare_parameter(
                'max_abs_steering', 45.0
            ).value,
            max_abs_speed=self.declare_parameter(
                'max_abs_speed', 7.5
            ).value,
            joy_timeout_s=self.declare_parameter('joy_timeout_s', 0.25).value,
            publish_rate_hz=self.declare_parameter(
                'publish_rate_hz', 20.0
            ).value,
        )
        self.gate = ManualDriveGate(self.config)

        joy_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        command_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            joy_qos,
        )
        self.motor_pub = self.create_publisher(
            Float32MultiArray,
            'xycar_motor',
            command_qos,
        )
        self.timer = self.create_timer(
            1.0 / self.config.publish_rate_hz,
            self.publish_cycle,
        )

        self.publish_command(STOP_COMMAND)
        deadman = (
            str(self.config.deadman_button)
            if self.config.deadman_button >= 0
            else 'UNCONFIGURED (STOP only)'
        )
        self.get_logger().warning(
            'Manual drive started: hold deadman for non-zero output; '
            f'deadman_button={deadman}, timeout={self.config.joy_timeout_s:.2f}s'
        )

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def joy_callback(self, msg: Joy) -> None:
        command = self.gate.record_joy(
            msg.axes,
            msg.buttons,
            self._now_seconds(),
        )
        # Release and malformed input publish STOP in the callback cycle.
        self.publish_command(command)

    def publish_cycle(self) -> None:
        self.publish_command(self.gate.command_at(self._now_seconds()))

    def publish_command(self, command: ManualDriveCommand) -> None:
        msg = Float32MultiArray()
        msg.data = [float(command.steering), float(command.speed)]
        self.motor_pub.publish(msg)

    def publish_stop(self) -> None:
        self.gate.latest_command = STOP_COMMAND
        self.publish_command(STOP_COMMAND)


def _finite_number(value) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )


def _clamp(value: float, maximum: float) -> float:
    return max(-maximum, min(maximum, value))


def main(args=None):
    rclpy.init(args=args)
    node = JoyToMotor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.publish_stop()
        except Exception:
            # Context shutdown may already have invalidated the publisher.
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
