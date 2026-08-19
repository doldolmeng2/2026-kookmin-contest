#!/usr/bin/env python3
"""Run the production MainNode against synthetic inputs with motor isolation."""

from __future__ import annotations

import json
import os
import sys
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, Int16

from main.main import MainNode
from main.race_fsm import Mode


DOMAIN_ID = 86
REAL_MOTOR_TOPIC = "/xycar_motor"
TEST_MOTOR_TOPIC = "/kmu_main_offline/xycar_motor"


def sensor_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def command_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class SyntheticSemanticHarness(Node):
    """Publish semantic inputs and observe only the isolated motor boundary."""

    def __init__(self, *, context: Context) -> None:
        super().__init__("kmu_main_offline_harness", context=context)
        self.lane_pub = self.create_publisher(Int16, "/lane_offset", sensor_qos())
        self.scan_pub = self.create_publisher(LaserScan, "/scan", sensor_qos())
        self.motor_messages: list[tuple[float, float]] = []
        self.mode_messages: list[int] = []
        self.create_subscription(
            Float32MultiArray,
            TEST_MOTOR_TOPIC,
            self._on_motor,
            command_qos(),
        )
        self.create_subscription(
            Int16,
            "/mode_info",
            self._on_mode,
            sensor_qos(),
        )

    def publish_fresh_inputs(self) -> None:
        lane = Int16()
        lane.data = 0
        self.lane_pub.publish(lane)

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.angle_min = -1.0
        scan.angle_max = 1.0
        scan.angle_increment = 1.0
        scan.range_min = 0.1
        scan.range_max = 10.0
        scan.ranges = [5.0, 5.0, 5.0]
        self.scan_pub.publish(scan)

    def publish_scan_only(self) -> None:
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.angle_min = -1.0
        scan.angle_max = 1.0
        scan.angle_increment = 1.0
        scan.range_min = 0.1
        scan.range_max = 10.0
        scan.ranges = [5.0, 5.0, 5.0]
        self.scan_pub.publish(scan)

    def _on_motor(self, message: Float32MultiArray) -> None:
        if len(message.data) != 2:
            raise RuntimeError(f"malformed isolated motor command: {message.data}")
        self.motor_messages.append((float(message.data[0]), float(message.data[1])))

    def _on_mode(self, message: Int16) -> None:
        self.mode_messages.append(int(message.data))


def _spin_for(
    executor: SingleThreadedExecutor,
    seconds: float,
    tick,
    sample,
) -> None:
    deadline = time.monotonic() + seconds
    next_tick = time.monotonic()
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_tick:
            tick()
            next_tick = now + 0.05
        executor.spin_once(timeout_sec=0.005)
        sample()


def run() -> dict[str, object]:
    if os.environ.get("ROS_DOMAIN_ID") != str(DOMAIN_ID):
        raise RuntimeError(f"ROS_DOMAIN_ID must be {DOMAIN_ID}")

    context = Context()
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "mode:=1",
            "-r",
            f"xycar_motor:={TEST_MOTOR_TOPIC}",
        ],
        context=context,
        domain_id=DOMAIN_ID,
    )
    main_node = None
    harness = None
    executor = SingleThreadedExecutor(context=context)
    state_trace: list[str] = []
    max_real_publishers = 0
    max_test_publishers = 0

    try:
        main_node = MainNode(context=context)
        harness = SyntheticSemanticHarness(context=context)
        executor.add_node(main_node)
        executor.add_node(harness)

        def sample_graph() -> None:
            nonlocal max_real_publishers, max_test_publishers
            max_real_publishers = max(
                max_real_publishers,
                harness.count_publishers(REAL_MOTOR_TOPIC),
            )
            max_test_publishers = max(
                max_test_publishers,
                harness.count_publishers(TEST_MOTOR_TOPIC),
            )
            state = main_node.runtime.fsm.state.value
            if not state_trace or state_trace[-1] != state:
                state_trace.append(state)

        _spin_for(
            executor,
            0.6,
            harness.publish_fresh_inputs,
            sample_graph,
        )
        lane_drive_seen = (
            main_node.runtime.fsm.state is Mode.LANE_DRIVE
            and any(speed > 0.0 for _, speed in harness.motor_messages)
        )

        # 0.8 s reaches lane staleness; the remaining second also proves the
        # command handoff has reached the exact zero message, not only STOP.
        _spin_for(executor, 1.8, harness.publish_scan_only, sample_graph)
        stale_stop_seen = (
            main_node.runtime.fsm.state is Mode.STOP
            and bool(harness.motor_messages)
            and harness.motor_messages[-1] == (0.0, 0.0)
        )

        recovery_start = len(harness.motor_messages)
        _spin_for(
            executor,
            0.9,
            harness.publish_fresh_inputs,
            sample_graph,
        )
        recovered = (
            main_node.runtime.fsm.state is Mode.LANE_DRIVE
            and any(
                speed > 0.0
                for _, speed in harness.motor_messages[recovery_start:]
            )
        )

        _spin_for(
            executor,
            0.1,
            harness.publish_fresh_inputs,
            sample_graph,
        )
        no_duplicate_transition = state_trace == [
            Mode.LANE_DRIVE.value,
            Mode.STOP.value,
            Mode.LANE_DRIVE.value,
        ]

        result = {
            "lane_drive_nonzero": lane_drive_seen,
            "lane_stale_stop_zero": stale_stop_seen,
            "recovered_after_0_5s": recovered,
            "no_duplicate_transition": no_duplicate_transition,
            "state_trace": state_trace,
            "external_mode_codes": sorted(set(harness.mode_messages)),
            "real_motor_max_publishers": max_real_publishers,
            "test_motor_max_publishers": max_test_publishers,
            "test_motor_samples": len(harness.motor_messages),
        }
        result["passed"] = all(
            (
                lane_drive_seen,
                stale_stop_seen,
                recovered,
                no_duplicate_transition,
                max_real_publishers == 0,
                max_test_publishers == 1,
            )
        )
        return result
    finally:
        for node in (harness, main_node):
            if node is not None:
                executor.remove_node(node)
                node.destroy_node()
        executor.shutdown()
        if context.ok():
            context.shutdown()


def main() -> int:
    try:
        result = run()
    except Exception as exc:
        print(json.dumps({"passed": False, "error": str(exc)}, sort_keys=True))
        return 1
    print(json.dumps(result, sort_keys=True))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
