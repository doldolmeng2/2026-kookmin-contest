"""Produce /road_surface once for each valid /pidnet_class_map frame."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

from .surface_classifier import (
    SurfaceThresholds,
    class_map_from_image,
    classify_surface,
)


# Full-lap 2026-08-13 evidence, 640x360 PIDNet label map, lower drivable
# trapezoid. During the sustained shortcut (143--153 s), class-5 ratio was
# 0.68--0.85. The stable post-shortcut road (157--161 s) had class-4 ratio
# 0.94--0.95. These thresholds defer ambiguous transition frames to UNKNOWN.
FULL_LAP_THRESHOLDS = SurfaceThresholds(
    road_min_ratio=0.90,
    shortcut_min_ratio=0.30,
    road_min_component_px=60000,
    shortcut_min_component_px=20000,
    roi_top=0.60,
    roi_bottom=1.0,
    roi_left=0.0,
    roi_right=1.0,
    roi_top_width_ratio=0.50,
)


def sensor_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class RoadSurfaceNode(Node):
    def __init__(self) -> None:
        super().__init__("road_surface_node")
        self.declare_parameter("class_map_topic", "/pidnet_class_map")
        self.declare_parameter("output_topic", "/road_surface")
        defaults = FULL_LAP_THRESHOLDS
        self.declare_parameter("road_min_ratio", defaults.road_min_ratio)
        self.declare_parameter("shortcut_min_ratio", defaults.shortcut_min_ratio)
        self.declare_parameter("road_min_component_px", defaults.road_min_component_px)
        self.declare_parameter(
            "shortcut_min_component_px",
            defaults.shortcut_min_component_px,
        )
        self.declare_parameter("roi_top", defaults.roi_top)
        self.declare_parameter("roi_bottom", defaults.roi_bottom)
        self.declare_parameter("roi_left", defaults.roi_left)
        self.declare_parameter("roi_right", defaults.roi_right)
        self.declare_parameter(
            "roi_top_width_ratio", defaults.roi_top_width_ratio
        )

        self.thresholds = SurfaceThresholds(
            road_min_ratio=float(self.get_parameter("road_min_ratio").value),
            shortcut_min_ratio=float(
                self.get_parameter("shortcut_min_ratio").value
            ),
            road_min_component_px=int(
                self.get_parameter("road_min_component_px").value
            ),
            shortcut_min_component_px=int(
                self.get_parameter("shortcut_min_component_px").value
            ),
            roi_top=float(self.get_parameter("roi_top").value),
            roi_bottom=float(self.get_parameter("roi_bottom").value),
            roi_left=float(self.get_parameter("roi_left").value),
            roi_right=float(self.get_parameter("roi_right").value),
            roi_top_width_ratio=float(
                self.get_parameter("roi_top_width_ratio").value
            ),
        )

        qos = sensor_qos()
        self.publisher = self.create_publisher(
            Int32,
            str(self.get_parameter("output_topic").value),
            qos,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("class_map_topic").value),
            self.on_class_map,
            qos,
        )

    def on_class_map(self, message: Image) -> None:
        try:
            class_map = class_map_from_image(message)
            surface, _ = classify_surface(class_map, self.thresholds)
        except Exception as exc:
            self.get_logger().error(f"malformed PIDNet class map ignored: {exc}")
            return
        output = Int32()
        output.data = int(surface)
        self.publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RoadSurfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
