"""Produce /road_surface once for each valid /pidnet_class_map frame."""

from __future__ import annotations

import rclpy
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

from .surface_classifier import (
    SurfaceThresholds,
    class_map_from_image,
    classify_surface,
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
        # Dataset separation has not been demonstrated, so these four values
        # deliberately have no production defaults.
        self.declare_parameter("road_min_ratio", Parameter.Type.DOUBLE)
        self.declare_parameter("shortcut_min_ratio", Parameter.Type.DOUBLE)
        self.declare_parameter("road_min_component_px", Parameter.Type.INTEGER)
        self.declare_parameter(
            "shortcut_min_component_px",
            Parameter.Type.INTEGER,
        )
        self.declare_parameter("roi_top", 0.0)
        self.declare_parameter("roi_bottom", 1.0)
        self.declare_parameter("roi_left", 0.0)
        self.declare_parameter("roi_right", 1.0)

        try:
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
            )
        except ParameterUninitializedException as exc:
            raise RuntimeError(
                "ROAD_SURFACE_THRESHOLD_UNVERIFIED: set both ratio and "
                "connected-component thresholds explicitly"
            ) from exc

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
