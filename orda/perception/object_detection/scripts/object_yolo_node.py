#!/usr/bin/env python3
"""ONNX Runtime YOLO frontend for the C++ LiDAR/lane fusion node."""

from __future__ import annotations

import os

import onnxruntime as ort
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

from object_detection.image_conversion import imgmsg_to_bgr
from object_detection.yolo_runtime import (
    closest_detection,
    decode_detections,
    letterbox_blob,
)


UNKNOWN = -1
FIXED = 0
MOVING = 1


class ObjectYoloNode(Node):
    def __init__(self) -> None:
        super().__init__("object_yolo_node")
        self.declare_parameter("model_path", "")
        self.declare_parameter("camera_topic", "/resized_image")
        self.declare_parameter("output_topic", "/object_yolo")
        self.declare_parameter("confidence_threshold", 0.50)
        self.declare_parameter("nms_threshold", 0.40)
        self.declare_parameter("input_size", 640)
        # Model class IDs are deliberately parameters.  The checked-in model
        # is one-class/fixed.  A teammate's multi-class model can be dropped in
        # without code changes once its exact ID mapping is known.
        self.declare_parameter("fixed_class_ids", [0])
        self.declare_parameter("moving_class_ids", [])

        model_path = str(self.get_parameter("model_path").value)
        if not model_path:
            model_path = os.path.join(
                get_package_share_directory("object_detection"),
                "model",
                "best.onnx",
            )
        if not os.path.isfile(model_path):
            raise FileNotFoundError(model_path)

        self.confidence = float(
            self.get_parameter("confidence_threshold").value
        )
        self.nms = float(self.get_parameter("nms_threshold").value)
        self.input_size = int(self.get_parameter("input_size").value)
        self.fixed_ids = {
            int(value) for value in self.get_parameter("fixed_class_ids").value
        }
        self.moving_ids = {
            int(value) for value in self.get_parameter("moving_class_ids").value
        }
        overlap = self.fixed_ids & self.moving_ids
        if overlap:
            raise ValueError(f"fixed/moving class IDs overlap: {sorted(overlap)}")
        self.allowed_ids = self.fixed_ids | self.moving_ids
        if not self.allowed_ids:
            raise ValueError("at least one obstacle class ID is required")

        self.session = ort.InferenceSession(
            model_path,
            providers=["CPUExecutionProvider"],
        )
        self.input_name = self.session.get_inputs()[0].name
        output_shape = self.session.get_outputs()[0].shape
        self.get_logger().info(
            f"ONNX Runtime ready: {model_path}, output={output_shape}, "
            f"fixed={sorted(self.fixed_ids)}, moving={sorted(self.moving_ids)}"
        )

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        output_topic = str(self.get_parameter("output_topic").value)
        camera_topic = str(self.get_parameter("camera_topic").value)
        self.publisher = self.create_publisher(
            Float32MultiArray, output_topic, qos
        )
        self.create_subscription(Image, camera_topic, self.on_image, qos)

    def on_image(self, message: Image) -> None:
        try:
            image = imgmsg_to_bgr(message)
            prepared = letterbox_blob(image, self.input_size)
            output = self.session.run(
                None,
                {self.input_name: prepared.blob},
            )[0]
            detections = decode_detections(
                output,
                image_shape=image.shape[:2],
                scale=prepared.scale,
                pad_x=prepared.pad_x,
                pad_y=prepared.pad_y,
                confidence_threshold=self.confidence,
                nms_threshold=self.nms,
                allowed_class_ids=self.allowed_ids,
            )
            detection = closest_detection(detections)
        except Exception as exc:
            self.get_logger().error(f"object YOLO inference failed: {exc}")
            return

        output_message = Float32MultiArray()
        if detection is None:
            output_message.data = [0.0, float(UNKNOWN)] + [0.0] * 8
        else:
            semantic_type = (
                FIXED if detection.class_id in self.fixed_ids else MOVING
            )
            center_x, center_y = detection.center
            output_message.data = [
                1.0,
                float(semantic_type),
                float(detection.confidence),
                float(detection.area),
                float(center_x),
                float(center_y),
                float(detection.x),
                float(detection.y),
                float(detection.width),
                float(detection.height),
            ]
        self.publisher.publish(output_message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObjectYoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
