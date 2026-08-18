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
        # Model class IDs are deliberately parameters rather than hardcoded,
        # so a retrained model with a different ID mapping can be dropped in
        # without code changes. Current model (train-5, 6-class merged
        # car+traffic model): 0=red_car(fixed) 1=green_car(moving).
        self.declare_parameter("fixed_class_ids", [0])
        self.declare_parameter("moving_class_ids", [1])
        # 신호등 색상 클래스. object_node 가 더 이상 traffic_node 를
        # 거치지 않고 이 노드가 보낸 박스로 직접 상태를 계산한다.
        # train-5 부터 "4-traffic"(몸체) 클래스가 빠져서 6클래스(0~5)다.
        self.declare_parameter("traffic_class_ids", [2, 3, 4, 5])
        self.declare_parameter("traffic_output_topic", "/traffic_boxes")

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
        self.traffic_ids = {
            int(value) for value in self.get_parameter("traffic_class_ids").value
        }

        self.session = ort.InferenceSession(
            model_path,
            providers=["CPUExecutionProvider"],
        )
        self.input_name = self.session.get_inputs()[0].name
        output_shape = self.session.get_outputs()[0].shape
        self.get_logger().info(
            f"ONNX Runtime ready: {model_path}, output={output_shape}, "
            f"fixed={sorted(self.fixed_ids)}, moving={sorted(self.moving_ids)}, "
            f"traffic={sorted(self.traffic_ids)}"
        )

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        output_topic = str(self.get_parameter("output_topic").value)
        traffic_output_topic = str(self.get_parameter("traffic_output_topic").value)
        camera_topic = str(self.get_parameter("camera_topic").value)
        self.publisher = self.create_publisher(
            Float32MultiArray, output_topic, qos
        )
        self.traffic_publisher = self.create_publisher(
            Float32MultiArray, traffic_output_topic, qos
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
            decode_kwargs = dict(
                output=output,
                image_shape=image.shape[:2],
                scale=prepared.scale,
                pad_x=prepared.pad_x,
                pad_y=prepared.pad_y,
                confidence_threshold=self.confidence,
                nms_threshold=self.nms,
            )
            detections = decode_detections(
                allowed_class_ids=self.allowed_ids, **decode_kwargs
            )
            detection = closest_detection(detections)
            # 신호등은 위치가 아니라 "보이는가"만 필요하다. 원거리 신호등이
            # 차량용 min_size_px(기본 12px) 필터에 걸려 누락되지 않도록
            # 최소 크기 제한 없이 별도로 디코드한다 (traffic_node 원래 동작과 동일).
            traffic_detections = decode_detections(
                allowed_class_ids=self.traffic_ids, min_size_px=1, **decode_kwargs
            )
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

        traffic_message = Float32MultiArray()
        flat: list[float] = []
        for det in traffic_detections:
            flat.extend([
                float(det.class_id), float(det.confidence),
                float(det.x), float(det.y), float(det.width), float(det.height),
            ])
        traffic_message.data = flat
        self.traffic_publisher.publish(traffic_message)


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
