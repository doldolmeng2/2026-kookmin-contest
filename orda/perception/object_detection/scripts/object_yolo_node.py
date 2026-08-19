#!/usr/bin/env python3
"""ONNX Runtime YOLO frontend for the C++ LiDAR/lane fusion node."""

from __future__ import annotations

import os

import onnxruntime as ort
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Int32

from object_detection.image_conversion import imgmsg_to_bgr
from object_detection.yolo_runtime import (
    closest_detection_for_classes,
    decode_detections,
    detection_slot,
    letterbox_blob,
    normalize_class_ids,
)
from object_detection.traffic_classifier import (
    classify_current_frame_candidate,
    parse_class_names,
    validate_classifier_classes,
    validate_detector_classes,
)


FIXED = 0
MOVING = 1


class ObjectYoloNode(Node):
    def __init__(self) -> None:
        super().__init__("object_yolo_node")
        self.declare_parameter("model_path", "")
        self.declare_parameter("detector_model_path", "")
        self.declare_parameter("traffic_classifier_model_path", "")
        self.declare_parameter("camera_topic", "/resized_image")
        self.declare_parameter("output_topic", "/object_yolo")
        self.declare_parameter("traffic_output_topic", "/traffic_detection")
        self.declare_parameter("confidence_threshold", 0.50)
        self.declare_parameter("classifier_confidence_threshold", 0.60)
        self.declare_parameter("nms_threshold", 0.40)
        self.declare_parameter("input_size", 640)
        # YAML supplies integer arrays. The moving mapping has no guessed
        # default: declaring its type explicitly lets a future non-empty YAML
        # array initialize it while standalone runs safely treat it as empty.
        self.declare_parameter("fixed_class_ids", [0])
        self.declare_parameter(
            "moving_class_ids",
            Parameter.Type.INTEGER_ARRAY,
        )

        legacy_model_path = str(self.get_parameter("model_path").value)
        detector_model_path = str(
            self.get_parameter("detector_model_path").value
        )
        if legacy_model_path and detector_model_path:
            raise ValueError(
                "set detector_model_path or legacy model_path, not both"
            )
        detector_model_path = detector_model_path or legacy_model_path
        if not detector_model_path:
            detector_model_path = os.path.join(
                get_package_share_directory("object_detection"),
                "model",
                "train10_detector_best.onnx",
            )
        classifier_model_path = str(
            self.get_parameter("traffic_classifier_model_path").value
        )
        if not classifier_model_path:
            classifier_model_path = os.path.join(
                get_package_share_directory("object_detection"),
                "model",
                "light1_classifier_best.onnx",
            )
        for label, path in (
            ("train-10 detector", detector_model_path),
            ("light1 traffic classifier", classifier_model_path),
        ):
            if not os.path.isfile(path):
                raise FileNotFoundError(f"{label} model missing: {path}")

        self.confidence = float(
            self.get_parameter("confidence_threshold").value
        )
        self.nms = float(self.get_parameter("nms_threshold").value)
        self.classifier_confidence = float(
            self.get_parameter("classifier_confidence_threshold").value
        )
        self.input_size = int(self.get_parameter("input_size").value)
        self.fixed_ids = normalize_class_ids(
            self.get_parameter("fixed_class_ids").value
        )
        try:
            moving_class_ids = self.get_parameter("moving_class_ids").value
        except ParameterUninitializedException:
            moving_class_ids = []
        self.moving_ids = normalize_class_ids(moving_class_ids)
        self.traffic_ids = {2, 3, 4, 5}
        overlap = self.fixed_ids & self.moving_ids
        if overlap:
            raise ValueError(f"fixed/moving class IDs overlap: {sorted(overlap)}")
        self.allowed_ids = self.fixed_ids | self.moving_ids | self.traffic_ids
        if not self.allowed_ids:
            raise ValueError("at least one obstacle class ID is required")

        self.session = ort.InferenceSession(
            detector_model_path,
            providers=["CPUExecutionProvider"],
        )
        detector_names = parse_class_names(
            self.session.get_modelmeta().custom_metadata_map.get("names")
        )
        validate_detector_classes(detector_names)
        self.input_name = self.session.get_inputs()[0].name
        output_shape = self.session.get_outputs()[0].shape

        self.classifier_session = ort.InferenceSession(
            classifier_model_path,
            providers=["CPUExecutionProvider"],
        )
        classifier_names = parse_class_names(
            self.classifier_session.get_modelmeta().custom_metadata_map.get(
                "names"
            )
        )
        validate_classifier_classes(classifier_names)
        classifier_input = self.classifier_session.get_inputs()[0]
        classifier_shape = classifier_input.shape
        if (
            len(classifier_shape) != 4
            or classifier_shape[0] != 1
            or classifier_shape[1] != 3
            or not isinstance(classifier_shape[2], int)
            or not isinstance(classifier_shape[3], int)
        ):
            raise ValueError(
                f"unsupported traffic classifier input shape: {classifier_shape}"
            )
        self.classifier_input_name = classifier_input.name
        self.classifier_height = classifier_shape[2]
        self.classifier_width = classifier_shape[3]
        self.get_logger().info(
            f"ONNX Runtime ready: detector={detector_model_path}, "
            f"output={output_shape}, classifier={classifier_model_path}, "
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
        self.traffic_publisher = self.create_publisher(
            Int32,
            str(self.get_parameter("traffic_output_topic").value),
            qos,
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
            fixed_detection = closest_detection_for_classes(
                detections,
                self.fixed_ids,
            )
            moving_detection = closest_detection_for_classes(
                detections,
                self.moving_ids,
            )
            traffic_candidate = closest_detection_for_classes(
                detections,
                self.traffic_ids,
            )
        except Exception as exc:
            self.get_logger().error(f"object YOLO inference failed: {exc}")
            return

        traffic_signal = 0
        if traffic_candidate is not None:
            try:
                traffic_signal = classify_current_frame_candidate(
                    image,
                    traffic_candidate,
                    self.classifier_session,
                    self.classifier_input_name,
                    self.classifier_height,
                    self.classifier_width,
                    self.classifier_confidence,
                )
            except Exception as exc:
                self.get_logger().error(
                    f"same-frame traffic classification failed: {exc}"
                )
                traffic_signal = 0

        output_message = Float32MultiArray()
        # Two independent slots preserve a fixed and a moving detection from
        # the same frame. The former single "closest overall" result discarded
        # one category whenever both were visible.
        output_message.data = (
            detection_slot(fixed_detection, FIXED)
            + detection_slot(moving_detection, MOVING)
        )
        self.publisher.publish(output_message)
        traffic_message = Int32()
        traffic_message.data = traffic_signal
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
