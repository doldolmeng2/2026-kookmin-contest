#!/usr/bin/env python3
"""Single-model ONNX Runtime frontend for C++ LiDAR/lane fusion.

One detector inference publishes all production interfaces:
  /object_yolo    closest fixed and moving vehicle boxes (10+10 slots)
  /traffic_detection  direct detector-derived traffic state
  /traffic_boxes  state-specific traffic detections as repeated
                  [signal_index, detector_confidence, x, y, w, h]

Signal indices are mapped directly from detector classes. Generic class 6 is
ambiguous and is never published as traffic-state evidence.
"""

from __future__ import annotations

import os
import time

import onnxruntime as ort
import rclpy
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Int32

from object_detection.image_conversion import imgmsg_to_bgr
from object_detection.yolo_runtime import (
    LatestFrameRateLimiter,
    closest_detection_for_classes,
    decode_detections,
    detection_slot,
    letterbox_blob,
    normalize_class_ids,
)
from object_detection.traffic_classifier import (
    detector_traffic_box_values,
    detector_traffic_to_state,
    parse_class_names,
    validate_detector_classes,
)


FIXED = 0
MOVING = 1


def diagnostic_value(key: str, value: object) -> KeyValue:
    item = KeyValue()
    item.key = key
    item.value = str(value)
    return item


class ObjectYoloNode(Node):
    def __init__(self) -> None:
        super().__init__("object_yolo_node")
        self.declare_parameter("model_path", "")
        self.declare_parameter("detector_model_path", "")
        self.declare_parameter("camera_topic", "/resized_image")
        self.declare_parameter("output_topic", "/object_yolo")
        self.declare_parameter("traffic_output_topic", "/traffic_detection")
        self.declare_parameter("traffic_boxes_topic", "/traffic_boxes")
        self.declare_parameter("confidence_threshold", 0.50)
        # 장애물(차량) 판정만 더 엄격하게 본다. 라바콘은 검출기에 없는 물체라
        # red_car 로 새어 나오는데, 실측(2026-08-23 bag 3종)에서 그 오검출은
        # 최대 0.85 에 그치고 FSM 진입 크기(1900px^2)를 넘는 건은 0.72 였다.
        # 반면 진짜 고정 방해차량은 0.89~0.93 으로 잡힌다. 신호등은 원거리에서
        # 신뢰도가 낮게 나오므로 이 값을 함께 올리면 안 된다.
        self.declare_parameter("obstacle_confidence_threshold", 0.83)
        self.declare_parameter("nms_threshold", 0.40)
        self.declare_parameter("input_size", 640)
        self.declare_parameter("max_inference_hz", 0.0)
        self.declare_parameter("publish_inference_diagnostics", False)
        self.declare_parameter(
            "inference_diagnostics_topic", "/object_yolo/inference_diagnostics"
        )
        # Canonical train-10 contract: 4=red_car(fixed), 0=green_car(moving).
        # Startup metadata validation rejects any incompatible mapping.
        self.declare_parameter("fixed_class_ids", [4])
        self.declare_parameter("moving_class_ids", [0])
        # State-specific traffic classes plus generic class 6. Class 6 remains
        # a decode candidate for diagnostics but is excluded from state output.
        self.declare_parameter("traffic_class_ids", [1, 2, 3, 5, 6])

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
        if not os.path.isfile(detector_model_path):
            raise FileNotFoundError(
                f"train-10 detector model missing: {detector_model_path}"
            )

        self.confidence = float(
            self.get_parameter("confidence_threshold").value
        )
        self.obstacle_confidence = float(
            self.get_parameter("obstacle_confidence_threshold").value
        )
        self.nms = float(self.get_parameter("nms_threshold").value)
        self.input_size = int(self.get_parameter("input_size").value)
        self.fixed_ids = normalize_class_ids(
            self.get_parameter("fixed_class_ids").value
        )
        self.moving_ids = normalize_class_ids(
            self.get_parameter("moving_class_ids").value
        )
        self.traffic_ids = normalize_class_ids(
            self.get_parameter("traffic_class_ids").value
        )
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
        validate_detector_classes(
            detector_names,
            self.fixed_ids,
            self.moving_ids,
            self.traffic_ids,
        )
        self.input_name = self.session.get_inputs()[0].name
        output_shape = self.session.get_outputs()[0].shape

        self.max_inference_hz = float(
            self.get_parameter("max_inference_hz").value
        )
        self.rate_limiter = LatestFrameRateLimiter(self.max_inference_hz)
        self.publish_inference_diagnostics = bool(
            self.get_parameter("publish_inference_diagnostics").value
        )
        self.get_logger().info(
            f"ONNX Runtime ready: detector={detector_model_path}, "
            f"output={output_shape}, "
            f"fixed={sorted(self.fixed_ids)}, moving={sorted(self.moving_ids)}, "
            f"traffic={sorted(self.traffic_ids)}, "
            f"conf(obstacle)={self.obstacle_confidence:.2f}, "
            f"conf(traffic)={self.confidence:.2f}"
        )

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        output_topic = str(self.get_parameter("output_topic").value)
        traffic_output_topic = str(self.get_parameter("traffic_output_topic").value)
        traffic_boxes_topic = str(self.get_parameter("traffic_boxes_topic").value)
        camera_topic = str(self.get_parameter("camera_topic").value)
        self.publisher = self.create_publisher(
            Float32MultiArray, output_topic, qos
        )
        self.traffic_publisher = self.create_publisher(
            Int32, traffic_output_topic, qos
        )
        self.traffic_boxes_publisher = self.create_publisher(
            Float32MultiArray, traffic_boxes_topic, qos
        )
        self.inference_diagnostics_publisher = None
        if self.publish_inference_diagnostics:
            diagnostics_topic = str(
                self.get_parameter("inference_diagnostics_topic").value
            )
            self.inference_diagnostics_publisher = self.create_publisher(
                DiagnosticArray, diagnostics_topic, qos
            )
        self.create_subscription(Image, camera_topic, self.on_image, qos)
        self.inference_timer = None
        if self.rate_limiter.enabled:
            self.inference_timer = self.create_timer(
                self.rate_limiter.period_s, self._on_inference_timer
            )

    def on_image(self, message: Image) -> None:
        if not self.rate_limiter.enabled:
            received_ns = (
                time.monotonic_ns() if self.publish_inference_diagnostics else 0
            )
            self._infer_and_publish(message, received_ns)
            return
        pending = (message, time.monotonic_ns())
        selected = self.rate_limiter.offer(pending, time.monotonic())
        if selected is not None:
            self._infer_and_publish(*selected)

    def _on_inference_timer(self) -> None:
        selected = self.rate_limiter.pop_due(time.monotonic())
        if selected is not None:
            self._infer_and_publish(*selected)

    def _infer_and_publish(self, message: Image, received_ns: int) -> None:
        started_ns = time.monotonic_ns()
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
                nms_threshold=self.nms,
            )
            detections = decode_detections(
                allowed_class_ids=self.allowed_ids,
                confidence_threshold=self.obstacle_confidence,
                **decode_kwargs,
            )
            fixed_detection = closest_detection_for_classes(
                detections, self.fixed_ids
            )
            moving_detection = closest_detection_for_classes(
                detections, self.moving_ids
            )
            # 신호등은 위치가 아니라 "보이는가"만 필요하다. 원거리 신호등이
            # 차량용 min_size_px(기본 12px) 필터에 걸려 누락되지 않도록
            # 최소 크기 제한 없이 별도로 디코드한다 (traffic_node 원래 동작과 동일).
            traffic_detections = decode_detections(
                allowed_class_ids=self.traffic_ids,
                confidence_threshold=self.confidence,
                min_size_px=1,
                **decode_kwargs,
            )
        except Exception as exc:
            self.get_logger().error(f"object YOLO inference failed: {exc}")
            return

        output_message = Float32MultiArray()
        output_message.data = (
            detection_slot(fixed_detection, FIXED)
            + detection_slot(moving_detection, MOVING)
        )
        self.publisher.publish(output_message)

        # Preserve the C++ priority contract using the same detector results:
        # LEFT(3) > STRAIGHT(2) > STOP(1) > UNKNOWN(0). Generic class 6 maps
        # to no state and therefore cannot become traffic evidence.
        traffic_states = {
            state
            for detection in traffic_detections
            if (state := detector_traffic_to_state(detection.class_id)) is not None
        }
        traffic_signal = (
            3 if 3 in traffic_states else
            2 if 2 in traffic_states else
            1 if 1 in traffic_states else
            0
        )
        signal_message = Int32()
        signal_message.data = traffic_signal
        self.traffic_publisher.publish(signal_message)

        # Publish only state-specific detector classes. Confidence and geometry
        # are the original detector values. Generic class 6 is intentionally
        # omitted because it cannot identify a traffic state.
        traffic_message = Float32MultiArray()
        flat: list[float] = []
        for det in traffic_detections:
            values = detector_traffic_box_values(
                det.class_id,
                det.confidence,
                det.x,
                det.y,
                det.width,
                det.height,
            )
            if values is None:
                continue
            flat.extend(values)
        traffic_message.data = flat
        self.traffic_boxes_publisher.publish(traffic_message)

        if self.inference_diagnostics_publisher is not None:
            completed_ns = time.monotonic_ns()
            diagnostics = DiagnosticArray()
            diagnostics.header = message.header
            status = DiagnosticStatus()
            status.level = DiagnosticStatus.OK
            status.name = "object_yolo/inference"
            status.message = "OK"
            status.hardware_id = "onnxruntime"
            status.values = [
                diagnostic_value("max_inference_hz", self.max_inference_hz),
                diagnostic_value(
                    "receive_to_start_us", (started_ns - received_ns) // 1000
                ),
                diagnostic_value(
                    "inference_work_us", (completed_ns - started_ns) // 1000
                ),
                diagnostic_value(
                    "callback_total_us", (completed_ns - received_ns) // 1000
                ),
                diagnostic_value(
                    "fixed_detected", int(fixed_detection is not None)
                ),
                diagnostic_value(
                    "moving_detected", int(moving_detection is not None)
                ),
                diagnostic_value("traffic_signal", traffic_signal),
                diagnostic_value("traffic_box_count", len(flat) // 6),
            ]
            diagnostics.status = [status]
            self.inference_diagnostics_publisher.publish(diagnostics)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObjectYoloNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
