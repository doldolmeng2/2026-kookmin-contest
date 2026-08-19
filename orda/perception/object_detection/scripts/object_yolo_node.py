#!/usr/bin/env python3
"""ONNX Runtime YOLO frontend for the C++ LiDAR/lane fusion node.

한 번의 추론으로 두 가지를 낸다:
  /object_yolo    차량(고정장애물·방해차량) 박스 1개 -> object_node(C++)
  /traffic_boxes  신호등처럼 생긴 위치 후보 전부       -> object_node(C++)

신호등은 2단계(ROI 크롭 + 분류) 방식이다. 검출기는 '신호등이 여기 있다'는
위치만 내고, 무슨 신호인지는 그 박스를 잘라 light_cls.onnx 에 넣어 정한다.
검출기의 신호등 클래스 판정은 신뢰할 수 없다: 홀드아웃에서 박스 찾기는
100% 였지만 클래스는 86.9% 였고, 같은 신호등을 축소만 해도
left_green_light -> green_light 로 뒤집혔다.

크롭·분류를 이 노드에서 하는 이유 (2026-08-19 C++ 로 옮겼다가 되돌림):
object_node(C++) 는 /traffic_boxes 로 박스를 받은 뒤 자기가 들고 있는 최신
프레임(last_raw_img_)에서 잘랐는데, 그건 이 박스를 만든 프레임이 아니다.
검출기 forward 가 CPU 에서 70ms 대(30fps 기준 2~3프레임)라 신호등에 근접할수록
크롭이 램프 줄을 통째로 비껴갔고, 램프가 안 잡힌 크롭을 분류기가 orange 로
0.99 이상 확신해서 초록불 바로 아래에서 정지(1) 판정이 나왔다. bag
rosbag2_2026_08_13-13_33_23 재현: 같은 프레임이면 orange 최대 확신도 0.61
(0.90 게이트에 걸려 보류)인데, 3프레임 밀리면 0.989, 8프레임이면 0.999 다.
검출에 쓴 image 를 그대로 잘라 쓰면 이 어긋남 자체가 생길 수 없다.

/traffic_boxes 형식은 6개씩 [class_id, confidence, x, y, w, h] 그대로지만,
class_id/confidence 자리에는 이제 **분류기** 인덱스(0=green 1=left_green
2=orange 3=red)와 그 확신도가 들어간다. 분류기를 못 띄웠으면 -1 을 넣어
보내고, 그때는 object_node 가 예전처럼 직접 잘라 분류한다(폴백).
"""

from __future__ import annotations

import os

import numpy as np
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
    crop_with_margin,
    decode_detections,
    letterbox_blob,
    light_blob,
    light_geometry_ok,
)


UNKNOWN = -1
FIXED = 0
MOVING = 1
# /traffic_boxes 의 class_id 자리에 넣는 "분류 못 했음" 표식. object_node 는
# 이 값을 보면 자기가 직접 크롭 분류하는 폴백 경로로 간다.
UNCLASSIFIED = -1


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
        # 신호등 후보 클래스. 실제 색 판정은 아래 크롭 분류기가 하므로,
        # 여기서는 "신호등처럼 생긴 위치" 후보를 고르는 용도로만 쓴다 —
        # 순서는 의미가 없다.
        self.declare_parameter("traffic_class_ids", [2, 3, 4, 5])
        self.declare_parameter("traffic_output_topic", "/traffic_boxes")
        # ── 신호등 크롭 분류기 ──────────────────────────────────────────
        # 빈 문자열이면 share 디렉터리(model/light_cls.onnx)를 탐색한다.
        # 전처리 상수는 object_detection.cpp 의 폴백 경로와 같은 기본값을
        # 쓴다 — 두 경로가 같은 박스에서 다른 판정을 내면 디버깅이 지옥이다.
        self.declare_parameter("light_classifier_path", "")
        self.declare_parameter("light_crop_margin", 0.15)
        self.declare_parameter("light_input_size", 64)
        # 잘려서 모양이 무너진 박스를 분류 전에 걸러내는 기하 게이트.
        # 근거와 임계값 선정은 yolo_runtime.light_geometry_ok() 참고.
        self.declare_parameter("light_max_aspect", 8.0)
        self.declare_parameter("light_edge_min_h", 20)

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

        self.light_margin = float(self.get_parameter("light_crop_margin").value)
        self.light_input_size = int(self.get_parameter("light_input_size").value)
        self.light_max_aspect = float(self.get_parameter("light_max_aspect").value)
        self.light_edge_min_h = int(self.get_parameter("light_edge_min_h").value)

        # 분류기를 못 띄워도 차량 경로는 그대로 돌아야 한다. 이 경우
        # /traffic_boxes 의 class_id 는 UNCLASSIFIED 로 나가고, object_node 가
        # 예전처럼 직접 크롭 분류하는 폴백 경로를 탄다.
        light_path = str(self.get_parameter("light_classifier_path").value)
        if not light_path:
            light_path = os.path.join(
                get_package_share_directory("object_detection"),
                "model",
                "light_cls.onnx",
            )
        self.light_session = None
        self.light_input_name = ""
        if os.path.isfile(light_path):
            try:
                self.light_session = ort.InferenceSession(
                    light_path,
                    providers=["CPUExecutionProvider"],
                )
                self.light_input_name = self.light_session.get_inputs()[0].name
                self.get_logger().info(
                    f"신호등 크롭 분류기 로드 완료: {light_path} "
                    f"(margin={self.light_margin}, size={self.light_input_size}, "
                    f"max_aspect={self.light_max_aspect}, "
                    f"edge_min_h={self.light_edge_min_h})"
                )
            except Exception as exc:
                self.light_session = None
                self.get_logger().error(
                    f"신호등 분류기 로드 실패, object_node 폴백으로 넘깁니다: {exc}"
                )
        else:
            self.get_logger().warn(
                f"신호등 분류기 모델이 없어 object_node 폴백으로 넘깁니다: {light_path}"
            )

        # ── 폴백 상태 알림 ────────────────────────────────────────────
        # 시작 시점 로그(위 warn/error) 는 한 번만 찍히고 스크롤로 사라진다.
        # 이 노드가 실행 중인 내내 -1(UNCLASSIFIED) 을 계속 내보내고 있다는
        # 사실을, 나중에 붙어서 로그를 보는 사람도 알 수 있어야 한다 —
        # object_node 의 classifyLight() 폴백은 last_raw_img_(최신 프레임)
        # 에서 다시 자르므로 원래 lag 버그의 조건을 그대로 갖는다(README 참고).
        # /object_yolo, /traffic_boxes 자체는 정상 발행되므로 겉보기엔 시스템이
        # 멀쩡해 보인다는 점이 이 알림이 필요한 이유다.
        if self.light_session is None:
            self.create_timer(5.0, self._warn_fallback_active)

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

    def _warn_fallback_active(self) -> None:
        self.get_logger().warn(
            "신호등 크롭 분류기가 없어 object_node 의 classifyLight() 폴백으로 "
            "동작 중입니다 (최신 프레임에서 재크롭 — 프레임 어긋남에 취약). "
            "light_classifier_path 를 확인하세요."
        )

    def classify_light(self, image, detection) -> tuple[int, float]:
        """검출에 쓴 바로 그 프레임에서 박스를 잘라 색을 판정한다.

        image 는 on_image() 가 이번 콜백에서 만든 배열 그대로다. 이 함수가
        object_node(C++) 가 아니라 여기 있어야 하는 이유는 파일 상단 참고 —
        토픽을 건너가면 크롭 원본이 다른 프레임이 되어버린다.

        반환값의 인덱스는 학습 시 폴더명 정렬 순서 = onnx 출력 순서다
        (0=green 1=left_green 2=orange 3=red). 재학습해서 순서가 바뀌면
        object_detection.cpp 의 스위치문도 같이 고쳐야 한다.
        """

        if self.light_session is None:
            return UNCLASSIFIED, 0.0
        try:
            crop = crop_with_margin(
                image,
                (detection.x, detection.y, detection.width, detection.height),
                self.light_margin,
            )
            blob = light_blob(crop, self.light_input_size)
            probabilities = self.light_session.run(
                None, {self.light_input_name: blob}
            )[0][0]
        except Exception as exc:
            # 크롭·추론이 프레임마다 실패할 수 있어(예: onnxruntime 세션이
            # 죽었는데 예외만 던지는 경우) 5초로 묶는다 — 매 프레임 로그면
            # 진짜 폴백 원인(모델 미로드)이 스크롤에 묻힌다.
            self.get_logger().error(
                f"신호등 크롭 분류 실패: {exc}", throttle_duration_sec=5.0
            )
            return UNCLASSIFIED, 0.0
        index = int(np.argmax(probabilities))
        return index, float(probabilities[index])

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

        # 박스마다 같은 프레임에서 잘라 분류한 뒤, 검출기 class_id/confidence
        # 자리에 분류기 결과를 실어 보낸다. 우선순위(좌회전>직진>정지) 판정과
        # 디바운스, 확신도 게이트는 계속 object_node(C++) 가 한다 — 박스를
        # 하나로 줄이지 않고 전부 내보내는 이유가 이것이다. 좌회전 화살표는
        # 초록 원과 같이 켜지는 경우가 많아 여러 박스를 한꺼번에 봐야 한다.
        #
        # 모양이 무너진 박스는 분류를 아예 건너뛰고 버린다. 분류기에는
        # background 클래스가 없어서 램프가 안 잡힌 크롭도 4개 중 하나를
        # 뱉어야 하고, 그 fallback 이 orange 다 (평평한 패치를 넣어도 orange
        # 가 argmax 로 나온다). orange -> 정지(1) 라 초록불 아래에서 서게 된다.
        traffic_message = Float32MultiArray()
        flat: list[float] = []
        for det in traffic_detections:
            if not light_geometry_ok(
                det,
                max_aspect=self.light_max_aspect,
                edge_min_height=self.light_edge_min_h,
            ):
                continue
            index, score = self.classify_light(image, det)
            flat.extend([
                float(index), float(score),
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
