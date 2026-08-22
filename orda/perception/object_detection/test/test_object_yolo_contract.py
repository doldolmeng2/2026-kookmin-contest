"""object_yolo_node.py ↔ object_detection.cpp 배선 계약 고정.

두 파일은 /object_yolo · /traffic_boxes 로만 이어져 있고 메시지가 전부
Float32MultiArray 라, 한쪽 필드 수나 토픽 이름이 바뀌어도 빌드는 통과하고
런타임에는 조용히 아무 값도 안 들어온다. 그 조합을 여기서 막는다.

이 패키지는 train-2 계열 7클래스 통합 모델(차량+신호등)을 단일 검출기로
쓴다. 신호등 색은 별도 크롭 분류기가 아니라 검출기 클래스 id 로 정해진다
(크롭 분류기 경로 traffic_classifier.py / light_cls.onnx 는 삭제했다).
"""

import ast
from pathlib import Path

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE / "scripts" / "object_yolo_node.py"
CPP = PACKAGE / "src" / "object_detection.cpp"
CONFIG = PACKAGE / "config" / "object_detection.yaml"


def _script_tree():
    return ast.parse(SCRIPT.read_text(encoding="utf-8"))


def test_parameters_are_declared_once_with_production_defaults():
    declarations = []
    for node in ast.walk(_script_tree()):
        if (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "declare_parameter"
            and node.args
            and isinstance(node.args[0], ast.Constant)
        ):
            declarations.append(node.args[0].value)

    assert len(declarations) == len(set(declarations))
    source = SCRIPT.read_text(encoding="utf-8")
    # 이 두 토픽 기본값이 아래 C++ 구독자와 짝이다.
    assert 'self.declare_parameter("output_topic", "/object_yolo")' in source
    assert 'self.declare_parameter("traffic_output_topic", "/traffic_boxes")' in source


def test_publishers_have_one_canonical_message_type_each():
    source = SCRIPT.read_text(encoding="utf-8")

    assert "Float32MultiArray, output_topic, qos" in source
    assert "Float32MultiArray, traffic_output_topic, qos" in source
    assert "self.publisher.publish(output_message)" in source
    assert "self.traffic_publisher.publish(traffic_message)" in source


def test_node_emits_both_slots_every_frame():
    source = SCRIPT.read_text(encoding="utf-8")

    assert "detection_slot(fixed_detection, FIXED)" in source
    assert "detection_slot(moving_detection, MOVING)" in source


def test_cpp_consumes_both_object_yolo_slots():
    cpp = CPP.read_text(encoding="utf-8")

    # 20필드 = fixed 슬롯(offset 0, type 0) + moving 슬롯(offset 10, type 1).
    # 슬롯 위치마다 타입을 강제해서, 순서가 뒤바뀐 메시지를 조용히 받아들이지
    # 않는다. 기록된 구형 bag 을 위해 10필드 단일 슬롯도 계속 받는다.
    assert "msg->data.size() != 10 && msg->data.size() != 20" in cpp
    assert "parse_slot(0, 0, fixed)" in cpp
    assert "parse_slot(10, 1, moving)" in cpp
    assert '"/object_yolo", qos_fast' in cpp
    assert '"/traffic_boxes", qos_fast' in cpp


def test_cpp_keeps_the_two_lane_labels_independent():
    """두 슬롯의 차선 확정 상태가 섞이지 않는지 확인한다.

    상태를 공유하면 고정장애물 검출이 방해차량의 확정 차선을 덮어써서
    /object_info 의 두 위치 필드가 같은 값으로 붙어 나온다.
    """
    cpp = CPP.read_text(encoding="utf-8")

    assert "fixed_lane_stabilizer_.update(" in cpp
    assert "moving_lane_stabilizer_.update(" in cpp
    assert "last_fixed_lane_label_" in cpp
    assert "last_moving_lane_label_" in cpp


def test_cpp_publishes_the_contracts_main_node_subscribes_to():
    cpp = CPP.read_text(encoding="utf-8")

    assert '"/object_info", qos_fast' in cpp
    assert '"/object_info_raw", qos_fast' in cpp
    assert '"/side_clearance", qos_sensor_output' in cpp


def test_traffic_class_ids_cover_the_detector_light_classes():
    source = SCRIPT.read_text(encoding="utf-8")

    # 1=green_light 2=left_green_light 3=orange_light 5=red_light.
    # 6=traffic(몸체)은 색을 못 정하므로 제외한다 — C++ 우선순위 판정도
    # 이 네 개만 다룬다.
    assert 'self.declare_parameter("traffic_class_ids", [1, 2, 3, 5])' in source
    assert "if      (cid == 1) tl_green  = true;" in CPP.read_text(encoding="utf-8")


def test_config_does_not_redirect_the_node_topic_defaults():
    """config 가 신호등 박스를 엉뚱한 토픽으로 돌려놓지 않는지 확인한다.

    옛 config 에는 traffic_output_topic: "/traffic_detection" 이 들어 있었다.
    지금 노드에서 그 값이 살아 있으면 박스가 /traffic_boxes 로 안 나가고
    C++ 구독자는 영원히 빈 상태가 된다 — 에러도 경고도 없이.
    """
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8")) or {}
    params = config.get("object_yolo_node", {}).get("ros__parameters", {})

    assert params.get("traffic_output_topic", "/traffic_boxes") == "/traffic_boxes"
    assert params.get("output_topic", "/object_yolo") == "/object_yolo"
