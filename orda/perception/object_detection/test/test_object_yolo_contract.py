import ast
from pathlib import Path

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE / "scripts" / "object_yolo_node.py"
CPP = PACKAGE / "src" / "object_detection.cpp"
CONFIG = PACKAGE / "config" / "object_detection.yaml"


def _script_tree():
    return ast.parse(SCRIPT.read_text(encoding="utf-8"))


def test_parameters_are_declared_once_with_safe_production_defaults():
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
    assert 'self.declare_parameter("max_inference_hz", 0.0)' in source
    assert 'self.declare_parameter("traffic_output_topic", "/traffic_detection")' in source
    assert 'self.declare_parameter("traffic_boxes_topic", "/traffic_boxes")' in source


def test_publishers_have_one_canonical_message_type_each():
    source = SCRIPT.read_text(encoding="utf-8")

    assert "Float32MultiArray, output_topic, qos" in source
    assert "Int32, traffic_output_topic, qos" in source
    assert "Float32MultiArray, traffic_boxes_topic, qos" in source
    assert "self.traffic_publisher.publish(signal_message)" in source
    assert "self.traffic_boxes_publisher.publish(traffic_message)" in source


def test_dual_slot_and_same_frame_classification_are_preserved():
    source = SCRIPT.read_text(encoding="utf-8")
    cpp = CPP.read_text(encoding="utf-8")

    assert "detection_slot(fixed_detection, FIXED)" in source
    assert "detection_slot(moving_detection, MOVING)" in source
    assert "classify_current_frame_candidate(" in source
    assert "classify_light(image, det)" in source
    assert "msg->data.size() != 10 && msg->data.size() != 20" in cpp
    assert "parse_slot(0, 0, fixed)" in cpp
    assert "parse_slot(10, 1, moving)" in cpp
    assert '"/side_clearance", qos_sensor_output' in cpp


def test_config_matches_the_seven_class_production_detector():
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    params = config["object_yolo_node"]["ros__parameters"]

    assert params["fixed_class_ids"] == [4]
    assert params["moving_class_ids"] == [0]
    assert params["traffic_class_ids"] == [1, 2, 3, 5, 6]
    assert params["max_inference_hz"] == 0.0
    assert params["traffic_output_topic"] == "/traffic_detection"
    assert params["traffic_boxes_topic"] == "/traffic_boxes"
