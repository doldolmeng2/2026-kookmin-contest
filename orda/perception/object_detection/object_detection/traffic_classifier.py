"""Detector metadata and direct traffic-class mapping helpers."""

from __future__ import annotations

import ast
from collections.abc import Mapping


DETECTOR_CLASS_NAMES = {
    0: "green_car",
    1: "green_light",
    2: "left_green_light",
    3: "orange_light",
    4: "red_car",
    5: "red_light",
    6: "traffic",
}

# /traffic_boxes signal index: 0=green, 1=left green, 2=orange, 3=red.
# Detector class 6 is intentionally absent: it is generic and cannot provide
# state-specific traffic evidence.
DETECTOR_TRAFFIC_TO_SIGNAL_INDEX = {
    1: 0,
    2: 1,
    3: 2,
    5: 3,
}
SIGNAL_INDEX_TO_TRAFFIC_STATE = {0: 2, 1: 3, 2: 1, 3: 1}


def detector_traffic_to_signal_index(class_id: int) -> int | None:
    """Return the /traffic_boxes index for a state-specific detector class."""

    return DETECTOR_TRAFFIC_TO_SIGNAL_INDEX.get(int(class_id))


def detector_traffic_to_state(class_id: int) -> int | None:
    """Return /object_info traffic state, or None for non-evidence classes."""

    signal_index = detector_traffic_to_signal_index(class_id)
    if signal_index is None:
        return None
    return SIGNAL_INDEX_TO_TRAFFIC_STATE[signal_index]


def detector_traffic_box_values(
    class_id: int,
    confidence: float,
    x: int,
    y: int,
    width: int,
    height: int,
) -> tuple[float, float, float, float, float, float] | None:
    """Build one direct-mapped /traffic_boxes record, preserving detector data."""

    signal_index = detector_traffic_to_signal_index(class_id)
    if signal_index is None:
        return None
    return (
        float(signal_index),
        float(confidence),
        float(x),
        float(y),
        float(width),
        float(height),
    )


def detector_object_type(
    class_id: int,
    fixed_class_ids=(4,),
    moving_class_ids=(0,),
) -> int | None:
    """Map a configured detector vehicle class to /object_yolo object_type."""

    if class_id in fixed_class_ids:
        return 0
    if class_id in moving_class_ids:
        return 1
    return None


def parse_class_names(value: object) -> dict[int, str]:
    """Parse the Ultralytics ONNX ``names`` metadata into a strict mapping."""

    if isinstance(value, str):
        try:
            value = ast.literal_eval(value)
        except (SyntaxError, ValueError) as exc:
            raise ValueError("invalid ONNX names metadata") from exc
    if not isinstance(value, Mapping):
        raise ValueError("ONNX names metadata must be a mapping")
    parsed: dict[int, str] = {}
    for class_id, name in value.items():
        try:
            normalized_id = int(class_id)
        except (TypeError, ValueError) as exc:
            raise ValueError("ONNX class IDs must be integers") from exc
        if normalized_id < 0 or not isinstance(name, str) or not name:
            raise ValueError("invalid ONNX class metadata entry")
        parsed[normalized_id] = name
    return parsed


def validate_detector_classes(
    names: Mapping[int, str],
    fixed_class_ids=(4,),
    moving_class_ids=(0,),
    traffic_class_ids=(1, 2, 3, 5, 6),
) -> None:
    """Reject a detector that cannot satisfy the train-10 class contract."""

    if set(fixed_class_ids) != {4}:
        raise ValueError("fixed detector class IDs must be [4]")
    if set(moving_class_ids) != {0}:
        raise ValueError("moving detector class IDs must be [0]")
    if set(traffic_class_ids) != {1, 2, 3, 5, 6}:
        raise ValueError("traffic detector class IDs must be [1, 2, 3, 5, 6]")
    if dict(names) != DETECTOR_CLASS_NAMES:
        raise ValueError("detector classes do not match the train-10 contract")
