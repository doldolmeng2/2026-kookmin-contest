"""Same-frame traffic-light crop classification helpers."""

from __future__ import annotations

import ast
from collections.abc import Mapping

import cv2
import numpy as np

from .yolo_runtime import Detection


TRAFFIC_CLASS_NAMES = {
    0: "green_light",
    1: "left_green_light",
    2: "orange_light",
    3: "red_light",
}
TRAFFIC_CLASS_TO_SIGNAL = {0: 2, 1: 3, 2: 1, 3: 1}


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
    fixed_class_ids=(0,),
    moving_class_ids=(1,),
    traffic_class_ids=range(2, 6),
) -> None:
    """Reject a detector that cannot satisfy the train-10 class contract."""

    fixed_names = {"fixed", "red_car"}
    moving_names = {"moving", "green_car"}
    invalid_fixed = sorted(
        class_id for class_id in fixed_class_ids if names.get(class_id) not in fixed_names
    )
    if invalid_fixed:
        raise ValueError(
            f"detector class {invalid_fixed[0]} must be fixed/red_car"
        )
    invalid_moving = sorted(
        class_id
        for class_id in moving_class_ids
        if names.get(class_id) not in moving_names
    )
    if invalid_moving:
        raise ValueError(
            f"detector class {invalid_moving[0]} must be moving/green_car"
        )
    missing = sorted(set(traffic_class_ids) - set(names))
    if missing:
        raise ValueError(f"detector traffic candidate classes missing: {missing}")


def validate_classifier_classes(names: Mapping[int, str]) -> None:
    if dict(names) != TRAFFIC_CLASS_NAMES:
        raise ValueError(
            "traffic classifier classes must be "
            "0 green, 1 left_green, 2 orange, 3 red"
        )


def crop_detection(image: np.ndarray, detection: Detection) -> np.ndarray:
    """Copy one current-frame crop; no result survives the calling callback."""

    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("expected an HxWx3 BGR image")
    height, width = image.shape[:2]
    left = max(0, min(detection.x, width))
    top = max(0, min(detection.y, height))
    right = max(left, min(detection.x + detection.width, width))
    bottom = max(top, min(detection.y + detection.height, height))
    if right <= left or bottom <= top:
        raise ValueError("traffic candidate crop is empty")
    return image[top:bottom, left:right].copy()


def classifier_blob(
    crop: np.ndarray,
    input_height: int,
    input_width: int,
) -> np.ndarray:
    """Apply the standard Ultralytics classification RGB normalization."""

    if crop.ndim != 3 or crop.shape[2] != 3 or crop.size == 0:
        raise ValueError("expected a non-empty HxWx3 BGR crop")
    if input_height <= 0 or input_width <= 0:
        raise ValueError("classifier input dimensions must be positive")
    rgb = cv2.cvtColor(
        cv2.resize(crop, (input_width, input_height)),
        cv2.COLOR_BGR2RGB,
    ).astype(np.float32) / 255.0
    mean = np.asarray((0.485, 0.456, 0.406), dtype=np.float32)
    std = np.asarray((0.229, 0.224, 0.225), dtype=np.float32)
    normalized = (rgb - mean) / std
    return np.ascontiguousarray(np.transpose(normalized, (2, 0, 1))[None])


def traffic_signal_from_output(
    output: np.ndarray,
    confidence_threshold: float,
) -> int:
    """Map one four-class classifier output to the official traffic code."""

    if not 0.0 <= confidence_threshold <= 1.0:
        raise ValueError("classifier confidence threshold must be in [0, 1]")
    scores = np.asarray(output, dtype=np.float64).reshape(-1)
    if scores.shape != (4,) or not np.all(np.isfinite(scores)):
        raise ValueError("traffic classifier output must contain four finite values")

    # Accept probabilities directly; otherwise interpret the output as logits.
    if np.all(scores >= 0.0) and np.isclose(scores.sum(), 1.0, atol=1e-3):
        probabilities = scores
    else:
        shifted = scores - np.max(scores)
        exponentials = np.exp(shifted)
        probabilities = exponentials / exponentials.sum()
    class_id = int(np.argmax(probabilities))
    if float(probabilities[class_id]) < confidence_threshold:
        return 0
    return TRAFFIC_CLASS_TO_SIGNAL[class_id]


def classify_current_frame_candidate(
    image: np.ndarray,
    detection: Detection | None,
    session,
    input_name: str,
    input_height: int,
    input_width: int,
    confidence_threshold: float,
) -> int:
    """Classify only the candidate passed for this frame; ``None`` means 0."""

    if detection is None:
        return 0
    crop = crop_detection(image, detection)
    blob = classifier_blob(crop, input_height, input_width)
    output = session.run(None, {input_name: blob})[0]
    return traffic_signal_from_output(output, confidence_threshold)
