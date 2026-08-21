"""ROS-independent Ultralytics ONNX preprocessing and decoding."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional

import cv2
import numpy as np


@dataclass(frozen=True)
class Detection:
    class_id: int
    confidence: float
    x: int
    y: int
    width: int
    height: int

    @property
    def area(self) -> int:
        return self.width * self.height

    @property
    def center(self) -> tuple[float, float]:
        return self.x + self.width * 0.5, self.y + self.height * 0.5


@dataclass(frozen=True)
class Letterbox:
    blob: np.ndarray
    scale: float
    pad_x: float
    pad_y: float


def letterbox_blob(image: np.ndarray, input_size: int = 640) -> Letterbox:
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("expected an HxWx3 BGR image")
    height, width = image.shape[:2]
    if height <= 0 or width <= 0:
        raise ValueError("image must not be empty")

    scale = min(input_size / width, input_size / height)
    resized_width = int(round(width * scale))
    resized_height = int(round(height * scale))
    pad_x = (input_size - resized_width) * 0.5
    pad_y = (input_size - resized_height) * 0.5
    canvas = np.full((input_size, input_size, 3), 114, dtype=np.uint8)
    resized = cv2.resize(image, (resized_width, resized_height))
    left = int(pad_x)
    top = int(pad_y)
    canvas[top:top + resized_height, left:left + resized_width] = resized
    rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
    blob = np.transpose(rgb.astype(np.float32) / 255.0, (2, 0, 1))[None]
    return Letterbox(np.ascontiguousarray(blob), scale, pad_x, pad_y)


def prediction_rows(output: np.ndarray) -> np.ndarray:
    """Normalize ``[1,C,N]`` or ``[1,N,C]`` into ``[N,C]`` rows."""

    value = np.asarray(output)
    if value.ndim == 3:
        if value.shape[0] != 1:
            raise ValueError(f"unsupported ONNX batch shape: {value.shape}")
        value = value[0]
    if value.ndim != 2:
        raise ValueError(f"unsupported ONNX output shape: {value.shape}")
    if value.shape[0] < 5 and value.shape[1] < 5:
        raise ValueError(f"YOLO output has fewer than five fields: {value.shape}")

    # Ultralytics exports commonly use [4 + classes, anchors].  A row count
    # much larger than the feature count means the output is already [N, C].
    if value.shape[1] < 5:
        value = value.T
    elif value.shape[0] < 5:
        pass
    elif value.shape[0] < value.shape[1]:
        value = value.T
    if value.shape[1] < 5:
        raise ValueError(f"YOLO row has fewer than five fields: {value.shape}")
    return np.ascontiguousarray(value)


def decode_detections(
    output: np.ndarray,
    *,
    image_shape: tuple[int, int],
    scale: float,
    pad_x: float,
    pad_y: float,
    confidence_threshold: float,
    nms_threshold: float,
    allowed_class_ids: Optional[Iterable[int]] = None,
    min_size_px: int = 12,
) -> list[Detection]:
    """Decode a YOLOv8 detection tensor without hard-coded class count."""

    rows = prediction_rows(output)
    allowed = None if allowed_class_ids is None else set(allowed_class_ids)
    image_height, image_width = image_shape
    boxes: list[list[int]] = []
    scores: list[float] = []
    class_ids: list[int] = []

    for row in rows:
        class_scores = row[4:]
        class_id = int(np.argmax(class_scores))
        confidence = float(class_scores[class_id])
        if confidence < confidence_threshold:
            continue
        if allowed is not None and class_id not in allowed:
            continue

        cx, cy, width, height = (float(v) for v in row[:4])
        x = ((cx - width * 0.5) - pad_x) / scale
        y = ((cy - height * 0.5) - pad_y) / scale
        width /= scale
        height /= scale
        left = max(0, min(int(round(x)), image_width - 1))
        top = max(0, min(int(round(y)), image_height - 1))
        box_width = max(1, min(int(round(width)), image_width - left))
        box_height = max(1, min(int(round(height)), image_height - top))
        if box_width < min_size_px or box_height < min_size_px:
            continue
        boxes.append([left, top, box_width, box_height])
        scores.append(confidence)
        class_ids.append(class_id)

    if not boxes:
        return []
    indices = cv2.dnn.NMSBoxes(
        boxes,
        scores,
        confidence_threshold,
        nms_threshold,
    )
    if len(indices) == 0:
        return []
    return [
        Detection(class_ids[index], scores[index], *boxes[index])
        for index in np.asarray(indices).reshape(-1)
    ]


def closest_detection(detections: Iterable[Detection]) -> Optional[Detection]:
    """Choose the largest box, matching the existing closest-object policy."""

    return max(detections, key=lambda item: item.area, default=None)
