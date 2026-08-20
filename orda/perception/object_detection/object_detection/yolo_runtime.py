"""ROS-independent Ultralytics ONNX preprocessing and decoding."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Generic, Iterable, Optional, TypeVar

import cv2
import numpy as np


T = TypeVar("T")


class LatestFrameRateLimiter(Generic[T]):
    """Keep at most one pending item and release only the newest when due."""

    def __init__(self, max_rate_hz: float) -> None:
        self.max_rate_hz = float(max_rate_hz)
        self.period_s = (
            1.0 / self.max_rate_hz if self.max_rate_hz > 0.0 else 0.0
        )
        self.next_allowed_s = 0.0
        self._latest: Optional[T] = None

    @property
    def enabled(self) -> bool:
        return self.period_s > 0.0

    @property
    def has_pending(self) -> bool:
        return self._latest is not None

    def offer(self, item: T, now_s: float) -> Optional[T]:
        if not self.enabled:
            return item
        self._latest = item
        return self.pop_due(now_s)

    def pop_due(self, now_s: float) -> Optional[T]:
        if self._latest is None or now_s < self.next_allowed_s:
            return None
        selected = self._latest
        self._latest = None
        self.next_allowed_s = now_s + self.period_s
        return selected


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


def normalize_class_ids(values: Iterable[int]) -> set[int]:
    """Return a validated, de-duplicated set of non-negative class IDs."""

    try:
        normalized = {int(value) for value in values}
    except (TypeError, ValueError) as exc:
        raise ValueError("class IDs must be integers") from exc
    if any(value < 0 for value in normalized):
        raise ValueError("class IDs must be non-negative")
    return normalized


def closest_detection_for_classes(
    detections: Iterable[Detection], class_ids: Iterable[int]
) -> Optional[Detection]:
    """Choose the largest detection whose class belongs to ``class_ids``."""

    allowed = normalize_class_ids(class_ids)
    return closest_detection(
        detection for detection in detections if detection.class_id in allowed
    )


def detection_slot(
    detection: Optional[Detection],
    semantic_type: int,
) -> list[float]:
    """Encode one fixed/moving detection as the canonical ten-field slot."""

    if semantic_type not in (0, 1):
        raise ValueError("semantic type must be fixed(0) or moving(1)")
    if detection is None:
        return [0.0, float(semantic_type)] + [0.0] * 8
    center_x, center_y = detection.center
    return [
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


def crop_with_margin(
    image: np.ndarray,
    box: tuple[int, int, int, int],
    margin: float = 0.15,
) -> np.ndarray:
    """신호등 박스에 margin 비율만큼 여백을 붙여 자른다.

    object_detection.cpp 의 classifyLight() 와 자르는 순서·클램핑이 같아야
    한다. 둘 중 하나만 바뀌면 같은 박스가 노드마다 다른 크롭이 되어, 폴백
    경로(C++ 직접 분류)와 주 경로(여기)의 판정이 갈린다.
    """

    x, y, width, height = box
    grow_x = width * margin * 0.5
    grow_y = height * margin * 0.5
    image_height, image_width = image.shape[:2]
    x1 = int(round(x - grow_x))
    y1 = int(round(y - grow_y))
    x2 = int(round(x + width + grow_x))
    y2 = int(round(y + height + grow_y))
    x1 = max(0, min(x1, image_width - 1))
    y1 = max(0, min(y1, image_height - 1))
    x2 = max(x1 + 1, min(x2, image_width))
    y2 = max(y1 + 1, min(y2, image_height))
    return image[y1:y2, x1:x2]


def light_blob(crop: np.ndarray, input_size: int = 64) -> np.ndarray:
    """크롭을 분류기 입력 텐서로 만든다 (정사각 stretch -> RGB -> /255 -> CHW).

    학습 크롭 생성(tools/make_light_crops.py)과 반드시 같은 전처리여야 한다.
    cv2.dnn.blobFromImage 를 쓰는 건 C++ 폴백 경로와 바이트 단위로 같은 텐서를
    만들기 위해서다 — 직접 transpose 하면 미묘하게 달라질 여지가 있다.
    """

    resized = cv2.resize(
        crop, (input_size, input_size), interpolation=cv2.INTER_CUBIC
    )
    return cv2.dnn.blobFromImage(
        resized,
        1.0 / 255.0,
        (input_size, input_size),
        (0, 0, 0),
        swapRB=True,
        crop=False,
    )


def light_geometry_ok(
    detection: Detection,
    *,
    max_aspect: float = 8.0,
    edge_min_height: int = 20,
) -> bool:
    """잘려서 모양이 무너진 신호등 박스인지 본다 (True = 분류해도 되는 모양).

    신호등에 근접하면 하우징이 화면 위로 빠져나가고, 검출기는 보이는 띠만
    잡는다. 종횡비가 4:1 에서 15:1 까지 무너진 박스를 64x64 정사각으로 늘리면
    학습 크롭 분포(약 4:1 하우징) 밖이라 분류기가 램프도 없는 띠를 보고 아무
    클래스나 고른다. 확신도로는 못 거른다 — 그렇게 나온 오답이 0.99 를 넘겼다.

    높이 단독으로는 자르지 않는다. 멀리 있는 신호등은 h 가 10px 대여도 정상이고
    (이 bag 에서 h<12 인 박스 24개가 전부 정답), 문제가 되는 건 "화면 최상단에
    붙어 있으면서 납작한" 조합뿐이다.
    """

    if detection.height <= 0:
        return False
    if detection.width / detection.height > max_aspect:
        return False
    if detection.y <= 1 and detection.height < edge_min_height:
        return False
    return True
