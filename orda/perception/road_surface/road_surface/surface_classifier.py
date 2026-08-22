"""ROS-independent PIDNet class-map validation and surface classification."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import math

import cv2
import numpy as np


class RoadSurface(IntEnum):
    UNKNOWN = 0
    NORMAL_ROAD = 1
    SHORTCUT = 2


@dataclass(frozen=True)
class SurfaceThresholds:
    road_min_ratio: float
    shortcut_min_ratio: float
    road_min_component_px: int
    shortcut_min_component_px: int
    roi_top: float = 0.0
    roi_bottom: float = 1.0
    roi_left: float = 0.0
    roi_right: float = 1.0
    roi_top_width_ratio: float = 1.0

    def __post_init__(self) -> None:
        for name in ("road_min_ratio", "shortcut_min_ratio"):
            value = getattr(self, name)
            if not math.isfinite(value) or not 0.0 <= value <= 1.0:
                raise ValueError(f"{name} must be in [0, 1]")
        for name in ("road_min_component_px", "shortcut_min_component_px"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ValueError(f"{name} must be a positive integer")
        if not (
            0.0 <= self.roi_top < self.roi_bottom <= 1.0
            and 0.0 <= self.roi_left < self.roi_right <= 1.0
        ):
            raise ValueError("ROI fractions must define a non-empty image region")
        if not 0.0 < self.roi_top_width_ratio <= 1.0:
            raise ValueError("roi_top_width_ratio must be in (0, 1]")


@dataclass(frozen=True)
class SurfaceEvidence:
    road_ratio: float
    shortcut_ratio: float
    road_largest_component_px: int
    shortcut_largest_component_px: int


def class_map_from_image(message) -> np.ndarray:
    """Decode a one-channel ROS Image without accepting truncated rows."""

    height = int(message.height)
    width = int(message.width)
    if height <= 0 or width <= 0:
        raise ValueError("class-map dimensions must be positive")
    encoding = str(message.encoding).lower()
    if encoding in ("mono8", "8uc1"):
        dtype = np.dtype(np.uint8)
    elif encoding == "32sc1":
        dtype = np.dtype(">i4" if message.is_bigendian else "<i4")
    else:
        raise ValueError(f"unsupported class-map encoding: {message.encoding}")
    row_bytes = width * dtype.itemsize
    step = int(message.step)
    if step < row_bytes:
        raise ValueError("class-map step is smaller than one row")
    raw = memoryview(message.data)
    if len(raw) != step * height:
        raise ValueError("class-map data length does not match height and step")
    rows = np.frombuffer(raw, dtype=np.uint8).reshape(height, step)
    packed = np.ascontiguousarray(rows[:, :row_bytes])
    return packed.view(dtype).reshape(height, width).astype(np.int32, copy=False)


def _largest_component(mask: np.ndarray) -> int:
    count, _, stats, _ = cv2.connectedComponentsWithStats(
        mask.astype(np.uint8),
        connectivity=8,
    )
    if count <= 1:
        return 0
    return int(np.max(stats[1:, cv2.CC_STAT_AREA]))


def surface_evidence(
    class_map: np.ndarray,
    thresholds: SurfaceThresholds,
) -> SurfaceEvidence:
    if class_map.ndim != 2 or class_map.size == 0:
        raise ValueError("class map must be a non-empty 2D array")
    if not np.issubdtype(class_map.dtype, np.integer):
        raise ValueError("class map must contain integer labels")
    height, width = class_map.shape
    top = int(math.floor(height * thresholds.roi_top))
    bottom = int(math.ceil(height * thresholds.roi_bottom))
    left = int(math.floor(width * thresholds.roi_left))
    right = int(math.ceil(width * thresholds.roi_right))
    roi = class_map[top:bottom, left:right]
    if roi.size == 0:
        raise ValueError("ROI is empty")
    selection = np.ones(roi.shape, dtype=np.uint8)
    if thresholds.roi_top_width_ratio < 1.0:
        selection.fill(0)
        roi_height, roi_width = roi.shape
        inset = (1.0 - thresholds.roi_top_width_ratio) * roi_width / 2.0
        cv2.fillConvexPoly(
            selection,
            np.array(
                [
                    [int(math.floor(inset)), 0],
                    [int(math.ceil(roi_width - inset)) - 1, 0],
                    [roi_width - 1, roi_height - 1],
                    [0, roi_height - 1],
                ],
                dtype=np.int32,
            ),
            1,
        )
    selected = selection.astype(bool)
    selected_pixels = int(np.count_nonzero(selected))
    road_mask = (roi == 4) & selected
    shortcut_mask = (roi == 5) & selected
    return SurfaceEvidence(
        road_ratio=float(np.count_nonzero(road_mask) / selected_pixels),
        shortcut_ratio=float(np.count_nonzero(shortcut_mask) / selected_pixels),
        road_largest_component_px=_largest_component(road_mask),
        shortcut_largest_component_px=_largest_component(shortcut_mask),
    )


def classify_surface(
    class_map: np.ndarray,
    thresholds: SurfaceThresholds,
) -> tuple[RoadSurface, SurfaceEvidence]:
    evidence = surface_evidence(class_map, thresholds)
    shortcut_ready = (
        evidence.shortcut_ratio >= thresholds.shortcut_min_ratio
        and evidence.shortcut_largest_component_px
        >= thresholds.shortcut_min_component_px
    )
    road_ready = (
        evidence.road_ratio >= thresholds.road_min_ratio
        and evidence.road_largest_component_px >= thresholds.road_min_component_px
    )
    if shortcut_ready:
        return RoadSurface.SHORTCUT, evidence
    if road_ready:
        return RoadSurface.NORMAL_ROAD, evidence
    return RoadSurface.UNKNOWN, evidence
