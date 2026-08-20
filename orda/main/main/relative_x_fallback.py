"""Pure relative-X fallback for obstacle-lane mission decisions."""

from __future__ import annotations

from dataclasses import dataclass
import math
import statistics
from typing import Optional

from .mission_types import LaneTarget, ObjectLane, ObjectType


FRAME_CENTER_X_PX = 320.0
LANE_ONE_RIGHT_THRESHOLD_PX = 62.5
LANE_TWO_LEFT_THRESHOLD_PX = -16.75
REQUIRED_INDEPENDENT_DETECTIONS = 3


@dataclass(frozen=True)
class ObjectDetectionSignature:
    """Fields that identify one YOLO result behind repeated raw publications."""

    object_type: ObjectType
    box_size: float
    box_cx: float
    box_cy: float
    confidence: float


class RelativeXObstacleLaneFallback:
    """Collect independent YOLO states and latch at mission-entry distance."""

    def __init__(self, *, encounter_timeout_s: float) -> None:
        if not math.isfinite(encounter_timeout_s) or encounter_timeout_s <= 0.0:
            raise ValueError("encounter_timeout_s must be finite and positive")
        self.encounter_timeout_s = float(encounter_timeout_s)
        self.reset()

    def reset(self) -> None:
        self._object_type = ObjectType.UNKNOWN
        self._last_signature: Optional[ObjectDetectionSignature] = None
        self._last_detection_at: Optional[float] = None
        self._relative_x_samples: list[float] = []
        self._latched_lane = ObjectLane.UNKNOWN
        self._median_relative_x: Optional[float] = None
        self._decided = False

    @property
    def independent_count(self) -> int:
        return len(self._relative_x_samples)

    @property
    def latched_lane(self) -> ObjectLane:
        return self._latched_lane

    @property
    def median_relative_x(self) -> Optional[float]:
        return self._median_relative_x

    @property
    def evidence_samples(self) -> tuple[float, ...]:
        return tuple(self._relative_x_samples)

    @property
    def decided(self) -> bool:
        return self._decided

    def expire(self, now: float) -> bool:
        """Reset after a valid-detection gap; return whether reset ran."""

        if not self._valid_time(now) or self._last_detection_at is None:
            return False
        age_s = float(now) - self._last_detection_at
        if age_s < 0.0 or age_s > self.encounter_timeout_s:
            self.reset()
            return True
        return False

    def observe(
        self,
        *,
        object_type: ObjectType,
        box_size: float,
        box_cx: float,
        box_cy: float,
        confidence: float,
        received_at: float,
    ) -> bool:
        """Accept one independent signature and return ``True`` when counted."""

        if object_type not in (ObjectType.FIXED, ObjectType.MOVING):
            return False
        values = (box_size, box_cx, box_cy, confidence, received_at)
        if any(not self._finite_number(value) for value in values):
            return False
        if box_size <= 0.0 or not 0.0 <= confidence <= 1.0:
            return False

        self.expire(received_at)
        if self._object_type not in (ObjectType.UNKNOWN, object_type):
            self.reset()
        self._object_type = object_type

        signature = ObjectDetectionSignature(
            object_type=object_type,
            box_size=float(box_size),
            box_cx=float(box_cx),
            box_cy=float(box_cy),
            confidence=float(confidence),
        )
        if signature == self._last_signature:
            self._last_detection_at = float(received_at)
            return False

        self._last_signature = signature
        self._last_detection_at = float(received_at)
        if self._decided:
            return True

        self._relative_x_samples.append(float(box_cx) - FRAME_CENTER_X_PX)
        self._relative_x_samples = self._relative_x_samples[
            -REQUIRED_INDEPENDENT_DETECTIONS:
        ]
        return True

    def latch_for_entry(self, ego_lane: LaneTarget) -> bool:
        """Latch the rolling last-three median at the entry-area decision."""

        if self._decided:
            return True
        if ego_lane not in (LaneTarget.LANE_ONE, LaneTarget.LANE_TWO):
            return False
        if len(self._relative_x_samples) < REQUIRED_INDEPENDENT_DETECTIONS:
            return False
        self._median_relative_x = statistics.median(self._relative_x_samples)
        self._latched_lane = fallback_lane_from_relative_x(
            ego_lane,
            self._median_relative_x,
        )
        self._decided = True
        return True

    @staticmethod
    def _valid_time(value: float) -> bool:
        return RelativeXObstacleLaneFallback._finite_number(value)

    @staticmethod
    def _finite_number(value: float) -> bool:
        return (
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(float(value))
        )


def fallback_lane_from_relative_x(
    ego_lane: LaneTarget,
    median_relative_x: float,
) -> ObjectLane:
    """Apply the two evidence-backed directional thresholds."""

    if ego_lane is LaneTarget.LANE_ONE:
        return (
            ObjectLane.RIGHT
            if median_relative_x > LANE_ONE_RIGHT_THRESHOLD_PX
            else ObjectLane.LEFT
        )
    if ego_lane is LaneTarget.LANE_TWO:
        return (
            ObjectLane.LEFT
            if median_relative_x < LANE_TWO_LEFT_THRESHOLD_PX
            else ObjectLane.RIGHT
        )
    return ObjectLane.UNKNOWN


def effective_object_lane(
    perception_lane: ObjectLane,
    fallback_lane: ObjectLane,
) -> ObjectLane:
    """Prefer a direct perception lane and use fallback only for UNKNOWN."""

    if perception_lane in (ObjectLane.LEFT, ObjectLane.RIGHT):
        return perception_lane
    if fallback_lane in (ObjectLane.LEFT, ObjectLane.RIGHT):
        return fallback_lane
    return ObjectLane.UNKNOWN


def object_mission_entry_allowed(
    ego_lane: LaneTarget,
    obstacle_lane: ObjectLane,
) -> bool:
    """Allow known same-lane obstacles, preserving CENTER's old behavior."""

    if obstacle_lane not in (ObjectLane.LEFT, ObjectLane.RIGHT):
        return False
    if ego_lane is LaneTarget.LANE_ONE:
        return obstacle_lane is ObjectLane.LEFT
    if ego_lane is LaneTarget.LANE_TWO:
        return obstacle_lane is ObjectLane.RIGHT
    return True
