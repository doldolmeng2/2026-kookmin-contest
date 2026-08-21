"""Typed mission contracts shared by the pure FSM and ROS adapter."""

from dataclasses import dataclass
from enum import Enum, IntEnum
from typing import Optional


class ObjectLane(Enum):
    """Lane label emitted in ``/object_info`` field 10 (index 9)."""

    # object_detection 의 lane_label 과 같은 값이다 (0=중앙선 근처, 1=왼쪽, 2=오른쪽).
    UNKNOWN = 0
    CENTER = 0   # README 표기용 별칭. 값은 UNKNOWN 과 같다.
    LEFT = 1
    RIGHT = 2


class LaneTarget(Enum):
    """Lane target contract consumed by the existing lane detector."""

    # car_lane 과 같은 정수 규약을 쓴다 (README): 0=중앙, 1=왼쪽, 2=오른쪽.
    # 기본 주행은 중앙이고, 장애물을 피할 때만 좌/우 차선으로 확정해서 들어간다.
    CENTER = 0
    LANE_ONE = 1
    LANE_TWO = 2


class RouteTrafficSignal(IntEnum):
    """Internal route-traffic contract; no production ROS topic owns it yet."""

    UNKNOWN = 0
    RED_AMBER = 1
    STRAIGHT = 2
    LEFT = 3


class ObjectType(IntEnum):
    """Semantic class carried by ``/object_info`` field 11."""

    UNKNOWN = -1
    FIXED = 0
    MOVING = 1


def opposite_lane_target(object_lane: ObjectLane) -> Optional[LaneTarget]:
    """Translate an object label to the opposite lane detector target."""

    if object_lane is ObjectLane.LEFT:
        return LaneTarget.LANE_TWO
    if object_lane is ObjectLane.RIGHT:
        return LaneTarget.LANE_ONE
    return None


@dataclass(frozen=True)
class ObjectSnapshot:
    """Validated subset of the backward-compatible ``/object_info`` message."""

    exists: bool
    distance: Optional[float]
    lane: ObjectLane
    received_at: float
    # YOLO 바운딩 박스 면적(px^2). exists/distance 는 LiDAR 산출물이지만 이 값과
    # lane 은 카메라 단독으로 나온다. 회피 방향은 이 둘만 보고 정한다.
    box_px: float = 0.0
    object_type: ObjectType = ObjectType.UNKNOWN
    confidence: float = 0.0


@dataclass(frozen=True)
class LaneChangeStateEvent:
    """Validated lane-change feedback with a deduplicated success edge."""

    changing: bool
    success: bool
    success_edge: bool
    received_at: float


@dataclass(frozen=True)
class RouteTrafficEvent:
    """Internal route signal and optional one-shot lap encounter edge."""

    signal: RouteTrafficSignal
    encounter_started: bool
    received_at: float
