"""Typed mission contracts shared by the pure FSM and ROS adapter."""

from dataclasses import dataclass
from enum import Enum, IntEnum
from typing import Optional


class ObjectLane(Enum):
    """Lane label emitted by the existing ten-field ``/object_info`` topic."""

    UNKNOWN = 0
    LEFT = 1
    RIGHT = 2


class LaneTarget(Enum):
    """Two-lane target contract consumed by the existing lane detector."""

    LANE_ONE = 0
    LANE_TWO = 1


class RouteTrafficSignal(IntEnum):
    """Internal route-traffic contract; no production ROS topic owns it yet."""

    UNKNOWN = 0
    RED_AMBER = 1
    STRAIGHT = 2
    LEFT = 3


def opposite_lane_target(object_lane: ObjectLane) -> Optional[LaneTarget]:
    """Translate an object label to the opposite lane detector target."""

    if object_lane is ObjectLane.LEFT:
        return LaneTarget.LANE_TWO
    if object_lane is ObjectLane.RIGHT:
        return LaneTarget.LANE_ONE
    return None


@dataclass(frozen=True)
class ObjectSnapshot:
    """Validated minimum subset of one existing ``/object_info`` message."""

    exists: bool
    distance: float
    lane: ObjectLane
    received_at: float


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
