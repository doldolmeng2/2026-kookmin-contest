"""ROS-independent runtime adapter for the authoritative race FSM."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Any, Callable, Deque, Optional, Sequence

from .control_selector import (
    CommandCandidate,
    ControlDecision,
    ControlCycleResult,
    ControlSelector,
    commit_fsm_then_select_control,
)
from .mission_observation import MissionObservation
from .race_context import RaceContext
from .race_fsm import Mode, RaceFSM, Transition
from .safety_monitor import (
    InputCategory,
    InputRequirement,
    SafetyDecision,
    SafetyMonitor,
)


DEFAULT_CONE_EVENT_QUEUE_CAPACITY = 16

_MOTION_MODES = frozenset(
    {
        Mode.LANE_DRIVE,
        Mode.CONE_DRIVE,
        Mode.REJOIN,
        Mode.FIXED_AVOID,
        Mode.OVERTAKE,
        Mode.SHORTCUT,
    }
)


@dataclass(frozen=True)
class ConeMessageEvent:
    """One validated ``/rubbercone_info`` callback receipt edge."""

    offset: int
    end_flag: bool
    confidence: int
    received_at: float


@dataclass(frozen=True)
class LaneValidityEvent:
    """One future explicit lane-validity callback receipt edge."""

    valid: bool
    received_at: float


@dataclass(frozen=True)
class TrafficMessageEvent:
    detected: bool
    received_at: float


@dataclass(frozen=True)
class ConeEnqueueResult:
    accepted: bool
    warning: Optional[str] = None
    dropped_oldest: bool = False


@dataclass(frozen=True)
class RuntimeCycleResult:
    observation: MissionObservation
    safety: SafetyDecision
    fsm_control: ControlCycleResult
    publish_cone_reset: bool
    discarded_pre_reset_events: int = 0
    discarded_pre_rejoin_lane_events: int = 0

    @property
    def transition(self) -> Transition:
        return self.fsm_control.transition

    @property
    def control(self) -> ControlDecision:
        return self.fsm_control.control


def dispatch_cone_reset(
    cycle: RuntimeCycleResult,
    publish_reset: Callable[[], None],
) -> bool:
    """Dispatch one reset command only for a committed cone-entry edge."""

    if not cycle.publish_cone_reset:
        return False
    publish_reset()
    return True


def runtime_safety_monitor() -> SafetyMonitor:
    """Build state-specific safety requirements from existing ROS inputs."""

    return SafetyMonitor(
        {
            Mode.INIT: (
                InputRequirement(
                    InputCategory.PERCEPTION,
                    "traffic_detection",
                    0.5,
                ),
                InputRequirement(
                    InputCategory.PERCEPTION,
                    "lane_offset",
                    0.5,
                ),
                InputRequirement(InputCategory.SENSOR, "scan", 0.5),
            ),
            Mode.WAIT_GREEN: (
                InputRequirement(
                    InputCategory.PERCEPTION,
                    "traffic_detection",
                    0.5,
                ),
            ),
            Mode.LANE_DRIVE: (
                InputRequirement(
                    InputCategory.PERCEPTION,
                    "lane_offset",
                    0.5,
                ),
            ),
            # A missing detector reset must leave CONE_DRIVE waiting for a
            # fresh zero, not force an immediate terminal STOP. Scan loss is
            # still a terminal sensor failure; cone command freshness is
            # enforced independently by ControlSelector.
            Mode.CONE_DRIVE: (
                InputRequirement(InputCategory.SENSOR, "scan", 0.5),
            ),
        },
    )


class RaceRuntimeAdapter:
    """Own the sole runtime ``RaceFSM`` and consume callback edges once."""

    def __init__(
        self,
        *,
        fsm: Optional[RaceFSM] = None,
        context: Optional[RaceContext] = None,
        safety_monitor: Optional[SafetyMonitor] = None,
        selector: Optional[ControlSelector] = None,
        cone_queue_capacity: int = DEFAULT_CONE_EVENT_QUEUE_CAPACITY,
    ) -> None:
        if (
            isinstance(cone_queue_capacity, bool)
            or not isinstance(cone_queue_capacity, int)
            or cone_queue_capacity < 1
        ):
            raise ValueError("cone_queue_capacity must be a positive integer")

        self.fsm = fsm if fsm is not None else RaceFSM()
        self.context = context if context is not None else RaceContext()
        self.safety_monitor = (
            safety_monitor if safety_monitor is not None else SafetyMonitor()
        )
        self.selector = selector if selector is not None else ControlSelector()
        self.cone_queue_capacity = cone_queue_capacity

        self._cone_events: Deque[ConeMessageEvent] = deque()
        self._lane_validity_events: Deque[LaneValidityEvent] = deque()
        self._traffic_events: Deque[TrafficMessageEvent] = deque()
        self.cone_queue_overflow_count = 0

        self.sensor_received_at: dict[str, float] = {}
        self.perception_received_at: dict[str, float] = {}
        self.green_detected = False
        self.latest_lane_offset: Optional[int] = None
        self.latest_lane_received_at: Optional[float] = None
        self.latest_cone_event: Optional[ConeMessageEvent] = None

    @property
    def pending_cone_event_count(self) -> int:
        return len(self._cone_events)

    def record_scan(self, received_at: float) -> bool:
        if not self._valid_timestamp(received_at):
            return False
        self.sensor_received_at["scan"] = received_at
        return True

    def record_lane_offset(self, offset: int, received_at: float) -> bool:
        if not self._valid_int(offset) or not self._valid_timestamp(received_at):
            return False
        self.latest_lane_offset = offset
        self.latest_lane_received_at = received_at
        self.perception_received_at["lane_offset"] = received_at
        return True

    def record_traffic(self, detected: bool, received_at: float) -> bool:
        if not isinstance(detected, bool) or not self._valid_timestamp(received_at):
            return False
        if len(self._traffic_events) == self.cone_queue_capacity:
            self._traffic_events.popleft()
        self._traffic_events.append(
            TrafficMessageEvent(detected=detected, received_at=received_at),
        )
        self.green_detected = detected
        self.perception_received_at["traffic_detection"] = received_at
        return True

    def record_lane_validity(self, valid: bool, received_at: float) -> bool:
        """Queue one explicit validity edge for the future lane publisher."""

        if not isinstance(valid, bool) or not self._valid_timestamp(received_at):
            return False
        if len(self._lane_validity_events) == self.cone_queue_capacity:
            self._lane_validity_events.popleft()
        self._lane_validity_events.append(
            LaneValidityEvent(valid=valid, received_at=received_at),
        )
        self.perception_received_at["lane_validity"] = received_at
        return True

    def record_cone_message(
        self,
        data: Sequence[Any],
        received_at: float,
    ) -> ConeEnqueueResult:
        """Validate and queue one callback without reusing an older event."""

        try:
            values = list(data)
        except TypeError:
            values = []

        if len(values) < 3:
            return ConeEnqueueResult(
                False,
                "rubbercone_info requires [offset, end_flag, confidence]",
            )
        if not self._valid_timestamp(received_at):
            return ConeEnqueueResult(False, "invalid rubbercone receipt timestamp")

        offset, end_flag, confidence = values[:3]
        if not self._valid_int(offset):
            return ConeEnqueueResult(False, "invalid rubbercone offset")
        if not self._valid_int(end_flag) or end_flag not in (0, 1):
            return ConeEnqueueResult(False, "rubbercone end_flag must be 0 or 1")
        if (
            not self._valid_int(confidence)
            or confidence < 0
            or confidence > 100
        ):
            return ConeEnqueueResult(
                False,
                "rubbercone confidence must be an integer from 0 to 100",
            )

        event = ConeMessageEvent(
            offset=offset,
            end_flag=bool(end_flag),
            confidence=confidence,
            received_at=received_at,
        )
        dropped_oldest = False
        if len(self._cone_events) == self.cone_queue_capacity:
            # Prefer the newest detector output. Losing an older 0 edge can only
            # leave cone exit disarmed, which is the fail-safe outcome.
            self._cone_events.popleft()
            self.cone_queue_overflow_count += 1
            dropped_oldest = True
        self._cone_events.append(event)
        self.latest_cone_event = event
        self.perception_received_at["rubbercone_info"] = received_at

        warning = (
            "rubbercone event queue full; dropped oldest event"
            if dropped_oldest
            else None
        )
        return ConeEnqueueResult(True, warning, dropped_oldest)

    def step(
        self,
        now: float,
        *,
        lane: Optional[CommandCandidate] = None,
        cone: Optional[CommandCandidate] = None,
        fault_reason: Optional[str] = None,
    ) -> RuntimeCycleResult:
        """Consume at most one cone event and run one authoritative FSM cycle."""

        if self.context.state_entered_at is None and self._valid_timestamp(now):
            self.context.state_entered_at = now

        cone_event = self._cone_events.popleft() if self._cone_events else None
        lane_validity_event = (
            self._lane_validity_events.popleft()
            if self._lane_validity_events
            else None
        )
        traffic_event = (
            self._traffic_events.popleft()
            if self._traffic_events
            else None
        )
        observation = MissionObservation(
            now=now,
            sensor_received_at=self.sensor_received_at,
            perception_received_at=self.perception_received_at,
            green_detected=(
                traffic_event.detected if traffic_event is not None else False
            ),
            traffic_message_received_at=(
                traffic_event.received_at if traffic_event is not None else None
            ),
            # There is no dedicated lane-validity input in the current graph.
            lane_valid=(
                lane_validity_event.valid
                if lane_validity_event is not None
                else False
            ),
            lane_valid_received_at=(
                lane_validity_event.received_at
                if lane_validity_event is not None
                else None
            ),
            cone_detected=(
                cone_event is not None
                and not cone_event.end_flag
                and cone_event.confidence > 0
            ),
            cone_finished=(
                cone_event is not None and cone_event.end_flag
            ),
            cone_confidence=(
                cone_event.confidence if cone_event is not None else None
            ),
            cone_end_flag=(
                cone_event.end_flag if cone_event is not None else None
            ),
            cone_message_received_at=(
                cone_event.received_at if cone_event is not None else None
            ),
            scan_received_at=self.sensor_received_at.get("scan"),
        )
        safety = self.safety_monitor.evaluate(
            self.fsm.state,
            self.context,
            observation,
            motion_enabled=self.fsm.state in _MOTION_MODES,
            fault_reason=fault_reason,
        )
        fsm_control = commit_fsm_then_select_control(
            self.fsm,
            observation,
            self.context,
            safety,
            self.selector,
            lane=lane,
            cone=cone,
        )
        transition = fsm_control.transition
        publish_cone_reset = (
            transition.changed
            and transition.source is Mode.LANE_DRIVE
            and transition.target is Mode.CONE_DRIVE
        )

        discarded = 0
        if publish_cone_reset:
            # Events already queued before the reset publish belong to the old
            # detector session and must never arm the new exit handshake.
            discarded = len(self._cone_events)
            self._cone_events.clear()
            self.latest_cone_event = None
            self.perception_received_at.pop("rubbercone_info", None)

        discarded_lane = 0
        if (
            transition.changed
            and transition.source is Mode.CONE_DRIVE
            and transition.target is Mode.REJOIN
        ):
            discarded_lane = len(self._lane_validity_events)
            self._lane_validity_events.clear()
            self.perception_received_at.pop("lane_validity", None)

        return RuntimeCycleResult(
            observation=observation,
            safety=safety,
            fsm_control=fsm_control,
            publish_cone_reset=publish_cone_reset,
            discarded_pre_reset_events=discarded,
            discarded_pre_rejoin_lane_events=discarded_lane,
        )

    @staticmethod
    def _valid_int(value: Any) -> bool:
        return isinstance(value, int) and not isinstance(value, bool)

    @staticmethod
    def _valid_timestamp(value: Any) -> bool:
        return (
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(value)
        )
