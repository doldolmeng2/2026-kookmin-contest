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
)
from .mission_observation import MissionObservation
from .mission_types import (
    LaneChangeStateEvent,
    LaneTarget,
    ObjectLane,
    ObjectSnapshot,
    RouteTrafficEvent,
    RouteTrafficSignal,
    opposite_lane_target,
)
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
class InputRecordResult:
    accepted: bool
    warning: Optional[str] = None


@dataclass(frozen=True)
class MissionEdgeEvent:
    received_at: float


@dataclass
class LaneActionStatus:
    """Action-level state; it deliberately does not add an FSM mode."""

    mode: Optional[Mode] = None
    safe_to_drive: bool = False
    pending: bool = False
    completed: bool = False
    target: Optional[LaneTarget] = None
    started_at: Optional[float] = None
    completed_at: Optional[float] = None


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
        object_max_age_s: float = 0.25,
        lane_change_max_age_s: float = 0.25,
    ) -> None:
        if (
            isinstance(cone_queue_capacity, bool)
            or not isinstance(cone_queue_capacity, int)
            or cone_queue_capacity < 1
        ):
            raise ValueError("cone_queue_capacity must be a positive integer")
        for name, value in (
            ("object_max_age_s", object_max_age_s),
            ("lane_change_max_age_s", lane_change_max_age_s),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value < 0.0
            ):
                raise ValueError(f"{name} must be finite and non-negative")

        self.fsm = fsm if fsm is not None else RaceFSM()
        self.context = context if context is not None else RaceContext()
        self.safety_monitor = (
            safety_monitor if safety_monitor is not None else SafetyMonitor()
        )
        self.selector = selector if selector is not None else ControlSelector()
        self.cone_queue_capacity = cone_queue_capacity
        self.object_max_age_s = float(object_max_age_s)
        self.lane_change_max_age_s = float(lane_change_max_age_s)

        self._cone_events: Deque[ConeMessageEvent] = deque()
        self._lane_validity_events: Deque[LaneValidityEvent] = deque()
        self._traffic_events: Deque[TrafficMessageEvent] = deque()
        self._lane_change_events: Deque[LaneChangeStateEvent] = deque()
        self._route_traffic_events: Deque[RouteTrafficEvent] = deque()
        self._mission_events: dict[str, Deque[MissionEdgeEvent]] = {
            "fixed_zone_entry": deque(),
            "fixed_zone_exit": deque(),
            "overtake_complete": deque(),
            "shortcut_complete": deque(),
        }
        self._last_internal_event_at: dict[str, float] = {}
        self.cone_queue_overflow_count = 0

        self.sensor_received_at: dict[str, float] = {}
        self.perception_received_at: dict[str, float] = {}
        self.green_detected = False
        self.latest_lane_offset: Optional[int] = None
        self.latest_lane_received_at: Optional[float] = None
        self.latest_cone_event: Optional[ConeMessageEvent] = None
        self.latest_object_snapshot: Optional[ObjectSnapshot] = None
        self._last_lane_change_received_at: Optional[float] = None
        self._lane_change_success_active = False
        self.traffic_stop_override = False
        self.lane_action = LaneActionStatus()

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

    def record_object_info(
        self,
        data: Sequence[Any],
        received_at: float,
    ) -> InputRecordResult:
        """Validate the object_info payload and retain its typed subset.

        앞 10필드는 고정 계약이고, 그 뒤는 추가 필드다. object_detection이
        측면 LiDAR 거리(side_left, side_right)를 11·12번째로 붙였으므로
        길이를 '정확히 10'이 아니라 '10 이상'으로 본다. 이 어댑터가 쓰는 것은
        앞 10필드뿐이고, 추가 필드는 main_node가 직접 읽는다.
        """

        try:
            values = list(data)
        except TypeError:
            values = []
        if len(values) < 10:
            return InputRecordResult(False, "object_info requires at least 10 fields")
        if not self._valid_timestamp(received_at):
            return InputRecordResult(False, "invalid object_info receipt timestamp")

        # 검증은 계약된 앞 10필드에만 적용한다. 뒤에 붙는 측면 LiDAR 거리는
        # 옆에 아무것도 없으면 정당하게 inf라, 여기서 함께 검사하면 정상
        # 메시지가 통째로 거부된다.
        values = values[:10]
        if any(
            not isinstance(value, (int, float)) or isinstance(value, bool)
            for value in values
        ):
            return InputRecordResult(False, "object_info fields must be numbers")

        numeric_values = [float(value) for value in values]
        if any(math.isnan(value) for value in numeric_values):
            return InputRecordResult(False, "object_info fields must not contain NaN")

        exists_value = numeric_values[0]
        if exists_value not in (0.0, 1.0):
            return InputRecordResult(False, "object_info exists must be 0 or 1")
        lane_value = numeric_values[9]
        if not math.isfinite(lane_value) or not lane_value.is_integer():
            return InputRecordResult(False, "object_info lane label must be an integer")
        try:
            lane = ObjectLane(int(lane_value))
        except ValueError:
            return InputRecordResult(False, "object_info lane label must be 0, 1, or 2")

        distance = numeric_values[1]
        if exists_value == 1.0:
            if any(not math.isfinite(value) for value in numeric_values):
                return InputRecordResult(
                    False,
                    "detected object_info fields must be finite numbers",
                )
            if distance < 0.0:
                return InputRecordResult(
                    False,
                    "object_info distance must be non-negative",
                )
            snapshot_distance: Optional[float] = distance
            snapshot_lane = lane
        else:
            # The existing detector intentionally emits +inf for min_dist when
            # no LiDAR cluster exists.  Keep the heartbeat/receipt, but do not
            # expose stale bbox/lane or the sentinel as actionable evidence.
            if distance < 0.0 or distance == -math.inf:
                return InputRecordResult(
                    False,
                    "absent object_info distance must be non-negative or +inf",
                )
            if any(not math.isfinite(value) for value in numeric_values[2:9]):
                return InputRecordResult(
                    False,
                    "absent object_info metadata must be finite numbers",
                )
            snapshot_distance = None
            snapshot_lane = ObjectLane.UNKNOWN

        self.latest_object_snapshot = ObjectSnapshot(
            exists=bool(exists_value),
            distance=snapshot_distance,
            lane=snapshot_lane,
            received_at=received_at,
        )
        self.perception_received_at["object_info"] = received_at
        return InputRecordResult(True)

    def record_lane_change_state(
        self,
        data: Sequence[Any],
        received_at: float,
    ) -> InputRecordResult:
        """Queue one validated feedback sample and deduplicate its success pulse."""

        try:
            values = list(data)
        except TypeError:
            values = []
        if len(values) != 2:
            return InputRecordResult(
                False,
                "lane_change_state requires [changing, success]",
            )
        if not self._valid_timestamp(received_at):
            return InputRecordResult(
                False,
                "invalid lane_change_state receipt timestamp",
            )
        if (
            self._last_lane_change_received_at is not None
            and received_at <= self._last_lane_change_received_at
        ):
            return InputRecordResult(
                False,
                "duplicate or regressed lane_change_state receipt",
            )
        if any(not self._valid_binary_int(value) for value in values):
            return InputRecordResult(
                False,
                "lane_change_state fields must be 0 or 1",
            )

        changing = bool(values[0])
        success = bool(values[1])
        success_edge = success and not self._lane_change_success_active
        self._lane_change_success_active = success
        self._last_lane_change_received_at = received_at
        if len(self._lane_change_events) == self.cone_queue_capacity:
            self._lane_change_events.popleft()
        self._lane_change_events.append(
            LaneChangeStateEvent(
                changing=changing,
                success=success,
                success_edge=success_edge,
                received_at=received_at,
            )
        )
        self.perception_received_at["lane_change_state"] = received_at
        return InputRecordResult(True)

    def record_route_traffic(
        self,
        signal: RouteTrafficSignal,
        received_at: float,
        *,
        encounter_started: bool = False,
    ) -> InputRecordResult:
        """Test/integration seam for a route source that production lacks."""

        if isinstance(signal, bool):
            return InputRecordResult(False, "invalid route traffic signal")
        try:
            typed_signal = RouteTrafficSignal(signal)
        except (TypeError, ValueError):
            return InputRecordResult(False, "invalid route traffic signal")
        if not isinstance(encounter_started, bool):
            return InputRecordResult(False, "encounter_started must be boolean")
        if not self._valid_timestamp(received_at):
            return InputRecordResult(False, "invalid route traffic timestamp")
        if len(self._route_traffic_events) == self.cone_queue_capacity:
            self._route_traffic_events.popleft()
        self._route_traffic_events.append(
            RouteTrafficEvent(typed_signal, encounter_started, received_at)
        )
        return InputRecordResult(True)

    def record_fixed_zone_entry(self, received_at: float) -> InputRecordResult:
        return self._record_mission_edge("fixed_zone_entry", received_at)

    def record_fixed_zone_exit(self, received_at: float) -> InputRecordResult:
        return self._record_mission_edge("fixed_zone_exit", received_at)

    def record_overtake_complete(self, received_at: float) -> InputRecordResult:
        return self._record_mission_edge("overtake_complete", received_at)

    def record_shortcut_complete(self, received_at: float) -> InputRecordResult:
        return self._record_mission_edge("shortcut_complete", received_at)

    def _record_mission_edge(
        self,
        name: str,
        received_at: float,
    ) -> InputRecordResult:
        if not self._valid_timestamp(received_at):
            return InputRecordResult(False, f"invalid {name} timestamp")
        previous = self._last_internal_event_at.get(name)
        if previous is not None and received_at <= previous:
            return InputRecordResult(False, f"duplicate or regressed {name} edge")
        self._last_internal_event_at[name] = received_at
        queue = self._mission_events[name]
        if len(queue) == self.cone_queue_capacity:
            queue.popleft()
        queue.append(MissionEdgeEvent(received_at))
        return InputRecordResult(True)

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
        lane_change_event = (
            self._lane_change_events.popleft()
            if self._lane_change_events
            else None
        )
        route_traffic_event = (
            self._route_traffic_events.popleft()
            if self._route_traffic_events
            else None
        )
        mission_events = {
            name: queue.popleft() if queue else None
            for name, queue in self._mission_events.items()
        }
        object_snapshot = self.latest_object_snapshot

        if route_traffic_event is not None and self._event_is_fresh(
            route_traffic_event.received_at,
            now,
            self.fsm.mission_event_max_age_s,
        ):
            if route_traffic_event.signal is RouteTrafficSignal.RED_AMBER:
                self.traffic_stop_override = True
            elif route_traffic_event.signal in (
                RouteTrafficSignal.STRAIGHT,
                RouteTrafficSignal.LEFT,
            ):
                self.traffic_stop_override = False

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
            object_exists=(
                object_snapshot.exists if object_snapshot is not None else False
            ),
            object_distance=(
                object_snapshot.distance if object_snapshot is not None else None
            ),
            object_lane=(
                object_snapshot.lane
                if object_snapshot is not None
                else ObjectLane.UNKNOWN
            ),
            object_received_at=(
                object_snapshot.received_at if object_snapshot is not None else None
            ),
            lane_change_changing=(
                lane_change_event.changing
                if lane_change_event is not None
                else False
            ),
            lane_change_success=(
                lane_change_event.success
                if lane_change_event is not None
                else False
            ),
            lane_change_success_edge=(
                lane_change_event.success_edge
                if lane_change_event is not None
                else False
            ),
            lane_change_received_at=(
                lane_change_event.received_at
                if lane_change_event is not None
                else None
            ),
            fixed_zone_entered=mission_events["fixed_zone_entry"] is not None,
            fixed_zone_entry_received_at=(
                mission_events["fixed_zone_entry"].received_at
                if mission_events["fixed_zone_entry"] is not None
                else None
            ),
            fixed_zone_exited=mission_events["fixed_zone_exit"] is not None,
            fixed_zone_exit_received_at=(
                mission_events["fixed_zone_exit"].received_at
                if mission_events["fixed_zone_exit"] is not None
                else None
            ),
            overtake_complete=mission_events["overtake_complete"] is not None,
            overtake_complete_received_at=(
                mission_events["overtake_complete"].received_at
                if mission_events["overtake_complete"] is not None
                else None
            ),
            shortcut_complete=mission_events["shortcut_complete"] is not None,
            shortcut_complete_received_at=(
                mission_events["shortcut_complete"].received_at
                if mission_events["shortcut_complete"] is not None
                else None
            ),
            route_traffic_signal=(
                route_traffic_event.signal
                if route_traffic_event is not None
                else RouteTrafficSignal.UNKNOWN
            ),
            route_traffic_received_at=(
                route_traffic_event.received_at
                if route_traffic_event is not None
                else None
            ),
            traffic_encounter_started=(
                route_traffic_event.encounter_started
                if route_traffic_event is not None
                else False
            ),
            traffic_encounter_received_at=(
                route_traffic_event.received_at
                if route_traffic_event is not None
                and route_traffic_event.encounter_started
                else None
            ),
        )
        safety = self.safety_monitor.evaluate(
            self.fsm.state,
            self.context,
            observation,
            motion_enabled=self.fsm.state in _MOTION_MODES,
            fault_reason=fault_reason,
        )
        transition = self.fsm.step(observation, self.context, safety)
        self._update_lane_action(observation, transition)
        control = self.selector.select(
            self.fsm.state,
            observation.now,
            lane=lane,
            cone=cone,
            mission_lane_authorized=self.lane_action.safe_to_drive,
            traffic_hold=self.traffic_stop_override,
        )
        fsm_control = ControlCycleResult(transition=transition, control=control)
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

    def _update_lane_action(
        self,
        observation: MissionObservation,
        transition: Transition,
    ) -> None:
        mode = self.fsm.state
        action_modes = (Mode.FIXED_AVOID, Mode.OVERTAKE)
        if mode not in action_modes:
            self.lane_action = LaneActionStatus()
            return
        if self.lane_action.mode is not mode or transition.changed:
            self.lane_action = LaneActionStatus(mode=mode)

        action = self.lane_action
        snapshot_at = observation.object_received_at
        object_fresh = (
            self._event_is_fresh(
                snapshot_at,
                observation.now,
                self.object_max_age_s,
            )
            and self.context.state_entered_at is not None
            and snapshot_at is not None
            and snapshot_at > self.context.state_entered_at
        )
        if not object_fresh:
            action.safe_to_drive = False
        elif not observation.object_exists:
            action.safe_to_drive = True
        else:
            target = opposite_lane_target(observation.object_lane)
            action.safe_to_drive = target is not None
            if (
                target is not None
                and target != self.context.lane_target
                and not action.pending
                and not action.completed
            ):
                self.context.lane_target = target
                action.target = target
                action.pending = True
                action.started_at = observation.now

        success_at = observation.lane_change_received_at
        if (
            action.pending
            and observation.lane_change_success_edge
            and action.started_at is not None
            and success_at is not None
            and success_at > action.started_at
            and self._event_is_fresh(
                success_at,
                observation.now,
                self.lane_change_max_age_s,
            )
        ):
            action.pending = False
            action.completed = True
            action.completed_at = success_at

    @staticmethod
    def _event_is_fresh(
        received_at: Optional[float],
        now: float,
        max_age_s: float,
    ) -> bool:
        if not RaceRuntimeAdapter._valid_timestamp(received_at):
            return False
        if not RaceRuntimeAdapter._valid_timestamp(now):
            return False
        age_s = now - received_at
        return 0.0 <= age_s <= max_age_s

    @staticmethod
    def _valid_int(value: Any) -> bool:
        return isinstance(value, int) and not isinstance(value, bool)

    @staticmethod
    def _valid_binary_int(value: Any) -> bool:
        if isinstance(value, bool):
            return False
        return isinstance(value, int) and value in (0, 1)

    @staticmethod
    def _valid_number(value: Any) -> bool:
        return (
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(value)
        )

    @staticmethod
    def _valid_timestamp(value: Any) -> bool:
        return (
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(value)
        )
