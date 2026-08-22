"""Race-wide mutable context for the ROS-independent FSM layer."""

from dataclasses import dataclass
from typing import ClassVar, Optional

from .mission_types import LaneTarget, PendingRouteAction


@dataclass(init=False)
class RaceContext:
    """Canonical state that must survive individual perception observations."""

    TOTAL_LAPS: ClassVar[int] = 3

    completed_laps: int
    shortcut_lap: Optional[int]
    lane_target: LaneTarget
    race_started_at: Optional[float]
    state_entered_at: Optional[float]
    cone_entered_at: Optional[float]
    last_traffic_encounter_at: Optional[float]
    pending_route_action: Optional[PendingRouteAction]
    stop_reason: Optional[str]

    def __init__(
        self,
        completed_laps: int = 0,
        shortcut_lap: Optional[int] = None,
        lane_target: LaneTarget = LaneTarget.CENTER,
        race_started_at: Optional[float] = None,
        state_entered_at: Optional[float] = None,
        cone_entered_at: Optional[float] = None,
        last_traffic_encounter_at: Optional[float] = None,
        pending_route_action: Optional[PendingRouteAction] = None,
        stop_reason: Optional[str] = None,
        *,
        # Compatibility-only constructor aliases. They are never stored as a
        # second mutable source of truth.
        finish_gate_passes: Optional[int] = None,
        last_gate_event_at: Optional[float] = None,
    ) -> None:
        if finish_gate_passes is not None:
            if completed_laps != 0 and completed_laps != finish_gate_passes:
                raise ValueError("conflicting completed-lap values")
            completed_laps = finish_gate_passes
        if last_gate_event_at is not None:
            if (
                last_traffic_encounter_at is not None
                and last_traffic_encounter_at != last_gate_event_at
            ):
                raise ValueError("conflicting traffic-encounter timestamps")
            last_traffic_encounter_at = last_gate_event_at

        self.completed_laps = self._validate_completed_laps(completed_laps)
        self.shortcut_lap = self._validate_shortcut_lap(shortcut_lap)
        self.lane_target = self._validate_lane_target(lane_target)
        self.race_started_at = race_started_at
        self.state_entered_at = state_entered_at
        self.cone_entered_at = cone_entered_at
        self.last_traffic_encounter_at = last_traffic_encounter_at
        self.pending_route_action = self._validate_pending_route_action(
            pending_route_action
        )
        self.stop_reason = stop_reason

    @property
    def lap_count(self) -> int:
        return self.completed_laps

    @property
    def current_lap(self) -> int:
        return min(self.completed_laps + 1, self.TOTAL_LAPS)

    @property
    def shortcut_used(self) -> bool:
        return self.shortcut_lap is not None

    @property
    def on_shortcut_lap(self) -> bool:
        return self.shortcut_lap == self.current_lap

    @property
    def finish_gate_passes(self) -> int:
        """Compatibility alias for code written before the traffic boundary."""

        return self.completed_laps

    @finish_gate_passes.setter
    def finish_gate_passes(self, value: int) -> None:
        self.completed_laps = self._validate_completed_laps(value)

    @property
    def last_gate_event_at(self) -> Optional[float]:
        """Compatibility alias; the organizer Gate is not a vehicle input."""

        return self.last_traffic_encounter_at

    @last_gate_event_at.setter
    def last_gate_event_at(self, value: Optional[float]) -> None:
        self.last_traffic_encounter_at = value

    @classmethod
    def _validate_completed_laps(cls, value: int) -> int:
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError("completed_laps must be an integer")
        if not 0 <= value <= cls.TOTAL_LAPS:
            raise ValueError("completed_laps must be between zero and three")
        return value

    @staticmethod
    def _validate_shortcut_lap(value: Optional[int]) -> Optional[int]:
        if value not in (None, 2, 3):
            raise ValueError("shortcut_lap must be None, 2, or 3")
        return value

    @staticmethod
    def _validate_lane_target(value: LaneTarget) -> LaneTarget:
        if isinstance(value, bool):
            raise ValueError("invalid lane_target")
        try:
            return LaneTarget(value)
        except (TypeError, ValueError) as exc:
            raise ValueError("invalid lane_target") from exc

    @staticmethod
    def _validate_pending_route_action(
        value: Optional[PendingRouteAction],
    ) -> Optional[PendingRouteAction]:
        if value is None:
            return None
        if isinstance(value, bool):
            raise ValueError("invalid pending_route_action")
        try:
            return PendingRouteAction(value)
        except (TypeError, ValueError) as exc:
            raise ValueError("invalid pending_route_action") from exc
