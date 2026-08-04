"""Pure 2026 finals FSM with only approved transitions enabled."""

from dataclasses import dataclass
from enum import Enum
import math
from typing import Optional

from .cone_entry import ConeEntryConfig, ConeEntryDebouncer, ConeEntryDecision
from .mission_observation import MissionObservation
from .race_context import RaceContext
from .safety_monitor import SafetyDecision


_TIMESTAMP_EPSILON_S = 1e-6


class Mode(str, Enum):
    INIT = "INIT"
    WAIT_GREEN = "WAIT_GREEN"
    LANE_DRIVE = "LANE_DRIVE"
    CONE_DRIVE = "CONE_DRIVE"
    REJOIN = "REJOIN"
    FIXED_AVOID = "FIXED_AVOID"
    OVERTAKE = "OVERTAKE"
    SHORTCUT = "SHORTCUT"
    FINISH = "FINISH"
    STOP = "STOP"


@dataclass(frozen=True)
class Transition:
    source: Mode
    target: Mode
    reason: str

    @property
    def changed(self) -> bool:
        return self.source is not self.target


class _GreenDebouncer:
    def __init__(
        self,
        min_consecutive_frames: int,
        min_duration_s: float,
    ) -> None:
        if min_consecutive_frames < 1:
            raise ValueError("min_consecutive_frames must be at least one")
        if min_duration_s < 0.0:
            raise ValueError("min_duration_s must not be negative")

        self._min_consecutive_frames = min_consecutive_frames
        self._min_duration_s = min_duration_s
        self.reset()

    def reset(self) -> None:
        self._consecutive_frames = 0
        self._first_detected_at: Optional[float] = None

    def update(self, detected: bool, now: float) -> bool:
        if not detected:
            self.reset()
            return False

        if self._consecutive_frames == 0:
            self._first_detected_at = now
        self._consecutive_frames += 1

        duration_s = now - self._first_detected_at
        return (
            self._consecutive_frames >= self._min_consecutive_frames
            and duration_s >= self._min_duration_s
        )


class RaceFSM:
    """Apply approved start, cone-entry, cone-exit, and safety transitions."""

    TERMINAL_STATES = frozenset({Mode.FINISH, Mode.STOP})

    def __init__(
        self,
        initial_state: Mode = Mode.INIT,
        *,
        green_min_consecutive_frames: int = 3,
        green_min_duration_s: float = 0.0,
        cone_entry_config: Optional[ConeEntryConfig] = None,
    ) -> None:
        self.state = initial_state
        self._green = _GreenDebouncer(
            green_min_consecutive_frames,
            green_min_duration_s,
        )
        self._cone_entry = ConeEntryDebouncer(cone_entry_config)
        self._last_cone_drive_message_at: Optional[float] = None
        self.last_cone_entry_decision: Optional[ConeEntryDecision] = None

    @property
    def cone_entry_config(self) -> ConeEntryConfig:
        return self._cone_entry.config

    @property
    def cone_entry_guard(self) -> ConeEntryDebouncer:
        return self._cone_entry

    def step(
        self,
        observation: MissionObservation,
        context: RaceContext,
        safety: SafetyDecision,
    ) -> Transition:
        self.last_cone_entry_decision = None

        if self.state in self.TERMINAL_STATES:
            return self._stay("terminal state")

        if safety.must_stop:
            reason = safety.reason or "safety stop"
            context.stop_reason = reason
            self._green.reset()
            self._cone_entry.deactivate()
            return self._change(
                Mode.STOP,
                reason,
                context,
                observation.now,
            )

        if self.state is Mode.INIT:
            if safety.inputs_ready:
                self._green.reset()
                return self._change(
                    Mode.WAIT_GREEN,
                    "required inputs ready",
                    context,
                    observation.now,
                )
            return self._stay("waiting for required inputs")

        if self.state is Mode.WAIT_GREEN:
            if not safety.inputs_ready:
                self._green.reset()
                return self._stay("waiting for required inputs")

            if self._green.update(
                observation.green_detected,
                observation.now,
            ):
                context.race_started_at = observation.now
                return self._change(
                    Mode.LANE_DRIVE,
                    "green signal debounced",
                    context,
                    observation.now,
                )
            return self._stay("waiting for debounced green")

        if self.state is Mode.LANE_DRIVE:
            decision = self._cone_entry.evaluate(observation)
            self.last_cone_entry_decision = decision
            if decision.triggered:
                context.cone_entered_at = observation.now
                self._last_cone_drive_message_at = (
                    observation.cone_message_received_at
                )
                return self._change(
                    Mode.CONE_DRIVE,
                    "cone entry confirmed",
                    context,
                    observation.now,
                )
            return self._stay(decision.reason)

        if self.state is Mode.CONE_DRIVE:
            if (
                self._accept_new_cone_drive_message(observation)
                and observation.cone_end_flag is True
            ):
                # The entry latch belongs to one cone episode. Rearm it when
                # that episode commits its exit so a later normal lap can
                # qualify independently.
                self._cone_entry.deactivate()
                return self._change(
                    Mode.REJOIN,
                    "fresh cone end flag",
                    context,
                    observation.now,
                )
            return self._stay("waiting for fresh cone end flag")

        # Cone-entry evidence must never accumulate outside LANE_DRIVE. Keep a
        # completed episode rearmed for a future LANE_DRIVE visit.
        self._cone_entry.deactivate()

        # finish_gate_crossed is intentionally ignored. The Gate event source
        # and debounce contract must be fixed before lap counting is enabled.
        #
        # Final shortcut policy contract (not implemented in this phase):
        # LANE_DRIVE transitions directly to SHORTCUT when a left-turn signal
        # is confirmed, current_lap is 2 or 3, shortcut_used is false, and the
        # shortcut-entry-ready input is true. The first valid opportunity is
        # used. shortcut_used becomes true only when that transition commits,
        # so repeated evaluation of one signal cannot consume it twice. There
        # is deliberately no intermediate route-decision mode.
        return self._stay("transition not implemented in this phase")

    def _accept_new_cone_drive_message(
        self,
        observation: MissionObservation,
    ) -> bool:
        """Accept one fresh cone receipt edge, regardless of its flag value."""

        timestamp = observation.cone_message_received_at
        if not self._valid_timestamp(timestamp):
            return False

        if (
            self._last_cone_drive_message_at is not None
            and timestamp <= self._last_cone_drive_message_at
        ):
            return False

        # Consume each new receipt edge once even when it is not end evidence.
        self._last_cone_drive_message_at = timestamp

        now = observation.now
        if not self._valid_timestamp(now):
            return False
        age_s = now - timestamp
        return (
            age_s >= -_TIMESTAMP_EPSILON_S
            and age_s - self.cone_entry_config.max_cone_age_s
            <= _TIMESTAMP_EPSILON_S
        )

    @staticmethod
    def _valid_timestamp(value: Optional[float]) -> bool:
        return (
            value is not None
            and not isinstance(value, bool)
            and isinstance(value, (int, float))
            and math.isfinite(value)
        )

    def _stay(self, reason: str) -> Transition:
        # A self-transition must not modify context.state_entered_at.
        return Transition(self.state, self.state, reason)

    def _change(
        self,
        target: Mode,
        reason: str,
        context: RaceContext,
        now: float,
    ) -> Transition:
        source = self.state
        if target is source:
            return self._stay(reason)

        self.state = target
        context.state_entered_at = now
        return Transition(source, target, reason)
