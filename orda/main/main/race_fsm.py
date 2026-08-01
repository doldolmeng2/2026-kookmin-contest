"""Pure 2026 finals FSM with only approved transitions enabled."""

from dataclasses import dataclass
from enum import Enum
from typing import Optional

from .cone_entry import ConeEntryConfig, ConeEntryDebouncer, ConeEntryDecision
from .mission_observation import MissionObservation
from .race_context import RaceContext
from .safety_monitor import SafetyDecision


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
    """Apply approved start, cone-entry, and safety transitions."""

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
                return self._change(
                    Mode.CONE_DRIVE,
                    "cone entry confirmed",
                    context,
                    observation.now,
                )
            return self._stay(decision.reason)

        # Cone-entry evidence must never accumulate outside LANE_DRIVE. Keep a
        # committed CONE_DRIVE episode latched so it cannot be consumed twice.
        if self.state is not Mode.CONE_DRIVE:
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
