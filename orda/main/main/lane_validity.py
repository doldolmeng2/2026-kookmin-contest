"""Pure debounce guard for a future explicit lane-validity input."""

from dataclasses import dataclass
import math
from typing import Optional

from .mission_observation import MissionObservation


_TIMESTAMP_EPSILON_S = 1e-6


@dataclass(frozen=True)
class LaneValidityConfig:
    """Receipt-time contract required to leave REJOIN safely."""

    min_messages: int = 3
    min_duration_s: float = 0.2
    max_age_s: float = 0.25

    def __post_init__(self) -> None:
        if (
            isinstance(self.min_messages, bool)
            or not isinstance(self.min_messages, int)
            or self.min_messages < 1
        ):
            raise ValueError("min_messages must be an integer of at least one")
        for name, value in (
            ("min_duration_s", self.min_duration_s),
            ("max_age_s", self.max_age_s),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value < 0.0
            ):
                raise ValueError(f"{name} must be a finite non-negative number")


@dataclass(frozen=True)
class LaneValidityDecision:
    triggered: bool
    reason: str
    new_message: bool = False
    sequence_reset: bool = False
    first_valid_at: Optional[float] = None
    valid_message_count: int = 0
    elapsed_s: float = 0.0


class LaneValidityDebouncer:
    """Count only fresh explicit validity messages received after REJOIN."""

    def __init__(self, config: Optional[LaneValidityConfig] = None) -> None:
        self.config = config or LaneValidityConfig()
        self.first_valid_at: Optional[float] = None
        self.valid_message_count = 0
        self.last_message_at: Optional[float] = None
        self.triggered = False

    def deactivate(self) -> None:
        self._reset_sequence()

    def evaluate(
        self,
        observation: MissionObservation,
        state_entered_at: Optional[float],
    ) -> LaneValidityDecision:
        timestamp = observation.lane_valid_received_at
        if timestamp is None:
            return self._decision(False, "no new lane-validity message")
        if not self._valid_timestamp(timestamp):
            return self._reset_decision("invalid lane-validity timestamp")
        if self.last_message_at is not None:
            if timestamp == self.last_message_at:
                return self._decision(False, "duplicate lane-validity timestamp")
            if timestamp < self.last_message_at:
                return self._reset_decision(
                    "lane-validity timestamp regression",
                )
        self.last_message_at = timestamp

        if not self._valid_timestamp(observation.now):
            return self._reset_decision(
                "invalid observation timestamp",
                new_message=True,
            )
        if not self._valid_timestamp(state_entered_at):
            return self._reset_decision(
                "invalid REJOIN entry timestamp",
                new_message=True,
            )

        age_s = observation.now - timestamp
        if (
            timestamp <= state_entered_at
            or age_s < -_TIMESTAMP_EPSILON_S
            or age_s - self.config.max_age_s > _TIMESTAMP_EPSILON_S
        ):
            return self._reset_decision(
                "lane-validity message outside fresh REJOIN session",
                new_message=True,
            )
        if observation.lane_valid is not True:
            return self._reset_decision(
                "lane validity not confirmed",
                new_message=True,
            )

        if self.first_valid_at is None:
            self.first_valid_at = timestamp
            self.valid_message_count = 1
        else:
            self.valid_message_count += 1

        elapsed_s = timestamp - self.first_valid_at
        if (
            self.valid_message_count >= self.config.min_messages
            and elapsed_s + _TIMESTAMP_EPSILON_S
            >= self.config.min_duration_s
        ):
            self.triggered = True
            return self._decision(
                True,
                "lane validity confirmed after REJOIN",
                new_message=True,
                elapsed_s=elapsed_s,
            )
        return self._decision(
            False,
            "waiting for lane-validity debounce",
            new_message=True,
            elapsed_s=elapsed_s,
        )

    def _reset_decision(
        self,
        reason: str,
        *,
        new_message: bool = False,
    ) -> LaneValidityDecision:
        reset = self._reset_sequence()
        return self._decision(
            False,
            reason,
            new_message=new_message,
            sequence_reset=reset,
        )

    def _reset_sequence(self) -> bool:
        changed = (
            self.first_valid_at is not None
            or self.valid_message_count != 0
            or self.triggered
        )
        self.first_valid_at = None
        self.valid_message_count = 0
        self.triggered = False
        return changed

    def _decision(
        self,
        triggered: bool,
        reason: str,
        *,
        new_message: bool = False,
        sequence_reset: bool = False,
        elapsed_s: float = 0.0,
    ) -> LaneValidityDecision:
        return LaneValidityDecision(
            triggered=triggered,
            reason=reason,
            new_message=new_message,
            sequence_reset=sequence_reset,
            first_valid_at=self.first_valid_at,
            valid_message_count=self.valid_message_count,
            elapsed_s=elapsed_s,
        )

    @staticmethod
    def _valid_timestamp(value: Optional[float]) -> bool:
        return (
            value is not None
            and not isinstance(value, bool)
            and isinstance(value, (int, float))
            and math.isfinite(value)
        )
