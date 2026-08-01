"""Pure, stateful guard for provisional rubbercone-section entry detection."""

from dataclasses import dataclass
import math
from typing import Optional

from .mission_observation import MissionObservation


_TIMESTAMP_EPSILON_S = 1e-6


@dataclass(frozen=True)
class ConeEntryConfig:
    """Provisional defaults derived only from four positive processed bags.

    These values are not final thresholds. They must be revalidated after
    trustworthy negative processed bags have been collected.
    """

    min_confidence: int = 75
    min_messages: int = 3
    min_duration_s: float = 0.2
    max_cone_age_s: float = 0.25
    max_scan_age_s: float = 0.25

    def __post_init__(self) -> None:
        if (
            isinstance(self.min_confidence, bool)
            or not isinstance(self.min_confidence, int)
            or not 0 <= self.min_confidence <= 100
        ):
            raise ValueError("min_confidence must be an integer from 0 to 100")
        if (
            isinstance(self.min_messages, bool)
            or not isinstance(self.min_messages, int)
            or self.min_messages < 1
        ):
            raise ValueError("min_messages must be an integer of at least one")

        for name, value in (
            ("min_duration_s", self.min_duration_s),
            ("max_cone_age_s", self.max_cone_age_s),
            ("max_scan_age_s", self.max_scan_age_s),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value < 0.0
            ):
                raise ValueError(f"{name} must be a finite non-negative number")


@dataclass(frozen=True)
class ConeEntryDecision:
    triggered: bool
    reason: str
    new_message: bool = False
    sequence_reset: bool = False
    first_qualifying_at: Optional[float] = None
    qualifying_message_count: int = 0
    elapsed_s: float = 0.0


class ConeEntryDebouncer:
    """Count only unique cone message edges in one qualifying episode."""

    def __init__(self, config: Optional[ConeEntryConfig] = None) -> None:
        self.config = config or ConeEntryConfig()
        self.first_qualifying_at: Optional[float] = None
        self.qualifying_message_count = 0
        self.last_cone_message_at: Optional[float] = None
        self.triggered = False

    def deactivate(self) -> None:
        """Stop accumulating outside LANE_DRIVE while retaining dedupe time."""

        self._reset_sequence()

    def reject_timestamp_regression(self) -> ConeEntryDecision:
        """Reset an active episode when an adapter rejects a regressed edge."""

        reset = self._reset_sequence()
        return self._decision(
            False,
            "cone message timestamp regression",
            sequence_reset=reset,
        )

    def evaluate(self, observation: MissionObservation) -> ConeEntryDecision:
        timestamp = observation.cone_message_received_at
        if timestamp is None:
            # No cone event edge exists on this FSM step. In particular, scan
            # and trace events must neither recount nor reset a cached frame.
            return self._decision(False, "no new cone message")

        if not self._valid_timestamp(timestamp):
            reset = self._reset_sequence()
            return self._decision(
                False,
                "invalid cone message timestamp",
                sequence_reset=reset,
            )

        if self.last_cone_message_at is not None:
            if timestamp == self.last_cone_message_at:
                return self._decision(
                    False,
                    "duplicate cone message timestamp",
                )
            if timestamp < self.last_cone_message_at:
                return self.reject_timestamp_regression()

        self.last_cone_message_at = timestamp
        now = observation.now
        if not self._valid_timestamp(now):
            reset = self._reset_sequence()
            return self._decision(
                False,
                "invalid observation timestamp",
                new_message=True,
                sequence_reset=reset,
            )

        cone_age_s = now - timestamp
        if (
            cone_age_s < -_TIMESTAMP_EPSILON_S
            or cone_age_s - self.config.max_cone_age_s > _TIMESTAMP_EPSILON_S
        ):
            reset = self._reset_sequence()
            return self._decision(
                False,
                "cone message stale",
                new_message=True,
                sequence_reset=reset,
            )

        scan_received_at = observation.scan_received_at
        if scan_received_at is None:
            reset = self._reset_sequence()
            return self._decision(
                False,
                "scan timestamp missing",
                new_message=True,
                sequence_reset=reset,
            )
        if not self._valid_timestamp(scan_received_at):
            reset = self._reset_sequence()
            return self._decision(
                False,
                "invalid scan timestamp",
                new_message=True,
                sequence_reset=reset,
            )

        scan_age_s = now - scan_received_at
        if (
            scan_age_s < -_TIMESTAMP_EPSILON_S
            or scan_age_s - self.config.max_scan_age_s > _TIMESTAMP_EPSILON_S
        ):
            reset = self._reset_sequence()
            return self._decision(
                False,
                "scan stale",
                new_message=True,
                sequence_reset=reset,
            )

        confidence = observation.cone_confidence
        if confidence is None:
            reset = self._reset_sequence()
            return self._decision(
                False,
                "cone confidence missing",
                new_message=True,
                sequence_reset=reset,
            )
        if (
            isinstance(confidence, bool)
            or not isinstance(confidence, int)
            or not 0 <= confidence <= 100
        ):
            reset = self._reset_sequence()
            return self._decision(
                False,
                "invalid cone confidence",
                new_message=True,
                sequence_reset=reset,
            )
        if confidence < self.config.min_confidence:
            reset = self._reset_sequence()
            return self._decision(
                False,
                "cone confidence below threshold",
                new_message=True,
                sequence_reset=reset,
            )
        if observation.cone_end_flag is not False:
            reset = self._reset_sequence()
            return self._decision(
                False,
                "cone end flag not clear",
                new_message=True,
                sequence_reset=reset,
            )

        if self.triggered:
            return self._decision(
                False,
                "cone entry already triggered",
                new_message=True,
            )

        if self.first_qualifying_at is None:
            self.first_qualifying_at = timestamp
            self.qualifying_message_count = 1
        else:
            self.qualifying_message_count += 1

        elapsed_s = timestamp - self.first_qualifying_at
        if (
            self.qualifying_message_count >= self.config.min_messages
            and elapsed_s + _TIMESTAMP_EPSILON_S
            >= self.config.min_duration_s
        ):
            self.triggered = True
            return self._decision(
                True,
                "cone entry confirmed",
                new_message=True,
                elapsed_s=elapsed_s,
            )

        return self._decision(
            False,
            "waiting for cone entry debounce",
            new_message=True,
            elapsed_s=elapsed_s,
        )

    def _reset_sequence(self) -> bool:
        changed = (
            self.first_qualifying_at is not None
            or self.qualifying_message_count != 0
            or self.triggered
        )
        self.first_qualifying_at = None
        self.qualifying_message_count = 0
        self.triggered = False
        return changed

    def _decision(
        self,
        triggered: bool,
        reason: str,
        *,
        new_message: bool = False,
        sequence_reset: bool = False,
        elapsed_s: Optional[float] = None,
    ) -> ConeEntryDecision:
        if elapsed_s is None:
            elapsed_s = 0.0
            if self.first_qualifying_at is not None:
                elapsed_s = max(
                    0.0,
                    (self.last_cone_message_at or self.first_qualifying_at)
                    - self.first_qualifying_at,
                )
        return ConeEntryDecision(
            triggered=triggered,
            reason=reason,
            new_message=new_message,
            sequence_reset=sequence_reset,
            first_qualifying_at=self.first_qualifying_at,
            qualifying_message_count=self.qualifying_message_count,
            elapsed_s=elapsed_s,
        )

    @staticmethod
    def _valid_timestamp(value: float) -> bool:
        return (
            not isinstance(value, bool)
            and isinstance(value, (int, float))
            and math.isfinite(value)
        )
