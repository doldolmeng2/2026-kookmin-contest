"""Pure physical traffic-fixture episode gate."""

from __future__ import annotations

import math
from typing import Optional

from main.mission_types import RouteTrafficSignal


class TrafficEncounterGate:
    """Emit one route encounter per fixture, resilient to brief UNKNOWN gaps."""

    def __init__(self, release_after_s: float) -> None:
        if not math.isfinite(release_after_s) or release_after_s <= 0.0:
            raise ValueError("release_after_s must be finite and positive")
        self.release_after_s = float(release_after_s)
        self.reset()

    def reset(self) -> None:
        self.episode_active = False
        self.encounter_emitted = False
        self.none_started_at: Optional[float] = None

    @staticmethod
    def _valid_time(now: float) -> bool:
        return not isinstance(now, bool) and math.isfinite(now)

    def update(self, signal: RouteTrafficSignal | int, now: float) -> bool:
        if not self._valid_time(now):
            return False
        try:
            signal = RouteTrafficSignal(signal)
        except (TypeError, ValueError):
            return False

        if signal is RouteTrafficSignal.UNKNOWN:
            if self.episode_active and self.none_started_at is None:
                self.none_started_at = float(now)
            if (
                self.episode_active
                and self.none_started_at is not None
                and now >= self.none_started_at
                and now - self.none_started_at >= self.release_after_s
            ):
                self.reset()
            return False

        # A signal after a continuously neutral interval belongs to a new
        # fixture even if no intermediate UNKNOWN callback arrived.
        if (
            self.episode_active
            and self.none_started_at is not None
            and now >= self.none_started_at
            and now - self.none_started_at >= self.release_after_s
        ):
            self.reset()

        self.none_started_at = None
        self.episode_active = True
        if signal is RouteTrafficSignal.RED_AMBER:
            return False
        if self.encounter_emitted:
            return False
        self.encounter_emitted = True
        return True
