"""Pure ownership gate for one physical obstacle-detection episode."""

from __future__ import annotations

import math
from typing import Optional


class ObjectMissionEpisodeGate:
    """Allow at most one mission until object evidence is continuously absent."""

    def __init__(self, release_after_s: float) -> None:
        if not math.isfinite(release_after_s) or release_after_s <= 0.0:
            raise ValueError("release_after_s must be finite and positive")
        self.release_after_s = float(release_after_s)
        self.reset()

    def reset(self) -> None:
        self.episode_active = False
        self.consumed = False
        self.last_valid_detection_at: Optional[float] = None

    @staticmethod
    def _valid_time(now: float) -> bool:
        return not isinstance(now, bool) and math.isfinite(now)

    def observe_valid_detection(self, now: float) -> bool:
        """Extend the current episode, creating it only after a real release."""

        if not self._valid_time(now):
            return False
        self.expire(now)
        if not self.episode_active:
            self.episode_active = True
            self.consumed = False
        self.last_valid_detection_at = float(now)
        return True

    def expire(self, now: float) -> bool:
        """Release after the existing 1.90 s encounter gap; return release edge."""

        if (
            not self.episode_active
            or self.last_valid_detection_at is None
            or not self._valid_time(now)
            or now < self.last_valid_detection_at
            or now - self.last_valid_detection_at < self.release_after_s
        ):
            return False
        self.reset()
        return True

    @property
    def entry_allowed(self) -> bool:
        return self.episode_active and not self.consumed

    def consume(self) -> bool:
        if not self.entry_allowed:
            return False
        self.consumed = True
        return True
