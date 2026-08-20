"""Pure shortcut exit guard for the CNN road-surface classifier."""

from dataclasses import dataclass
from enum import IntEnum
import math
from typing import Optional


class RoadSurface(IntEnum):
    UNKNOWN = 0
    STANDARD_BLACK = 1
    SHORTCUT_WHITE = 2


@dataclass(frozen=True)
class ShortcutExitConfig:
    min_black_frames: int = 3
    min_black_duration_s: float = 0.20

    def __post_init__(self) -> None:
        if self.min_black_frames < 1:
            raise ValueError("min_black_frames must be positive")
        if not math.isfinite(self.min_black_duration_s) or self.min_black_duration_s < 0:
            raise ValueError("min_black_duration_s must be finite and non-negative")


class ShortcutExitGuard:
    """Require white-shortcut evidence before accepting continuous black road."""

    def __init__(self, config: Optional[ShortcutExitConfig] = None) -> None:
        self.config = config or ShortcutExitConfig()
        self.reset()

    def reset(self) -> None:
        self.white_seen = False
        self.black_frames = 0
        self.black_started_at: Optional[float] = None

    def update(self, surface: RoadSurface | int, now: float) -> bool:
        if isinstance(surface, bool) or not math.isfinite(now):
            return False
        try:
            surface = RoadSurface(surface)
        except (TypeError, ValueError):
            return False

        if surface is RoadSurface.SHORTCUT_WHITE:
            self.white_seen = True
            self.black_frames = 0
            self.black_started_at = None
            return False

        if not self.white_seen or surface is not RoadSurface.STANDARD_BLACK:
            self.black_frames = 0
            self.black_started_at = None
            return False

        if self.black_frames == 0:
            self.black_started_at = now
        self.black_frames += 1
        elapsed = now - self.black_started_at
        return (
            self.black_frames >= self.config.min_black_frames
            and elapsed + 1e-6 >= self.config.min_black_duration_s
        )
