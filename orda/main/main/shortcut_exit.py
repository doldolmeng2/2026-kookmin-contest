"""Pure shortcut exit guard for shortcut-label disappearance."""

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
    min_missing_frames: int = 3

    def __post_init__(self) -> None:
        if self.min_missing_frames < 1:
            raise ValueError("min_missing_frames must be positive")


class ShortcutExitGuard:
    """Exit after a seen shortcut label disappears for several frames."""

    def __init__(self, config: Optional[ShortcutExitConfig] = None) -> None:
        self.config = config or ShortcutExitConfig()
        self.reset()

    def reset(self) -> None:
        self.shortcut_seen = False
        self.missing_frames = 0

    def update(self, surface: RoadSurface | int, now: float) -> bool:
        if isinstance(surface, bool) or not math.isfinite(now):
            return False
        try:
            surface = RoadSurface(surface)
        except (TypeError, ValueError):
            return False

        if surface is RoadSurface.SHORTCUT_WHITE:
            self.shortcut_seen = True
            self.missing_frames = 0
            return False

        if not self.shortcut_seen:
            self.missing_frames = 0
            return False

        self.missing_frames += 1
        return self.missing_frames >= self.config.min_missing_frames
