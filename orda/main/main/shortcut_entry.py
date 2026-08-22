"""Pure shortcut entry guard for left-turn traffic signal timing."""

from dataclasses import dataclass
import math
from typing import Optional


_EPSILON_S = 1e-6


@dataclass(frozen=True)
class ShortcutEntryConfig:
    arm_duration_s: float = 1.0
    missing_frames_to_enter: int = 3
    armed_timeout_s: float = 2.0

    def __post_init__(self) -> None:
        if not math.isfinite(self.arm_duration_s) or self.arm_duration_s < 0.0:
            raise ValueError("arm_duration_s must be finite and non-negative")
        if self.missing_frames_to_enter < 1:
            raise ValueError("missing_frames_to_enter must be positive")
        if not math.isfinite(self.armed_timeout_s) or self.armed_timeout_s < 0.0:
            raise ValueError("armed_timeout_s must be finite and non-negative")


@dataclass(frozen=True)
class ShortcutEntryDecision:
    armed: bool
    enter: bool
    visible_duration_s: float
    missing_frames: int
    reason: str


class ShortcutEntryGuard:
    """Arm on sustained left-turn signal, then enter after it disappears."""

    def __init__(self, config: Optional[ShortcutEntryConfig] = None) -> None:
        self.config = config or ShortcutEntryConfig()
        self.reset()

    @property
    def armed(self) -> bool:
        return self.armed_at is not None and not self.entry_sent

    def reset(self) -> None:
        self.visible_since: Optional[float] = None
        self.last_seen_at: Optional[float] = None
        self.armed_at: Optional[float] = None
        self.missing_frames = 0
        self.entry_sent = False

    def update(self, left_visible: bool, now: float) -> ShortcutEntryDecision:
        if not isinstance(left_visible, bool) or not math.isfinite(now):
            self.reset()
            return ShortcutEntryDecision(False, False, 0.0, 0, "invalid input")

        if self.entry_sent:
            return ShortcutEntryDecision(False, False, 0.0, 0, "entry already sent")

        if left_visible:
            if self.visible_since is None:
                self.visible_since = now
            self.last_seen_at = now
            self.missing_frames = 0
            visible_duration = max(0.0, now - self.visible_since)
            if (
                self.armed_at is None
                and visible_duration + _EPSILON_S >= self.config.arm_duration_s
            ):
                self.armed_at = now
                return ShortcutEntryDecision(
                    True,
                    False,
                    visible_duration,
                    0,
                    "left signal armed",
                )
            return ShortcutEntryDecision(
                self.armed,
                False,
                visible_duration,
                0,
                "left signal visible",
            )

        if self.armed_at is None:
            self.visible_since = None
            self.last_seen_at = None
            self.missing_frames = 0
            return ShortcutEntryDecision(False, False, 0.0, 0, "waiting for left")

        if (
            self.last_seen_at is None
            or now - self.last_seen_at > self.config.armed_timeout_s + _EPSILON_S
        ):
            self.reset()
            return ShortcutEntryDecision(False, False, 0.0, 0, "armed timeout")

        self.missing_frames += 1
        enter = self.missing_frames >= self.config.missing_frames_to_enter
        if enter:
            self.entry_sent = True
        return ShortcutEntryDecision(
            self.armed,
            enter,
            0.0,
            self.missing_frames,
            "left signal disappeared" if enter else "left signal missing",
        )
