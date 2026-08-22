"""Pure, throttled formatting for Main runtime decisions."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Tuple


@dataclass(frozen=True)
class RuntimeDiagnosticSnapshot:
    mode: str
    control_source: str
    control_reason: str
    safety_reason: Optional[str]
    missing_inputs: Tuple[str, ...]
    stale_inputs: Tuple[str, ...]
    traffic_stop_override: bool
    lane_action_safe_to_drive: bool
    lane_action_pending: bool
    lane_action_completed: bool
    zone_elapsed_s: Optional[float]
    side_obstacle_seen: bool
    clear_timer_elapsed_s: Optional[float]
    same_lane_brake_reason: str

    @property
    def signature(self) -> tuple:
        # Continuously increasing timers are display values, not state changes.
        return (
            self.mode,
            self.control_source,
            self.control_reason,
            self.safety_reason,
            self.missing_inputs,
            self.stale_inputs,
            self.traffic_stop_override,
            self.lane_action_safe_to_drive,
            self.lane_action_pending,
            self.lane_action_completed,
            self.zone_elapsed_s is not None,
            self.side_obstacle_seen,
            self.clear_timer_elapsed_s is not None,
            self.same_lane_brake_reason,
        )


def _format_number(value: Optional[float]) -> str:
    if value is None:
        return "inactive"
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return "invalid"
    if math.isnan(value):
        return "nan"
    if math.isinf(value):
        return "inf" if value > 0 else "-inf"
    return f"{value:.2f}s"


def format_runtime_diagnostic(snapshot: RuntimeDiagnosticSnapshot) -> str:
    missing = ",".join(snapshot.missing_inputs) or "none"
    stale = ",".join(snapshot.stale_inputs) or "none"
    safety = snapshot.safety_reason or "ready"
    return (
        f"runtime mode={snapshot.mode} control={snapshot.control_source} "
        f"decision={snapshot.control_reason!r} safety={safety!r} "
        f"missing=[{missing}] stale=[{stale}] "
        f"traffic_stop_override={snapshot.traffic_stop_override} "
        "lane_action="
        f"safe:{snapshot.lane_action_safe_to_drive},"
        f"pending:{snapshot.lane_action_pending},"
        f"completed:{snapshot.lane_action_completed} "
        f"zone_elapsed={_format_number(snapshot.zone_elapsed_s)} "
        f"side_seen={snapshot.side_obstacle_seen} "
        f"clear_timer={_format_number(snapshot.clear_timer_elapsed_s)} "
        f"same_lane_brake={snapshot.same_lane_brake_reason!r}"
    )


class RuntimeDiagnosticReporter:
    """Log state changes immediately and an unchanged HOLD at most once/s."""

    def __init__(self, hold_period_s: float = 1.0) -> None:
        if not math.isfinite(hold_period_s) or hold_period_s <= 0.0:
            raise ValueError("hold_period_s must be finite and positive")
        self.hold_period_s = float(hold_period_s)
        self.reset()

    def reset(self) -> None:
        self._last_signature: Optional[tuple] = None
        self._last_emitted_at: Optional[float] = None

    def update(
        self,
        snapshot: RuntimeDiagnosticSnapshot,
        now: float,
    ) -> Optional[str]:
        changed = snapshot.signature != self._last_signature
        valid_now = (
            not isinstance(now, bool)
            and isinstance(now, (int, float))
            and math.isfinite(now)
        )
        periodic_hold = False
        if (
            not changed
            and snapshot.control_source == "HOLD"
            and valid_now
            and self._last_emitted_at is not None
        ):
            elapsed = now - self._last_emitted_at
            periodic_hold = elapsed < 0.0 or elapsed >= self.hold_period_s

        self._last_signature = snapshot.signature
        if not changed and not periodic_hold:
            return None
        if valid_now:
            self._last_emitted_at = float(now)
        return format_runtime_diagnostic(snapshot)
