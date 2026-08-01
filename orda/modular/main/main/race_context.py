"""Race-wide mutable context for the ROS-independent FSM layer."""

from dataclasses import dataclass
from typing import ClassVar, Optional


@dataclass
class RaceContext:
    """State that must survive individual perception observations."""

    TOTAL_LAPS: ClassVar[int] = 3

    # The Gate event source and its debounce contract are not decided yet.
    # The current FSM must therefore never increment this field.
    finish_gate_passes: int = 0
    shortcut_used: bool = False

    race_started_at: Optional[float] = None
    state_entered_at: Optional[float] = None
    cone_entered_at: Optional[float] = None
    last_gate_event_at: Optional[float] = None
    stop_reason: Optional[str] = None

    @property
    def lap_count(self) -> int:
        """Return completed laps, currently identical to Gate passes."""

        return self.finish_gate_passes

    @property
    def current_lap(self) -> int:
        """Return the one-based lap in progress, capped at the final lap."""

        return min(self.finish_gate_passes + 1, self.TOTAL_LAPS)
