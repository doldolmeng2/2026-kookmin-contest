"""Immutable input snapshot consumed by the pure race FSM."""

from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Mapping, Optional


@dataclass(frozen=True)
class MissionObservation:
    """One coherent mission snapshot.

    ``now`` and every value in the timestamp maps must use the same clock
    domain. Sensor receipt times and derived-perception receipt times remain
    separate so safety policy can require and diagnose them independently.
    The dedicated cone and scan receipt fields use that same domain.
    ``cone_message_received_at=None`` means this snapshot has no new cone
    message edge; cached cone data must not be counted again.
    """

    now: float
    sensor_received_at: Mapping[str, float] = field(default_factory=dict)
    perception_received_at: Mapping[str, float] = field(default_factory=dict)

    green_detected: bool = False
    traffic_message_received_at: Optional[float] = None
    lane_valid: bool = False
    lane_valid_received_at: Optional[float] = None
    cone_detected: bool = False
    cone_finished: bool = False
    cone_confidence: Optional[int] = None
    cone_end_flag: Optional[bool] = None
    cone_message_received_at: Optional[float] = None
    scan_received_at: Optional[float] = None
    fixed_vehicle_detected: bool = False
    fixed_avoid_complete: bool = False
    interfering_vehicle_detected: bool = False
    overtake_complete: bool = False
    left_turn_signal: bool = False
    shortcut_complete: bool = False
    finish_gate_crossed: bool = False
    fixed_avoid_complete_received_at: Optional[float] = None
    overtake_complete_received_at: Optional[float] = None
    left_turn_signal_received_at: Optional[float] = None
    shortcut_complete_received_at: Optional[float] = None
    finish_gate_crossed_received_at: Optional[float] = None

    def __post_init__(self) -> None:
        # Copy caller-owned dictionaries so this object remains a true snapshot.
        object.__setattr__(
            self,
            "sensor_received_at",
            MappingProxyType(dict(self.sensor_received_at)),
        )
        object.__setattr__(
            self,
            "perception_received_at",
            MappingProxyType(dict(self.perception_received_at)),
        )

    def last_received_at(self, category: str, name: str) -> Optional[float]:
        """Return a receipt time from the requested input category."""

        if category == "sensor":
            return self.sensor_received_at.get(name)
        if category == "perception":
            return self.perception_received_at.get(name)
        raise ValueError(f"unknown input category: {category}")
