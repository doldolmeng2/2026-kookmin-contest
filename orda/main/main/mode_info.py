"""Explicit adapter from internal RaceFSM state to legacy ROS diagnostics."""

from enum import IntEnum
from typing import Any

from .race_fsm import Mode


class LegacyModeInfoCode(IntEnum):
    """Numeric values consumed by the current lane detector implementation."""

    STOP = 0
    CONE_DRIVE = 1
    REJOIN = 2
    LANE_DRIVE = 3
    BEFORE = 4
    LANE_CHANGE = 5


_CONFIRMED_MODE_CODES = {
    Mode.INIT: LegacyModeInfoCode.STOP,
    Mode.WAIT_GREEN: LegacyModeInfoCode.STOP,
    Mode.LANE_DRIVE: LegacyModeInfoCode.LANE_DRIVE,
    Mode.CONE_DRIVE: LegacyModeInfoCode.CONE_DRIVE,
    Mode.REJOIN: LegacyModeInfoCode.REJOIN,
    Mode.FINISH: LegacyModeInfoCode.STOP,
    Mode.STOP: LegacyModeInfoCode.STOP,
}


def external_mode_code(mode: Any) -> LegacyModeInfoCode:
    """Return a confirmed external code or the fail-safe STOP code."""

    if not isinstance(mode, Mode):
        return LegacyModeInfoCode.STOP
    return _CONFIRMED_MODE_CODES.get(mode, LegacyModeInfoCode.STOP)


def mode_info_data(mode: Any, lane: Any) -> list[int]:
    """Build the current two-field ``[mode, lane]`` external contract."""

    lane_value = lane if isinstance(lane, int) and lane in (0, 1) else 1
    return [int(external_mode_code(mode)), lane_value]
