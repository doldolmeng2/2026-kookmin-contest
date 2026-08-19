"""PPT-facing mode/lane contracts and the lane detector compatibility seam."""

from enum import IntEnum
from typing import Any, Optional

from .race_fsm import Mode


class ExternalModeInfoCode(IntEnum):
    """Authoritative values published on ``/mode_info``."""

    WAIT_GREEN = 0
    LANE_DRIVE = 1
    CONE_DRIVE = 2
    FIXED_AVOID = 3
    OVERTAKE = 4
    SHORTCUT = 5


_EXTERNAL_MODE_CODES = {
    Mode.WAIT_GREEN: ExternalModeInfoCode.WAIT_GREEN,
    Mode.LANE_DRIVE: ExternalModeInfoCode.LANE_DRIVE,
    Mode.CONE_DRIVE: ExternalModeInfoCode.CONE_DRIVE,
    Mode.FIXED_AVOID: ExternalModeInfoCode.FIXED_AVOID,
    Mode.OVERTAKE: ExternalModeInfoCode.OVERTAKE,
    Mode.SHORTCUT: ExternalModeInfoCode.SHORTCUT,
}


def external_mode_code(mode: Any) -> Optional[ExternalModeInfoCode]:
    """Return the approved external code, or ``None`` for internal states.

    FINISH deliberately has no external value. Main therefore does not publish
    a fabricated ``/mode_info`` value while it is active.
    """

    if not isinstance(mode, Mode):
        return None
    return _EXTERNAL_MODE_CODES.get(mode)


def lane_info_value(lane: Any) -> int:
    """Translate the internal target into the PPT ``/lane_info`` contract.

    Internal: 0=center, 1=lane one, 2=lane two.
    External: 1=lane one, 2=lane two, 3=center.
    Invalid values fail safe to center.
    """

    if isinstance(lane, bool) or not isinstance(lane, int):
        return 3
    return {0: 3, 1: 1, 2: 2}.get(lane, 3)


class LegacyLaneCommandCode(IntEnum):
    """Values consumed only by the existing lane detector implementation."""

    HOLD = 0
    CONE_DRIVE = 1
    LANE_DRIVE = 3
    LANE_CHANGE = 5


_LEGACY_LANE_CODES = {
    Mode.WAIT_GREEN: LegacyLaneCommandCode.HOLD,
    Mode.LANE_DRIVE: LegacyLaneCommandCode.LANE_DRIVE,
    Mode.CONE_DRIVE: LegacyLaneCommandCode.CONE_DRIVE,
    # SHORTCUT uses the ordinary lane detector for steering.
    Mode.SHORTCUT: LegacyLaneCommandCode.LANE_DRIVE,
    Mode.FINISH: LegacyLaneCommandCode.HOLD,
}


def lane_command_data(
    mode: Any,
    lane: Any,
    *,
    mission_lane_control_enabled: bool = False,
    lane_change_active: bool = False,
) -> list[int]:
    """Build the private ``[legacy_mode, internal_lane]`` lane command."""

    valid_lane = (
        isinstance(lane, int)
        and not isinstance(lane, bool)
        and lane in (0, 1, 2)
    )
    lane_value = lane if valid_lane else 0
    code = _LEGACY_LANE_CODES.get(mode, LegacyLaneCommandCode.HOLD)
    if mode in (Mode.FIXED_AVOID, Mode.OVERTAKE):
        code = (
            LegacyLaneCommandCode.LANE_CHANGE
            if lane_change_active and mission_lane_control_enabled
            else LegacyLaneCommandCode.LANE_DRIVE
        )
    return [int(code), lane_value]
