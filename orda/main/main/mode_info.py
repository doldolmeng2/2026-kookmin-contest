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


def mode_info_data(
    mode: Any,
    lane: Any,
    *,
    mission_lane_control_enabled: bool = False,
    lane_change_active: bool = False,
) -> list[int]:
    """Build the existing two-field contract with action-level lane control."""

    # bool은 int의 서브클래스라 별도로 막지 않으면 lane 값으로 새어 들어간다.
    valid_lane = (
        isinstance(lane, int)
        and not isinstance(lane, bool)
        and lane in (0, 1, 2)
    )
    lane_value = lane if valid_lane else 0
    code = external_mode_code(mode)
    if mode in (Mode.FIXED_AVOID, Mode.OVERTAKE):
        # 구간 안에서는 정지 코드(0)를 내보내지 않는다. 인지가 잠깐 신선하지
        # 않다고 해서 "정지"를 지시하면 lane_detection 이 기준선을 갱신하지
        # 못하고, 구간 진입 직후 [0, 0] 이 한 박자 튀어나온다. 차선 변경 중이
        # 아니면 차선 주행(3)으로 둔다. 실제 정지 판단은 ControlSelector 가
        # 따로 하므로 이 코드가 모터를 멈추지는 않는다.
        code = (
            LegacyModeInfoCode.LANE_CHANGE
            if lane_change_active and mission_lane_control_enabled
            else LegacyModeInfoCode.LANE_DRIVE
        )
    return [int(code), lane_value]
