import pytest

from main.mode_info import (
    ExternalModeInfoCode,
    LegacyLaneCommandCode,
    external_mode_code,
    lane_command_data,
    lane_info_value,
)
from main.race_fsm import Mode


def test_internal_mode_values_are_not_external_numeric_codes():
    assert all(isinstance(mode.value, str) for mode in Mode)
    assert Mode.LANE_DRIVE.value == "LANE_DRIVE"
    assert external_mode_code(Mode.LANE_DRIVE) == 1


@pytest.mark.parametrize(
    ("mode", "expected"),
    [
        (Mode.WAIT_GREEN, ExternalModeInfoCode.WAIT_GREEN),
        (Mode.LANE_DRIVE, ExternalModeInfoCode.LANE_DRIVE),
        (Mode.CONE_DRIVE, ExternalModeInfoCode.CONE_DRIVE),
        (Mode.FIXED_AVOID, ExternalModeInfoCode.FIXED_AVOID),
        (Mode.OVERTAKE, ExternalModeInfoCode.OVERTAKE),
        (Mode.SHORTCUT, ExternalModeInfoCode.SHORTCUT),
    ],
)
def test_ppt_external_mode_mapping(mode, expected):
    assert external_mode_code(mode) is expected


@pytest.mark.parametrize(
    "internal_or_invalid",
    [Mode.FINISH, None, 3, "LANE_DRIVE", True, object()],
)
def test_unassigned_or_invalid_modes_have_no_external_code(internal_or_invalid):
    assert external_mode_code(internal_or_invalid) is None


@pytest.mark.parametrize(
    ("internal", "external"),
    [(0, 3), (1, 1), (2, 2)],
)
def test_lane_info_mapping(internal, external):
    assert lane_info_value(internal) == external


@pytest.mark.parametrize("invalid", [-1, 3, None, True, "1"])
def test_invalid_lane_info_falls_back_to_center(invalid):
    assert lane_info_value(invalid) == 3


def test_private_lane_command_preserves_existing_detector_contract():
    assert lane_command_data(Mode.WAIT_GREEN, 0) == [0, 0]
    assert lane_command_data(Mode.LANE_DRIVE, 0) == [3, 0]
    assert lane_command_data(Mode.CONE_DRIVE, 1) == [1, 1]


@pytest.mark.parametrize("invalid_lane", [-1, 3, None, True, "1"])
def test_invalid_private_lane_target_falls_back_to_center(invalid_lane):
    assert lane_command_data(Mode.LANE_DRIVE, invalid_lane) == [3, 0]


def test_numeric_five_is_scoped_by_topic_contract():
    assert LegacyLaneCommandCode.LANE_CHANGE == 5
    assert ExternalModeInfoCode.SHORTCUT == 5
    assert lane_command_data(
        Mode.FIXED_AVOID,
        1,
        mission_lane_control_enabled=True,
        lane_change_active=True,
    ) == [5, 1]


@pytest.mark.parametrize("mode", [Mode.FIXED_AVOID, Mode.OVERTAKE])
def test_action_level_private_command_never_stops_inside_zone(mode):
    unauthorized = lane_command_data(mode, 0)
    changing = lane_command_data(
        mode,
        1,
        mission_lane_control_enabled=True,
        lane_change_active=True,
    )
    settled = lane_command_data(
        mode,
        1,
        mission_lane_control_enabled=True,
        lane_change_active=False,
    )

    assert unauthorized == [3, 0]
    assert changing == [5, 1]
    assert settled == [3, 1]
