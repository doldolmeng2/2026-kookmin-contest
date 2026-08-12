import pytest

from main.lane_validity import LaneValidityConfig
from main.mission_observation import MissionObservation
from main.mode_info import (
    LegacyModeInfoCode,
    external_mode_code,
    mode_info_data,
)
from main.race_context import RaceContext
from main.race_fsm import Mode, RaceFSM
from main.safety_monitor import SafetyDecision


def test_internal_mode_values_are_not_external_numeric_codes():
    assert all(isinstance(mode.value, str) for mode in Mode)
    assert Mode.LANE_DRIVE.value == "LANE_DRIVE"
    assert external_mode_code(Mode.LANE_DRIVE) == 3


@pytest.mark.parametrize(
    ("mode", "expected"),
    [
        (Mode.INIT, LegacyModeInfoCode.STOP),
        (Mode.WAIT_GREEN, LegacyModeInfoCode.STOP),
        (Mode.LANE_DRIVE, LegacyModeInfoCode.LANE_DRIVE),
        (Mode.CONE_DRIVE, LegacyModeInfoCode.CONE_DRIVE),
        (Mode.REJOIN, LegacyModeInfoCode.REJOIN),
        (Mode.FINISH, LegacyModeInfoCode.STOP),
        (Mode.STOP, LegacyModeInfoCode.STOP),
    ],
)
def test_confirmed_external_mode_mapping(mode, expected):
    assert external_mode_code(mode) is expected


@pytest.mark.parametrize(
    "mode",
    [Mode.FIXED_AVOID, Mode.OVERTAKE, Mode.SHORTCUT],
)
def test_unconfirmed_future_modes_publish_stop_instead_of_invented_codes(mode):
    assert external_mode_code(mode) is LegacyModeInfoCode.STOP


@pytest.mark.parametrize("invalid", [None, 3, "LANE_DRIVE", True, object()])
def test_invalid_mode_input_maps_to_external_stop(invalid):
    assert external_mode_code(invalid) is LegacyModeInfoCode.STOP


def test_mode_info_data_preserves_current_lane_detector_two_field_contract():
    assert mode_info_data(Mode.LANE_DRIVE, 0) == [3, 0]
    assert mode_info_data(Mode.CONE_DRIVE, 1) == [1, 1]
    assert mode_info_data(Mode.REJOIN, 1) == [2, 1]


@pytest.mark.parametrize("invalid_lane", [-1, 2, None, True, "1"])
def test_invalid_lane_value_uses_safe_existing_lane_value(invalid_lane):
    assert mode_info_data(Mode.LANE_DRIVE, invalid_lane) == [3, 1]


def test_lane_change_code_exists_only_as_consumer_contract_not_fsm_mapping():
    assert LegacyModeInfoCode.LANE_CHANGE == 5
    assert all(
        external_mode_code(mode) is not LegacyModeInfoCode.LANE_CHANGE
        for mode in Mode
    )


@pytest.mark.parametrize("mode", [Mode.FIXED_AVOID, Mode.OVERTAKE])
def test_action_level_mode_info_orders_stop_change_then_lane(mode):
    stopped = mode_info_data(mode, 0)
    changing = mode_info_data(
        mode,
        1,
        mission_lane_control_enabled=True,
        lane_change_active=True,
    )
    settled = mode_info_data(
        mode,
        1,
        mission_lane_control_enabled=True,
        lane_change_active=False,
    )

    assert stopped == [0, 0]
    assert changing == [5, 1]
    assert settled == [3, 1]


def test_lane_change_code_is_not_emitted_while_action_is_unsafe():
    assert mode_info_data(
        Mode.FIXED_AVOID,
        0,
        mission_lane_control_enabled=False,
        lane_change_active=True,
    ) == [0, 0]


def test_rejoin_completion_enters_fixed_avoid_with_fail_safe_external_mode():
    fsm = RaceFSM(
        initial_state=Mode.REJOIN,
        lane_validity_config=LaneValidityConfig(
            min_messages=1,
            min_duration_s=0.0,
        ),
    )
    context = RaceContext(state_entered_at=1.0)

    assert external_mode_code(fsm.state) is LegacyModeInfoCode.REJOIN
    transition = fsm.step(
        MissionObservation(
            now=1.1,
            lane_valid=True,
            lane_valid_received_at=1.1,
        ),
        context,
        SafetyDecision(inputs_ready=True),
    )

    assert transition.target is Mode.FIXED_AVOID
    assert fsm.state is Mode.FIXED_AVOID
    assert external_mode_code(fsm.state) is LegacyModeInfoCode.STOP
