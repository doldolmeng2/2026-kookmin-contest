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


@pytest.mark.parametrize("invalid_lane", [-1, 3, None, True, "1"])
def test_invalid_lane_value_falls_back_to_center(invalid_lane):
    # 기본 주행이 중앙이므로 알 수 없는 값은 중앙(2)으로 떨어뜨린다.
    assert mode_info_data(Mode.LANE_DRIVE, invalid_lane) == [3, 0]


def test_center_is_a_valid_lane_value():
    assert mode_info_data(Mode.LANE_DRIVE, 0) == [3, 0]


def test_lane_change_code_exists_only_as_consumer_contract_not_fsm_mapping():
    assert LegacyModeInfoCode.LANE_CHANGE == 5
    assert all(
        external_mode_code(mode) is not LegacyModeInfoCode.LANE_CHANGE
        for mode in Mode
    )


@pytest.mark.parametrize("mode", [Mode.FIXED_AVOID, Mode.OVERTAKE])
def test_action_level_mode_info_never_emits_stop_inside_a_zone(mode):
    # 구간 안에서는 정지 코드(0)를 쓰지 않는다. 인지가 신선하지 않으면
    # 차선 변경(5) 대신 차선 주행(3)으로 둔다.
    unauthorized = mode_info_data(mode, 0)
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

    assert unauthorized == [3, 0]
    assert changing == [5, 1]
    assert settled == [3, 1]


def test_lane_change_code_is_not_emitted_while_action_is_unsafe():
    # 미허가 상태에서는 변경(5)이 아니라 차선 주행(3)으로 떨어진다.
    assert mode_info_data(
        Mode.FIXED_AVOID,
        0,
        mission_lane_control_enabled=False,
        lane_change_active=True,
    ) == [3, 0]


def test_rejoin_completion_changes_external_mode_to_lane_not_fixed_avoid():
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

    assert transition.target is Mode.LANE_DRIVE
    assert fsm.state is Mode.LANE_DRIVE
    assert external_mode_code(fsm.state) is LegacyModeInfoCode.LANE_DRIVE
