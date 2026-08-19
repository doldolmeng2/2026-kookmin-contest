import pytest

from main.control_selector import (
    CommandCandidate,
    ControlSelector,
    ControlSource,
    DriveCommand,
)
from main.mission_observation import MissionObservation
from main.mode_info import external_mode_code
from main.race_context import RaceContext
from main.race_fsm import Mode, RaceFSM
from main.safety_monitor import SafetyDecision


STATE_CONTRACT = (
    (Mode.WAIT_GREEN, ControlSource.STOP, 0),
    (Mode.LANE_DRIVE, ControlSource.LANE, 1),
    (Mode.CONE_DRIVE, ControlSource.CONE, 2),
    (Mode.FIXED_AVOID, ControlSource.STOP, 3),
    (Mode.OVERTAKE, ControlSource.STOP, 4),
    (Mode.SHORTCUT, ControlSource.LANE, 5),
    (Mode.FINISH, ControlSource.STOP, None),
    (Mode.STOP, ControlSource.STOP, None),
)


@pytest.mark.parametrize(
    ("mode", "expected_source", "expected_external_code"),
    STATE_CONTRACT,
)
def test_every_state_has_explicit_control_and_external_mode_contract(
    mode,
    expected_source,
    expected_external_code,
):
    lane = CommandCandidate(DriveCommand(1.0, 5.0), 2.0)
    cone = CommandCandidate(DriveCommand(-2.0, 4.0), 2.0)

    control = ControlSelector().select(mode, 2.0, lane=lane, cone=cone)

    assert control.source is expected_source
    code = external_mode_code(mode)
    assert (None if code is None else int(code)) == expected_external_code
    if expected_source is ControlSource.STOP:
        assert control.command == DriveCommand(0.0, 0.0)


@pytest.mark.parametrize("mode", list(Mode))
def test_safety_fault_precedes_every_nonterminal_mission_transition(mode):
    fsm = RaceFSM(initial_state=mode)
    context = RaceContext(state_entered_at=1.0)

    transition = fsm.step(
        MissionObservation(
            now=2.0,
            cone_end_flag=True,
            cone_message_received_at=2.0,
            fixed_avoid_complete=True,
            fixed_avoid_completed_at=2.0,
            overtake_complete=True,
            overtake_complete_received_at=2.0,
            shortcut_complete=True,
            shortcut_complete_received_at=2.0,
        ),
        context,
        SafetyDecision(must_stop=True, reason="contract fault"),
    )

    if mode in RaceFSM.TERMINAL_STATES:
        assert transition.changed is False
        assert fsm.state is mode
    else:
        assert transition.target is Mode.STOP
        assert context.stop_reason == "contract fault"


def test_cone_completion_returns_directly_to_lane_drive():
    fsm = RaceFSM(initial_state=Mode.CONE_DRIVE)
    context = RaceContext(state_entered_at=1.0)

    fsm.step(
        MissionObservation(
            now=1.1,
            cone_end_flag=False,
            cone_message_received_at=1.1,
        ),
        context,
        SafetyDecision(inputs_ready=True),
    )
    transition = fsm.step(
        MissionObservation(
            now=1.2,
            cone_end_flag=True,
            cone_message_received_at=1.2,
        ),
        context,
        SafetyDecision(inputs_ready=True),
    )

    assert transition.source is Mode.CONE_DRIVE
    assert transition.target is Mode.LANE_DRIVE
