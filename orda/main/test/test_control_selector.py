import math

import pytest

from main.control_selector import (
    CommandCandidate,
    ControlSelector,
    ControlSelectorConfig,
    ControlSource,
    DriveCommand,
    commit_fsm_then_select_control,
)
from main.mission_observation import MissionObservation
from main.race_context import RaceContext
from main.race_fsm import Mode, RaceFSM
from main.safety_monitor import SafetyDecision


LANE_COMMAND = DriveCommand(angle=3.0, speed=7.0)
CONE_COMMAND = DriveCommand(angle=-12.0, speed=4.0)


def candidate(command, received_at):
    return CommandCandidate(command=command, received_at=received_at)


def safe():
    return SafetyDecision(inputs_ready=True)


def cone_observation(timestamp, confidence=90):
    return MissionObservation(
        now=timestamp,
        cone_confidence=confidence,
        cone_end_flag=False,
        cone_message_received_at=timestamp,
        scan_received_at=timestamp,
    )


def test_control_source_values_are_exact():
    assert [(source.name, source.value) for source in ControlSource] == [
        ("STOP", "STOP"),
        ("LANE", "LANE"),
        ("CONE", "CONE"),
    ]


@pytest.mark.parametrize(
    "mode",
    [
        Mode.WAIT_GREEN,
        Mode.FIXED_AVOID,
        Mode.OVERTAKE,
        Mode.FINISH,
        Mode.STOP,
    ],
)
def test_non_driving_or_unimplemented_modes_select_zero_stop(mode):
    decision = ControlSelector().select(
        mode,
        10.0,
        lane=candidate(LANE_COMMAND, 10.0),
        cone=candidate(CONE_COMMAND, 10.0),
    )

    assert decision.source is ControlSource.STOP
    assert decision.command == DriveCommand(0.0, 0.0)


def test_lane_drive_selects_fresh_lane_and_ignores_fresh_cone():
    decision = ControlSelector().select(
        Mode.LANE_DRIVE,
        10.0,
        lane=candidate(LANE_COMMAND, 9.9),
        cone=candidate(CONE_COMMAND, 10.0),
    )

    assert decision.source is ControlSource.LANE
    assert decision.command is LANE_COMMAND


def test_shortcut_follows_fresh_lane_until_cnn_exit_edge():
    decision = ControlSelector().select(
        Mode.SHORTCUT,
        10.0,
        lane=candidate(LANE_COMMAND, 9.9),
        cone=candidate(CONE_COMMAND, 10.0),
    )

    assert decision.source is ControlSource.LANE
    assert decision.command is LANE_COMMAND


@pytest.mark.parametrize(
    "lane",
    [None, candidate(LANE_COMMAND, 9.0)],
)
def test_lane_drive_missing_or_stale_lane_stops_without_cone_fallback(lane):
    decision = ControlSelector().select(
        Mode.LANE_DRIVE,
        10.0,
        lane=lane,
        cone=candidate(CONE_COMMAND, 10.0),
    )

    assert decision.source is ControlSource.STOP
    assert decision.command == DriveCommand(0.0, 0.0)


def test_cone_drive_selects_fresh_cone_and_ignores_fresh_lane():
    decision = ControlSelector().select(
        Mode.CONE_DRIVE,
        10.0,
        lane=candidate(LANE_COMMAND, 10.0),
        cone=candidate(CONE_COMMAND, 9.9),
    )

    assert decision.source is ControlSource.CONE
    assert decision.command is CONE_COMMAND


@pytest.mark.parametrize("mode", [Mode.FIXED_AVOID, Mode.OVERTAKE])
def test_action_modes_select_lane_only_with_explicit_authorization(mode):
    selector = ControlSelector()

    unauthorized = selector.select(
        mode,
        10.0,
        lane=candidate(LANE_COMMAND, 10.0),
    )
    authorized = selector.select(
        mode,
        10.0,
        lane=candidate(LANE_COMMAND, 10.0),
        mission_lane_authorized=True,
    )

    assert unauthorized.source is ControlSource.STOP
    assert authorized.source is ControlSource.LANE
    assert authorized.command is LANE_COMMAND


def test_authorized_action_mode_still_rejects_stale_lane_command():
    decision = ControlSelector().select(
        Mode.FIXED_AVOID,
        10.0,
        lane=candidate(LANE_COMMAND, 9.0),
        mission_lane_authorized=True,
    )

    assert decision.source is ControlSource.STOP


@pytest.mark.parametrize(
    "mode",
    [
        Mode.LANE_DRIVE,
        Mode.CONE_DRIVE,
        Mode.FIXED_AVOID,
        Mode.OVERTAKE,
        Mode.SHORTCUT,
    ],
)
def test_route_traffic_hold_overrides_motion_without_changing_mode(mode):
    decision = ControlSelector().select(
        mode,
        10.0,
        lane=candidate(LANE_COMMAND, 10.0),
        cone=candidate(CONE_COMMAND, 10.0),
        mission_lane_authorized=True,
        traffic_hold=True,
    )

    assert decision.source is ControlSource.STOP
    assert decision.reason == "recoverable route-traffic hold"


@pytest.mark.parametrize(
    "cone",
    [None, candidate(CONE_COMMAND, 9.0)],
)
def test_cone_drive_missing_or_stale_cone_stops_without_lane_fallback(cone):
    decision = ControlSelector().select(
        Mode.CONE_DRIVE,
        10.0,
        lane=candidate(LANE_COMMAND, 10.0),
        cone=cone,
    )

    assert decision.source is ControlSource.STOP
    assert decision.command == DriveCommand(0.0, 0.0)


@pytest.mark.parametrize("mode", [Mode.LANE_DRIVE, Mode.CONE_DRIVE, Mode.SHORTCUT])
def test_age_boundary_is_inclusive(mode):
    selected = LANE_COMMAND if mode is Mode.LANE_DRIVE else CONE_COMMAND
    decision = ControlSelector().select(
        mode,
        10.25,
        lane=candidate(selected, 10.0),
        cone=candidate(selected, 10.0),
    )

    expected = (
        ControlSource.CONE if mode is Mode.CONE_DRIVE else ControlSource.LANE
    )
    assert decision.source is expected


@pytest.mark.parametrize("received_at", [math.nan, math.inf, True, 10.01])
def test_invalid_or_future_candidate_timestamp_stops(received_at):
    decision = ControlSelector().select(
        Mode.CONE_DRIVE,
        10.0,
        lane=candidate(LANE_COMMAND, 10.0),
        cone=candidate(CONE_COMMAND, received_at),
    )

    assert decision.source is ControlSource.STOP


@pytest.mark.parametrize("now", [math.nan, math.inf, True])
def test_invalid_selection_timestamp_stops(now):
    decision = ControlSelector().select(
        Mode.CONE_DRIVE,
        now,
        cone=candidate(CONE_COMMAND, 10.0),
    )

    assert decision.source is ControlSource.STOP
    assert decision.reason == "invalid selection timestamp"


def test_invalid_command_value_stops():
    decision = ControlSelector().select(
        Mode.CONE_DRIVE,
        10.0,
        cone=candidate(DriveCommand(math.nan, 4.0), 10.0),
    )

    assert decision.source is ControlSource.STOP
    assert decision.reason == "invalid cone command value"


def test_malformed_command_candidate_stops():
    decision = ControlSelector().select(
        Mode.CONE_DRIVE,
        10.0,
        cone=CommandCandidate(command=None, received_at=10.0),
    )

    assert decision.source is ControlSource.STOP
    assert decision.reason == "invalid cone command candidate"


def test_clock_shift_does_not_change_freshness_decision():
    selector = ControlSelector()
    baseline = selector.select(
        Mode.CONE_DRIVE,
        10.2,
        cone=candidate(CONE_COMMAND, 10.0),
    )

    for shift in (5.0, 20.0, 60.0, 3600.0):
        shifted = selector.select(
            Mode.CONE_DRIVE,
            10.2 + shift,
            cone=candidate(CONE_COMMAND, 10.0 + shift),
        )
        assert shifted == baseline


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("max_lane_age_s", -0.1),
        ("max_lane_age_s", math.nan),
        ("max_cone_age_s", math.inf),
        ("max_cone_age_s", True),
    ],
)
def test_selector_config_rejects_invalid_age(field, value):
    with pytest.raises(ValueError):
        ControlSelectorConfig(**{field: value})


def test_fsm_commit_changes_source_to_cone_in_the_same_cycle():
    fsm = RaceFSM(initial_state=Mode.LANE_DRIVE)
    context = RaceContext(state_entered_at=0.0)
    selector = ControlSelector()

    for timestamp in (1.0, 1.1):
        result = commit_fsm_then_select_control(
            fsm,
            cone_observation(timestamp),
            context,
            safe(),
            selector,
            lane=candidate(LANE_COMMAND, timestamp),
            cone=candidate(CONE_COMMAND, timestamp),
        )
        assert result.transition.changed is False
        assert result.control.source is ControlSource.LANE

    committed = commit_fsm_then_select_control(
        fsm,
        cone_observation(1.21),
        context,
        safe(),
        selector,
        lane=candidate(LANE_COMMAND, 1.21),
        cone=candidate(CONE_COMMAND, 1.21),
    )

    assert committed.transition.source is Mode.LANE_DRIVE
    assert committed.transition.target is Mode.CONE_DRIVE
    assert committed.control.source is ControlSource.CONE
    assert context.state_entered_at == 1.21
    assert context.cone_entered_at == 1.21


def test_same_cached_cone_event_does_not_retransition_after_commit():
    fsm = RaceFSM(initial_state=Mode.LANE_DRIVE)
    context = RaceContext()
    selector = ControlSelector()

    for timestamp in (1.0, 1.1, 1.21):
        result = commit_fsm_then_select_control(
            fsm,
            cone_observation(timestamp),
            context,
            safe(),
            selector,
            cone=candidate(CONE_COMMAND, timestamp),
        )

    repeated = commit_fsm_then_select_control(
        fsm,
        cone_observation(1.21),
        context,
        safe(),
        selector,
        lane=candidate(LANE_COMMAND, 1.21),
        cone=candidate(CONE_COMMAND, 1.21),
    )

    assert result.transition.target is Mode.CONE_DRIVE
    assert repeated.transition.changed is False
    assert repeated.control.source is ControlSource.CONE


def test_fresh_lane_update_cannot_take_priority_while_cone_drive():
    fsm = RaceFSM(initial_state=Mode.CONE_DRIVE)
    result = commit_fsm_then_select_control(
        fsm,
        MissionObservation(now=4.1),
        RaceContext(state_entered_at=3.0),
        safe(),
        ControlSelector(),
        lane=candidate(LANE_COMMAND, 4.1),
        cone=candidate(CONE_COMMAND, 4.0),
    )

    assert result.transition.changed is False
    assert result.control.source is ControlSource.CONE


def test_transition_cycle_without_cone_command_stops_instead_of_using_lane():
    fsm = RaceFSM(initial_state=Mode.LANE_DRIVE)
    context = RaceContext()
    selector = ControlSelector()

    for timestamp in (1.0, 1.1):
        fsm.step(cone_observation(timestamp), context, safe())

    committed = commit_fsm_then_select_control(
        fsm,
        cone_observation(1.21),
        context,
        safe(),
        selector,
        lane=candidate(LANE_COMMAND, 1.21),
        cone=None,
    )

    assert committed.transition.target is Mode.CONE_DRIVE
    assert committed.control.source is ControlSource.STOP
    assert committed.control.command == DriveCommand(0.0, 0.0)
