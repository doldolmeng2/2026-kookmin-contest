import pytest

from main.control_selector import CommandCandidate, DriveCommand
from main.mission_observation import MissionObservation
from main.mission_types import RouteTrafficSignal
from main.race_context import RaceContext
from main.race_fsm import Mode, RaceFSM
from main.runtime_adapter import RaceRuntimeAdapter
from main.safety_monitor import SafetyDecision


SAFE = SafetyDecision(inputs_ready=True)


def route(now, signal, *, received_at=None, encounter=True):
    received_at = now if received_at is None else received_at
    return MissionObservation(
        now=now,
        route_traffic_signal=signal,
        route_traffic_received_at=received_at,
        traffic_encounter_started=encounter,
        traffic_encounter_received_at=received_at if encounter else None,
    )


def test_lap_one_left_without_a_committed_encounter_cannot_enter_shortcut():
    fsm = RaceFSM(initial_state=Mode.LANE_DRIVE)
    context = RaceContext(state_entered_at=0.0)
    result = fsm.step(route(0.1, RouteTrafficSignal.LEFT, encounter=False), context, SAFE)
    assert result.changed is False
    assert context.current_lap == 1
    assert context.shortcut_used is False


def test_lap_two_and_lap_three_left_select_shortcut():
    lap_two_fsm = RaceFSM(initial_state=Mode.LANE_DRIVE)
    lap_two_context = RaceContext(state_entered_at=0.0)
    assert lap_two_fsm.step(
        route(1.0, RouteTrafficSignal.LEFT), lap_two_context, SAFE
    ).target is Mode.SHORTCUT


@pytest.mark.parametrize(
    "signal",
    [
        RouteTrafficSignal.RED_AMBER,
        RouteTrafficSignal.STRAIGHT,
        RouteTrafficSignal.UNKNOWN,
    ],
)
def test_red_orange_straight_and_unknown_cannot_enter_shortcut(signal):
    fsm = RaceFSM(initial_state=Mode.LANE_DRIVE)
    context = RaceContext(completed_laps=1, state_entered_at=1.0)
    result = fsm.step(route(1.1, signal), context, SAFE)
    assert result.target is Mode.LANE_DRIVE
    assert context.shortcut_used is False

    lap_three_fsm = RaceFSM(initial_state=Mode.LANE_DRIVE)
    lap_three_context = RaceContext(completed_laps=1, state_entered_at=1.0)
    assert lap_three_fsm.step(
        route(2.0, RouteTrafficSignal.LEFT), lap_three_context, SAFE
    ).target is Mode.SHORTCUT


def test_shortcut_is_one_shot_and_non_left_signals_do_not_enter():
    fsm = RaceFSM(initial_state=Mode.LANE_DRIVE)
    context = RaceContext(state_entered_at=0.0)
    entered = fsm.step(route(1.0, RouteTrafficSignal.LEFT), context, SAFE)
    assert entered.target is Mode.SHORTCUT
    assert context.shortcut_lap == 2
    assert fsm.step(
        MissionObservation(
            now=1.1,
            shortcut_complete=True,
            shortcut_complete_received_at=1.1,
        ),
        context,
        SAFE,
    ).target is Mode.LANE_DRIVE
    repeated = fsm.step(route(2.0, RouteTrafficSignal.LEFT), context, SAFE)
    assert repeated.changed is False
    assert context.shortcut_lap == 2


@pytest.mark.parametrize(
    ("now", "received_at"),
    [(2.0, 1.0), (1.0, 2.0), (2.0, 0.5)],
    ids=("stale", "future", "pre-session"),
)
def test_invalid_left_receipt_cannot_enter_shortcut(now, received_at):
    fsm = RaceFSM(initial_state=Mode.LANE_DRIVE, mission_event_max_age_s=0.25)
    context = RaceContext(completed_laps=1, state_entered_at=1.0)
    result = fsm.step(
        route(now, RouteTrafficSignal.LEFT, received_at=received_at),
        context,
        SAFE,
    )
    assert result.changed is False
    assert context.shortcut_used is False


def test_safety_stop_precedes_simultaneous_shortcut_selection():
    fsm = RaceFSM(initial_state=Mode.LANE_DRIVE)
    context = RaceContext(completed_laps=1, state_entered_at=1.0)
    result = fsm.step(
        route(1.1, RouteTrafficSignal.LEFT),
        context,
        SafetyDecision(must_stop=True, reason="synthetic stale lane"),
    )
    assert result.target is Mode.STOP
    assert context.shortcut_used is False


def test_stop_discards_old_shortcut_exit_evidence_before_recovery():
    adapter = RaceRuntimeAdapter(
        fsm=RaceFSM(initial_state=Mode.SHORTCUT),
        context=RaceContext(
            completed_laps=1,
            shortcut_lap=2,
            state_entered_at=1.0,
        ),
    )
    adapter.record_shortcut_complete(1.1)
    stopped = adapter.step(1.1, fault_reason="synthetic fault")
    assert stopped.transition.target is Mode.STOP

    held = adapter.step(1.2)
    recovered = adapter.step(1.8)
    assert held.transition.changed is False
    assert recovered.transition.target is Mode.SHORTCUT

    command = CommandCandidate(DriveCommand(0.0, 5.0), received_at=1.9)
    next_cycle = adapter.step(1.9, lane=command)
    assert next_cycle.transition.changed is False
    assert adapter.fsm.state is Mode.SHORTCUT
