import pytest

from main.mission_types import PendingRouteAction, RouteTrafficSignal
from main.race_context import RaceContext
from main.race_fsm import Mode, RaceFSM
from main.runtime_adapter import RaceRuntimeAdapter


def runtime(mode, completed_laps=0):
    return RaceRuntimeAdapter(
        fsm=RaceFSM(initial_state=mode),
        context=RaceContext(
            completed_laps=completed_laps,
            state_entered_at=0.0,
        ),
    )


def encounter(adapter, signal, timestamp, *, step_at=None):
    assert adapter.record_route_traffic(
        signal,
        timestamp,
        encounter_started=True,
    ).accepted
    return adapter.step(timestamp if step_at is None else step_at)


def test_straight_encounter_during_cone_counts_without_interrupting_cone():
    adapter = runtime(Mode.CONE_DRIVE)
    cycle = encounter(adapter, RouteTrafficSignal.STRAIGHT, 1.0)
    assert cycle.transition.changed is False
    assert adapter.fsm.state is Mode.CONE_DRIVE
    assert adapter.context.completed_laps == 1
    assert adapter.context.pending_route_action is None


def test_cone_exit_then_commits_pending_shortcut_without_ttl_loss():
    adapter = runtime(Mode.CONE_DRIVE, completed_laps=1)
    encounter(adapter, RouteTrafficSignal.LEFT, 1.0, step_at=10.0)
    assert adapter.context.completed_laps == 2
    assert adapter.context.pending_route_action is PendingRouteAction.SHORTCUT
    assert adapter.fsm.state is Mode.CONE_DRIVE

    assert adapter.record_cone_message([0, 0, 80], 10.1).accepted
    adapter.step(10.1)
    assert adapter.record_cone_message([0, 1, 0], 10.2).accepted
    assert adapter.step(10.2).transition.target is Mode.LANE_DRIVE
    committed = adapter.step(11.0)
    assert committed.transition.target is Mode.SHORTCUT
    assert adapter.context.pending_route_action is None


@pytest.mark.parametrize("mode", [Mode.FIXED_AVOID, Mode.OVERTAKE])
def test_encounter_during_object_mission_is_not_lost(mode):
    adapter = runtime(mode)
    encounter(adapter, RouteTrafficSignal.STRAIGHT, 1.0)
    assert adapter.fsm.state is mode
    assert adapter.context.completed_laps == 1


@pytest.mark.parametrize(
    ("mode", "exit_recorder"),
    [
        (Mode.FIXED_AVOID, "record_fixed_zone_exit"),
        (Mode.OVERTAKE, "record_overtake_complete"),
    ],
)
def test_third_encounter_finishes_only_after_object_mission_exit(
    mode,
    exit_recorder,
):
    adapter = runtime(mode, completed_laps=2)
    encounter(adapter, RouteTrafficSignal.STRAIGHT, 1.0)
    assert adapter.fsm.state is mode
    assert adapter.context.completed_laps == 3
    assert adapter.context.pending_route_action is PendingRouteAction.FINISH

    assert getattr(adapter, exit_recorder)(2.0).accepted
    assert adapter.step(2.0).transition.target is Mode.LANE_DRIVE
    assert adapter.step(5.0).transition.target is Mode.FINISH


def test_pending_finish_overrides_pending_shortcut():
    adapter = runtime(Mode.OVERTAKE, completed_laps=1)
    encounter(adapter, RouteTrafficSignal.LEFT, 1.0)
    assert adapter.context.pending_route_action is PendingRouteAction.SHORTCUT
    encounter(adapter, RouteTrafficSignal.STRAIGHT, 2.0)
    assert adapter.context.completed_laps == 3
    assert adapter.context.pending_route_action is PendingRouteAction.FINISH


def test_approved_encounter_survives_safety_hold_and_normal_ttl():
    adapter = runtime(Mode.CONE_DRIVE)
    assert adapter.record_route_traffic(
        RouteTrafficSignal.STRAIGHT,
        1.0,
        encounter_started=True,
    ).accepted

    held = adapter.step(1.0, fault_reason="synthetic hold")
    assert held.transition.target is Mode.CONE_DRIVE
    assert adapter.context.completed_laps == 0

    recovered = adapter.step(10.0)
    assert recovered.transition.target is Mode.CONE_DRIVE
    assert adapter.context.completed_laps == 1


def test_new_context_clears_pending_route_action():
    context = RaceContext(pending_route_action=PendingRouteAction.FINISH)
    assert context.pending_route_action is PendingRouteAction.FINISH
    assert RaceContext().pending_route_action is None
