import math

import pytest

from main.cone_entry import ConeEntryConfig
from main.mission_observation import MissionObservation
from main.mission_types import RouteTrafficSignal
from main.race_context import RaceContext
from main.race_fsm import Mode, RaceFSM
from main.safety_monitor import SafetyDecision


SAFE = SafetyDecision(inputs_ready=True)


def route_observation(now, signal, *, encounter=True, received_at=None):
    timestamp = now if received_at is None else received_at
    return MissionObservation(
        now=now,
        route_traffic_signal=signal,
        route_traffic_received_at=timestamp,
        traffic_encounter_started=encounter,
        traffic_encounter_received_at=timestamp if encounter else None,
    )


_UNSET = object()


def edge_observation(now, name, received_at=_UNSET):
    timestamp = now if received_at is _UNSET else received_at
    fields = {
        "fixed_avoid_complete": {
            "fixed_avoid_complete": True,
            "fixed_avoid_completed_at": timestamp,
        },
        "overtake_complete": {
            "overtake_complete": True,
            "overtake_complete_received_at": timestamp,
        },
        "shortcut_complete": {
            "shortcut_complete": True,
            "shortcut_complete_received_at": timestamp,
        },
    }
    return MissionObservation(now=now, **fields[name])


def cone_observation(now, *, end_flag=False, confidence=90):
    return MissionObservation(
        now=now,
        cone_detected=not end_flag and confidence > 0,
        cone_finished=end_flag,
        cone_confidence=confidence,
        cone_end_flag=end_flag,
        cone_message_received_at=now,
        scan_received_at=now,
    )


def test_initial_green_starts_lap_one_without_incrementing_completed_laps():
    fsm = RaceFSM(
        initial_state=Mode.WAIT_GREEN,
        green_min_consecutive_frames=1,
    )
    context = RaceContext(state_entered_at=1.0)

    transition = fsm.step(
        MissionObservation(
            now=1.1,
            green_detected=True,
            traffic_message_received_at=1.1,
        ),
        context,
        SAFE,
    )

    assert transition.target is Mode.LANE_DRIVE
    assert context.completed_laps == 0
    assert context.current_lap == 1


def test_three_route_encounters_commit_finish_exactly_once():
    fsm = RaceFSM(initial_state=Mode.LANE_DRIVE)
    context = RaceContext(state_entered_at=0.0)

    for lap, timestamp in enumerate((1.0, 2.0), start=1):
        transition = fsm.step(
            route_observation(timestamp, RouteTrafficSignal.STRAIGHT),
            context,
            SAFE,
        )
        assert transition.changed is False
        assert context.completed_laps == lap

    finish = fsm.step(
        route_observation(3.0, RouteTrafficSignal.STRAIGHT),
        context,
        SAFE,
    )
    repeated = fsm.step(
        route_observation(3.1, RouteTrafficSignal.LEFT),
        context,
        SAFE,
    )

    assert finish.target is Mode.FINISH
    assert context.completed_laps == 3
    assert repeated.changed is False
    assert fsm.state is Mode.FINISH


def test_left_signal_without_encounter_cannot_select_shortcut_on_lap_one():
    fsm = RaceFSM(initial_state=Mode.LANE_DRIVE)
    context = RaceContext(state_entered_at=0.0)

    transition = fsm.step(
        route_observation(
            1.0,
            RouteTrafficSignal.LEFT,
            encounter=False,
        ),
        context,
        SAFE,
    )

    assert transition.changed is False
    assert context.current_lap == 1
    assert context.shortcut_lap is None


def test_lap_two_shortcut_is_used_once_and_lap_three_guard_is_restored():
    fsm = RaceFSM(initial_state=Mode.LANE_DRIVE)
    context = RaceContext(state_entered_at=0.0)

    selected = fsm.step(
        route_observation(1.0, RouteTrafficSignal.LEFT),
        context,
        SAFE,
    )
    assert selected.target is Mode.SHORTCUT
    assert context.shortcut_lap == 2

    completed = fsm.step(
        edge_observation(1.2, "shortcut_complete"),
        context,
        SAFE,
    )
    assert completed.target is Mode.LANE_DRIVE

    suppressed = fsm.step(
        MissionObservation(
            now=1.3,
            cone_confidence=90,
            cone_end_flag=False,
            cone_message_received_at=1.3,
            scan_received_at=1.3,
        ),
        context,
        SAFE,
    )
    assert suppressed.changed is False
    assert fsm.state is Mode.LANE_DRIVE

    next_lap = fsm.step(
        route_observation(2.0, RouteTrafficSignal.STRAIGHT),
        context,
        SAFE,
    )
    for timestamp in (2.1, 2.2, 2.31):
        cone_entry = fsm.step(cone_observation(timestamp), context, SAFE)

    assert next_lap.changed is False
    assert context.current_lap == 3
    assert context.on_shortcut_lap is False
    assert cone_entry.target is Mode.CONE_DRIVE


def test_lap_three_shortcut_is_available_if_lap_two_was_straight():
    fsm = RaceFSM(initial_state=Mode.LANE_DRIVE)
    context = RaceContext(state_entered_at=0.0)

    lap_two = fsm.step(
        route_observation(1.0, RouteTrafficSignal.STRAIGHT),
        context,
        SAFE,
    )
    lap_three = fsm.step(
        route_observation(2.0, RouteTrafficSignal.LEFT),
        context,
        SAFE,
    )

    assert lap_two.changed is False
    assert lap_three.target is Mode.SHORTCUT
    assert context.shortcut_lap == 3


def test_duplicate_traffic_encounter_does_not_increment_twice():
    fsm = RaceFSM(initial_state=Mode.LANE_DRIVE)
    context = RaceContext(state_entered_at=0.0)
    observation = route_observation(1.0, RouteTrafficSignal.STRAIGHT)

    fsm.step(observation, context, SAFE)
    duplicate = fsm.step(
        route_observation(
            1.1,
            RouteTrafficSignal.LEFT,
            received_at=1.0,
        ),
        context,
        SAFE,
    )

    assert duplicate.changed is False
    assert context.completed_laps == 1
    assert context.shortcut_lap is None


def test_object_detection_alone_never_enters_fixed_avoid():
    fsm = RaceFSM(initial_state=Mode.LANE_DRIVE)
    context = RaceContext(state_entered_at=1.0)

    transition = fsm.step(
        MissionObservation(
            now=1.1,
            object_exists=True,
            object_received_at=1.1,
        ),
        context,
        SAFE,
    )

    assert transition.changed is False
    assert fsm.state is Mode.LANE_DRIVE


def test_normal_route_rejoin_and_completion_edges_form_mission_chain():
    fsm = RaceFSM(initial_state=Mode.CONE_DRIVE)
    context = RaceContext(state_entered_at=1.0)

    armed = fsm.step(cone_observation(1.1), context, SAFE)
    rejoin = fsm.step(
        cone_observation(1.2, end_flag=True, confidence=0),
        context,
        SAFE,
    )
    first_lane_valid = fsm.step(
        MissionObservation(
            now=1.3,
            lane_valid=True,
            lane_valid_received_at=1.3,
        ),
        context,
        SAFE,
    )
    second_lane_valid = fsm.step(
        MissionObservation(
            now=1.4,
            lane_valid=True,
            lane_valid_received_at=1.4,
        ),
        context,
        SAFE,
    )
    fixed = fsm.step(
        MissionObservation(
            now=1.51,
            lane_valid=True,
            lane_valid_received_at=1.51,
        ),
        context,
        SAFE,
    )
    fixed_lane_success = fsm.step(
        MissionObservation(
            now=1.6,
            lane_change_success=True,
            lane_change_success_edge=True,
            lane_change_received_at=1.6,
        ),
        context,
        SAFE,
    )
    overtake = fsm.step(
        edge_observation(1.7, "fixed_avoid_complete"),
        context,
        SAFE,
    )
    overtake_lane_success = fsm.step(
        MissionObservation(
            now=1.8,
            lane_change_success=True,
            lane_change_success_edge=True,
            lane_change_received_at=1.8,
        ),
        context,
        SAFE,
    )
    lane = fsm.step(
        edge_observation(1.9, "overtake_complete"),
        context,
        SAFE,
    )

    assert armed.changed is False
    assert armed.reason == "cone exit session armed"
    assert rejoin.target is Mode.REJOIN
    assert first_lane_valid.changed is False
    assert second_lane_valid.changed is False
    assert fixed.target is Mode.FIXED_AVOID
    assert fixed_lane_success.changed is False
    assert fixed_lane_success.target is Mode.FIXED_AVOID
    assert overtake.target is Mode.OVERTAKE
    assert overtake_lane_success.changed is False
    assert overtake_lane_success.target is Mode.OVERTAKE
    assert lane.target is Mode.LANE_DRIVE


@pytest.mark.parametrize(
    ("state", "edge_name"),
    [
        (Mode.FIXED_AVOID, "fixed_avoid_complete"),
        (Mode.OVERTAKE, "overtake_complete"),
        (Mode.SHORTCUT, "shortcut_complete"),
    ],
)
def test_pre_entry_stale_and_duplicate_mission_edges_are_ignored(
    state,
    edge_name,
):
    fsm = RaceFSM(initial_state=state, mission_event_max_age_s=0.25)
    context = RaceContext(state_entered_at=10.0)

    pre_entry = fsm.step(
        edge_observation(10.1, edge_name, received_at=9.9),
        context,
        SAFE,
    )
    stale = fsm.step(
        edge_observation(11.0, edge_name, received_at=10.1),
        context,
        SAFE,
    )
    duplicate = fsm.step(
        edge_observation(11.1, edge_name, received_at=10.1),
        context,
        SAFE,
    )

    assert pre_entry.changed is False
    assert stale.changed is False
    assert duplicate.changed is False
    assert fsm.state is state


@pytest.mark.parametrize(
    ("state", "edge_name", "target"),
    [
        (Mode.FIXED_AVOID, "fixed_avoid_complete", Mode.OVERTAKE),
        (Mode.OVERTAKE, "overtake_complete", Mode.LANE_DRIVE),
        (Mode.SHORTCUT, "shortcut_complete", Mode.LANE_DRIVE),
    ],
)
def test_fresh_completion_is_consumed_once_per_state_session(
    state,
    edge_name,
    target,
):
    fsm = RaceFSM(initial_state=state)
    context = RaceContext(state_entered_at=10.0)

    first = fsm.step(edge_observation(10.1, edge_name), context, SAFE)
    assert first.target is target

    fsm.state = state
    context.state_entered_at = 20.0
    replayed = fsm.step(
        edge_observation(20.1, edge_name, received_at=10.1),
        context,
        SAFE,
    )
    assert replayed.changed is False
    assert fsm.state is state

    fresh = fsm.step(edge_observation(20.2, edge_name), context, SAFE)
    assert fresh.target is target


@pytest.mark.parametrize(
    ("state", "edge_name"),
    [
        (Mode.FIXED_AVOID, "fixed_avoid_complete"),
        (Mode.OVERTAKE, "overtake_complete"),
        (Mode.SHORTCUT, "shortcut_complete"),
    ],
)
def test_regressed_future_and_invalid_completion_timestamps_are_ignored(
    state,
    edge_name,
):
    fsm = RaceFSM(initial_state=state, mission_event_max_age_s=0.25)
    context = RaceContext(state_entered_at=10.0)

    stale_seed = fsm.step(
        edge_observation(11.0, edge_name, received_at=10.6),
        context,
        SAFE,
    )
    regressed = fsm.step(
        edge_observation(11.1, edge_name, received_at=10.5),
        context,
        SAFE,
    )

    assert stale_seed.changed is False
    assert regressed.changed is False
    assert fsm.state is state

    for timestamp in (10.2, None, math.nan, math.inf, -math.inf, True):
        isolated_fsm = RaceFSM(initial_state=state)
        isolated_context = RaceContext(state_entered_at=10.0)
        rejected = isolated_fsm.step(
            edge_observation(10.1, edge_name, received_at=timestamp),
            isolated_context,
            SAFE,
        )

        assert rejected.changed is False
        assert isolated_fsm.state is state


def test_finish_has_priority_over_other_simultaneous_lane_events():
    fsm = RaceFSM(
        initial_state=Mode.LANE_DRIVE,
        cone_entry_config=ConeEntryConfig(min_messages=1, min_duration_s=0.0),
    )
    context = RaceContext(completed_laps=2, state_entered_at=1.0)

    transition = fsm.step(
        MissionObservation(
            now=1.1,
            route_traffic_signal=RouteTrafficSignal.LEFT,
            route_traffic_received_at=1.1,
            traffic_encounter_started=True,
            traffic_encounter_received_at=1.1,
            cone_confidence=90,
            cone_end_flag=False,
            cone_message_received_at=1.1,
            scan_received_at=1.1,
        ),
        context,
        SAFE,
    )

    assert transition.target is Mode.FINISH
    assert context.shortcut_lap is None


def test_safety_stop_precedes_all_simultaneous_mission_edges():
    fsm = RaceFSM(initial_state=Mode.FIXED_AVOID)
    context = RaceContext(state_entered_at=1.0)

    transition = fsm.step(
        edge_observation(1.1, "fixed_avoid_complete"),
        context,
        SafetyDecision(must_stop=True, reason="synthetic fault"),
    )

    assert transition.target is Mode.STOP
    assert context.stop_reason == "synthetic fault"
