import math

import pytest

from main.control_selector import (
    CommandCandidate,
    ControlSelector,
    ControlSource,
    DriveCommand,
    commit_fsm_then_select_control,
)
from main.lane_validity import LaneValidityConfig, LaneValidityDebouncer
from main.mission_observation import MissionObservation
from main.race_context import RaceContext
from main.race_fsm import Mode, RaceFSM
from main.runtime_adapter import RaceRuntimeAdapter
from main.safety_monitor import SafetyDecision


def observation(now, valid, received_at):
    return MissionObservation(
        now=now,
        lane_valid=valid,
        lane_valid_received_at=received_at,
    )


def safe():
    return SafetyDecision(inputs_ready=True)


def lane_candidate(received_at):
    return CommandCandidate(DriveCommand(2.0, 5.0), received_at)


def test_rejoin_requires_three_fresh_validity_edges_and_duration():
    fsm = RaceFSM(initial_state=Mode.REJOIN)
    context = RaceContext(state_entered_at=1.0)

    first = fsm.step(observation(1.1, True, 1.1), context, safe())
    second = fsm.step(observation(1.2, True, 1.2), context, safe())
    third = fsm.step(observation(1.31, True, 1.31), context, safe())

    assert first.changed is False
    assert second.changed is False
    assert third.source is Mode.REJOIN
    assert third.target is Mode.LANE_DRIVE
    assert third.reason == "fresh lane validity confirmed"
    assert context.state_entered_at == 1.31


def test_no_lane_validity_publisher_keeps_rejoin_and_selects_stop():
    fsm = RaceFSM(initial_state=Mode.REJOIN)
    context = RaceContext(state_entered_at=1.0)

    result = commit_fsm_then_select_control(
        fsm,
        MissionObservation(now=1.1),
        context,
        safe(),
        ControlSelector(),
        lane=lane_candidate(1.1),
    )

    assert result.transition.changed is False
    assert fsm.state is Mode.REJOIN
    assert result.control.source is ControlSource.STOP
    assert result.control.command == DriveCommand(0.0, 0.0)


def test_legacy_rejoin_stops_until_commit_then_enters_lane():
    fsm = RaceFSM(initial_state=Mode.REJOIN)
    context = RaceContext(state_entered_at=1.0)
    selector = ControlSelector()

    for timestamp in (1.1, 1.2):
        result = commit_fsm_then_select_control(
            fsm,
            observation(timestamp, True, timestamp),
            context,
            safe(),
            selector,
            lane=lane_candidate(timestamp),
        )
        assert result.control.source is ControlSource.STOP

    committed = commit_fsm_then_select_control(
        fsm,
        observation(1.31, True, 1.31),
        context,
        safe(),
        selector,
        lane=lane_candidate(1.31),
    )

    assert committed.transition.target is Mode.LANE_DRIVE
    assert committed.control.source is ControlSource.LANE
    assert committed.control.command == DriveCommand(2.0, 5.0)


# 0.4초 수신은 now=1.31 기준 0.91초 전이라 max_lane_age_s(0.8)를 넘는다.
@pytest.mark.parametrize("candidate", [None, lane_candidate(0.4)])
def test_rejoin_to_lane_without_fresh_lane_command_selects_stop(candidate):
    config = LaneValidityConfig(min_messages=1, min_duration_s=0.0)
    fsm = RaceFSM(
        initial_state=Mode.REJOIN,
        lane_validity_config=config,
    )
    context = RaceContext(state_entered_at=1.0)

    committed = commit_fsm_then_select_control(
        fsm,
        observation(1.31, True, 1.31),
        context,
        safe(),
        ControlSelector(),
        lane=candidate,
    )

    assert committed.transition.target is Mode.LANE_DRIVE
    assert committed.control.source is ControlSource.STOP
    assert committed.control.command == DriveCommand(0.0, 0.0)


def test_false_validity_resets_rejoin_debounce():
    fsm = RaceFSM(initial_state=Mode.REJOIN)
    context = RaceContext(state_entered_at=1.0)
    fsm.step(observation(1.1, True, 1.1), context, safe())
    reset = fsm.step(observation(1.2, False, 1.2), context, safe())

    for timestamp in (1.3, 1.4):
        transition = fsm.step(
            observation(timestamp, True, timestamp),
            context,
            safe(),
        )
        assert transition.changed is False

    assert reset.changed is False
    assert reset.reason == "lane validity not confirmed"
    assert fsm.state is Mode.REJOIN


@pytest.mark.parametrize(
    ("now", "received_at"),
    [
        (1.1, 0.9),
        (1.1, 1.0),
        (2.0, 1.1),
        (1.1, 1.2),
        (1.1, math.nan),
        (1.1, math.inf),
    ],
)
def test_invalid_preentry_stale_future_or_nan_validity_does_not_transition(
    now,
    received_at,
):
    fsm = RaceFSM(initial_state=Mode.REJOIN)
    context = RaceContext(state_entered_at=1.0)

    transition = fsm.step(
        observation(now, True, received_at),
        context,
        safe(),
    )

    assert transition.changed is False
    assert fsm.state is Mode.REJOIN


def test_duplicate_and_regressed_validity_do_not_advance_guard():
    fsm = RaceFSM(initial_state=Mode.REJOIN)
    context = RaceContext(state_entered_at=1.0)
    fsm.step(observation(1.1, True, 1.1), context, safe())
    duplicate = fsm.step(observation(1.11, True, 1.1), context, safe())
    regressed = fsm.step(observation(1.12, True, 1.05), context, safe())

    assert duplicate.reason == "duplicate lane-validity timestamp"
    assert regressed.reason == "lane-validity timestamp regression"
    assert fsm.lane_validity_guard.valid_message_count == 0
    assert fsm.state is Mode.REJOIN


def test_safety_stop_has_priority_over_completed_lane_validity_sequence():
    config = LaneValidityConfig(min_messages=1, min_duration_s=0.0)
    fsm = RaceFSM(
        initial_state=Mode.REJOIN,
        lane_validity_config=config,
    )
    context = RaceContext(state_entered_at=1.0)

    transition = fsm.step(
        observation(1.1, True, 1.1),
        context,
        SafetyDecision(must_stop=True, reason="camera fault"),
    )

    assert transition.target is Mode.STOP
    assert context.stop_reason == "camera fault"


def test_runtime_lane_validity_seam_consumes_each_edge_once():
    adapter = RaceRuntimeAdapter(
        fsm=RaceFSM(initial_state=Mode.REJOIN),
        context=RaceContext(state_entered_at=1.0),
    )
    adapter.record_lane_validity(True, 1.1)

    first = adapter.step(1.1)
    second = adapter.step(1.12)

    assert first.observation.lane_valid is True
    assert first.observation.lane_valid_received_at == 1.1
    assert second.observation.lane_valid is False
    assert second.observation.lane_valid_received_at is None


def test_rejoin_guard_rearms_for_a_later_normal_lap():
    fsm = RaceFSM(initial_state=Mode.REJOIN)
    context = RaceContext(state_entered_at=1.0)
    for timestamp in (1.1, 1.2, 1.31):
        first = fsm.step(observation(timestamp, True, timestamp), context, safe())
    assert first.target is Mode.LANE_DRIVE

    fsm.state = Mode.REJOIN
    context.state_entered_at = 2.0
    for timestamp in (2.1, 2.2):
        second = fsm.step(observation(timestamp, True, timestamp), context, safe())
        assert second.changed is False
    second = fsm.step(observation(2.31, True, 2.31), context, safe())

    assert second.target is Mode.LANE_DRIVE
    assert context.state_entered_at == 2.31


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("min_messages", 0),
        ("min_messages", True),
        ("min_duration_s", -0.1),
        ("max_age_s", math.nan),
    ],
)
def test_lane_validity_config_rejects_invalid_values(field, value):
    with pytest.raises(ValueError):
        LaneValidityConfig(**{field: value})


def test_lane_validity_debouncer_no_event_does_not_reset_active_sequence():
    guard = LaneValidityDebouncer()
    guard.evaluate(observation(1.1, True, 1.1), 1.0)

    decision = guard.evaluate(MissionObservation(now=1.15), 1.0)

    assert decision.reason == "no new lane-validity message"
    assert guard.valid_message_count == 1
