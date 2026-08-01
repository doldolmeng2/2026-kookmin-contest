import math

import pytest

from main.cone_entry import ConeEntryConfig, ConeEntryDebouncer
from main.mission_observation import MissionObservation
from main.race_context import RaceContext
from main.race_fsm import Mode, RaceFSM
from main.safety_monitor import SafetyDecision


DEFAULT_SCAN = object()


def observation(
    now,
    *,
    cone_at=None,
    confidence=90,
    end_flag=False,
    scan_at=DEFAULT_SCAN,
):
    if scan_at is DEFAULT_SCAN:
        scan_at = cone_at
    return MissionObservation(
        now=now,
        cone_confidence=confidence,
        cone_end_flag=end_flag,
        cone_message_received_at=cone_at,
        scan_received_at=scan_at,
    )


def test_provisional_default_config_is_centralized():
    assert ConeEntryConfig() == ConeEntryConfig(
        min_confidence=75,
        min_messages=3,
        min_duration_s=0.2,
        max_cone_age_s=0.25,
        max_scan_age_s=0.25,
    )


@pytest.mark.parametrize("value", [-1, 101, 75.0, True])
def test_config_rejects_invalid_confidence(value):
    with pytest.raises(ValueError):
        ConeEntryConfig(min_confidence=value)


@pytest.mark.parametrize("value", [0, -1, 3.0, True])
def test_config_rejects_invalid_message_count(value):
    with pytest.raises(ValueError):
        ConeEntryConfig(min_messages=value)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("min_duration_s", -0.1),
        ("max_cone_age_s", -0.1),
        ("max_scan_age_s", -0.1),
        ("min_duration_s", math.nan),
        ("max_cone_age_s", math.inf),
        ("max_scan_age_s", True),
    ],
)
def test_config_rejects_invalid_duration_and_age(field, value):
    with pytest.raises(ValueError):
        ConeEntryConfig(**{field: value})


def test_one_high_confidence_message_does_not_trigger():
    guard = ConeEntryDebouncer()

    decision = guard.evaluate(observation(1.0, cone_at=1.0))

    assert decision.triggered is False
    assert guard.qualifying_message_count == 1
    assert guard.first_qualifying_at == 1.0


def test_duplicate_cone_timestamp_does_not_increment_or_move_start():
    guard = ConeEntryDebouncer()
    guard.evaluate(observation(1.0, cone_at=1.0))

    duplicate = guard.evaluate(observation(1.1, cone_at=1.0))

    assert duplicate.reason == "duplicate cone message timestamp"
    assert duplicate.new_message is False
    assert guard.qualifying_message_count == 1
    assert guard.first_qualifying_at == 1.0


def test_only_distinct_new_cone_messages_are_counted():
    guard = ConeEntryDebouncer(
        ConeEntryConfig(min_messages=4, min_duration_s=1.0)
    )

    guard.evaluate(observation(1.0, cone_at=1.0))
    guard.evaluate(observation(1.0, cone_at=1.0))
    guard.evaluate(observation(1.1, cone_at=1.1))
    guard.evaluate(observation(1.1, cone_at=1.1))

    assert guard.qualifying_message_count == 2


def test_message_count_without_duration_does_not_trigger():
    guard = ConeEntryDebouncer()

    for timestamp in (1.0, 1.05, 1.1):
        decision = guard.evaluate(
            observation(timestamp, cone_at=timestamp)
        )

    assert guard.qualifying_message_count == 3
    assert decision.elapsed_s == pytest.approx(0.1)
    assert decision.triggered is False


def test_duration_without_message_count_does_not_trigger():
    guard = ConeEntryDebouncer()

    guard.evaluate(observation(1.0, cone_at=1.0))
    decision = guard.evaluate(observation(1.3, cone_at=1.3))

    assert guard.qualifying_message_count == 2
    assert decision.elapsed_s == pytest.approx(0.3)
    assert decision.triggered is False


def test_message_count_and_duration_together_trigger_once():
    guard = ConeEntryDebouncer()

    guard.evaluate(observation(1.0, cone_at=1.0))
    guard.evaluate(observation(1.1, cone_at=1.1))
    decision = guard.evaluate(observation(1.21, cone_at=1.21))
    later = guard.evaluate(observation(1.31, cone_at=1.31))

    assert decision.triggered is True
    assert decision.reason == "cone entry confirmed"
    assert later.triggered is False
    assert later.reason == "cone entry already triggered"


def test_duration_and_freshness_boundaries_are_inclusive():
    guard = ConeEntryDebouncer()

    guard.evaluate(
        observation(1.25, cone_at=1.0, scan_at=1.0)
    )
    guard.evaluate(
        observation(1.35, cone_at=1.1, scan_at=1.1)
    )
    decision = guard.evaluate(
        observation(1.45, cone_at=1.2, scan_at=1.2)
    )

    assert decision.triggered is True


@pytest.mark.parametrize(
    ("confidence", "end_flag", "reason"),
    [
        (74, False, "cone confidence below threshold"),
        (None, False, "cone confidence missing"),
        (90, True, "cone end flag not clear"),
    ],
)
def test_nonqualifying_new_cone_message_resets_sequence(
    confidence,
    end_flag,
    reason,
):
    guard = ConeEntryDebouncer()
    guard.evaluate(observation(1.0, cone_at=1.0))
    guard.evaluate(observation(1.1, cone_at=1.1))

    decision = guard.evaluate(
        observation(
            1.2,
            cone_at=1.2,
            confidence=confidence,
            end_flag=end_flag,
        )
    )

    assert decision.reason == reason
    assert decision.sequence_reset is True
    assert guard.qualifying_message_count == 0
    assert guard.first_qualifying_at is None


def test_missing_scan_prevents_entry_and_resets_sequence():
    guard = ConeEntryDebouncer()
    guard.evaluate(observation(1.0, cone_at=1.0))

    decision = guard.evaluate(
        observation(1.1, cone_at=1.1, scan_at=None)
    )

    assert decision.reason == "scan timestamp missing"
    assert decision.sequence_reset is True
    assert guard.qualifying_message_count == 0


def test_stale_scan_over_limit_resets_sequence():
    guard = ConeEntryDebouncer()
    guard.evaluate(observation(1.0, cone_at=1.0))

    decision = guard.evaluate(
        observation(1.4, cone_at=1.4, scan_at=1.0)
    )

    assert decision.reason == "scan stale"
    assert decision.sequence_reset is True
    assert guard.qualifying_message_count == 0


def test_stale_cone_over_limit_resets_sequence():
    guard = ConeEntryDebouncer()
    guard.evaluate(observation(1.0, cone_at=1.0))

    decision = guard.evaluate(
        observation(1.4, cone_at=1.1, scan_at=1.4)
    )

    assert decision.reason == "cone message stale"
    assert decision.sequence_reset is True
    assert guard.qualifying_message_count == 0


def test_timestamp_regression_resets_and_rejects_event():
    guard = ConeEntryDebouncer()
    guard.evaluate(observation(2.0, cone_at=2.0))
    guard.evaluate(observation(2.1, cone_at=2.1))

    regression = guard.evaluate(observation(2.05, cone_at=2.05))
    next_new = guard.evaluate(observation(2.2, cone_at=2.2))

    assert regression.reason == "cone message timestamp regression"
    assert regression.new_message is False
    assert regression.sequence_reset is True
    assert next_new.qualifying_message_count == 1


def test_invalid_cone_timestamp_resets_and_rejects_event():
    guard = ConeEntryDebouncer()
    guard.evaluate(observation(1.0, cone_at=1.0))

    decision = guard.evaluate(observation(1.1, cone_at=math.nan))

    assert decision.reason == "invalid cone message timestamp"
    assert decision.new_message is False
    assert decision.sequence_reset is True
    assert guard.qualifying_message_count == 0


def test_non_cone_event_does_not_recount_or_reset_cached_sequence():
    guard = ConeEntryDebouncer()
    guard.evaluate(observation(1.0, cone_at=1.0))

    no_event = guard.evaluate(MissionObservation(now=1.1, scan_received_at=1.1))

    assert no_event.reason == "no new cone message"
    assert guard.qualifying_message_count == 1
    assert guard.first_qualifying_at == 1.0


def test_short_confidence_spike_is_rejected_by_persistence():
    guard = ConeEntryDebouncer()
    high = guard.evaluate(observation(1.0, cone_at=1.0, confidence=100))
    low = guard.evaluate(observation(1.1, cone_at=1.1, confidence=20))

    assert high.triggered is False
    assert low.triggered is False
    assert guard.qualifying_message_count == 0


def test_repeated_below_threshold_values_never_start_episode():
    guard = ConeEntryDebouncer()

    for timestamp in (1.0, 1.1, 1.2, 1.3):
        decision = guard.evaluate(
            observation(timestamp, cone_at=timestamp, confidence=74)
        )

    assert decision.triggered is False
    assert guard.qualifying_message_count == 0
    assert guard.first_qualifying_at is None


def test_two_high_messages_then_break_do_not_carry_into_next_episode():
    guard = ConeEntryDebouncer()
    guard.evaluate(observation(1.0, cone_at=1.0))
    guard.evaluate(observation(1.1, cone_at=1.1))
    guard.evaluate(observation(1.2, cone_at=1.2, confidence=10))
    guard.evaluate(observation(1.3, cone_at=1.3))
    decision = guard.evaluate(observation(1.6, cone_at=1.6))

    assert decision.triggered is False
    assert guard.qualifying_message_count == 2
    assert guard.first_qualifying_at == 1.3


def test_stale_scan_between_high_messages_breaks_episode():
    guard = ConeEntryDebouncer()
    guard.evaluate(observation(1.0, cone_at=1.0))
    guard.evaluate(observation(1.1, cone_at=1.1))
    guard.evaluate(observation(1.5, cone_at=1.5, scan_at=1.1))
    guard.evaluate(observation(1.6, cone_at=1.6))
    decision = guard.evaluate(observation(1.9, cone_at=1.9))

    assert decision.triggered is False
    assert guard.qualifying_message_count == 2


def test_cached_end_flag_message_cannot_be_reinterpreted_as_entry():
    guard = ConeEntryDebouncer()
    guard.evaluate(observation(1.0, cone_at=1.0))
    end = guard.evaluate(
        observation(1.1, cone_at=1.1, confidence=100, end_flag=True)
    )
    cached = guard.evaluate(
        observation(1.2, cone_at=1.1, confidence=100, end_flag=False)
    )

    assert end.reason == "cone end flag not clear"
    assert cached.reason == "duplicate cone message timestamp"
    assert guard.qualifying_message_count == 0


def run_shifted_fsm_sequence(shift):
    fsm = RaceFSM(initial_state=Mode.LANE_DRIVE)
    context = RaceContext(state_entered_at=shift)
    transitions = []

    for relative_time in (1.0, 1.1, 1.21):
        timestamp = shift + relative_time
        transition = fsm.step(
            observation(timestamp, cone_at=timestamp),
            context,
            SafetyDecision(inputs_ready=True),
        )
        if transition.changed:
            transitions.append(transition)

    return {
        "final_mode": fsm.state,
        "transition_count": len(transitions),
        "reason": transitions[0].reason,
        "transition_time": context.cone_entered_at,
        "first_qualifying_time": fsm.cone_entry_guard.first_qualifying_at,
        "trigger_delay": (
            context.cone_entered_at
            - fsm.cone_entry_guard.first_qualifying_at
        ),
        "qualifying_messages": fsm.cone_entry_guard.qualifying_message_count,
    }


@pytest.mark.parametrize("shift", [5.0, 20.0, 60.0, 3600.0])
def test_cone_entry_is_invariant_to_finite_timestamp_shift(shift):
    baseline = run_shifted_fsm_sequence(0.0)
    shifted = run_shifted_fsm_sequence(shift)

    assert shifted["final_mode"] is baseline["final_mode"] is Mode.CONE_DRIVE
    assert shifted["transition_count"] == baseline["transition_count"] == 1
    assert shifted["reason"] == baseline["reason"] == "cone entry confirmed"
    assert shifted["qualifying_messages"] == baseline["qualifying_messages"] == 3
    assert shifted["trigger_delay"] == pytest.approx(
        baseline["trigger_delay"],
        abs=1e-9,
    )
    assert shifted["transition_time"] - baseline["transition_time"] == (
        pytest.approx(shift, abs=1e-9)
    )
    assert shifted["first_qualifying_time"] - baseline[
        "first_qualifying_time"
    ] == pytest.approx(shift, abs=1e-9)
