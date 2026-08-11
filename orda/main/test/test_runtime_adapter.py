import pytest

from main.cone_entry import ConeEntryConfig
from main.control_selector import (
    CommandCandidate,
    ControlSource,
    DriveCommand,
)
from main.mission_observation import MissionObservation
from main.race_context import RaceContext
from main.race_fsm import Mode, RaceFSM
from main.runtime_adapter import (
    RaceRuntimeAdapter,
    dispatch_cone_reset,
    runtime_safety_monitor,
)
from main.safety_monitor import SafetyDecision


def runtime(mode, *, queue_capacity=16, one_message_entry=False):
    config = None
    if one_message_entry:
        config = ConeEntryConfig(min_messages=1, min_duration_s=0.0)
    return RaceRuntimeAdapter(
        fsm=RaceFSM(initial_state=mode, cone_entry_config=config),
        context=RaceContext(state_entered_at=0.5),
        cone_queue_capacity=queue_capacity,
    )


def candidate(angle, speed, received_at):
    return CommandCandidate(DriveCommand(angle, speed), received_at)


def enter_cone(adapter, timestamp=1.0):
    adapter.record_scan(timestamp)
    assert adapter.record_cone_message([4, 0, 90], timestamp).accepted
    cycle = adapter.step(
        timestamp,
        lane=candidate(1.0, 5.0, timestamp),
        cone=candidate(-2.0, 4.0, timestamp),
    )
    assert cycle.transition.source is Mode.LANE_DRIVE
    assert cycle.transition.target is Mode.CONE_DRIVE
    return cycle


def test_one_cone_callback_is_consumed_once_and_next_tick_has_no_edge():
    adapter = runtime(Mode.CONE_DRIVE)
    assert adapter.record_cone_message([7, 1, 0], 1.0).accepted

    first = adapter.step(1.0)
    second = adapter.step(1.02)

    assert first.observation.cone_message_received_at == 1.0
    assert first.observation.cone_end_flag is True
    assert second.observation.cone_message_received_at is None
    assert second.observation.cone_end_flag is None
    assert adapter.fsm.state is Mode.CONE_DRIVE


def test_cached_end_flag_is_not_repeated_on_timer_ticks():
    adapter = runtime(Mode.CONE_DRIVE)
    adapter.record_cone_message([0, 1, 0], 1.0)

    adapter.step(1.0)
    for now in (1.02, 1.04, 1.06):
        cycle = adapter.step(now)
        assert cycle.observation.cone_message_received_at is None
        assert cycle.transition.changed is False


def test_consecutive_zero_then_one_callbacks_preserve_order():
    adapter = runtime(Mode.CONE_DRIVE)
    adapter.record_cone_message([3, 0, 80], 1.0)
    adapter.record_cone_message([4, 1, 0], 1.1)

    zero = adapter.step(1.1)
    one = adapter.step(1.11)

    assert zero.observation.cone_message_received_at == 1.0
    assert zero.observation.cone_end_flag is False
    assert zero.transition.changed is False
    assert adapter.fsm.cone_exit_armed is False
    assert one.observation.cone_message_received_at == 1.1
    assert one.observation.cone_end_flag is True
    assert one.transition.target is Mode.REJOIN


def test_malformed_short_array_is_ignored_without_reusing_normal_event():
    adapter = runtime(Mode.CONE_DRIVE)
    adapter.record_cone_message([3, 0, 80], 1.0)
    armed = adapter.step(1.0)

    malformed = adapter.record_cone_message([9, 1], 1.1)
    no_event = adapter.step(1.1)

    assert armed.transition.reason == "cone exit session armed"
    assert malformed.accepted is False
    assert "requires" in malformed.warning
    assert no_event.observation.cone_message_received_at is None
    assert no_event.observation.cone_end_flag is None
    assert adapter.fsm.state is Mode.CONE_DRIVE


@pytest.mark.parametrize(
    "data",
    ([0, -1, 90], [0, 2, 90], [0, 0, -1], [0, 0, 101]),
)
def test_invalid_end_flag_or_confidence_is_not_queued(data):
    adapter = runtime(Mode.CONE_DRIVE)

    result = adapter.record_cone_message(data, 1.0)

    assert result.accepted is False
    assert adapter.pending_cone_event_count == 0
    assert adapter.step(1.0).observation.cone_message_received_at is None


def test_lane_to_cone_dispatches_reset_once_and_selects_cone_control():
    adapter = runtime(Mode.LANE_DRIVE, one_message_entry=True)
    first = enter_cone(adapter)
    published = []

    assert dispatch_cone_reset(first, lambda: published.append("reset")) is True
    stayed = adapter.step(1.02)
    assert dispatch_cone_reset(stayed, lambda: published.append("reset")) is False

    assert published == ["reset"]
    assert first.control.source is ControlSource.CONE
    assert adapter.fsm.state is Mode.CONE_DRIVE


def test_pre_reset_queue_is_discarded_and_latched_one_cannot_exit():
    adapter = runtime(Mode.LANE_DRIVE, one_message_entry=True)
    adapter.record_scan(1.1)
    adapter.record_cone_message([1, 0, 90], 1.0)
    adapter.record_cone_message([2, 0, 90], 1.1)

    entry = adapter.step(1.1)
    adapter.record_cone_message([3, 1, 0], 1.2)
    latched_one = adapter.step(1.2)

    assert entry.publish_cone_reset is True
    assert entry.discarded_pre_reset_events == 1
    assert latched_one.transition.changed is False
    assert adapter.fsm.state is Mode.CONE_DRIVE
    assert adapter.fsm.cone_exit_armed is False


def test_reset_then_fresh_zero_and_separate_one_enters_rejoin():
    adapter = runtime(Mode.LANE_DRIVE, one_message_entry=True)
    enter_cone(adapter)

    adapter.record_cone_message([0, 1, 0], 1.1)
    old_one = adapter.step(1.1)
    adapter.record_cone_message([0, 0, 80], 1.2)
    fresh_zero = adapter.step(1.2)
    adapter.record_cone_message([0, 1, 0], 1.3)
    fresh_one = adapter.step(1.3)

    assert old_one.transition.changed is False
    assert fresh_zero.transition.changed is False
    assert fresh_zero.transition.reason == "cone exit session armed"
    assert fresh_one.transition.target is Mode.REJOIN


def test_stop_transition_never_dispatches_cone_reset():
    adapter = runtime(Mode.LANE_DRIVE, one_message_entry=True)
    adapter.record_scan(1.0)
    adapter.record_cone_message([0, 0, 90], 1.0)

    stopped = adapter.step(1.0, fault_reason="test fault")
    published = []

    assert stopped.transition.target is Mode.STOP
    assert stopped.publish_cone_reset is False
    assert dispatch_cone_reset(stopped, lambda: published.append("reset")) is False
    assert published == []


def test_second_normal_cone_session_dispatches_one_new_reset():
    adapter = runtime(Mode.LANE_DRIVE, one_message_entry=True)
    resets = []

    first_entry = enter_cone(adapter, 1.0)
    dispatch_cone_reset(first_entry, lambda: resets.append(1))
    adapter.record_cone_message([0, 0, 80], 1.1)
    adapter.step(1.1)
    adapter.record_cone_message([0, 1, 0], 1.2)
    assert adapter.step(1.2).transition.target is Mode.REJOIN

    for timestamp in (1.3, 1.4):
        adapter.record_lane_validity(True, timestamp)
        waiting = adapter.step(
            timestamp,
            lane=candidate(1.0, 5.0, timestamp),
        )
        assert waiting.transition.changed is False
        assert waiting.control.source is ControlSource.STOP
    adapter.record_lane_validity(True, 1.51)
    first_rejoin = adapter.step(
        1.51,
        lane=candidate(1.0, 5.0, 1.51),
    )
    assert first_rejoin.transition.target is Mode.LANE_DRIVE
    assert first_rejoin.control.source is ControlSource.LANE

    second_entry = enter_cone(adapter, 2.1)
    dispatch_cone_reset(second_entry, lambda: resets.append(2))
    adapter.record_cone_message([0, 0, 80], 2.2)
    adapter.step(2.2)
    adapter.record_cone_message([0, 1, 0], 2.3)
    assert adapter.step(2.3).transition.target is Mode.REJOIN

    for timestamp in (2.4, 2.5):
        adapter.record_lane_validity(True, timestamp)
        assert adapter.step(timestamp).transition.changed is False
    adapter.record_lane_validity(True, 2.61)
    second_rejoin = adapter.step(
        2.61,
        lane=candidate(1.0, 5.0, 2.61),
    )
    assert second_rejoin.transition.target is Mode.LANE_DRIVE
    assert second_rejoin.control.source is ControlSource.LANE

    stayed = adapter.step(2.63)
    dispatch_cone_reset(stayed, lambda: resets.append(99))

    assert resets == [1, 2]


def test_queue_overflow_drops_oldest_and_keeps_fail_safe_order():
    adapter = runtime(Mode.CONE_DRIVE, queue_capacity=2)
    adapter.record_cone_message([0, 0, 80], 1.0)
    adapter.record_cone_message([0, 1, 0], 1.1)
    overflow = adapter.record_cone_message([0, 0, 80], 1.2)

    first = adapter.step(1.2)
    second = adapter.step(1.21)

    assert overflow.dropped_oldest is True
    assert adapter.cone_queue_overflow_count == 1
    assert first.observation.cone_message_received_at == 1.1
    assert first.observation.cone_end_flag is True
    assert second.observation.cone_message_received_at == 1.2
    assert second.observation.cone_end_flag is False
    assert adapter.fsm.state is Mode.CONE_DRIVE
    assert adapter.fsm.cone_exit_armed is True


def test_timestamp_regression_is_passed_through_and_rejected_by_fsm():
    adapter = runtime(Mode.CONE_DRIVE)
    adapter.record_cone_message([0, 0, 80], 2.0)
    adapter.step(2.0)
    adapter.record_cone_message([0, 1, 0], 1.9)

    regressed = adapter.step(2.1)

    assert regressed.observation.now == 2.1
    assert regressed.observation.cone_message_received_at == 1.9
    assert regressed.transition.changed is False
    assert adapter.fsm.state is Mode.CONE_DRIVE


def test_receipt_and_step_values_share_the_supplied_clock_domain():
    adapter = runtime(Mode.CONE_DRIVE)
    fake_ros_time = 42.0
    adapter.record_cone_message([0, 0, 80], fake_ros_time)
    fake_ros_time += 0.02

    cycle = adapter.step(fake_ros_time)

    assert cycle.observation.now == 42.02
    assert cycle.observation.cone_message_received_at == 42.0
    assert cycle.observation.now - cycle.observation.cone_message_received_at == pytest.approx(0.02)


def test_runtime_init_waits_for_existing_required_inputs():
    adapter = RaceRuntimeAdapter(
        fsm=RaceFSM(initial_state=Mode.INIT),
        context=RaceContext(state_entered_at=0.0),
        safety_monitor=runtime_safety_monitor(),
    )

    missing = adapter.step(1.0)
    adapter.record_traffic(False, 1.1)
    adapter.record_lane_offset(0, 1.1)
    adapter.record_scan(1.1)
    ready = adapter.step(1.1)

    assert missing.transition.changed is False
    assert missing.safety.inputs_ready is False
    assert ready.transition.target is Mode.WAIT_GREEN
    assert ready.safety.inputs_ready is True


def test_one_green_callback_is_not_recounted_by_timer_ticks():
    adapter = RaceRuntimeAdapter(
        fsm=RaceFSM(initial_state=Mode.WAIT_GREEN),
        context=RaceContext(state_entered_at=0.0),
    )
    adapter.record_traffic(True, 1.0)

    first = adapter.step(1.0)
    repeated = [adapter.step(now) for now in (1.02, 1.04, 1.06)]
    adapter.record_traffic(True, 1.1)
    second = adapter.step(1.1)
    adapter.record_traffic(True, 1.21)
    third = adapter.step(1.21)

    assert first.transition.changed is False
    assert all(cycle.transition.changed is False for cycle in repeated)
    assert second.transition.changed is False
    assert third.transition.target is Mode.LANE_DRIVE


@pytest.mark.parametrize(
    ("now", "received_at"),
    [(2.0, 1.0), (1.0, 1.1), (1.0, float("nan"))],
)
def test_stale_future_or_nan_green_receipt_does_not_start_race(
    now,
    received_at,
):
    fsm = RaceFSM(initial_state=Mode.WAIT_GREEN)
    context = RaceContext(state_entered_at=0.0)

    transition = fsm.step(
        MissionObservation(
            now=now,
            green_detected=True,
            traffic_message_received_at=received_at,
        ),
        context,
        SafetyDecision(inputs_ready=True),
    )

    assert transition.changed is False
    assert fsm.state is Mode.WAIT_GREEN
    assert context.race_started_at is None


def test_stale_lane_input_commits_safety_stop_and_zero_control():
    adapter = RaceRuntimeAdapter(
        fsm=RaceFSM(initial_state=Mode.LANE_DRIVE),
        context=RaceContext(state_entered_at=0.0),
        safety_monitor=runtime_safety_monitor(),
    )
    adapter.record_lane_offset(0, 1.0)

    cycle = adapter.step(
        1.6,
        lane=candidate(1.0, 5.0, 1.0),
    )

    assert cycle.transition.target is Mode.STOP
    assert cycle.safety.must_stop is True
    assert cycle.control.source is ControlSource.STOP


def test_stale_cone_command_stops_motor_without_bypassing_exit_handshake():
    adapter = RaceRuntimeAdapter(
        fsm=RaceFSM(initial_state=Mode.CONE_DRIVE),
        context=RaceContext(state_entered_at=1.0, cone_entered_at=1.0),
        safety_monitor=runtime_safety_monitor(),
    )
    adapter.record_scan(2.0)

    cycle = adapter.step(
        2.0,
        cone=candidate(-2.0, 4.0, 1.0),
    )

    assert cycle.transition.changed is False
    assert adapter.fsm.state is Mode.CONE_DRIVE
    assert cycle.safety.must_stop is False
    assert cycle.control.source is ControlSource.STOP


@pytest.mark.parametrize(
    "mode",
    [Mode.FIXED_AVOID, Mode.OVERTAKE, Mode.SHORTCUT],
)
def test_unwired_future_states_ignore_typed_events_and_select_stop(mode):
    adapter = RaceRuntimeAdapter(
        fsm=RaceFSM(initial_state=mode),
        context=RaceContext(state_entered_at=1.0),
    )

    observation_time = 1.1
    cycle = adapter.step(
        observation_time,
        lane=candidate(1.0, 5.0, observation_time),
        cone=candidate(-2.0, 4.0, observation_time),
    )

    assert cycle.transition.changed is False
    assert adapter.fsm.state is mode
    assert cycle.control.source is ControlSource.STOP
