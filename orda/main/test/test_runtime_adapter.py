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
    dispatch_cone_session_state,
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
    assert one.transition.target is Mode.LANE_DRIVE


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


def test_lane_to_cone_dispatches_active_once_and_selects_cone_control():
    adapter = runtime(Mode.LANE_DRIVE, one_message_entry=True)
    first = enter_cone(adapter)
    published = []

    assert dispatch_cone_session_state(first, published.append) is True
    stayed = adapter.step(1.02)
    assert dispatch_cone_session_state(stayed, published.append) is False

    assert published == [True]
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

    assert entry.cone_session_active_command is True
    assert entry.discarded_pre_phase_events == 1
    assert latched_one.transition.changed is False
    assert adapter.fsm.state is Mode.CONE_DRIVE
    assert adapter.fsm.cone_exit_armed is False


def test_reset_then_fresh_zero_and_separate_one_returns_to_lane():
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
    assert fresh_one.transition.target is Mode.LANE_DRIVE


def test_committed_cone_exit_dispatches_search_exactly_once():
    adapter = runtime(Mode.CONE_DRIVE)
    published = []
    adapter.record_cone_message([0, 0, 80], 1.0)
    adapter.step(1.0)
    adapter.record_cone_message([0, 1, 0], 1.1)

    exited = adapter.step(1.1)
    repeated = adapter.step(1.2)

    assert exited.transition.source is Mode.CONE_DRIVE
    assert exited.transition.target is Mode.LANE_DRIVE
    assert exited.transition.reason == "fresh cone end flag"
    assert exited.observation.cone_end_flag is True
    assert exited.cone_session_active_command is False
    assert dispatch_cone_session_state(exited, published.append) is True
    assert repeated.transition.changed is False
    assert repeated.cone_session_active_command is None
    assert dispatch_cone_session_state(repeated, published.append) is False
    assert published == [False]


def test_safety_stop_consumes_cone_end_without_dispatching_or_reusing_reset():
    adapter = runtime(Mode.CONE_DRIVE)
    published = []
    adapter.record_cone_message([0, 0, 80], 1.0)
    adapter.step(1.0)
    adapter.record_cone_message([0, 1, 0], 1.1)

    stopped = adapter.step(1.1, fault_reason="forced safety stop")
    holding = adapter.step(1.2)
    recovered = adapter.step(1.8)

    assert stopped.observation.cone_end_flag is True
    assert stopped.transition.source is Mode.CONE_DRIVE
    assert stopped.transition.target is Mode.CONE_DRIVE
    assert stopped.cone_session_active_command is None
    assert dispatch_cone_session_state(stopped, published.append) is False
    assert holding.transition.changed is False
    assert holding.cone_session_active_command is None
    assert recovered.transition.source is Mode.CONE_DRIVE
    assert recovered.transition.target is Mode.CONE_DRIVE
    assert recovered.cone_session_active_command is None
    assert dispatch_cone_session_state(recovered, published.append) is False
    assert published == []


@pytest.mark.parametrize(
    ("end_received_at", "now"),
    ((1.1, 2.0), (2.0, 1.1), (0.4, 1.1)),
    ids=("stale", "future", "pre-session"),
)
def test_invalid_cone_end_edges_never_dispatch_exit_reset(end_received_at, now):
    adapter = runtime(Mode.CONE_DRIVE)
    adapter.record_cone_message([0, 0, 80], 1.0)
    adapter.step(1.0)
    adapter.record_cone_message([0, 1, 0], end_received_at)

    cycle = adapter.step(now)

    assert cycle.transition.changed is False
    assert cycle.transition.target is Mode.CONE_DRIVE
    assert cycle.cone_session_active_command is None
    assert dispatch_cone_session_state(
        cycle, lambda _: pytest.fail("unexpected phase command")
    ) is False


def test_cone_lifecycle_dispatches_explicit_state_for_each_committed_boundary():
    adapter = runtime(Mode.LANE_DRIVE, one_message_entry=True)
    phases = []

    first_entry = enter_cone(adapter, 1.0)
    dispatch_cone_session_state(first_entry, phases.append)
    adapter.record_cone_message([0, 0, 80], 1.1)
    adapter.step(1.1)
    adapter.record_cone_message([0, 1, 0], 1.2)
    first_exit = adapter.step(1.2)
    dispatch_cone_session_state(first_exit, phases.append)

    stayed = adapter.step(1.3)
    dispatch_cone_session_state(stayed, phases.append)

    second_entry = enter_cone(adapter, 2.1)
    dispatch_cone_session_state(second_entry, phases.append)

    assert first_entry.transition.source is Mode.LANE_DRIVE
    assert first_entry.transition.target is Mode.CONE_DRIVE
    assert first_exit.transition.source is Mode.CONE_DRIVE
    assert first_exit.transition.target is Mode.LANE_DRIVE
    assert stayed.transition.changed is False
    assert second_entry.transition.source is Mode.LANE_DRIVE
    assert second_entry.transition.target is Mode.CONE_DRIVE
    assert phases == [True, False, True]


def test_stop_transition_never_dispatches_cone_phase():
    adapter = runtime(Mode.LANE_DRIVE, one_message_entry=True)
    adapter.record_scan(1.0)
    adapter.record_cone_message([0, 0, 90], 1.0)

    stopped = adapter.step(1.0, fault_reason="test fault")
    published = []

    assert stopped.transition.target is Mode.LANE_DRIVE
    assert stopped.cone_session_active_command is None
    assert dispatch_cone_session_state(stopped, published.append) is False
    assert published == []


def test_second_normal_cone_session_dispatches_one_new_active_command():
    adapter = runtime(Mode.LANE_DRIVE, one_message_entry=True)
    resets = []

    first_entry = enter_cone(adapter, 1.0)
    dispatch_cone_session_state(first_entry, resets.append)
    adapter.record_cone_message([0, 0, 80], 1.1)
    adapter.step(1.1)
    adapter.record_cone_message([0, 1, 0], 1.2)
    first_exit = adapter.step(1.2)
    dispatch_cone_session_state(first_exit, resets.append)

    second_entry = enter_cone(adapter, 2.1)
    dispatch_cone_session_state(second_entry, resets.append)
    adapter.record_cone_message([0, 0, 80], 2.2)
    adapter.step(2.2)
    adapter.record_cone_message([0, 1, 0], 2.3)
    second_exit = adapter.step(2.3)
    assert second_exit.transition.target is Mode.LANE_DRIVE
    dispatch_cone_session_state(second_exit, resets.append)

    stayed = adapter.step(2.63)
    dispatch_cone_session_state(stayed, resets.append)

    assert resets == [True, False, True, False]


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


def test_lane_position_rejects_invalid_or_regressed_receipts():
    adapter = runtime(Mode.FIXED_AVOID)

    assert adapter.record_lane_position(1, 1.0) is True
    assert adapter.record_lane_position(2, 1.0) is False
    assert adapter.record_lane_position(2, 0.9) is False
    assert adapter.record_lane_position(3, 1.1) is False
    assert adapter.record_lane_position(1, float("nan")) is False
    assert adapter.measured_lane == 1
    assert adapter.measured_lane_received_at == 1.0
    assert adapter.perception_received_at["lane_position"] == 1.0


def test_side_clearance_rejects_invalid_duplicate_or_regressed_receipts():
    adapter = runtime(Mode.OVERTAKE)

    assert adapter.record_side_clearance(float("inf"), 0.3, 1.0) is True
    assert adapter.record_side_clearance(0.2, 0.4, 1.0) is False
    assert adapter.record_side_clearance(0.2, 0.4, 0.9) is False
    assert adapter.record_side_clearance(-0.1, 0.4, 1.1) is False
    assert adapter.record_side_clearance(float("nan"), 0.4, 1.1) is False
    assert adapter.record_side_clearance(float("-inf"), 0.4, 1.1) is False
    assert adapter.latest_side_left == float("inf")
    assert adapter.latest_side_right == pytest.approx(0.3)
    assert adapter.side_clearance_received_at == 1.0
    assert adapter.perception_received_at["side_clearance"] == 1.0


def test_lane_guardrail_rejects_invalid_duplicate_or_regressed_receipts():
    adapter = runtime(Mode.LANE_DRIVE)

    assert adapter.record_lane_guardrail(120.0, 240.0, 1.0) is True
    assert adapter.record_lane_guardrail(130.0, 250.0, 1.0) is False
    assert adapter.record_lane_guardrail(130.0, 250.0, 0.9) is False
    assert adapter.record_lane_guardrail(float("nan"), 250.0, 1.1) is False
    assert adapter.record_lane_guardrail(130.0, float("inf"), 1.1) is False
    assert adapter.record_lane_guardrail(130.0, 250.0, float("nan")) is False
    assert adapter.latest_lane_guardrail == (120.0, 240.0)
    assert adapter.lane_guardrail_received_at == 1.0
    assert adapter.perception_received_at["lane_guardrail"] == 1.0


def test_lane_guardrail_accepts_unobserved_rails():
    """음수는 '레일을 못 봤다'는 정상적인 관측 결과다.

    바깥 실선이 안 보이는 건 흔한 일이고(실측 관측률 84~90%), 그때 메시지를
    버리면 제어기가 오래된 여유를 계속 붙들게 된다. 값은 받아들이고, 감쇠는
    제어기가 판단한다.
    """
    adapter = runtime(Mode.LANE_DRIVE)

    assert adapter.record_lane_guardrail(-1.0, -1.0, 1.0) is True
    assert adapter.latest_lane_guardrail == (-1.0, -1.0)


def test_lane_change_in_progress_only_while_fresh_and_changing():
    adapter = runtime(Mode.LANE_DRIVE)
    max_age = adapter.lane_change_max_age_s

    assert adapter.lane_change_in_progress(1.0) is False

    assert adapter.record_lane_change_state([1, 0], 1.0).accepted is True
    assert adapter.lane_change_in_progress(1.0) is True
    assert adapter.lane_change_in_progress(1.0 + max_age) is True
    # 토픽이 끊기면 차선 변경 중인지 더 이상 알 수 없다.
    assert adapter.lane_change_in_progress(1.0 + max_age + 0.01) is False

    assert adapter.record_lane_change_state([0, 1], 2.0).accepted is True
    assert adapter.lane_change_in_progress(2.0) is False


def test_runtime_wait_green_absorbs_startup_readiness_gate():
    adapter = RaceRuntimeAdapter(
        fsm=RaceFSM(initial_state=Mode.WAIT_GREEN),
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
    assert ready.transition.changed is False
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

    assert first.transition.target is Mode.LANE_DRIVE
    assert all(cycle.transition.changed is False for cycle in repeated)


def test_live_cone_entry_ready_bypasses_legacy_confidence_debounce():
    adapter = runtime(Mode.LANE_DRIVE)
    assert adapter.record_cone_message([0, 0, 10, 1], 1.0).accepted

    cycle = adapter.step(1.0)

    assert cycle.transition.target is Mode.CONE_DRIVE
    assert cycle.transition.reason == "perception-approved cone entry"


def test_live_cone_not_ready_never_falls_back_to_legacy_guard():
    adapter = runtime(Mode.LANE_DRIVE)

    for received_at in (1.0, 1.1, 1.21):
        assert adapter.record_cone_message([0, 0, 100, 0], received_at).accepted
        cycle = adapter.step(received_at)
        assert cycle.transition.changed is False

    assert adapter.fsm.state is Mode.LANE_DRIVE


@pytest.mark.parametrize(
    "payload",
    ([0, 0, 90, 2], [0, 1, 0, 1], [0, 0, 90, 1, 0]),
)
def test_live_cone_schema_and_semantic_conflicts_are_rejected(payload):
    adapter = runtime(Mode.LANE_DRIVE)

    result = adapter.record_cone_message(payload, 1.0)

    assert result.accepted is False
    assert adapter.pending_cone_event_count == 0


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

    # 1.2초 공백. LANE_OFFSET_MAX_AGE_S(1.0)를 넘어 진짜 인지 유실로 본다.
    cycle = adapter.step(
        2.2,
        lane=candidate(1.0, 5.0, 1.0),
    )

    assert cycle.transition.target is Mode.LANE_DRIVE
    assert cycle.safety.must_stop is True
    assert cycle.control.source is ControlSource.HOLD


def test_stop_recovery_waits_for_real_inputs_not_just_the_hold_timer():
    """hold 복귀는 타이머가 아니라 입력 회복을 봐야 한다.

    예전에는 runtime_safety_monitor() 에 hold 상태의 필수 입력이 등록돼 있지
    않아 inputs_ready 가 항상 True 였고, 복귀가 사실상 0.5초 타이머였다.
    실측 bag 에서 hold 구간 길이가 전부 0.50~0.52초로 똑같고, 복귀 직후 다시
    hold 으로 떨어지기를 28번 반복한 이유다.
    """

    adapter = RaceRuntimeAdapter(
        fsm=RaceFSM(initial_state=Mode.LANE_DRIVE),
        context=RaceContext(state_entered_at=0.0),
        safety_monitor=runtime_safety_monitor(),
    )
    adapter.record_lane_offset(0, 1.0)
    adapter.record_scan(1.0)

    # 인지 유실 -> hold
    stopped = adapter.step(2.2, lane=candidate(1.0, 5.0, 2.2))
    assert stopped.transition.target is Mode.LANE_DRIVE

    # 입력이 안 돌아오는 동안에는 아무리 기다려도 복귀하지 않는다.
    for now in (3.0, 4.0, 5.0):
        held = adapter.step(now, lane=candidate(1.0, 5.0, now))
        assert held.safety.inputs_ready is False
        assert adapter.fsm.state is Mode.LANE_DRIVE

    # 입력이 돌아오면 유지 시간(0.5초)을 채운 뒤 복귀한다.
    for now in (5.2, 5.4, 5.6, 5.8):
        adapter.record_lane_offset(0, now)
        adapter.record_scan(now)
        cycle = adapter.step(now, lane=candidate(1.0, 5.0, now))
    assert cycle.transition.target is Mode.LANE_DRIVE
    assert adapter.fsm.state is Mode.LANE_DRIVE


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
    assert cycle.control.source is ControlSource.HOLD


@pytest.mark.parametrize(
    "mode",
    [Mode.FIXED_AVOID, Mode.OVERTAKE],
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
    assert cycle.control.source is ControlSource.HOLD


def test_shortcut_uses_fresh_lane_control_while_waiting_for_cnn_exit():
    adapter = RaceRuntimeAdapter(
        fsm=RaceFSM(initial_state=Mode.SHORTCUT),
        context=RaceContext(
            completed_laps=1,
            shortcut_lap=2,
            state_entered_at=1.0,
        ),
        safety_monitor=runtime_safety_monitor(),
    )
    adapter.record_lane_offset(0, 1.1)

    cycle = adapter.step(1.1, lane=candidate(1.0, 5.0, 1.1))

    assert cycle.transition.changed is False
    assert cycle.control.source is ControlSource.LANE
