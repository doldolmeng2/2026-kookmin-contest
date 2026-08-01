import pytest

from main.cone_entry import ConeEntryConfig
from main.mission_observation import MissionObservation
from main.race_context import RaceContext
from main.race_fsm import Mode, RaceFSM
from main.safety_monitor import SafetyDecision


def safe(inputs_ready=True):
    return SafetyDecision(inputs_ready=inputs_ready)


def cone_observation(timestamp, confidence=90, end_flag=False):
    return MissionObservation(
        now=timestamp,
        cone_confidence=confidence,
        cone_end_flag=end_flag,
        cone_message_received_at=timestamp,
        scan_received_at=timestamp,
    )


def test_mode_set_matches_2026_skeleton():
    assert [(mode.name, mode.value) for mode in Mode] == [
        ("INIT", "INIT"),
        ("WAIT_GREEN", "WAIT_GREEN"),
        ("LANE_DRIVE", "LANE_DRIVE"),
        ("CONE_DRIVE", "CONE_DRIVE"),
        ("REJOIN", "REJOIN"),
        ("FIXED_AVOID", "FIXED_AVOID"),
        ("OVERTAKE", "OVERTAKE"),
        ("SHORTCUT", "SHORTCUT"),
        ("FINISH", "FINISH"),
        ("STOP", "STOP"),
    ]
    assert len(Mode) == 10
    assert "ROUTE_DECISION" not in Mode.__members__


def test_mode_values_are_exact_strings():
    assert [mode.value for mode in Mode] == [
        "INIT",
        "WAIT_GREEN",
        "LANE_DRIVE",
        "CONE_DRIVE",
        "REJOIN",
        "FIXED_AVOID",
        "OVERTAKE",
        "SHORTCUT",
        "FINISH",
        "STOP",
    ]


def test_init_waits_for_inputs_without_changing_entry_time():
    fsm = RaceFSM()
    context = RaceContext(state_entered_at=1.0)

    transition = fsm.step(
        MissionObservation(now=2.0),
        context,
        safe(inputs_ready=False),
    )

    assert transition.changed is False
    assert fsm.state is Mode.INIT
    assert context.state_entered_at == 1.0
    assert context.race_started_at is None


def test_init_ready_enters_wait_green_without_starting_race_clock():
    fsm = RaceFSM()
    context = RaceContext(state_entered_at=1.0)

    transition = fsm.step(
        MissionObservation(now=2.0),
        context,
        safe(),
    )

    assert transition.changed is True
    assert transition.target is Mode.WAIT_GREEN
    assert context.state_entered_at == 2.0
    assert context.race_started_at is None


def test_green_requires_consecutive_frames_and_resets_on_gap():
    fsm = RaceFSM(
        initial_state=Mode.WAIT_GREEN,
        green_min_consecutive_frames=3,
    )
    context = RaceContext(state_entered_at=1.0)

    for now in (2.0, 2.1):
        transition = fsm.step(
            MissionObservation(now=now, green_detected=True),
            context,
            safe(),
        )
        assert transition.changed is False
        assert context.state_entered_at == 1.0

    fsm.step(
        MissionObservation(now=2.2, green_detected=False),
        context,
        safe(),
    )

    for now in (3.0, 3.1):
        transition = fsm.step(
            MissionObservation(now=now, green_detected=True),
            context,
            safe(),
        )
        assert transition.changed is False

    transition = fsm.step(
        MissionObservation(now=3.2, green_detected=True),
        context,
        safe(),
    )

    assert transition.target is Mode.LANE_DRIVE
    assert context.state_entered_at == 3.2
    assert context.race_started_at == 3.2


def test_green_debounce_can_also_require_duration():
    fsm = RaceFSM(
        initial_state=Mode.WAIT_GREEN,
        green_min_consecutive_frames=2,
        green_min_duration_s=0.5,
    )
    context = RaceContext(state_entered_at=1.0)

    first = fsm.step(
        MissionObservation(now=5.0, green_detected=True),
        context,
        safe(),
    )
    second = fsm.step(
        MissionObservation(now=5.5, green_detected=True),
        context,
        safe(),
    )

    assert first.changed is False
    assert second.target is Mode.LANE_DRIVE
    assert context.race_started_at == 5.5


def test_safety_stop_records_reason_and_entry_time():
    fsm = RaceFSM(initial_state=Mode.LANE_DRIVE)
    context = RaceContext(state_entered_at=3.0)
    safety = SafetyDecision(
        must_stop=True,
        reason="stale required inputs: sensor:lidar",
        inputs_ready=False,
        stale_inputs=("sensor:lidar",),
    )

    transition = fsm.step(
        MissionObservation(now=4.0),
        context,
        safety,
    )

    assert transition.target is Mode.STOP
    assert context.state_entered_at == 4.0
    assert context.stop_reason == safety.reason


def test_lane_drive_enters_cone_drive_once_after_debounce_commit():
    fsm = RaceFSM(initial_state=Mode.LANE_DRIVE)
    context = RaceContext(state_entered_at=0.5)

    first = fsm.step(cone_observation(1.0), context, safe())
    second = fsm.step(cone_observation(1.1), context, safe())
    assert context.state_entered_at == 0.5
    assert context.cone_entered_at is None

    transition = fsm.step(cone_observation(1.21), context, safe())
    repeated = fsm.step(cone_observation(1.21), context, safe())

    assert first.changed is False
    assert second.changed is False
    assert context.cone_entered_at == 1.21
    assert transition.source is Mode.LANE_DRIVE
    assert transition.target is Mode.CONE_DRIVE
    assert transition.reason == "cone entry confirmed"
    assert repeated.changed is False
    assert fsm.state is Mode.CONE_DRIVE
    assert fsm.cone_entry_guard.triggered is True
    assert context.state_entered_at == 1.21
    assert context.cone_entered_at == 1.21


def test_lane_self_transition_preserves_state_and_cone_entry_times():
    fsm = RaceFSM(initial_state=Mode.LANE_DRIVE)
    context = RaceContext(state_entered_at=2.0, cone_entered_at=1.0)

    transition = fsm.step(cone_observation(3.0), context, safe())

    assert transition.changed is False
    assert context.state_entered_at == 2.0
    assert context.cone_entered_at == 1.0


@pytest.mark.parametrize(
    "mode",
    [Mode.CONE_DRIVE, Mode.REJOIN, Mode.FIXED_AVOID, Mode.OVERTAKE],
)
def test_cone_entry_sequence_does_not_accumulate_outside_lane_drive(mode):
    fsm = RaceFSM(initial_state=mode)
    context = RaceContext(state_entered_at=0.0)

    for timestamp in (1.0, 1.1, 1.3):
        transition = fsm.step(
            cone_observation(timestamp),
            context,
            safe(),
        )

    assert transition.changed is False
    assert fsm.state is mode
    assert fsm.cone_entry_guard.qualifying_message_count == 0
    assert context.cone_entered_at is None


def test_custom_cone_config_is_used_without_scattered_thresholds():
    config = ConeEntryConfig(
        min_confidence=90,
        min_messages=2,
        min_duration_s=0.5,
        max_cone_age_s=0.1,
        max_scan_age_s=0.1,
    )
    fsm = RaceFSM(initial_state=Mode.LANE_DRIVE, cone_entry_config=config)
    context = RaceContext()

    fsm.step(cone_observation(1.0, confidence=90), context, safe())
    transition = fsm.step(
        cone_observation(1.5, confidence=90),
        context,
        safe(),
    )

    assert fsm.cone_entry_config is config
    assert transition.target is Mode.CONE_DRIVE


@pytest.mark.parametrize("terminal", [Mode.FINISH, Mode.STOP])
def test_terminal_states_ignore_further_events_and_faults(terminal):
    fsm = RaceFSM(initial_state=terminal)
    context = RaceContext(state_entered_at=7.0)

    transition = fsm.step(
        MissionObservation(now=8.0, green_detected=True),
        context,
        SafetyDecision(must_stop=True, reason="late fault"),
    )

    assert transition.changed is False
    assert fsm.state is terminal
    assert context.state_entered_at == 7.0


@pytest.mark.parametrize(
    "state",
    [
        Mode.LANE_DRIVE,
        Mode.CONE_DRIVE,
        Mode.REJOIN,
        Mode.FIXED_AVOID,
        Mode.OVERTAKE,
        Mode.SHORTCUT,
    ],
)
def test_unimplemented_mission_events_self_transition(state):
    fsm = RaceFSM(initial_state=state)
    context = RaceContext(
        finish_gate_passes=1,
        state_entered_at=10.0,
        last_gate_event_at=9.0,
    )
    observation = MissionObservation(
        now=11.0,
        lane_valid=True,
        cone_detected=True,
        cone_finished=True,
        fixed_vehicle_detected=True,
        fixed_avoid_complete=True,
        interfering_vehicle_detected=True,
        overtake_complete=True,
        left_turn_signal=True,
        shortcut_complete=True,
        finish_gate_crossed=True,
    )

    transition = fsm.step(observation, context, safe())

    assert transition.changed is False
    assert fsm.state is state
    assert context.state_entered_at == 10.0
    assert context.finish_gate_passes == 1
    assert context.last_gate_event_at == 9.0
