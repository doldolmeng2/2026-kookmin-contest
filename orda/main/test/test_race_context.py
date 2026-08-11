import pytest

from main.mission_types import LaneTarget
from main.race_context import RaceContext


def test_context_defaults_before_start():
    context = RaceContext()

    assert context.finish_gate_passes == 0
    assert context.lap_count == 0
    assert context.current_lap == 1
    assert context.shortcut_used is False
    assert context.shortcut_lap is None
    assert context.on_shortcut_lap is False
    assert context.lane_target is LaneTarget.LANE_TWO
    assert context.race_started_at is None
    assert context.state_entered_at is None
    assert context.cone_entered_at is None
    assert context.last_gate_event_at is None
    assert context.stop_reason is None


def test_lap_properties_are_derived_from_finish_gate_passes():
    context = RaceContext(finish_gate_passes=1)
    assert context.lap_count == 1
    assert context.current_lap == 2

    context.finish_gate_passes = 2
    assert context.lap_count == 2
    assert context.current_lap == 3

    context.finish_gate_passes = 3
    assert context.lap_count == 3
    assert context.current_lap == 3


def test_canonical_lap_and_shortcut_fields_have_no_duplicate_mutable_state():
    context = RaceContext(completed_laps=1, shortcut_lap=2)

    assert context.finish_gate_passes == 1
    assert context.current_lap == 2
    assert context.shortcut_used is True
    assert context.on_shortcut_lap is True

    context.completed_laps = 2
    assert context.current_lap == 3
    assert context.on_shortcut_lap is False


def test_compatibility_aliases_write_the_canonical_fields():
    context = RaceContext(last_gate_event_at=4.0)

    context.finish_gate_passes = 2
    context.last_gate_event_at = 5.0

    assert context.completed_laps == 2
    assert context.last_traffic_encounter_at == 5.0


@pytest.mark.parametrize("value", [-1, 4, 1.0, True])
def test_invalid_completed_laps_are_rejected(value):
    with pytest.raises(ValueError):
        RaceContext(completed_laps=value)


@pytest.mark.parametrize("value", [0, 1, 4, True])
def test_invalid_shortcut_lap_is_rejected(value):
    with pytest.raises(ValueError):
        RaceContext(shortcut_lap=value)
