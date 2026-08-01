from main.race_context import RaceContext


def test_context_defaults_before_start():
    context = RaceContext()

    assert context.finish_gate_passes == 0
    assert context.lap_count == 0
    assert context.current_lap == 1
    assert context.shortcut_used is False
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
