from main.mission_types import RouteTrafficSignal
from main.traffic_encounter import TrafficEncounterGate


RELEASE_S = 1.0


def test_green_brief_unknown_same_green_emits_once():
    gate = TrafficEncounterGate(RELEASE_S)
    assert gate.update(RouteTrafficSignal.STRAIGHT, 1.0) is True
    assert gate.update(RouteTrafficSignal.UNKNOWN, 2.0) is False
    assert gate.update(RouteTrafficSignal.STRAIGHT, 2.5) is False


def test_sustained_unknown_allows_next_fixture():
    gate = TrafficEncounterGate(RELEASE_S)
    assert gate.update(RouteTrafficSignal.STRAIGHT, 1.0) is True
    assert gate.update(RouteTrafficSignal.UNKNOWN, 2.0) is False
    assert gate.update(RouteTrafficSignal.STRAIGHT, 3.0) is True


def test_red_then_green_same_fixture_emits_at_green_once():
    gate = TrafficEncounterGate(RELEASE_S)
    assert gate.update(RouteTrafficSignal.RED_AMBER, 1.0) is False
    assert gate.update(RouteTrafficSignal.STRAIGHT, 1.2) is True
    assert gate.update(RouteTrafficSignal.STRAIGHT, 1.4) is False


def test_green_to_left_without_release_is_same_episode():
    gate = TrafficEncounterGate(RELEASE_S)
    assert gate.update(RouteTrafficSignal.STRAIGHT, 1.0) is True
    assert gate.update(RouteTrafficSignal.LEFT, 1.2) is False


def test_three_physical_episodes_emit_exactly_three_edges():
    gate = TrafficEncounterGate(RELEASE_S)
    edges = []
    for signal, now in (
        (RouteTrafficSignal.STRAIGHT, 1.0),
        (RouteTrafficSignal.UNKNOWN, 2.0),
        (RouteTrafficSignal.LEFT, 3.1),
        (RouteTrafficSignal.UNKNOWN, 4.0),
        (RouteTrafficSignal.UNKNOWN, 5.1),
        (RouteTrafficSignal.LEFT, 6.0),
    ):
        edges.append(gate.update(signal, now))
    assert sum(edges) == 3
