"""Mission boundaries are places on the track; crossing one is a single event.

The same bag replayed twice produced LANE_DRIVE<->OVERTAKE thirteen times in half
a second once and not at all the other time, so this has to be detected rather
than eyeballed in a timeline.
"""

import pytest

from drive_eval.modes import ModeTransition, chatter_spans


def _edges(pairs):
    return [
        ModeTransition(time_s=time_s, source=source, target=target)
        for time_s, source, target in pairs
    ]


LANE, OVERTAKE, CONE = 1, 4, 2


def test_a_burst_of_flips_is_reported():
    # 104.75-105.28 s of the 2026-08-13 confirmation run, in miniature.
    edges = _edges(
        [
            (104.75, LANE, OVERTAKE),
            (104.79, OVERTAKE, LANE),
            (104.81, LANE, OVERTAKE),
            (104.85, OVERTAKE, LANE),
            (104.89, LANE, OVERTAKE),
        ]
    )

    spans = chatter_spans(edges)

    assert len(spans) == 1
    assert spans[0].flips == 5
    assert 'LANE_DRIVE' in spans[0].modes and 'OVERTAKE' in spans[0].modes


def test_ordinary_mission_transitions_are_not_chatter():
    edges = _edges(
        [
            (18.0, LANE, CONE),
            (27.5, CONE, LANE),
            (29.0, LANE, 3),
            (33.1, 3, LANE),
        ]
    )

    assert chatter_spans(edges) == []


def test_two_transitions_close_together_are_allowed():
    # Leaving a cone section and immediately entering the fixed-obstacle zone is
    # a real sequence, not indecision.
    edges = _edges([(27.5, CONE, LANE), (27.9, LANE, 3)])

    assert chatter_spans(edges) == []


def test_the_window_and_threshold_are_configurable():
    edges = _edges(
        [(0.0, LANE, OVERTAKE), (0.1, OVERTAKE, LANE), (0.2, LANE, OVERTAKE)]
    )

    assert chatter_spans(edges, min_flips=3) != []
    assert chatter_spans(edges, min_flips=4) == []


def test_two_separate_bursts_are_reported_separately():
    edges = _edges(
        [
            (10.0, LANE, OVERTAKE),
            (10.1, OVERTAKE, LANE),
            (10.2, LANE, OVERTAKE),
            (10.3, OVERTAKE, LANE),
            (60.0, LANE, OVERTAKE),
            (60.1, OVERTAKE, LANE),
            (60.2, LANE, OVERTAKE),
            (60.3, OVERTAKE, LANE),
        ]
    )

    spans = chatter_spans(edges)

    assert len(spans) == 2
    assert spans[0].start_s == pytest.approx(10.0)
    assert spans[1].start_s == pytest.approx(60.0)


def test_an_empty_timeline_has_no_chatter():
    assert chatter_spans([]) == []


def test_the_span_reports_the_time_it_covered():
    edges = _edges(
        [
            (5.0, LANE, OVERTAKE),
            (5.1, OVERTAKE, LANE),
            (5.2, LANE, OVERTAKE),
            (5.4, OVERTAKE, LANE),
        ]
    )

    span = chatter_spans(edges)[0]

    assert span.duration_s == pytest.approx(0.4)
    assert '0.40 s' in span.format()
