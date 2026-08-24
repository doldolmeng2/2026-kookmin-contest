import pytest

from drive_eval.cone_truth import ConeWindow
from drive_eval.modes import (
    CONE_DRIVE,
    Interval,
    evaluate_cone_sessions,
    intervals_of,
    mode_occupancy,
    transitions,
)
from drive_eval.timebase import Series


def _mode_series(pairs):
    return Series(tuple(t for t, _ in pairs), tuple(float(v) for _, v in pairs))


def test_transitions_are_the_edges_not_every_sample():
    series = _mode_series([(0.0, 1), (0.1, 1), (0.2, 2), (0.3, 2), (0.4, 1)])

    edges = transitions(series)

    assert [(edge.time_s, edge.source, edge.target) for edge in edges] == [
        (0.2, 1, 2),
        (0.4, 2, 1),
    ]


def test_lane_to_cone_and_back_are_declared_transitions():
    series = _mode_series([(0.0, 1), (1.0, 2), (2.0, 1)])

    assert all(edge.legal for edge in transitions(series))


def test_an_undeclared_transition_is_flagged():
    # CONE_DRIVE -> SHORTCUT is not in race_fsm.py.
    series = _mode_series([(0.0, 2), (1.0, 5)])

    assert transitions(series)[0].legal is False


def test_intervals_of_finds_each_stretch_of_one_mode():
    series = _mode_series(
        [(0.0, 1), (1.0, 2), (2.0, 2), (3.0, 1), (4.0, 2), (5.0, 1)]
    )

    spans = intervals_of(series, CONE_DRIVE)

    assert [(span.start_s, span.end_s) for span in spans] == [(1.0, 3.0), (4.0, 5.0)]


def test_an_unfinished_interval_runs_to_the_end_of_the_run():
    series = _mode_series([(0.0, 1), (1.0, 2)])

    spans = intervals_of(series, CONE_DRIVE, end_s=9.0)

    assert spans[0].end_s == 9.0


def test_a_corridor_driven_in_cone_mode_counts_as_entered():
    windows = [ConeWindow(start_s=10.0, end_s=18.0, peak_cones=6, scans=70)]
    intervals = [Interval(10.4, 19.0)]

    report = evaluate_cone_sessions(windows, intervals)

    assert report.all_entered()
    assert report.matches[0].entry_delay_s == pytest.approx(0.4)
    # The 0.4 s the stack spent still in LANE_DRIVE is 5% of the corridor.
    assert report.matches[0].coverage == pytest.approx(0.95)
    assert report.spurious == []


def test_a_missed_corridor_is_reported():
    windows = [ConeWindow(start_s=10.0, end_s=18.0, peak_cones=6, scans=70)]

    report = evaluate_cone_sessions(windows, [])

    assert not report.all_entered()
    assert report.matches[0].entered is False
    assert 'NOT ENTERED' in report.matches[0].format()


def test_cone_mode_with_no_cones_anywhere_is_reported_as_spurious():
    windows = [ConeWindow(start_s=10.0, end_s=18.0, peak_cones=6, scans=70)]
    intervals = [Interval(10.2, 17.0), Interval(60.0, 64.0)]

    report = evaluate_cone_sessions(windows, intervals)

    assert report.all_entered()
    assert [(span.start_s, span.end_s) for span in report.spurious] == [(60.0, 64.0)]


def test_partial_coverage_is_measured_not_rounded_to_entered():
    windows = [ConeWindow(start_s=10.0, end_s=20.0, peak_cones=6, scans=90)]
    intervals = [Interval(10.0, 13.0)]

    report = evaluate_cone_sessions(windows, intervals)

    assert report.matches[0].coverage == pytest.approx(0.3)


def test_a_session_starting_just_before_the_corridor_still_matches():
    # The node can switch on a single early corridor scan; the ground truth waits
    # for two, so a small lead is expected rather than wrong.
    windows = [ConeWindow(start_s=10.0, end_s=18.0, peak_cones=6, scans=70)]
    intervals = [Interval(9.0, 18.5)]

    report = evaluate_cone_sessions(windows, intervals, slack_s=2.0)

    assert report.all_entered()
    assert report.matches[0].entry_delay_s == pytest.approx(-1.0)


def test_mode_occupancy_sums_the_time_in_each_mode():
    series = _mode_series([(0.0, 1), (2.0, 2), (5.0, 1)])

    occupancy = mode_occupancy(series, end_s=8.0)

    assert occupancy['LANE_DRIVE'] == pytest.approx(5.0)
    assert occupancy['CONE_DRIVE'] == pytest.approx(3.0)
