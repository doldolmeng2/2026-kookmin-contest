"""A node can publish steadily and still not be driving; these rules catch that."""

import pytest

from drive_eval.modes import silent_spans
from drive_eval.steering import zero_output_spans
from drive_eval.timebase import Series


def _series(pairs):
    return Series(tuple(t for t, _ in pairs), tuple(float(v) for _, v in pairs))


def test_a_long_flat_zero_command_is_reported():
    pairs = [(index * 0.02, 0.0) for index in range(250)]  # 5 s of zeros
    pairs += [(5.0 + index * 0.02, 20.0) for index in range(50)]

    spans = zero_output_spans(_series(pairs))

    assert len(spans) == 1
    assert spans[0][0] == pytest.approx(0.0)
    assert spans[0][1] == pytest.approx(4.98)


def test_a_brief_zero_crossing_on_a_straight_is_not_reported():
    pairs = [(index * 0.02, 0.0 if 10 <= index < 30 else 15.0) for index in range(200)]

    assert zero_output_spans(_series(pairs)) == []


def test_a_zero_stretch_running_to_the_end_of_the_run_is_reported():
    pairs = [(index * 0.02, 15.0) for index in range(50)]
    pairs += [(1.0 + index * 0.02, 0.0) for index in range(200)]

    spans = zero_output_spans(_series(pairs))

    assert len(spans) == 1
    assert spans[0][1] == pytest.approx(4.98)


def test_steering_that_never_rests_reports_nothing():
    pairs = [(index * 0.02, 10.0 + index) for index in range(200)]

    assert zero_output_spans(_series(pairs)) == []


def test_a_gap_in_mode_info_is_found():
    # Main stops publishing /mode_info in FINISH, so the topic simply goes quiet.
    series = _series([(0.0, 1), (0.5, 1), (1.0, 1), (30.0, 1)])

    gaps = silent_spans(series, max_gap_s=2.0)

    assert [(gap.start_s, gap.end_s) for gap in gaps] == [(1.0, 30.0)]


def test_silence_running_to_the_end_of_the_run_is_found():
    series = _series([(0.0, 1), (0.5, 1)])

    gaps = silent_spans(series, max_gap_s=2.0, end_s=60.0)

    assert [(gap.start_s, gap.end_s) for gap in gaps] == [(0.5, 60.0)]


def test_a_steady_mode_stream_has_no_silence():
    series = _series([(index * 0.02, 1) for index in range(500)])

    assert silent_spans(series, max_gap_s=2.0, end_s=10.0) == []
