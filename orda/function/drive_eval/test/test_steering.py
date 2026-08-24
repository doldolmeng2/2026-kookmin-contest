import math

import pytest

from drive_eval.steering import (
    build_grid,
    command_rate_hz,
    compare_steering,
    saturation_ratio,
)
from drive_eval.timebase import Series


def _wave(duration_s=20.0, hz=20.0, amplitude=30.0, period_s=4.0, shift_s=0.0, gain=1.0):
    step = 1.0 / hz
    count = int(duration_s / step)
    times = tuple(index * step for index in range(count))
    values = tuple(
        gain * amplitude * math.sin(2.0 * math.pi * (t - shift_s) / period_s)
        for t in times
    )
    return Series(times, values)


def test_identical_steering_scores_perfectly():
    reference = _wave()

    stats = compare_steering(reference, reference, 0.0, 20.0)

    assert stats.correlation == pytest.approx(1.0, abs=1e-6)
    assert stats.sign_agreement == pytest.approx(1.0)
    assert stats.rmse_deg == pytest.approx(0.0, abs=1e-9)
    assert stats.amplitude_ratio == pytest.approx(1.0, abs=1e-6)


def test_opposite_steering_is_caught_by_direction_agreement():
    reference = _wave()
    inverted = Series(reference.times_s, tuple(-value for value in reference.values))

    stats = compare_steering(reference, inverted, 0.0, 20.0)

    assert stats.correlation < -0.9
    assert stats.sign_agreement == pytest.approx(0.0)


def test_a_late_but_correct_controller_is_reported_as_lag_not_as_wrong():
    reference = _wave()
    late = _wave(shift_s=0.5)

    stats = compare_steering(reference, late, 0.0, 20.0)

    assert stats.best_lag_s == pytest.approx(0.5, abs=0.06)
    assert stats.correlation_at_best_lag > 0.99
    assert stats.correlation_at_best_lag > stats.correlation


def test_a_barely_steering_controller_shows_a_small_amplitude_ratio():
    reference = _wave()
    timid = _wave(gain=0.1)

    stats = compare_steering(reference, timid, 0.0, 20.0)

    assert stats.correlation == pytest.approx(1.0, abs=1e-6)
    assert stats.amplitude_ratio == pytest.approx(0.1, abs=1e-6)


def test_direction_agreement_ignores_the_near_zero_straights():
    # Both sit at ~0 on a straight; counting sign matches there would report
    # agreement for noise.
    reference = Series((0.0, 1.0, 2.0, 3.0), (0.5, -0.5, 40.0, -40.0))
    candidate = Series((0.0, 1.0, 2.0, 3.0), (-0.5, 0.5, 35.0, -35.0))

    stats = compare_steering(
        reference, candidate, 0.0, 3.0, grid_hz=1.0, active_threshold_deg=5.0
    )

    assert stats.active_samples == 2
    assert stats.sign_agreement == pytest.approx(1.0)


def test_no_overlap_yields_empty_statistics_rather_than_a_crash():
    reference = Series((0.0, 1.0), (10.0, 10.0))
    candidate = Series((50.0, 51.0), (10.0, 10.0))

    stats = compare_steering(reference, candidate, 0.0, 1.0)

    assert stats.samples == 0
    assert 'no overlapping samples' in stats.format('x')


def test_bias_reports_a_constant_offset():
    reference = Series(tuple(i * 0.05 for i in range(100)), (10.0,) * 100)
    candidate = Series(tuple(i * 0.05 for i in range(100)), (13.0,) * 100)

    stats = compare_steering(reference, candidate, 0.0, 4.9)

    assert stats.bias_deg == pytest.approx(3.0)
    assert stats.rmse_deg == pytest.approx(3.0)


def test_saturation_ratio_flags_a_bang_bang_controller():
    pinned = Series((0.0, 1.0, 2.0, 3.0), (100.0, -100.0, 100.0, 5.0))

    assert saturation_ratio(pinned) == pytest.approx(0.75)


def test_command_rate_measures_publish_frequency():
    series = Series(tuple(index * 0.05 for index in range(21)), (0.0,) * 21)

    assert command_rate_hz(series) == pytest.approx(20.0)


def test_grid_is_empty_when_the_window_is():
    assert build_grid(5.0, 5.0, 20.0) == []
