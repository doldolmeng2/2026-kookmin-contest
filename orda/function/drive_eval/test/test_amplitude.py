"""An RMS ratio against a human driver is not a measure of how much a controller steers.

In the 2026-08-13 cone corridors the driver's steering is bimodal: median 1.5 deg
with roughly a third of the samples at the 100 deg stop.  The stack steers
continuously - mean 19 deg, peak 44 deg, never reaching its own 45 deg clamp - and
still scores 0.31 on candidate-RMS over reference-RMS.  Reading that as "barely
steering" sent an earlier diagnosis at the wrong file.
"""

import math

import pytest

from drive_eval.steering import compare_steering
from drive_eval.timebase import Series


def _series(values, hz=20.0):
    step = 1.0 / hz
    return Series(
        tuple(index * step for index in range(len(values))),
        tuple(float(value) for value in values),
    )


def _bimodal_driver(count=400, period=40, spike=100.0):
    """Mostly straight, with occasional full-lock flicks."""
    return [
        (spike if (index % period) < 12 else 1.5) * (1 if (index // period) % 2 else -1)
        for index in range(count)
    ]


def _smooth_controller(count=400, period=40, amplitude=30.0):
    return [
        amplitude * math.sin(2.0 * math.pi * index / period) for index in range(count)
    ]


def test_the_rms_ratio_understates_a_controller_that_steers_continuously():
    driver = _series(_bimodal_driver())
    stack = _series(_smooth_controller())

    stats = compare_steering(driver, stack, 0.0, 19.9)

    # The very number that misled: low, while the stack is steering the whole time.
    assert stats.amplitude_ratio < 0.6
    assert stats.candidate_median_abs_deg > 15.0


def test_percentiles_show_the_two_distributions_apart():
    driver = _series(_bimodal_driver())
    stack = _series(_smooth_controller())

    stats = compare_steering(driver, stack, 0.0, 19.9)

    assert stats.reference_median_abs_deg == pytest.approx(1.5, abs=0.5)
    assert stats.reference_max_abs_deg == pytest.approx(100.0, abs=0.5)
    assert stats.candidate_max_abs_deg == pytest.approx(30.0, abs=0.5)


def test_a_controller_that_really_does_nothing_is_still_caught():
    driver = _series(_bimodal_driver())
    asleep = _series([0.2] * 400)

    stats = compare_steering(driver, asleep, 0.0, 19.9)

    assert stats.candidate_p90_abs_deg < 5.0


def test_a_working_controller_clears_the_absolute_floor():
    driver = _series(_bimodal_driver())
    stack = _series(_smooth_controller())

    stats = compare_steering(driver, stack, 0.0, 19.9)

    assert stats.candidate_p90_abs_deg >= 5.0


def test_percentiles_are_reported_for_both_sides_in_the_text():
    driver = _series(_bimodal_driver())
    stack = _series(_smooth_controller())

    text = compare_steering(driver, stack, 0.0, 19.9).format('cone corridor')

    assert 'stack/recorded' in text
    assert 'p90' in text and 'max' in text
