import pytest

from drive_eval.timebase import Series, Timebase, TimebaseError


def _samples(count=11, wall_start=1_000_000_000, sim_start=500_000_000, step=5_000_000):
    return [
        (wall_start + index * step, sim_start + index * step)
        for index in range(count)
    ]


def test_maps_a_wall_stamp_onto_the_bag_timeline():
    timebase = Timebase.from_clock_samples(_samples())

    assert timebase.sim_at(1_000_000_000) == 500_000_000
    assert timebase.sim_at(1_010_000_000) == 510_000_000


def test_interpolates_between_clock_samples():
    # /clock at 200 Hz leaves 5 ms holes; a 20 Hz command lands inside one.
    timebase = Timebase.from_clock_samples(_samples())

    assert timebase.sim_at(1_002_500_000) == 502_500_000


def test_extrapolates_before_the_first_sample_instead_of_dropping_it():
    timebase = Timebase.from_clock_samples(_samples())

    assert timebase.sim_at(999_000_000) == 499_000_000


def test_extrapolates_after_the_last_sample():
    timebase = Timebase.from_clock_samples(_samples())

    assert timebase.sim_at(1_060_000_000) == 560_000_000


def test_playback_rate_reports_slow_playback():
    slow = [(index * 10_000_000, index * 5_000_000) for index in range(20)]

    assert Timebase.from_clock_samples(slow).rate() == pytest.approx(0.5)


def test_duplicate_wall_stamps_do_not_break_the_lookup():
    samples = _samples() + [(1_010_000_000, 510_000_001)]

    timebase = Timebase.from_clock_samples(samples)

    assert timebase.sim_at(1_010_000_000) == 510_000_001


def test_a_run_without_clock_is_rejected_with_an_actionable_message():
    with pytest.raises(TimebaseError) as excinfo:
        Timebase.from_clock_samples([(1, 1)])

    assert '--clock' in str(excinfo.value)


def test_series_holds_the_last_value_forward():
    series = Series((0.0, 1.0, 2.0), (10.0, 20.0, 30.0))

    assert series.value_at(1.5) == 20.0
    assert series.value_at(2.0) == 30.0
    assert series.value_at(9.0) == 30.0


def test_series_has_no_value_before_its_first_sample():
    series = Series((1.0, 2.0), (10.0, 20.0))

    assert series.value_at(0.5) is None


def test_series_clipping_keeps_only_the_window():
    series = Series((0.0, 1.0, 2.0, 3.0), (0.0, 1.0, 2.0, 3.0))

    assert Series.clipped(series, 1.0, 2.0).values == (1.0, 2.0)
