"""Lap counting decides when a zero output is a finish and when it is a failure."""

import pytest

from drive_eval.modes import (
    TOTAL_LAPS,
    laps_completed_by,
    traffic_encounters,
)
from drive_eval.timebase import Series


def _signal_series(spans, hz=20.0):
    """Build /object_info[0] over time from (start_s, end_s, value) spans."""
    times = []
    values = []
    step = 1.0 / hz
    end = max(span[1] for span in spans)
    time_s = 0.0
    while time_s <= end:
        value = 0
        for start, stop, signal in spans:
            if start <= time_s <= stop:
                value = signal
                break
        times.append(time_s)
        values.append(float(value))
        time_s += step
    return Series(tuple(times), tuple(values))


# The 2026-08-13 recording: the light is seen four times, twice showing left.
BAG_PASSES = [(1.0, 7.0, 2), (69.0, 73.0, 2), (136.0, 139.5, 3), (177.0, 181.0, 3)]


def test_each_pass_of_the_fixture_is_one_encounter():
    encounters = traffic_encounters(_signal_series(BAG_PASSES))

    assert [round(encounter.time_s) for encounter in encounters] == [1, 69, 136, 177]


def test_a_flickering_detection_within_one_pass_is_not_two_encounters():
    flickering = [(1.0, 3.0, 2), (3.5, 7.0, 2)]

    assert len(traffic_encounters(_signal_series(flickering))) == 1


def test_a_stop_signal_is_not_a_lap():
    assert traffic_encounters(_signal_series([(1.0, 7.0, 1)])) == []


def test_the_left_signal_counts_as_a_pass():
    encounters = traffic_encounters(_signal_series([(1.0, 7.0, 3)]))

    assert len(encounters) == 1
    assert encounters[0].is_left


def test_starting_in_wait_green_does_not_count_the_start_line():
    encounters = traffic_encounters(_signal_series(BAG_PASSES))

    # Three laps are complete at the fourth sighting, not the third.
    assert laps_completed_by(encounters, 176.0, started_in_wait_green=True) == 2
    assert laps_completed_by(encounters, 178.0, started_in_wait_green=True) == 3


def test_starting_in_lane_drive_counts_the_start_line_and_finishes_a_lap_early():
    encounters = traffic_encounters(_signal_series(BAG_PASSES))

    # Same recording, same sightings: FINISH lands a whole lap sooner.
    assert laps_completed_by(encounters, 137.0, started_in_wait_green=False) == 3
    assert laps_completed_by(encounters, 137.0, started_in_wait_green=True) == 2


def test_lap_count_never_exceeds_the_race_length():
    encounters = traffic_encounters(_signal_series(BAG_PASSES))

    assert laps_completed_by(encounters, 999.0, started_in_wait_green=False) == TOTAL_LAPS


def test_no_sightings_means_no_laps():
    assert laps_completed_by([], 100.0, started_in_wait_green=True) == 0
