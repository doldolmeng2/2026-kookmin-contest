import math

import pytest

from drive_eval.cone_truth import (
    ConeDetectorConfig,
    ConeObservation,
    ScanSample,
    cone_windows,
    detect_cones,
    observe,
)

BEAMS = 500
ANGLE_MIN = -math.pi
ANGLE_INCREMENT = 2.0 * math.pi / BEAMS


def _scan(objects, time_s=0.0, background=float('inf')):
    """Build a scan with small objects at the given (x, y) positions."""
    ranges = [background] * BEAMS
    for x, y in objects:
        distance = math.hypot(x, y)
        angle = math.atan2(y, x)
        centre = int(round((angle - ANGLE_MIN) / ANGLE_INCREMENT))
        # A cone subtends a few beams at this distance.
        for offset in (-1, 0, 1):
            index = (centre + offset) % BEAMS
            ranges[index] = distance
    return ScanSample(
        time_s=time_s,
        ranges=tuple(ranges),
        angle_min=ANGLE_MIN,
        angle_increment=ANGLE_INCREMENT,
        range_min=0.10,
        range_max=16.0,
    )


def test_a_cone_on_each_side_is_found():
    cones = detect_cones(_scan([(0.6, 0.35), (0.6, -0.35)]))

    assert len(cones) == 2
    assert any(y > 0 for _, y in cones)
    assert any(y < 0 for _, y in cones)


def test_returns_beyond_the_search_range_are_ignored():
    # 3 m is outside the node's 1.10 m envelope, so it is not a cone it missed.
    assert detect_cones(_scan([(3.0, 0.2)])) == []


def test_returns_outside_the_lateral_window_are_ignored():
    assert detect_cones(_scan([(0.4, 1.5)])) == []


def test_a_wall_is_not_a_cone():
    wall = [(0.8, y / 100.0) for y in range(-80, 81, 2)]

    assert detect_cones(_scan(wall)) == []


def test_a_single_side_is_not_a_corridor():
    observation = observe(_scan([(0.6, 0.35), (0.9, 0.40)]))

    assert observation.count == 2
    assert observation.is_corridor is False


def _corridor(time_s):
    return ConeObservation(time_s=time_s, cones=((0.6, 0.3), (0.6, -0.3)), left=1, right=1)


def _empty(time_s):
    return ConeObservation(time_s=time_s, cones=(), left=0, right=0)


def test_a_sustained_corridor_becomes_one_window():
    observations = [_corridor(index * 0.1) for index in range(40)]

    windows = cone_windows(observations)

    assert len(windows) == 1
    assert windows[0].start_s == pytest.approx(0.0)
    assert windows[0].end_s == pytest.approx(3.9)


def test_a_single_dropped_scan_does_not_split_a_window():
    observations = [_corridor(index * 0.1) for index in range(20)]
    observations[10] = _empty(1.0)
    observations += [_corridor(2.0 + index * 0.1) for index in range(20)]

    windows = cone_windows(observations)

    assert len(windows) == 1


def test_four_consecutive_empty_scans_close_the_window():
    # Same patience as rubbercone_end_missing_frames=4.
    first = [_corridor(index * 0.1) for index in range(20)]
    gap = [_empty(2.0 + index * 0.1) for index in range(10)]
    second = [_corridor(3.0 + index * 0.1) for index in range(20)]

    windows = cone_windows(first + gap + second)

    assert len(windows) == 2


def test_a_one_frame_glimpse_is_not_a_window():
    observations = [_empty(index * 0.1) for index in range(20)]
    observations[5] = _corridor(0.5)

    assert cone_windows(observations) == []


def test_short_windows_are_dropped():
    observations = [_corridor(index * 0.1) for index in range(5)]

    assert cone_windows(observations, min_duration_s=1.0) == []


def test_config_envelope_is_honoured():
    narrow = ConeDetectorConfig(max_range_m=0.5)

    assert detect_cones(_scan([(0.8, 0.2)]), narrow) == []
    assert detect_cones(_scan([(0.8, 0.2)])) != []
