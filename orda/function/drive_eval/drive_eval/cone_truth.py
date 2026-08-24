"""Where the rubber-cone corridor actually was, read straight from ``/scan``.

The bag records no mode and no cone label, so "did the stack drive the cone
section" needs an independent answer.  This module builds one from the same LiDAR
the stack saw, using a deliberately different and much simpler rule than
``rubbercone_node``: cluster the returns inside the search envelope, keep the
cone-sized ones, and call it a corridor while there is at least one cluster on
each side of the car.

That independence is the point.  A ground truth computed by the code under test
would agree with it by construction; this one can disagree, which is what makes a
missed or spurious cone session visible.

Pure geometry — no ROS types cross this boundary, so every rule here is testable
against hand-written scans.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple
import math


@dataclass(frozen=True)
class ConeDetectorConfig:
    """Search envelope, matched to the ``rubbercone_node`` launch defaults.

    Using the node's own envelope keeps the comparison fair: a cone the node was
    never allowed to look at should not count as one it missed.
    """

    #: rubbercone_scan_max_range
    max_range_m: float = 1.10
    #: rubbercone_scan_max_angle
    max_angle_deg: float = 85.0
    #: rubbercone_max_lateral_distance
    max_lateral_m: float = 0.85
    #: Two returns further apart than this start a new cluster.
    cluster_gap_m: float = 0.12
    #: A cone is a small object; a wall is not.
    max_cluster_extent_m: float = 0.30
    min_cluster_points: int = 2
    max_cluster_points: int = 40
    #: Points closer to the axis than this are not counted for either side.
    side_deadband_m: float = 0.05


@dataclass(frozen=True)
class ScanSample:
    """One LaserScan, reduced to what the geometry needs."""

    time_s: float
    ranges: Tuple[float, ...]
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float


@dataclass(frozen=True)
class ConeObservation:
    """What one scan says about the corridor."""

    time_s: float
    cones: Tuple[Tuple[float, float], ...]
    left: int
    right: int

    @property
    def count(self) -> int:
        return len(self.cones)

    @property
    def is_corridor(self) -> bool:
        """A corridor needs cones on both sides; one wall on one side does not."""
        return self.left >= 1 and self.right >= 1


@dataclass(frozen=True)
class ConeWindow:
    """A stretch of the recording where the car was inside a cone corridor."""

    start_s: float
    end_s: float
    peak_cones: int
    scans: int

    @property
    def duration_s(self) -> float:
        return self.end_s - self.start_s

    def contains(self, time_s: float, slack_s: float = 0.0) -> bool:
        return self.start_s - slack_s <= time_s <= self.end_s + slack_s


def detect_cones(
    sample: ScanSample, config: ConeDetectorConfig = ConeDetectorConfig()
) -> List[Tuple[float, float]]:
    """Cone-sized clusters in front of the car, as ``(x, y)`` metres."""
    points: List[Tuple[float, float]] = []
    upper_range = min(sample.range_max, config.max_range_m)
    for index, distance in enumerate(sample.ranges):
        if not math.isfinite(distance):
            continue
        if not (sample.range_min <= distance <= upper_range):
            continue
        angle = sample.angle_min + index * sample.angle_increment
        if abs(math.degrees(angle)) > config.max_angle_deg:
            continue
        x = distance * math.cos(angle)
        y = distance * math.sin(angle)
        if x <= 0.0 or abs(y) > config.max_lateral_m:
            continue
        points.append((x, y))

    clusters: List[List[Tuple[float, float]]] = []
    current: List[Tuple[float, float]] = []
    for point in points:
        if current and math.dist(point, current[-1]) <= config.cluster_gap_m:
            current.append(point)
            continue
        if current:
            clusters.append(current)
        current = [point]
    if current:
        clusters.append(current)

    cones: List[Tuple[float, float]] = []
    for cluster in clusters:
        if not (
            config.min_cluster_points <= len(cluster) <= config.max_cluster_points
        ):
            continue
        xs = [p[0] for p in cluster]
        ys = [p[1] for p in cluster]
        extent = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        if extent > config.max_cluster_extent_m:
            continue
        cones.append((sum(xs) / len(cluster), sum(ys) / len(cluster)))
    return cones


def observe(
    sample: ScanSample, config: ConeDetectorConfig = ConeDetectorConfig()
) -> ConeObservation:
    cones = detect_cones(sample, config)
    left = sum(1 for _, y in cones if y > config.side_deadband_m)
    right = sum(1 for _, y in cones if y < -config.side_deadband_m)
    return ConeObservation(
        time_s=sample.time_s, cones=tuple(cones), left=left, right=right
    )


def cone_windows(
    observations: Sequence[ConeObservation],
    enter_hits: int = 2,
    exit_misses: int = 4,
    min_duration_s: float = 1.0,
) -> List[ConeWindow]:
    """Group corridor scans into windows with entry/exit hysteresis.

    ``exit_misses`` mirrors ``rubbercone_end_missing_frames``: the node needs four
    consecutive empty scans to declare the section over, so the ground truth uses
    the same patience instead of splitting a corridor at every dropped scan.
    """
    windows: List[ConeWindow] = []
    hits = 0
    misses = 0
    start: Optional[float] = None
    last_hit_time: Optional[float] = None
    peak = 0
    scans = 0
    first_hit_time: Optional[float] = None

    for observation in observations:
        if observation.is_corridor:
            misses = 0
            hits += 1
            if first_hit_time is None:
                first_hit_time = observation.time_s
            if start is None and hits >= enter_hits:
                start = first_hit_time
                peak = 0
                scans = 0
            if start is not None:
                peak = max(peak, observation.count)
                scans += 1
                last_hit_time = observation.time_s
        else:
            hits = 0
            first_hit_time = None
            if start is not None:
                misses += 1
                if misses >= exit_misses:
                    windows.append(
                        ConeWindow(
                            start_s=start,
                            end_s=last_hit_time if last_hit_time else start,
                            peak_cones=peak,
                            scans=scans,
                        )
                    )
                    start = None
                    misses = 0

    if start is not None:
        windows.append(
            ConeWindow(
                start_s=start,
                end_s=last_hit_time if last_hit_time else start,
                peak_cones=peak,
                scans=scans,
            )
        )

    return [window for window in windows if window.duration_s >= min_duration_s]


def describe_windows(windows: Sequence[ConeWindow]) -> str:
    if not windows:
        return '  (no cone corridor found in /scan)'
    lines = []
    for index, window in enumerate(windows, start=1):
        lines.append(
            '  #%d  %7.2f s .. %7.2f s  (%.2f s, %d scans, peak %d cones)'
            % (
                index,
                window.start_s,
                window.end_s,
                window.duration_s,
                window.scans,
                window.peak_cones,
            )
        )
    return '\n'.join(lines)
