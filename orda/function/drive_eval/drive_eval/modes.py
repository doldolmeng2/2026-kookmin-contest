"""Read the mode timeline the stack published and judge it against the track.

``/mode_info`` is a step signal, so the interesting content is its edges: when the
FSM changed mode, and whether each change lines up with something that was really
there.  For the cone section the "really there" comes from :mod:`.cone_truth`,
computed from ``/scan`` independently of the code being graded.

Pure module: it takes a step series and a list of windows and returns findings.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from .cone_truth import ConeWindow
from .timebase import Series

#: ``ExternalModeInfoCode`` in main/main/mode_info.py.
MODE_NAMES: Dict[int, str] = {
    0: 'WAIT_GREEN',
    1: 'LANE_DRIVE',
    2: 'CONE_DRIVE',
    3: 'FIXED_AVOID',
    4: 'OVERTAKE',
    5: 'SHORTCUT',
}

CONE_DRIVE = 2
LANE_DRIVE = 1

#: The transitions race_fsm.py is allowed to make.  Anything else on the wire is
#: either a bug or an undocumented path, and both are worth surfacing.
LEGAL_TRANSITIONS = frozenset(
    {
        (0, 1),
        (1, 2),
        (2, 1),
        (1, 3),
        (3, 1),
        (1, 4),
        (4, 1),
        (1, 5),
        (5, 1),
    }
)


def mode_name(code: int) -> str:
    return MODE_NAMES.get(int(code), 'UNKNOWN(%d)' % code)


@dataclass(frozen=True)
class ModeTransition:
    time_s: float
    source: int
    target: int

    @property
    def legal(self) -> bool:
        return (self.source, self.target) in LEGAL_TRANSITIONS

    def format(self) -> str:
        return '  %8.2f s  %-12s -> %-12s %s' % (
            self.time_s,
            mode_name(self.source),
            mode_name(self.target),
            '' if self.legal else '  <-- not a declared FSM transition',
        )


@dataclass(frozen=True)
class Interval:
    start_s: float
    end_s: float

    @property
    def duration_s(self) -> float:
        return self.end_s - self.start_s

    def overlap_s(self, other: 'Interval') -> float:
        return max(
            0.0, min(self.end_s, other.end_s) - max(self.start_s, other.start_s)
        )


def transitions(series: Series) -> List[ModeTransition]:
    """Edges of a step series, ignoring repeats of the same value."""
    edges: List[ModeTransition] = []
    previous: Optional[int] = None
    for time_s, value in zip(series.times_s, series.values):
        code = int(value)
        if previous is None:
            previous = code
            continue
        if code != previous:
            edges.append(ModeTransition(time_s=time_s, source=previous, target=code))
            previous = code
    return edges


def intervals_of(series: Series, code: int, end_s: Optional[float] = None) -> List[Interval]:
    """Stretches during which the series held ``code``."""
    result: List[Interval] = []
    start: Optional[float] = None
    for time_s, value in zip(series.times_s, series.values):
        if int(value) == code:
            if start is None:
                start = time_s
        elif start is not None:
            result.append(Interval(start, time_s))
            start = None
    if start is not None:
        last = end_s if end_s is not None else (
            series.times_s[-1] if series.times_s else start
        )
        result.append(Interval(start, max(last, start)))
    return result


@dataclass
class ConeSessionMatch:
    """One ground-truth cone window and what the stack did about it."""

    window_index: int
    window_start_s: float
    window_end_s: float
    entered: bool = False
    entry_time_s: Optional[float] = None
    exit_time_s: Optional[float] = None
    entry_delay_s: Optional[float] = None
    exit_delay_s: Optional[float] = None
    coverage: float = 0.0

    def as_dict(self) -> Dict[str, object]:
        return asdict(self)

    def format(self) -> str:
        if not self.entered:
            return (
                '  #%d  %7.2f s .. %7.2f s   NOT ENTERED - the stack stayed out of '
                'CONE_DRIVE for this corridor'
                % (self.window_index, self.window_start_s, self.window_end_s)
            )
        return (
            '  #%d  %7.2f s .. %7.2f s   entered %+.2f s, left %+.2f s, '
            'covered %5.1f%% of the corridor'
            % (
                self.window_index,
                self.window_start_s,
                self.window_end_s,
                self.entry_delay_s if self.entry_delay_s is not None else 0.0,
                self.exit_delay_s if self.exit_delay_s is not None else 0.0,
                100.0 * self.coverage,
            )
        )


@dataclass
class ConeSessionReport:
    """Everything the cone check found."""

    matches: List[ConeSessionMatch] = field(default_factory=list)
    spurious: List[Interval] = field(default_factory=list)
    windows: int = 0
    entered: int = 0

    def all_entered(self) -> bool:
        return self.windows > 0 and self.entered == self.windows

    def as_dict(self) -> Dict[str, object]:
        return {
            'windows': self.windows,
            'entered': self.entered,
            'all_entered': self.all_entered(),
            'matches': [match.as_dict() for match in self.matches],
            'spurious': [
                {'start_s': item.start_s, 'end_s': item.end_s}
                for item in self.spurious
            ],
        }

    def format(self) -> str:
        lines = [
            'rubber-cone sessions (ground truth from /scan, %d corridor(s)):'
            % self.windows
        ]
        for match in self.matches:
            lines.append(match.format())
        if self.spurious:
            lines.append(
                'CONE_DRIVE outside any corridor (%d):' % len(self.spurious)
            )
            for item in self.spurious:
                lines.append(
                    '  %7.2f s .. %7.2f s  (%.2f s)'
                    % (item.start_s, item.end_s, item.duration_s)
                )
        else:
            lines.append('  no CONE_DRIVE outside a corridor')
        return '\n'.join(lines)


def evaluate_cone_sessions(
    windows: Sequence[ConeWindow],
    cone_intervals: Sequence[Interval],
    slack_s: float = 2.0,
) -> ConeSessionReport:
    """Match CONE_DRIVE stretches to the corridors that were really there.

    ``slack_s`` allows a session to start slightly before the ground truth
    declares a corridor: the node needs two consecutive corridor scans to be
    confident, but a single early scan is a legitimate reason to switch, and at
    ~10 Hz that is a real fraction of a second.
    """
    report = ConeSessionReport(windows=len(windows))
    claimed: List[int] = []

    for index, window in enumerate(windows, start=1):
        window_interval = Interval(window.start_s, window.end_s)
        overlapping = [
            (position, interval)
            for position, interval in enumerate(cone_intervals)
            if interval.overlap_s(
                Interval(window.start_s - slack_s, window.end_s + slack_s)
            )
            > 0.0
        ]
        match = ConeSessionMatch(
            window_index=index,
            window_start_s=window.start_s,
            window_end_s=window.end_s,
        )
        if overlapping:
            claimed.extend(position for position, _ in overlapping)
            first = overlapping[0][1]
            last = overlapping[-1][1]
            covered = sum(
                interval.overlap_s(window_interval) for _, interval in overlapping
            )
            match.entered = True
            match.entry_time_s = first.start_s
            match.exit_time_s = last.end_s
            match.entry_delay_s = first.start_s - window.start_s
            match.exit_delay_s = last.end_s - window.end_s
            match.coverage = (
                covered / window_interval.duration_s
                if window_interval.duration_s > 0.0
                else 0.0
            )
            report.entered += 1
        report.matches.append(match)

    claimed_set = set(claimed)
    report.spurious = [
        interval
        for position, interval in enumerate(cone_intervals)
        if position not in claimed_set
    ]
    return report


#: ``/object_info[0]``: 0 none, 1 stop, 2 straight green, 3 left green.
ROUTE_SIGNALS = (2, 3)

#: ``race_context.RaceContext.TOTAL_LAPS``.
TOTAL_LAPS = 3


@dataclass(frozen=True)
class TrafficEncounter:
    """One pass of the four-light fixture, as the stack perceived it."""

    time_s: float
    signal: int

    @property
    def is_left(self) -> bool:
        return self.signal == 3


def traffic_encounters(
    series: Series, release_after_s: float = 5.0
) -> List[TrafficEncounter]:
    """Group route-signal detections into one encounter per fixture pass.

    ``race_fsm`` counts a lap per encounter, so reproducing the same grouping is
    what makes it possible to say whether a FINISH was earned or premature.  The
    release window mirrors ``TrafficEncounterGate``: a signal that returns after
    a continuously neutral gap belongs to the next pass, not the current one.
    """
    encounters: List[TrafficEncounter] = []
    last_signal_at: Optional[float] = None
    for time_s, value in zip(series.times_s, series.values):
        signal = int(value)
        if signal not in ROUTE_SIGNALS:
            continue
        if last_signal_at is None or time_s - last_signal_at >= release_after_s:
            encounters.append(TrafficEncounter(time_s=time_s, signal=signal))
        last_signal_at = time_s
    return encounters


def laps_completed_by(
    encounters: Sequence[TrafficEncounter],
    time_s: float,
    started_in_wait_green: bool,
) -> int:
    """Laps ``race_fsm`` would have counted by ``time_s``.

    ``RaceFSM`` skips ``_record_traffic_encounter`` while in ``WAIT_GREEN``, so
    the starting green is consumed by the start and is not a lap.  Starting the
    stack directly in ``LANE_DRIVE`` — which every bag-test profile does — makes
    that first sighting count, and the race then finishes a whole lap early.
    """
    seen = sum(1 for encounter in encounters if encounter.time_s <= time_s)
    if started_in_wait_green:
        seen = max(0, seen - 1)
    return min(seen, TOTAL_LAPS)


@dataclass(frozen=True)
class ChatterSpan:
    """A stretch where the FSM could not make up its mind."""

    start_s: float
    end_s: float
    flips: int
    modes: Tuple[str, ...]

    @property
    def duration_s(self) -> float:
        return self.end_s - self.start_s

    def format(self) -> str:
        return '  %7.2f s .. %7.2f s  %2d changes in %.2f s between %s' % (
            self.start_s,
            self.end_s,
            self.flips,
            self.duration_s,
            ' and '.join(self.modes),
        )


def chatter_spans(
    edges: Sequence[ModeTransition],
    window_s: float = 1.0,
    min_flips: int = 4,
) -> List[ChatterSpan]:
    """Bursts of value changes on ``/mode_info`` too fast to be real decisions.

    A mission boundary is a place on the track, so crossing it changes the mode
    once.  Four or more changes inside a second means a consumer of this topic is
    being told about transitions that did not happen.

    What this does *not* say is that the FSM flipped.  Measured on the
    2026-08-13 bag: ``/mode_info`` reported 15 changes across the overtake
    boundary and 25 across the shortcut boundary, while ``main_node`` logged a
    single ``FSM ... -> ...`` line each time.  In the same windows every output
    topic - ``/mode_info``, ``/lane_info``, ``/internal/lane_command`` and the
    motor command - jumped from 50 Hz to about 72 Hz together, so the publish
    block ran more often than the 50 Hz control cycle rather than the state
    machine changing its mind.  Confirm against the node's own FSM log before
    reading a burst here as a state-machine defect.
    """
    spans: List[ChatterSpan] = []
    index = 0
    while index < len(edges):
        end = index
        while (
            end + 1 < len(edges)
            and edges[end + 1].time_s - edges[index].time_s <= window_s
        ):
            end += 1
        flips = end - index + 1
        if flips >= min_flips:
            involved = []
            for edge in edges[index:end + 1]:
                for code in (edge.source, edge.target):
                    name = mode_name(code)
                    if name not in involved:
                        involved.append(name)
            spans.append(
                ChatterSpan(
                    start_s=edges[index].time_s,
                    end_s=edges[end].time_s,
                    flips=flips,
                    modes=tuple(involved),
                )
            )
            index = end + 1
        else:
            index += 1
    return spans


def silent_spans(
    series: Series, max_gap_s: float = 2.0, end_s: Optional[float] = None
) -> List[Interval]:
    """Gaps where ``/mode_info`` stopped arriving.

    ``mode_info.external_mode_code`` returns ``None`` for ``FINISH``, so Main
    deliberately publishes nothing while the race is over.  A ``/mode_info``
    consumer therefore cannot distinguish "still in the last mode I heard" from
    "the FSM has finished and is holding zero", and a mode timeline read on its
    own will attribute that whole stretch to the wrong mode.  Finding the silence
    is the only way to see it.
    """
    gaps: List[Interval] = []
    times = list(series.times_s)
    if not times:
        return gaps
    for index in range(1, len(times)):
        if times[index] - times[index - 1] > max_gap_s:
            gaps.append(Interval(times[index - 1], times[index]))
    if end_s is not None and end_s - times[-1] > max_gap_s:
        gaps.append(Interval(times[-1], end_s))
    return gaps


def mode_occupancy(series: Series, end_s: Optional[float] = None) -> Dict[str, float]:
    """Seconds spent in each mode, for a one-line shape check of the run."""
    occupancy: Dict[str, float] = {}
    times = list(series.times_s)
    values = list(series.values)
    if not times:
        return occupancy
    limit = end_s if end_s is not None else times[-1]
    for index, (time_s, value) in enumerate(zip(times, values)):
        following = times[index + 1] if index + 1 < len(times) else limit
        name = mode_name(int(value))
        occupancy[name] = occupancy.get(name, 0.0) + max(0.0, following - time_s)
    return occupancy


def format_transitions(edges: Sequence[ModeTransition]) -> str:
    if not edges:
        return '  (mode never changed)'
    return '\n'.join(edge.format() for edge in edges)
