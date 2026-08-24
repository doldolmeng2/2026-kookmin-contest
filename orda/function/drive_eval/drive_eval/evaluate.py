"""Put the pieces together: load both bags, line them up, grade the run."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Tuple
import json

from .bagsource import BagReadError, decode_records, load_reference, topic_types
from .cone_truth import (
    ConeDetectorConfig,
    ConeWindow,
    ScanSample,
    cone_windows,
    describe_windows,
    observe,
)
from .modes import (
    CONE_DRIVE,
    TOTAL_LAPS,
    ChatterSpan,
    ConeSessionReport,
    Interval,
    ModeTransition,
    chatter_spans,
    evaluate_cone_sessions,
    format_transitions,
    intervals_of,
    laps_completed_by,
    mode_name,
    mode_occupancy,
    silent_spans,
    traffic_encounters,
    transitions,
)
from .steering import (
    SteeringStats,
    command_rate_hz,
    compare_steering,
    saturation_ratio,
    zero_output_spans,
)
from .timebase import Series, Timebase, TimebaseError

#: What the recording provides as reference.
REFERENCE_STEERING_TOPIC = '/xycar_motor'
REFERENCE_SCAN_TOPIC = '/scan'

#: What the isolated bag-test launch publishes.
RUN_STEERING_TOPIC = '/kmu_main_offline/xycar_motor'
RUN_MODE_TOPIC = '/mode_info'
RUN_CLOCK_TOPIC = '/clock'
OBJECT_INFO_TOPIC = '/object_info'
RUN_OPTIONAL_TOPICS = ('/rubbercone_info', '/lane_offset', OBJECT_INFO_TOPIC)

#: ``ExternalModeInfoCode.WAIT_GREEN``.
WAIT_GREEN = 0

SIGNAL_NAMES = {0: 'none', 1: 'stop', 2: 'green', 3: 'left'}


@dataclass(frozen=True)
class Thresholds:
    """Where "well enough" is drawn, so a verdict is never a matter of taste.

    These are pass marks for *agreement with a human lap*, not for lap quality.
    A controller that steers the right way at the right moment through the cone
    corridor is doing its job even when its line differs from the driver's.
    """

    #: Fraction of active samples where both turn the same way.
    min_sign_agreement: float = 0.65
    #: Pearson correlation of the two steering signals.
    min_correlation: float = 0.35
    #: The stack must actually turn the wheel, measured in degrees rather than
    #: against the driver.
    #:
    #: An RMS ratio against a human reference is not a usable pass mark here: in
    #: this recording the driver's cone-corridor steering is bimodal - median
    #: 1.5 deg with 27-33% of samples at the 100 deg stop - so the RMS is set by
    #: the spikes.  The stack steers continuously at a mean of 19 deg and peaks
    #: at 44 deg, and still scores 0.31 on that ratio.  The ratio stays in the
    #: report as context; the verdict uses an absolute floor.
    min_candidate_p90_deg: float = 5.0
    #: A cone session may start this late and still count as caught.
    max_cone_entry_delay_s: float = 2.50
    #: How much of each corridor must be driven in CONE_DRIVE.
    min_cone_coverage: float = 0.50
    #: A corridor shorter than this is listed but never fails the run.  The
    #: /scan ground truth cannot tell two obstacle cars either side of the lane
    #: from a two-second cone gate, and a false alarm here would train the team
    #: to ignore the report.
    cone_confident_min_s: float = 3.0
    #: Commands pinned at the steering limit for more than this look like a
    #: bang-bang controller rather than tracking.
    max_saturation_ratio: float = 0.35
    #: Playback that fell behind invalidates every timing conclusion.
    min_playback_rate: float = 0.90
    max_playback_rate: float = 1.10
    #: main_node publishes /mode_info once per control cycle (50 Hz, see
    #: CONTROL_PERIOD_S in control.py).  Meaningfully more than that means two
    #: main_nodes were alive on the same topics — a leftover from a previous run
    #: — and the whole recording mixes two FSMs.
    max_mode_rate_hz: float = 80.0
    #: A flat-zero command run longer than this is a stretch the car stood still
    #: for, not a straight.
    dead_output_min_s: float = 2.0
    #: /mode_info quiet for longer than this means the FSM left the published
    #: contract (FINISH has no external code).
    mode_silence_min_s: float = 2.0
    #: Leaving this much of the recording undriven at the end is the stack
    #: parking itself while the course still had track left.
    gave_up_min_s: float = 10.0


@dataclass
class EvaluationReport:
    """Result of grading one run against one recording."""

    source_bag: str = ''
    run_bag: str = ''
    steering_topic: str = ''
    playback_rate: float = 0.0
    overlap_start_s: float = 0.0
    overlap_end_s: float = 0.0
    #: Nothing past this is graded: the race ended, or the recording did.
    grading_end_s: float = 0.0
    started_in_wait_green: bool = False
    traffic_encounters: List[Tuple[float, int]] = field(default_factory=list)
    laps_completed: int = 0
    race_finished_at_s: Optional[float] = None
    reference_rate_hz: float = 0.0
    candidate_rate_hz: float = 0.0
    mode_rate_hz: float = 0.0
    candidate_saturation: float = 0.0
    cone_windows: List[Dict[str, float]] = field(default_factory=list)
    cone_sessions: Optional[ConeSessionReport] = None
    mode_transitions: List[ModeTransition] = field(default_factory=list)
    mode_occupancy_s: Dict[str, float] = field(default_factory=dict)
    illegal_transitions: int = 0
    mode_chatter: List[ChatterSpan] = field(default_factory=list)
    dead_output_spans: List[Tuple[float, float]] = field(default_factory=list)
    dead_output_s: float = 0.0
    mode_silent_spans: List[Tuple[float, float]] = field(default_factory=list)
    #: When the stack stopped driving for good, if it did so before the sensors
    #: ran out.  On track this is the moment the car parks itself mid-race.
    gave_up_at_s: Optional[float] = None
    abandoned_s: float = 0.0
    steering_overall: SteeringStats = field(default_factory=SteeringStats)
    steering_driving: SteeringStats = field(default_factory=SteeringStats)
    steering_in_cone: SteeringStats = field(default_factory=SteeringStats)
    steering_outside_cone: SteeringStats = field(default_factory=SteeringStats)
    steering_per_window: Dict[str, SteeringStats] = field(default_factory=dict)
    findings: List[str] = field(default_factory=list)
    verdicts: Dict[str, str] = field(default_factory=dict)
    notes: List[str] = field(default_factory=list)

    def passed(self) -> bool:
        return all(value == 'PASS' for value in self.verdicts.values())

    def as_dict(self) -> Dict[str, object]:
        return {
            'source_bag': self.source_bag,
            'run_bag': self.run_bag,
            'steering_topic': self.steering_topic,
            'playback_rate': self.playback_rate,
            'overlap_start_s': self.overlap_start_s,
            'overlap_end_s': self.overlap_end_s,
            'grading_end_s': self.grading_end_s,
            'started_in_wait_green': self.started_in_wait_green,
            'traffic_encounters': self.traffic_encounters,
            'laps_completed': self.laps_completed,
            'race_finished_at_s': self.race_finished_at_s,
            'reference_rate_hz': self.reference_rate_hz,
            'candidate_rate_hz': self.candidate_rate_hz,
            'mode_rate_hz': self.mode_rate_hz,
            'candidate_saturation': self.candidate_saturation,
            'cone_windows': self.cone_windows,
            'cone_sessions': (
                self.cone_sessions.as_dict() if self.cone_sessions else {}
            ),
            'mode_transitions': [
                {
                    'time_s': edge.time_s,
                    'source': edge.source,
                    'target': edge.target,
                    'legal': edge.legal,
                }
                for edge in self.mode_transitions
            ],
            'mode_occupancy_s': self.mode_occupancy_s,
            'illegal_transitions': self.illegal_transitions,
            'mode_chatter': [
                {
                    'start_s': span.start_s,
                    'end_s': span.end_s,
                    'flips': span.flips,
                    'modes': list(span.modes),
                }
                for span in self.mode_chatter
            ],
            'dead_output_spans': self.dead_output_spans,
            'dead_output_s': self.dead_output_s,
            'mode_silent_spans': self.mode_silent_spans,
            'gave_up_at_s': self.gave_up_at_s,
            'abandoned_s': self.abandoned_s,
            'steering_overall': self.steering_overall.as_dict(),
            'steering_driving': self.steering_driving.as_dict(),
            'steering_in_cone': self.steering_in_cone.as_dict(),
            'steering_outside_cone': self.steering_outside_cone.as_dict(),
            'steering_per_window': {
                name: stats.as_dict()
                for name, stats in self.steering_per_window.items()
            },
            'findings': self.findings,
            'verdicts': self.verdicts,
            'notes': self.notes,
            'passed': self.passed(),
        }

    def to_json(self, indent: int = 2) -> str:
        return json.dumps(self.as_dict(), indent=indent, sort_keys=True)

    def format(self) -> str:
        rule = '=' * 78
        lines = [
            rule,
            'drive_eval - xycar_ws stack vs. recorded run',
            rule,
            '  source bag       %s' % self.source_bag,
            '  run bag          %s' % self.run_bag,
            '  stack steering   %s' % self.steering_topic,
            '  playback rate    %.4f bag-s per wall-s' % self.playback_rate,
            '  compared window  %.2f s .. %.2f s%s'
            % (
                self.overlap_start_s,
                self.grading_end_s,
                ''
                if self.race_finished_at_s is None
                else '  (race ended here; recording runs to %.2f s)'
                % self.overlap_end_s,
            ),
            '  laps counted     %d of %d, started in %s'
            % (
                self.laps_completed,
                TOTAL_LAPS,
                'WAIT_GREEN' if self.started_in_wait_green else 'LANE_DRIVE',
            ),
            '  command rates    recorded %.2f Hz, stack %.2f Hz, /mode_info %.2f Hz'
            % (self.reference_rate_hz, self.candidate_rate_hz, self.mode_rate_hz),
            '',
            'mode timeline (/mode_info):',
            format_transitions(self.mode_transitions),
            '',
        ]
        if self.mode_chatter:
            lines.append('')
            lines.append(
                '/mode_info changed faster than a mission boundary can be crossed:'
            )
            for span in self.mode_chatter:
                lines.append(span.format())
            lines.append(
                '  (check the node FSM log before calling this a state-machine '
                'defect - see the output-rate bursts below)'
            )
        lines.append('')
        lines.append('time in each mode:')
        for name in sorted(self.mode_occupancy_s, key=lambda k: -self.mode_occupancy_s[k]):
            lines.append('  %-14s %7.2f s' % (name, self.mode_occupancy_s[name]))
        lines.append('')
        lines.append('traffic-light passes the stack perceived (/object_info[0]):')
        if self.traffic_encounters:
            for index, (time_s, signal) in enumerate(self.traffic_encounters, start=1):
                counted = (not self.started_in_wait_green) or index > 1
                lines.append(
                    '  #%d  %7.2f s  %-6s %s'
                    % (
                        index,
                        time_s,
                        SIGNAL_NAMES.get(signal, str(signal)),
                        'counted as a completed lap' if counted else 'consumed by the start',
                    )
                )
        else:
            lines.append('  (none)')
        lines.append('')
        lines.append('cone corridors found in /scan:')
        lines.append(
            describe_windows(
                [
                    ConeWindow(
                        start_s=window['start_s'],
                        end_s=window['end_s'],
                        peak_cones=int(window['peak_cones']),
                        scans=int(window['scans']),
                    )
                    for window in self.cone_windows
                ]
            )
        )
        if self.mode_silent_spans:
            lines.append('')
            lines.append(
                '/mode_info went quiet (FINISH has no external code, so a mode '
                'timeline alone cannot see these):'
            )
            for start_s, end_s in self.mode_silent_spans:
                lines.append(
                    '  %7.2f s .. %7.2f s  (%.2f s)'
                    % (start_s, end_s, end_s - start_s)
                )
        lines.append('')
        if self.cone_sessions is not None:
            lines.append(self.cone_sessions.format())
        lines.append('')
        if self.race_finished_at_s is not None:
            lines.append(
                'the stack finished its %d laps at %.2f s and held zero from '
                'there, as FINISH requires' % (TOTAL_LAPS, self.race_finished_at_s)
            )
            lines.append('')
        if self.gave_up_at_s is not None:
            lines.append(
                'the stack stopped driving at %.2f s with only %d of %d laps '
                'counted, and never resumed (%.2f s of the recording left '
                'undriven)'
                % (
                    self.gave_up_at_s,
                    self.laps_completed,
                    TOTAL_LAPS,
                    self.abandoned_s,
                )
            )
            lines.append('')
        if self.dead_output_spans:
            lines.append(
                'stack commanded a flat zero steering angle for %.2f s total:'
                % self.dead_output_s
            )
            for start_s, end_s in self.dead_output_spans:
                lines.append(
                    '  %7.2f s .. %7.2f s  (%.2f s)'
                    % (start_s, end_s, end_s - start_s)
                )
            lines.append('')
        lines.append('steering agreement with the recorded command:')
        lines.append(self.steering_overall.format('whole run'))
        if self.dead_output_spans:
            lines.append(
                self.steering_driving.format('whole run, dead time removed')
            )
        lines.append(self.steering_in_cone.format('inside cone corridors'))
        lines.append(self.steering_outside_cone.format('outside cone corridors'))
        for name in sorted(self.steering_per_window):
            lines.append(self.steering_per_window[name].format(name))
        lines.append(
            '  %-28s %.1f%% of stack commands sit at the steering limit'
            % ('saturation', 100.0 * self.candidate_saturation)
        )
        if self.findings:
            lines.append('')
            lines.append('findings:')
            for finding in self.findings:
                lines.append('  - %s' % finding)
        if self.notes:
            lines.append('')
            for note in self.notes:
                lines.append('note: %s' % note)
        lines.append('')
        lines.append('verdicts:')
        for name in sorted(self.verdicts):
            lines.append('  %-22s %s' % (name, self.verdicts[name]))
        lines.append('')
        lines.append(
            '  RESULT: %s' % ('PASS' if self.passed() else 'ATTENTION NEEDED')
        )
        lines.append(rule)
        return '\n'.join(lines)


def bag_start_ns(bag_path: Path, fallback_ns: int) -> int:
    """Timestamp the source bag calls t=0, so every report shares one origin."""
    metadata = bag_path / 'metadata.yaml'
    if metadata.is_file():
        try:
            import yaml

            with metadata.open('r', encoding='utf-8') as handle:
                info = (yaml.safe_load(handle) or {}).get(
                    'rosbag2_bagfile_information', {}
                )
            starting = (info.get('starting_time') or {}).get(
                'nanoseconds_since_epoch'
            )
            if starting:
                return int(starting)
        except Exception:
            pass
    return fallback_ns


def _series_from(
    records: Sequence[Tuple[int, object]],
    getter: Callable[[object], Optional[float]],
    epoch_ns: int,
    timebase: Optional[Timebase] = None,
) -> Series:
    times: List[float] = []
    values: List[float] = []
    for timestamp_ns, message in records:
        value = getter(message)
        if value is None:
            continue
        sim_ns = timebase.sim_at(timestamp_ns) if timebase else timestamp_ns
        times.append((sim_ns - epoch_ns) / 1e9)
        values.append(float(value))
    order = sorted(range(len(times)), key=lambda index: times[index])
    return Series(
        tuple(times[index] for index in order),
        tuple(values[index] for index in order),
    )


def scan_samples(
    records: Sequence[Tuple[int, object]], epoch_ns: int
) -> List[ScanSample]:
    """LaserScan messages reduced to the geometry the cone rules need."""
    return [
        ScanSample(
            time_s=(timestamp_ns - epoch_ns) / 1e9,
            ranges=tuple(float(value) for value in message.ranges),
            angle_min=float(message.angle_min),
            angle_increment=float(message.angle_increment),
            range_min=float(message.range_min),
            range_max=float(message.range_max),
        )
        for timestamp_ns, message in records
    ]


def _motor_angle(message) -> Optional[float]:
    data = list(getattr(message, 'data', []) or [])
    return float(data[0]) if data else None


def _first_field(message) -> Optional[float]:
    data = list(getattr(message, 'data', []) or [])
    return float(data[0]) if data else None


def _scalar(message) -> Optional[float]:
    value = getattr(message, 'data', None)
    return float(value) if value is not None else None


def evaluate(
    source_bag: str | Path,
    run_bag: str | Path,
    steering_topic: str = RUN_STEERING_TOPIC,
    mode_topic: str = RUN_MODE_TOPIC,
    thresholds: Thresholds = Thresholds(),
    cone_config: ConeDetectorConfig = ConeDetectorConfig(),
    grid_hz: float = 20.0,
    use_cache: bool = True,
    log: Optional[Callable[[str], None]] = None,
) -> EvaluationReport:
    """Grade one recorded stack run against the bag that drove it."""
    source_dir = Path(source_bag).expanduser()
    run_dir = Path(run_bag).expanduser()
    report = EvaluationReport(
        source_bag=str(source_dir),
        run_bag=str(run_dir),
        steering_topic=steering_topic,
    )

    reference = load_reference(
        source_dir,
        (REFERENCE_STEERING_TOPIC, REFERENCE_SCAN_TOPIC),
        use_cache=use_cache,
        log=log,
    )

    available = topic_types(run_dir)
    needed = [RUN_CLOCK_TOPIC, steering_topic, mode_topic]
    missing = [topic for topic in needed if topic not in available]
    if missing:
        raise BagReadError(
            'run bag %s is missing %s. Record it with the drive_eval runner so '
            '/clock and the isolated motor topic are captured.'
            % (run_dir, ', '.join(missing))
        )
    optional = [topic for topic in RUN_OPTIONAL_TOPICS if topic in available]
    run = decode_records(run_dir, needed + optional)

    clock_samples = [
        (
            timestamp_ns,
            int(message.clock.sec) * 1_000_000_000 + int(message.clock.nanosec),
        )
        for timestamp_ns, message in run[RUN_CLOCK_TOPIC]
    ]
    try:
        timebase = Timebase.from_clock_samples(clock_samples)
    except TimebaseError as exc:
        raise BagReadError(str(exc)) from exc
    report.playback_rate = timebase.rate()

    first_reference_ns = min(
        records[0][0]
        for records in reference.values()
        if records
    )
    epoch_ns = bag_start_ns(source_dir, first_reference_ns)

    reference_steering = _series_from(
        reference[REFERENCE_STEERING_TOPIC], _motor_angle, epoch_ns
    )
    candidate_steering = _series_from(
        run[steering_topic], _motor_angle, epoch_ns, timebase
    )
    mode_series = _series_from(run[mode_topic], _scalar, epoch_ns, timebase)

    scans = scan_samples(reference[REFERENCE_SCAN_TOPIC], epoch_ns)

    run_start, run_end = mode_series.span()
    if len(candidate_steering):
        candidate_span = candidate_steering.span()
        run_start = min(run_start, candidate_span[0])
        run_end = max(run_end, candidate_span[1])
    report.overlap_start_s = max(run_start, reference_steering.span()[0])
    report.overlap_end_s = min(run_end, reference_steering.span()[1])

    # The recording is a practice session: the driver keeps going after the third
    # lap.  The stack is only responsible for the race, so find where its race
    # ended and grade nothing past it — otherwise a correct FINISH reads as the
    # stack abandoning the course.
    signal_series = (
        _series_from(run[OBJECT_INFO_TOPIC], _first_field, epoch_ns, timebase)
        if OBJECT_INFO_TOPIC in run
        else Series((), ())
    )
    report.traffic_encounters = [
        (encounter.time_s, encounter.signal)
        for encounter in traffic_encounters(signal_series)
    ]
    report.started_in_wait_green = bool(
        len(mode_series) and int(mode_series.values[0]) == WAIT_GREEN
    )
    report.dead_output_spans = zero_output_spans(
        candidate_steering, min_duration_s=thresholds.dead_output_min_s
    )
    report.dead_output_s = sum(
        end_s - start_s for start_s, end_s in report.dead_output_spans
    )
    trailing = [
        span
        for span in report.dead_output_spans
        if report.overlap_end_s - span[1] <= thresholds.mode_silence_min_s
    ]
    report.grading_end_s = report.overlap_end_s
    if trailing:
        stopped_at = min(span[0] for span in trailing)
        report.laps_completed = laps_completed_by(
            traffic_encounters(signal_series),
            stopped_at,
            report.started_in_wait_green,
        )
        if report.laps_completed >= TOTAL_LAPS:
            # Three laps counted: the stack finished the race it was given.  The
            # rest of the recording is the driver's extra practice lap.
            report.race_finished_at_s = stopped_at
            report.grading_end_s = stopped_at
        else:
            report.gave_up_at_s = stopped_at
            report.abandoned_s = report.overlap_end_s - stopped_at
    else:
        report.laps_completed = laps_completed_by(
            traffic_encounters(signal_series),
            report.overlap_end_s,
            report.started_in_wait_green,
        )

    # Zero output after a legitimate FINISH is the contract, not a defect.
    report.dead_output_spans = [
        (start_s, min(end_s, report.grading_end_s))
        for start_s, end_s in report.dead_output_spans
        if start_s < report.grading_end_s
    ]
    report.dead_output_s = sum(
        end_s - start_s for start_s, end_s in report.dead_output_spans
    )

    windows = [
        window
        for window in cone_windows(
            [observe(scan, cone_config) for scan in scans]
        )
        if window.end_s >= report.overlap_start_s
        and window.start_s <= report.grading_end_s
    ]
    report.cone_windows = [
        {
            'start_s': window.start_s,
            'end_s': window.end_s,
            'peak_cones': window.peak_cones,
            'scans': window.scans,
        }
        for window in windows
    ]

    cone_intervals = intervals_of(mode_series, CONE_DRIVE, end_s=report.grading_end_s)
    report.cone_sessions = evaluate_cone_sessions(windows, cone_intervals)
    report.mode_transitions = transitions(mode_series)
    report.illegal_transitions = sum(
        1 for edge in report.mode_transitions if not edge.legal
    )
    report.mode_chatter = chatter_spans(
        [
            edge
            for edge in report.mode_transitions
            if edge.time_s <= report.grading_end_s
        ]
    )
    report.mode_occupancy_s = mode_occupancy(mode_series, end_s=report.grading_end_s)

    report.mode_silent_spans = [
        (span.start_s, span.end_s)
        for span in silent_spans(
            mode_series,
            max_gap_s=thresholds.mode_silence_min_s,
            end_s=report.grading_end_s,
        )
    ]

    report.reference_rate_hz = command_rate_hz(
        reference_steering.clipped(report.overlap_start_s, report.grading_end_s)
    )
    report.candidate_rate_hz = command_rate_hz(candidate_steering)
    report.mode_rate_hz = command_rate_hz(mode_series)
    report.candidate_saturation = saturation_ratio(candidate_steering)

    report.steering_overall = compare_steering(
        reference_steering,
        candidate_steering,
        report.overlap_start_s,
        report.grading_end_s,
        grid_hz=grid_hz,
    )

    # Flat-zero stretches are already reported on their own; leaving them inside
    # the tracking figures would blame the controller for time the FSM had taken
    # it out of the loop, and hide how it steers when it is actually driving.
    report.steering_driving = (
        _concat_stats(
            reference_steering,
            candidate_steering,
            _complement(
                report.overlap_start_s,
                report.grading_end_s,
                report.dead_output_spans,
            ),
            grid_hz,
        )
        if report.dead_output_spans
        else report.steering_overall
    )

    inside = _concat_stats(
        reference_steering,
        candidate_steering,
        [(window.start_s, window.end_s) for window in windows],
        grid_hz,
    )
    report.steering_in_cone = inside
    report.steering_outside_cone = _concat_stats(
        reference_steering,
        candidate_steering,
        _complement(
            report.overlap_start_s,
            report.grading_end_s,
            [(window.start_s, window.end_s) for window in windows],
        ),
        grid_hz,
    )
    for index, window in enumerate(windows, start=1):
        report.steering_per_window['cone #%d' % index] = compare_steering(
            reference_steering,
            candidate_steering,
            window.start_s,
            window.end_s,
            grid_hz=grid_hz,
        )

    _judge(report, thresholds)
    return report


def _concat_stats(
    reference: Series,
    candidate: Series,
    spans: Sequence[Tuple[float, float]],
    grid_hz: float,
) -> SteeringStats:
    """Compare over the union of several spans by stitching them into one series.

    Concatenating the *samples* rather than averaging the per-span statistics
    keeps every sample equally weighted, so a two-second corridor does not count
    as much as a twenty-second one.
    """
    times: List[float] = []
    reference_values: List[float] = []
    candidate_values: List[float] = []
    cursor = 0.0
    for start_s, end_s in spans:
        if end_s <= start_s:
            continue
        step = 1.0 / grid_hz
        count = int((end_s - start_s) / step) + 1
        for index in range(count):
            time_s = start_s + index * step
            reference_value = reference.value_at(time_s)
            candidate_value = candidate.value_at(time_s)
            if reference_value is None or candidate_value is None:
                continue
            times.append(cursor + index * step)
            reference_values.append(reference_value)
            candidate_values.append(candidate_value)
        cursor += (end_s - start_s) + step

    if len(times) < 2:
        return SteeringStats()
    stitched_reference = Series(tuple(times), tuple(reference_values))
    stitched_candidate = Series(tuple(times), tuple(candidate_values))
    return compare_steering(
        stitched_reference,
        stitched_candidate,
        times[0],
        times[-1],
        grid_hz=grid_hz,
    )


def _complement(
    start_s: float, end_s: float, spans: Sequence[Tuple[float, float]]
) -> List[Tuple[float, float]]:
    """The parts of ``[start, end]`` not covered by ``spans``."""
    result: List[Tuple[float, float]] = []
    cursor = start_s
    for span_start, span_end in sorted(spans):
        if span_start > cursor:
            result.append((cursor, min(span_start, end_s)))
        cursor = max(cursor, span_end)
        if cursor >= end_s:
            break
    if cursor < end_s:
        result.append((cursor, end_s))
    return [(a, b) for a, b in result if b > a]


def _judge(report: EvaluationReport, thresholds: Thresholds) -> None:
    """Turn the measurements into verdicts, recording why each one landed."""
    # Check the capture itself first: if two stacks were publishing, every other
    # number below is a blend of two FSMs and means nothing.
    if report.mode_rate_hz > thresholds.max_mode_rate_hz:
        report.verdicts['run capture'] = 'FAIL'
        report.findings.append(
            '/mode_info arrived at %.0f Hz, far above the 50 Hz control cycle: '
            'more than one main_node was publishing during this recording, so '
            'nothing else in this report can be trusted. Kill every leftover node '
            'before recording again.' % report.mode_rate_hz
        )
    else:
        report.verdicts['run capture'] = 'PASS'

    rate = report.playback_rate
    if thresholds.min_playback_rate <= rate <= thresholds.max_playback_rate:
        report.verdicts['playback'] = 'PASS'
    else:
        report.verdicts['playback'] = 'FAIL'
        report.findings.append(
            'playback ran at %.3f bag-s per wall-s; the stack did not see the '
            'recording at its real speed, so every timing figure below is '
            'suspect' % rate
        )

    if report.candidate_rate_hz <= 0.0:
        report.verdicts['stack output'] = 'FAIL'
        report.findings.append(
            'the stack published no steering command on %s' % report.steering_topic
        )
    elif report.dead_output_spans:
        report.verdicts['stack output'] = 'FAIL'
        longest = max(
            report.dead_output_spans, key=lambda span: span[1] - span[0]
        )
        report.findings.append(
            'the stack kept publishing but commanded a flat zero steering angle '
            'for %.1f s of the run (longest %.1f s from %.1f s); it was alive but '
            'not driving'
            % (
                report.dead_output_s,
                longest[1] - longest[0],
                longest[0],
            )
        )
    else:
        report.verdicts['stack output'] = 'PASS'

    if report.gave_up_at_s is not None and report.abandoned_s > thresholds.gave_up_min_s:
        report.verdicts['finishes the course'] = 'FAIL'
        report.findings.append(
            'the stack stopped driving for good at %.1f s with only %d of %d laps '
            'counted, leaving %.1f s of the recording undriven'
            % (
                report.gave_up_at_s,
                report.laps_completed,
                TOTAL_LAPS,
                report.abandoned_s,
            )
        )
    else:
        report.verdicts['finishes the course'] = 'PASS'

    if not report.started_in_wait_green and report.traffic_encounters:
        report.notes.append(
            'the stack was started in %s, so the first traffic-light pass at '
            '%.1f s was counted as a completed lap. Lap counting only skips the '
            'start when the FSM begins in WAIT_GREEN (race_fsm.py guards '
            '_record_traffic_encounter on that state), so a bag test started in '
            'LANE_DRIVE finishes one whole lap early. Use mode:=0 for a run whose '
            'lap count matches the contest.'
            % (
                mode_name(int(report.mode_transitions[0].source))
                if report.mode_transitions
                else 'a driving mode',
                report.traffic_encounters[0][0],
            )
        )

    if report.mode_silent_spans:
        quiet = sum(end_s - start_s for start_s, end_s in report.mode_silent_spans)
        report.notes.append(
            '/mode_info was quiet for %.1f s in %d stretch(es). Main publishes no '
            'external code for FINISH, so the timeline above holds the last mode '
            'it heard through those stretches rather than showing the real one.'
            % (quiet, len(report.mode_silent_spans))
        )

    sessions = report.cone_sessions
    if sessions is None or sessions.windows == 0:
        report.verdicts['rubbercone'] = 'N/A'
        report.notes.append(
            'no cone corridor was found in /scan over the compared window, so '
            'rubber-cone behaviour was not exercised'
        )
    else:
        problems: List[str] = []
        for match in sessions.matches:
            corridor_s = match.window_end_s - match.window_start_s
            confident = corridor_s >= thresholds.cone_confident_min_s
            if not match.entered:
                message = (
                    'corridor #%d (%.2f-%.2f s, %.2f s) never entered CONE_DRIVE'
                    % (
                        match.window_index,
                        match.window_start_s,
                        match.window_end_s,
                        corridor_s,
                    )
                )
                if confident:
                    problems.append(message)
                else:
                    report.notes.append(
                        message
                        + ' - too short (< %.1f s) for the /scan ground truth to '
                        'be sure it was cones rather than two obstacles either '
                        'side, so it does not fail the run'
                        % thresholds.cone_confident_min_s
                    )
                continue
            if not confident:
                continue
            if (
                match.entry_delay_s is not None
                and match.entry_delay_s > thresholds.max_cone_entry_delay_s
            ):
                problems.append(
                    'corridor #%d entered %.2f s late (limit %.2f s)'
                    % (
                        match.window_index,
                        match.entry_delay_s,
                        thresholds.max_cone_entry_delay_s,
                    )
                )
            if match.coverage < thresholds.min_cone_coverage:
                problems.append(
                    'corridor #%d only %.0f%% covered by CONE_DRIVE (limit %.0f%%)'
                    % (
                        match.window_index,
                        100.0 * match.coverage,
                        100.0 * thresholds.min_cone_coverage,
                    )
                )
        if sessions.spurious:
            problems.append(
                'CONE_DRIVE entered %d time(s) with no cone corridor in /scan'
                % len(sessions.spurious)
            )
        report.verdicts['rubbercone'] = 'PASS' if not problems else 'FAIL'
        report.findings.extend(problems)

    if report.illegal_transitions:
        report.verdicts['mode changes'] = 'FAIL'
        report.findings.append(
            '%d /mode_info transition(s) are not declared in race_fsm.py'
            % report.illegal_transitions
        )
    elif report.mode_chatter:
        report.verdicts['mode changes'] = 'FAIL'
        worst = max(report.mode_chatter, key=lambda span: span.flips)
        report.findings.append(
            '/mode_info reported %d mode changes in %.2f s at %.1f s between %s. '
            'A consumer of that topic acts on changes that a mission boundary '
            'cannot produce.'
            % (
                worst.flips,
                worst.duration_s,
                worst.start_s,
                ' and '.join(worst.modes),
            )
        )
        report.findings.append(
            'this is a /mode_info contract problem, not necessarily an FSM one: '
            'on the 2026-08-13 bag the node logged a single FSM transition and '
            'its own runtime diagnostic held the old mode throughout the burst. '
            'Check the node FSM log for the same window before changing race_fsm.'
        )
    elif not report.mode_transitions:
        report.verdicts['mode changes'] = 'FAIL'
        report.findings.append(
            'the mode never changed over the whole run; the FSM did not react to '
            'anything in the recording'
        )
    else:
        report.verdicts['mode changes'] = 'PASS'

    # Judge the controller on the time it was actually driving; the flat-zero
    # stretches already have their own verdict under 'stack output'.
    steering = (
        report.steering_driving
        if report.steering_driving.samples
        else report.steering_overall
    )
    if not steering.samples:
        report.verdicts['steering'] = 'FAIL'
        report.findings.append(
            'no overlapping steering samples to compare; check that the run and '
            'the recording cover the same part of the bag'
        )
    else:
        problems = []
        if steering.sign_agreement < thresholds.min_sign_agreement:
            problems.append(
                'steering turns the same way as the driver only %.0f%% of the '
                'time (limit %.0f%%)'
                % (
                    100.0 * steering.sign_agreement,
                    100.0 * thresholds.min_sign_agreement,
                )
            )
        if steering.correlation < thresholds.min_correlation:
            problems.append(
                'steering correlation with the recorded command is %.2f '
                '(limit %.2f)' % (steering.correlation, thresholds.min_correlation)
            )
        if steering.candidate_p90_abs_deg < thresholds.min_candidate_p90_deg:
            problems.append(
                'the stack barely turned the wheel: 90%% of its commands stayed '
                'under %.1f deg (floor %.1f deg)'
                % (
                    steering.candidate_p90_abs_deg,
                    thresholds.min_candidate_p90_deg,
                )
            )
        if report.candidate_saturation > thresholds.max_saturation_ratio:
            problems.append(
                '%.0f%% of stack commands sit at the steering limit (limit %.0f%%)'
                % (
                    100.0 * report.candidate_saturation,
                    100.0 * thresholds.max_saturation_ratio,
                )
            )
        report.verdicts['steering'] = 'PASS' if not problems else 'FAIL'
        report.findings.extend(problems)

        if abs(steering.best_lag_s) >= 0.25:
            report.notes.append(
                'steering correlates best with the recorded command shifted by '
                '%+.2f s (r=%.2f vs %.2f unshifted): the stack reacts %s'
                % (
                    steering.best_lag_s,
                    steering.correlation_at_best_lag,
                    steering.correlation,
                    'late' if steering.best_lag_s > 0 else 'early',
                )
            )
