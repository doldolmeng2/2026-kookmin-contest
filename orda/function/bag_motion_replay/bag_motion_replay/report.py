"""Turn a run into numbers that can be checked, not adjectives.

"The timing matches" is only meaningful next to the error it was measured with,
so every run ends with the same table: how far each publish landed from its
recorded position on the timeline, and whether a single payload byte differed.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional, Sequence
import json
import math


def _percentile(sorted_values: Sequence[float], fraction: float) -> float:
    if not sorted_values:
        return 0.0
    if len(sorted_values) == 1:
        return float(sorted_values[0])
    position = fraction * (len(sorted_values) - 1)
    low = int(math.floor(position))
    high = min(low + 1, len(sorted_values) - 1)
    weight = position - low
    return float(sorted_values[low] * (1.0 - weight) + sorted_values[high] * weight)


@dataclass(frozen=True)
class TimingStats:
    """Distribution of signed timing errors, in nanoseconds."""

    count: int = 0
    mean_ns: float = 0.0
    median_ns: float = 0.0
    min_ns: float = 0.0
    max_ns: float = 0.0
    p95_abs_ns: float = 0.0
    p99_abs_ns: float = 0.0
    max_abs_ns: float = 0.0
    rms_ns: float = 0.0

    @classmethod
    def from_errors(cls, errors: Sequence[float]) -> 'TimingStats':
        if not errors:
            return cls()
        ordered = sorted(errors)
        absolute = sorted(abs(value) for value in errors)
        total = float(sum(errors))
        return cls(
            count=len(errors),
            mean_ns=total / len(errors),
            median_ns=_percentile(ordered, 0.5),
            min_ns=float(ordered[0]),
            max_ns=float(ordered[-1]),
            p95_abs_ns=_percentile(absolute, 0.95),
            p99_abs_ns=_percentile(absolute, 0.99),
            max_abs_ns=float(absolute[-1]),
            rms_ns=math.sqrt(sum(value * value for value in errors) / len(errors)),
        )

    def as_dict(self) -> Dict[str, float]:
        return asdict(self)

    def format_row(self, label: str) -> str:
        if not self.count:
            return '  %-34s (no samples)' % label
        return '  %-34s n=%6d  mean=%+8.1f us  p95=%7.1f us  p99=%7.1f us  max=%8.1f us' % (
            label,
            self.count,
            self.mean_ns / 1000.0,
            self.p95_abs_ns / 1000.0,
            self.p99_abs_ns / 1000.0,
            self.max_abs_ns / 1000.0,
        )


@dataclass
class ReplayReport:
    """Everything one replay run is judged on."""

    bag: str = ''
    cue: str = ''
    topic_set: str = ''
    topics: List[str] = field(default_factory=list)
    rate: float = 1.0
    timing_mode: str = 'wall'
    loop_count: int = 1
    scheduled: int = 0
    published: int = 0
    failed: int = 0
    retried: int = 0
    skipped: int = 0
    aborted: bool = False
    abort_reason: str = ''
    scheduler: str = ''
    wall_duration_s: float = 0.0
    bag_duration_s: float = 0.0
    overall: TimingStats = field(default_factory=TimingStats)
    per_topic: Dict[str, TimingStats] = field(default_factory=dict)
    missing_topics: List[str] = field(default_factory=list)
    notes: List[str] = field(default_factory=list)

    def complete(self) -> bool:
        """Every scheduled message went out, and nothing was cut short."""
        return (
            not self.aborted
            and self.failed == 0
            and self.skipped == 0
            and self.published == self.scheduled
            and self.scheduled > 0
        )

    def as_dict(self) -> Dict[str, object]:
        data = asdict(self)
        data['overall'] = self.overall.as_dict()
        data['per_topic'] = {
            name: stats.as_dict() for name, stats in self.per_topic.items()
        }
        data['complete'] = self.complete()
        return data

    def to_json(self, indent: int = 2) -> str:
        return json.dumps(self.as_dict(), indent=indent, sort_keys=True)

    def format(self) -> str:
        lines = [
            '=' * 78,
            'bag_motion_replay run report',
            '=' * 78,
            '  bag              %s' % self.bag,
            '  cue              %s' % self.cue,
            '  topic set        %s (%s)' % (self.topic_set, ', '.join(self.topics)),
            '  rate             x%g   timing mode: %s   passes: %d'
            % (self.rate, self.timing_mode, self.loop_count),
            '  scheduler        %s' % self.scheduler,
            '  recorded span    %.3f s' % self.bag_duration_s,
            '  wall duration    %.3f s' % self.wall_duration_s,
            '  published        %d / %d  (failed %d, retried %d, skipped %d)'
            % (self.published, self.scheduled, self.failed, self.retried, self.skipped),
        ]
        if self.missing_topics:
            lines.append(
                '  not in bag       %s' % ', '.join(self.missing_topics)
            )
        lines.append('')
        lines.append('publish-time error vs. the recorded timeline:')
        lines.append(self.overall.format_row('all topics'))
        for name in sorted(self.per_topic):
            lines.append(self.per_topic[name].format_row(name))
        if self.notes:
            lines.append('')
            for note in self.notes:
                lines.append('note: %s' % note)
        lines.append('')
        lines.append(
            '  RESULT: %s'
            % (
                'complete - every scheduled message published'
                if self.complete()
                else 'INCOMPLETE - see counts above'
            )
        )
        lines.append('=' * 78)
        return '\n'.join(lines)


@dataclass
class VerificationReport:
    """Result of comparing what was received against what was recorded."""

    topics: List[str] = field(default_factory=list)
    expected: Dict[str, int] = field(default_factory=dict)
    received: Dict[str, int] = field(default_factory=dict)
    payload_mismatches: Dict[str, int] = field(default_factory=dict)
    first_mismatch: Optional[str] = None
    interval_stats: Dict[str, TimingStats] = field(default_factory=dict)
    offset_stats: Dict[str, TimingStats] = field(default_factory=dict)
    overall_interval: TimingStats = field(default_factory=TimingStats)
    overall_offset: TimingStats = field(default_factory=TimingStats)
    notes: List[str] = field(default_factory=list)

    def content_exact(self) -> bool:
        return (
            bool(self.topics)
            and not any(self.payload_mismatches.values())
            and all(
                self.received.get(topic, 0) == self.expected.get(topic, 0)
                for topic in self.topics
            )
        )

    def as_dict(self) -> Dict[str, object]:
        data = asdict(self)
        data['interval_stats'] = {
            name: stats.as_dict() for name, stats in self.interval_stats.items()
        }
        data['offset_stats'] = {
            name: stats.as_dict() for name, stats in self.offset_stats.items()
        }
        data['overall_interval'] = self.overall_interval.as_dict()
        data['overall_offset'] = self.overall_offset.as_dict()
        data['content_exact'] = self.content_exact()
        return data

    def to_json(self, indent: int = 2) -> str:
        return json.dumps(self.as_dict(), indent=indent, sort_keys=True)

    def format(self) -> str:
        lines = [
            '=' * 78,
            'bag_motion_replay verification report',
            '=' * 78,
            'message content (received CDR bytes vs. recorded CDR bytes):',
        ]
        for topic in self.topics:
            expected = self.expected.get(topic, 0)
            received = self.received.get(topic, 0)
            mismatched = self.payload_mismatches.get(topic, 0)
            verdict = 'identical' if received == expected and not mismatched else 'MISMATCH'
            lines.append(
                '  %-34s %6d/%-6d  differing bytes in %d msg  %s'
                % (topic, received, expected, mismatched, verdict)
            )
        lines.append('')
        lines.append('inter-message interval error (received gap vs. recorded gap):')
        lines.append(self.overall_interval.format_row('all topics'))
        for topic in self.topics:
            if topic in self.interval_stats:
                lines.append(self.interval_stats[topic].format_row(topic))
        lines.append('')
        lines.append('position-on-timeline error (offset from each topic first message):')
        lines.append(self.overall_offset.format_row('all topics'))
        for topic in self.topics:
            if topic in self.offset_stats:
                lines.append(self.offset_stats[topic].format_row(topic))
        if self.first_mismatch:
            lines.append('')
            lines.append('first mismatch: %s' % self.first_mismatch)
        if self.notes:
            lines.append('')
            for note in self.notes:
                lines.append('note: %s' % note)
        lines.append('')
        lines.append(
            '  RESULT: %s'
            % (
                'content is byte-identical to the recording'
                if self.content_exact()
                else 'CONTENT DOES NOT MATCH THE RECORDING'
            )
        )
        lines.append('=' * 78)
        return '\n'.join(lines)
