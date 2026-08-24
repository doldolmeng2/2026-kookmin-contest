"""Turn recorded timestamps into absolute publish deadlines.

Every deadline is computed from a single playback epoch, never from "now plus the
gap to the next message".  Sleeping gap-by-gap accumulates every oversleep, so a
200 s run drifts by however much the OS was late in total; anchoring each deadline
to the epoch keeps a late publish from moving the ones after it.

Pure module: integers and dataclasses only, so the whole schedule can be asserted
in tests without a clock.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple


class ScheduleError(ValueError):
    """Raised when the requested playback window cannot produce a schedule."""


@dataclass(frozen=True)
class ScheduleEntry:
    """One publish, placed on the playback timeline."""

    #: Index into ``CueData.records``.
    record_index: int
    #: Index into ``CueData.topics``.
    topic_index: int
    #: Timestamp as recorded in the bag.
    bag_timestamp_ns: int
    #: Offset from the first replayed record, as recorded (rate not applied).
    bag_offset_ns: int
    #: Offset from the playback epoch, after ``rate`` scaling and loop placement.
    delay_ns: int
    #: 0 for the first pass through the cue, 1 for the first repeat, and so on.
    loop_index: int = 0


@dataclass(frozen=True)
class Schedule:
    """The full, immutable playback plan."""

    entries: Tuple[ScheduleEntry, ...]
    rate: float
    loop_count: int
    #: Bag timestamp the playback epoch corresponds to.
    epoch_bag_timestamp_ns: int
    #: Wall duration of one pass, after rate scaling.
    pass_duration_ns: int

    def __len__(self) -> int:
        return len(self.entries)

    def total_duration_ns(self) -> int:
        return self.entries[-1].delay_ns if self.entries else 0

    def deadline_ns(self, entry: ScheduleEntry, epoch_ns: int) -> int:
        return epoch_ns + entry.delay_ns


def _window_bounds(
    timestamps: Sequence[int],
    start_offset_s: float,
    end_offset_s: Optional[float],
) -> Tuple[int, int]:
    first = timestamps[0]
    last = timestamps[-1]
    if start_offset_s < 0.0:
        raise ScheduleError('start_offset must not be negative')
    start_ns = first + int(round(start_offset_s * 1e9))
    if end_offset_s is None or end_offset_s < 0.0:
        end_ns = last
    else:
        end_ns = first + int(round(end_offset_s * 1e9))
    if end_ns < start_ns:
        raise ScheduleError(
            'playback window is empty: start_offset=%.6f end_offset=%s'
            % (start_offset_s, end_offset_s)
        )
    return (start_ns, end_ns)


def build_schedule(
    records: Sequence[Tuple[int, int, bytes]],
    rate: float = 1.0,
    start_offset_s: float = 0.0,
    end_offset_s: Optional[float] = None,
    loop_count: int = 1,
    loop_gap_s: float = 0.0,
) -> Schedule:
    """Place every selected record on the playback timeline.

    ``loop_count`` of 0 means "one pass"; repeats are laid out end to end with
    ``loop_gap_s`` between them, keeping the within-pass spacing identical to the
    recording so a looped run is still a faithful repeat of the same motion.
    """
    if rate <= 0.0:
        raise ScheduleError('rate must be positive, got %r' % rate)
    if loop_gap_s < 0.0:
        raise ScheduleError('loop_gap must not be negative')
    if not records:
        raise ScheduleError('no records to schedule')

    timestamps = [timestamp for _, timestamp, _ in records]
    start_ns, end_ns = _window_bounds(timestamps, start_offset_s, end_offset_s)

    selected: List[Tuple[int, int, int]] = []
    for record_index, (topic_index, timestamp, _) in enumerate(records):
        if start_ns <= timestamp <= end_ns:
            selected.append((record_index, topic_index, timestamp))

    if not selected:
        raise ScheduleError(
            'playback window [%.6f, %s] s selects no record'
            % (start_offset_s, 'end' if end_offset_s is None else '%.6f' % end_offset_s)
        )

    epoch_bag_ns = selected[0][2]
    span_ns = selected[-1][2] - epoch_bag_ns
    pass_duration_ns = int(round(span_ns / rate))
    gap_ns = int(round(loop_gap_s * 1e9))
    passes = max(1, int(loop_count) if loop_count else 1)

    entries: List[ScheduleEntry] = []
    for loop_index in range(passes):
        loop_base = loop_index * (pass_duration_ns + gap_ns)
        for record_index, topic_index, timestamp in selected:
            bag_offset = timestamp - epoch_bag_ns
            entries.append(
                ScheduleEntry(
                    record_index=record_index,
                    topic_index=topic_index,
                    bag_timestamp_ns=timestamp,
                    bag_offset_ns=bag_offset,
                    delay_ns=loop_base + int(round(bag_offset / rate)),
                    loop_index=loop_index,
                )
            )

    return Schedule(
        entries=tuple(entries),
        rate=float(rate),
        loop_count=passes,
        epoch_bag_timestamp_ns=epoch_bag_ns,
        pass_duration_ns=pass_duration_ns,
    )


def infinite_loop_requested(loop_count: int) -> bool:
    """``loop:=-1`` means keep repeating until the node is asked to stop."""
    return int(loop_count) < 0


def per_topic_offsets(schedule: Schedule, loop_index: int = 0) -> dict:
    """Recorded offsets of each topic within one pass, for the verifier."""
    offsets: dict = {}
    for entry in schedule.entries:
        if entry.loop_index != loop_index:
            continue
        offsets.setdefault(entry.topic_index, []).append(entry.bag_offset_ns)
    return offsets


def describe_schedule(schedule: Schedule, topic_names: Sequence[str]) -> str:
    """Human-readable plan summary, printed before the first publish."""
    lines = [
        'schedule: %d publishes over %.3f s (rate x%g, %d pass%s)'
        % (
            len(schedule.entries),
            schedule.total_duration_ns() / 1e9,
            schedule.rate,
            schedule.loop_count,
            '' if schedule.loop_count == 1 else 'es',
        )
    ]
    tally: dict = {}
    for entry in schedule.entries:
        tally[entry.topic_index] = tally.get(entry.topic_index, 0) + 1
    for topic_index in sorted(tally):
        name = (
            topic_names[topic_index]
            if topic_index < len(topic_names)
            else 'topic[%d]' % topic_index
        )
        lines.append('  %-34s %6d publishes' % (name, tally[topic_index]))
    return '\n'.join(lines)
