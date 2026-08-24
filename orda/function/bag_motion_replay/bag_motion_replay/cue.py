"""The replay cue: the recorded command stream, extracted once and kept verbatim.

A cue holds every selected record as ``(topic, recorded timestamp, CDR payload)``
in recorded order.  Building one up front matters for timing: the source bag here
is 21 GB, and pulling a message off disk in the middle of a 20 Hz command stream
would show up directly as publish jitter.  The cue for the command topics is a few
hundred kilobytes, so a whole run is resident in RAM before the first publish.

The container is deliberately trivial — a magic string, a JSON header, then packed
records — so that it can be read without ROS, and so a corrupted or half-written
cue is detected (payload SHA-256) instead of replayed.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple
import hashlib
import json
import os
import struct
import tempfile
import time

from .bagio import BagError, BagRecord, BagTopic, open_bag

MAGIC = b'ORDACUE\x01'
RECORD_HEADER = struct.Struct('<HQI')
CUE_FORMAT_VERSION = 1


class CueError(RuntimeError):
    """Raised when a cue file is missing, damaged, or does not match its bag."""


@dataclass(frozen=True)
class CueTopic:
    """One replayable topic inside a cue."""

    index: int
    name: str
    type_name: str
    offered_qos_profiles: str = ''
    count: int = 0
    payload_bytes: int = 0


@dataclass
class CueData:
    """A whole replay cue held in memory."""

    topics: List[CueTopic]
    records: List[Tuple[int, int, bytes]]
    source: Dict[str, object] = field(default_factory=dict)

    def topic_names(self) -> Tuple[str, ...]:
        return tuple(topic.name for topic in self.topics)

    def name_of(self, topic_index: int) -> str:
        return self.topics[topic_index].name

    def span_ns(self) -> Tuple[int, int]:
        if not self.records:
            return (0, 0)
        return (self.records[0][1], self.records[-1][1])

    def duration_ns(self) -> int:
        first, last = self.span_ns()
        return last - first

    def counts(self) -> Dict[str, int]:
        tally: Dict[str, int] = {topic.name: 0 for topic in self.topics}
        for topic_index, _, _ in self.records:
            tally[self.topics[topic_index].name] += 1
        return tally

    def records_for(self, topic: str) -> List[Tuple[int, bytes]]:
        try:
            index = self.topic_names().index(topic)
        except ValueError:
            return []
        return [
            (timestamp, payload)
            for topic_index, timestamp, payload in self.records
            if topic_index == index
        ]

    def total_payload_bytes(self) -> int:
        return sum(len(payload) for _, _, payload in self.records)


def collect_cue(
    bag_path: str | Path,
    topic_names: Sequence[str],
    progress: Optional[Callable[[int, int], None]] = None,
) -> CueData:
    """Read a bag once and hold the selected topics in memory as a cue."""
    source = open_bag(bag_path)
    bag_topics = source.topics()

    missing = [name for name in topic_names if name not in bag_topics]
    if missing:
        raise CueError(
            'bag %s does not contain: %s' % (bag_path, ', '.join(sorted(missing)))
        )

    ordered = list(topic_names)
    index_of = {name: position for position, name in enumerate(ordered)}
    counts = {name: 0 for name in ordered}
    byte_totals = {name: 0 for name in ordered}
    records: List[Tuple[int, int, bytes]] = []

    expected = sum(bag_topics[name].count for name in ordered)
    for record in source.iter_records(ordered):
        position = index_of[record.topic]
        records.append((position, record.timestamp_ns, record.payload))
        counts[record.topic] += 1
        byte_totals[record.topic] += len(record.payload)
        if progress is not None and len(records) % 2000 == 0:
            progress(len(records), expected)

    records.sort(key=lambda item: item[1])
    if progress is not None:
        progress(len(records), max(expected, len(records)))

    topics = [
        CueTopic(
            index=position,
            name=name,
            type_name=bag_topics[name].type_name,
            offered_qos_profiles=bag_topics[name].offered_qos_profiles,
            count=counts[name],
            payload_bytes=byte_totals[name],
        )
        for position, name in enumerate(ordered)
    ]

    return CueData(
        topics=topics,
        records=records,
        source=_source_fingerprint(bag_path, len(records)),
    )


def _source_fingerprint(bag_path: str | Path, record_count: int) -> Dict[str, object]:
    bag_dir = Path(bag_path).expanduser()
    files = []
    if bag_dir.is_dir():
        for path in sorted(bag_dir.iterdir()):
            if path.suffix in ('.db3', '.mcap'):
                stat = path.stat()
                files.append(
                    {
                        'name': path.name,
                        'size': stat.st_size,
                        'mtime_ns': stat.st_mtime_ns,
                    }
                )
    return {
        'bag_uri': str(bag_dir),
        'files': files,
        'record_count': record_count,
        'built_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
    }


def write_cue(cue: CueData, out_path: str | Path) -> Path:
    """Serialize a cue next to the bag (or anywhere the caller points at)."""
    target = Path(out_path).expanduser()
    target.parent.mkdir(parents=True, exist_ok=True)

    digest = hashlib.sha256()
    payload_bytes = 0
    handle = tempfile.NamedTemporaryFile(
        dir=str(target.parent), prefix=target.name + '.', suffix='.part', delete=False
    )
    payload_path = Path(handle.name)
    try:
        with handle:
            for topic_index, timestamp_ns, payload in cue.records:
                head = RECORD_HEADER.pack(topic_index, timestamp_ns, len(payload))
                handle.write(head)
                handle.write(payload)
                digest.update(head)
                digest.update(payload)
                payload_bytes += len(head) + len(payload)

        header = {
            'format': CUE_FORMAT_VERSION,
            'topics': [
                {
                    'index': topic.index,
                    'name': topic.name,
                    'type': topic.type_name,
                    'offered_qos_profiles': topic.offered_qos_profiles,
                    'count': topic.count,
                    'payload_bytes': topic.payload_bytes,
                }
                for topic in cue.topics
            ],
            'record_count': len(cue.records),
            'payload_bytes': payload_bytes,
            'payload_sha256': digest.hexdigest(),
            'source': cue.source,
        }
        header_blob = json.dumps(header, sort_keys=True).encode('utf-8')

        final = tempfile.NamedTemporaryFile(
            dir=str(target.parent),
            prefix=target.name + '.',
            suffix='.tmp',
            delete=False,
        )
        final_path = Path(final.name)
        with final:
            final.write(MAGIC)
            final.write(struct.pack('<I', len(header_blob)))
            final.write(header_blob)
            with payload_path.open('rb') as payload_handle:
                while True:
                    chunk = payload_handle.read(1 << 20)
                    if not chunk:
                        break
                    final.write(chunk)
            final.flush()
            os.fsync(final.fileno())
        final_path.replace(target)
    finally:
        payload_path.unlink(missing_ok=True)
    return target


def read_cue(cue_path: str | Path, verify: bool = True) -> CueData:
    """Load a cue file, refusing anything that does not hash back to its header."""
    path = Path(cue_path).expanduser()
    if not path.is_file():
        raise CueError('cue file does not exist: %s' % path)

    with path.open('rb') as handle:
        magic = handle.read(len(MAGIC))
        if magic != MAGIC:
            raise CueError('%s is not a bag_motion_replay cue file' % path)
        (header_length,) = struct.unpack('<I', handle.read(4))
        header = json.loads(handle.read(header_length).decode('utf-8'))
        if header.get('format') != CUE_FORMAT_VERSION:
            raise CueError(
                'cue %s has format %r, this build reads format %d'
                % (path, header.get('format'), CUE_FORMAT_VERSION)
            )
        payload = handle.read()

    if verify:
        digest = hashlib.sha256(payload).hexdigest()
        if digest != header.get('payload_sha256'):
            raise CueError(
                'cue %s is damaged: payload sha256 %s does not match header %s'
                % (path, digest, header.get('payload_sha256'))
            )

    topics = [
        CueTopic(
            index=int(entry['index']),
            name=entry['name'],
            type_name=entry['type'],
            offered_qos_profiles=entry.get('offered_qos_profiles', ''),
            count=int(entry.get('count', 0)),
            payload_bytes=int(entry.get('payload_bytes', 0)),
        )
        for entry in header['topics']
    ]
    topics.sort(key=lambda topic: topic.index)

    records: List[Tuple[int, int, bytes]] = []
    offset = 0
    size = len(payload)
    head_size = RECORD_HEADER.size
    while offset < size:
        if offset + head_size > size:
            raise CueError('cue %s ends inside a record header' % path)
        topic_index, timestamp_ns, length = RECORD_HEADER.unpack_from(payload, offset)
        offset += head_size
        if offset + length > size:
            raise CueError('cue %s ends inside a record payload' % path)
        records.append((topic_index, timestamp_ns, payload[offset:offset + length]))
        offset += length

    if len(records) != int(header.get('record_count', len(records))):
        raise CueError(
            'cue %s holds %d records, header claims %s'
            % (path, len(records), header.get('record_count'))
        )

    return CueData(topics=topics, records=records, source=header.get('source', {}))


def default_cue_path(bag_path: str | Path, topic_names: Sequence[str]) -> Path:
    """Where a cue for this bag and this topic selection is cached.

    The selection is folded into the file name so switching topic sets rebuilds
    instead of silently replaying the previous selection.  Cues live under
    ``~/.cache`` rather than next to the bag, which is often read-only.
    """
    bag_dir = Path(bag_path).expanduser()
    digest = hashlib.sha256('\n'.join(sorted(topic_names)).encode('utf-8')).hexdigest()
    cache_root = Path(
        os.environ.get('XDG_CACHE_HOME', str(Path.home() / '.cache'))
    ) / 'bag_motion_replay'
    return cache_root / ('%s.%s.orcue' % (bag_dir.name, digest[:12]))


def cue_matches_bag(cue: CueData, bag_path: str | Path) -> Tuple[bool, str]:
    """Whether a cached cue still describes the bag on disk."""
    expected = _source_fingerprint(bag_path, len(cue.records))
    recorded_files = cue.source.get('files') or []
    current_files = expected['files']
    if [f['name'] for f in recorded_files] != [f['name'] for f in current_files]:
        return (False, 'bag storage files changed')
    for recorded, current in zip(recorded_files, current_files):
        if recorded.get('size') != current.get('size'):
            return (False, 'bag file %s changed size' % current.get('name'))
    return (True, 'cue matches bag')


def load_or_build_cue(
    bag_path: str | Path,
    topic_names: Sequence[str],
    cue_path: Optional[str | Path] = None,
    rebuild: bool = False,
    progress: Optional[Callable[[int, int], None]] = None,
    log: Optional[Callable[[str], None]] = None,
) -> Tuple[CueData, Path]:
    """Return a usable cue, building and caching one when needed."""
    target = Path(cue_path).expanduser() if cue_path else default_cue_path(
        bag_path, topic_names
    )

    def announce(message: str) -> None:
        if log is not None:
            log(message)

    if not rebuild and target.is_file():
        try:
            cue = read_cue(target)
        except CueError as exc:
            announce('cue %s unusable (%s); rebuilding' % (target, exc))
        else:
            same, reason = cue_matches_bag(cue, bag_path)
            selection_matches = set(cue.topic_names()) == set(topic_names)
            if same and selection_matches:
                announce('reusing cue %s (%d records)' % (target, len(cue.records)))
                return (cue, target)
            announce(
                'cue %s stale (%s); rebuilding'
                % (target, reason if not same else 'topic selection changed')
            )

    announce('building cue from %s' % bag_path)
    cue = collect_cue(bag_path, topic_names, progress=progress)
    try:
        write_cue(cue, target)
        announce('wrote cue %s (%d records)' % (target, len(cue.records)))
    except OSError as exc:
        announce('could not cache cue at %s (%s); continuing from memory' % (target, exc))
    return (cue, target)


def summarize_cue(cue: CueData) -> str:
    """One-screen description of what a cue will publish."""
    first, last = cue.span_ns()
    lines = [
        'cue: %d records, %.3f s, %.1f kB'
        % (
            len(cue.records),
            (last - first) / 1e9,
            cue.total_payload_bytes() / 1000.0,
        )
    ]
    for topic in cue.topics:
        stamps = [ts for index, ts, _ in cue.records if index == topic.index]
        if len(stamps) >= 2:
            gaps = [stamps[i + 1] - stamps[i] for i in range(len(stamps) - 1)]
            mean_hz = 1e9 / (sum(gaps) / len(gaps))
            span = '%.3f s, %.2f Hz' % ((stamps[-1] - stamps[0]) / 1e9, mean_hz)
        else:
            span = 'single sample' if stamps else 'empty'
        lines.append(
            '  %-34s %-32s %6d msg  %s'
            % (topic.name, topic.type_name, topic.count, span)
        )
    return '\n'.join(lines)


def iter_bag_topics(bag_path: str | Path) -> Iterable[BagTopic]:
    """List what a bag holds, without reading any message."""
    return open_bag(bag_path).topics().values()


__all__ = [
    'BagError',
    'BagRecord',
    'CueData',
    'CueError',
    'CueTopic',
    'collect_cue',
    'cue_matches_bag',
    'default_cue_path',
    'iter_bag_topics',
    'load_or_build_cue',
    'read_cue',
    'summarize_cue',
    'write_cue',
]
