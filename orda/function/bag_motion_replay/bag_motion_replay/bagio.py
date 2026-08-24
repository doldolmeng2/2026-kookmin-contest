"""Read a rosbag2 without deserializing anything.

Replay fidelity starts here: the recorded CDR payload is carried through to
``publish()`` untouched, so the bytes that reach a subscriber are the bytes that
were recorded.  Deserializing and re-serializing would round-trip every float and
rewrite every message header.

The sqlite3 reader is used directly rather than through ``rosbag2_py`` because it
is available without a sourced workspace (which keeps this module unit-testable),
and because it lets a 21 GB bag be filtered down to the command topics with a
single indexed scan.  ``rosbag2_py`` is used for storage backends sqlite3 cannot
open, e.g. mcap.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterator, List, Optional, Sequence, Tuple
import sqlite3


class BagError(RuntimeError):
    """Raised when a bag cannot be opened or read."""


@dataclass(frozen=True)
class BagTopic:
    """One topic as the bag describes it."""

    name: str
    type_name: str
    offered_qos_profiles: str = ''
    count: int = 0


@dataclass(frozen=True)
class BagRecord:
    """One recorded message, still serialized."""

    topic: str
    timestamp_ns: int
    payload: bytes


def _read_metadata(bag_dir: Path) -> Optional[dict]:
    metadata_path = bag_dir / 'metadata.yaml'
    if not metadata_path.is_file():
        return None
    try:
        import yaml
    except ImportError:
        return None
    try:
        with metadata_path.open('r', encoding='utf-8') as handle:
            loaded = yaml.safe_load(handle)
    except Exception:
        return None
    if not isinstance(loaded, dict):
        return None
    return loaded.get('rosbag2_bagfile_information')


class Sqlite3BagSource:
    """Read the ``.db3`` files of a rosbag2 in recorded order."""

    def __init__(self, bag_dir: Path) -> None:
        self.bag_dir = bag_dir
        self.files = self._resolve_files(bag_dir)
        if not self.files:
            raise BagError('no .db3 storage file under %s' % bag_dir)
        self._metadata = _read_metadata(bag_dir)
        self._topics: Optional[Dict[str, BagTopic]] = None

    @staticmethod
    def _resolve_files(bag_dir: Path) -> List[Path]:
        info = _read_metadata(bag_dir)
        if info:
            listed = [
                bag_dir / rel
                for rel in info.get('relative_file_paths', [])
                if str(rel).endswith('.db3')
            ]
            existing = [path for path in listed if path.is_file()]
            if existing:
                return existing
        return sorted(bag_dir.glob('*.db3'))

    @staticmethod
    def _connect(path: Path) -> sqlite3.Connection:
        return sqlite3.connect('file:%s?mode=ro' % path.as_posix(), uri=True)

    def _counts_from_metadata(self) -> Dict[str, int]:
        counts: Dict[str, int] = {}
        if not self._metadata:
            return counts
        for entry in self._metadata.get('topics_with_message_count', []) or []:
            meta = (entry or {}).get('topic_metadata') or {}
            name = meta.get('name')
            if name:
                counts[name] = int(entry.get('message_count', 0) or 0)
        return counts

    def topics(self) -> Dict[str, BagTopic]:
        if self._topics is not None:
            return self._topics

        counts = self._counts_from_metadata()
        merged: Dict[str, BagTopic] = {}
        for path in self.files:
            connection = self._connect(path)
            try:
                rows = connection.execute(
                    'SELECT name, type, offered_qos_profiles FROM topics'
                ).fetchall()
            except sqlite3.DatabaseError as exc:
                raise BagError('cannot read topics from %s: %s' % (path, exc)) from exc
            finally:
                connection.close()
            for name, type_name, qos in rows:
                if name in merged:
                    continue
                merged[name] = BagTopic(
                    name=name,
                    type_name=type_name,
                    offered_qos_profiles=qos or '',
                    count=counts.get(name, 0),
                )

        if not counts:
            merged = self._fill_counts(merged)
        self._topics = merged
        return merged

    def _fill_counts(self, topics: Dict[str, BagTopic]) -> Dict[str, BagTopic]:
        tally: Dict[str, int] = {name: 0 for name in topics}
        for path in self.files:
            connection = self._connect(path)
            try:
                id_to_name = {
                    row[0]: row[1]
                    for row in connection.execute('SELECT id, name FROM topics')
                }
                for topic_id, count in connection.execute(
                    'SELECT topic_id, COUNT(*) FROM messages GROUP BY topic_id'
                ):
                    name = id_to_name.get(topic_id)
                    if name is not None:
                        tally[name] = tally.get(name, 0) + int(count)
            finally:
                connection.close()
        return {
            name: BagTopic(
                name=topic.name,
                type_name=topic.type_name,
                offered_qos_profiles=topic.offered_qos_profiles,
                count=tally.get(name, 0),
            )
            for name, topic in topics.items()
        }

    def iter_records(self, topic_names: Sequence[str]) -> Iterator[BagRecord]:
        wanted = set(topic_names)
        if not wanted:
            return
        for path in self.files:
            connection = self._connect(path)
            try:
                id_to_name = {}
                for topic_id, name in connection.execute(
                    'SELECT id, name FROM topics'
                ):
                    if name in wanted:
                        id_to_name[topic_id] = name
                if not id_to_name:
                    continue
                placeholders = ','.join('?' for _ in id_to_name)
                query = (
                    'SELECT topic_id, timestamp, data FROM messages '
                    'WHERE topic_id IN (%s) ORDER BY timestamp ASC, id ASC'
                    % placeholders
                )
                for topic_id, timestamp, data in connection.execute(
                    query, tuple(id_to_name)
                ):
                    yield BagRecord(
                        topic=id_to_name[topic_id],
                        timestamp_ns=int(timestamp),
                        payload=bytes(data),
                    )
            finally:
                connection.close()


class Rosbag2PyBagSource:
    """Fallback reader for storage backends sqlite3 cannot open (e.g. mcap)."""

    def __init__(self, bag_dir: Path, storage_id: str) -> None:
        self.bag_dir = bag_dir
        self.storage_id = storage_id
        self._topics: Optional[Dict[str, BagTopic]] = None

    def _reader(self, topic_names: Sequence[str] = ()):
        try:
            from rosbag2_py import (
                ConverterOptions,
                SequentialReader,
                StorageFilter,
                StorageOptions,
            )
        except ImportError as exc:  # pragma: no cover - needs a sourced workspace
            raise BagError(
                'reading %s storage needs a sourced ROS 2 environment providing '
                'rosbag2_py' % self.storage_id
            ) from exc
        reader = SequentialReader()
        reader.open(
            StorageOptions(uri=str(self.bag_dir), storage_id=self.storage_id),
            ConverterOptions('', ''),
        )
        if topic_names:
            reader.set_filter(StorageFilter(topics=list(topic_names)))
        return reader

    def topics(self) -> Dict[str, BagTopic]:
        if self._topics is not None:
            return self._topics
        reader = self._reader()
        counts = {}
        info = _read_metadata(self.bag_dir) or {}
        for entry in info.get('topics_with_message_count', []) or []:
            meta = (entry or {}).get('topic_metadata') or {}
            if meta.get('name'):
                counts[meta['name']] = int(entry.get('message_count', 0) or 0)
        merged = {}
        for item in reader.get_all_topics_and_types():
            qos = getattr(item, 'offered_qos_profiles', '') or ''
            if not isinstance(qos, str):
                qos = str(qos)
            merged[item.name] = BagTopic(
                name=item.name,
                type_name=item.type,
                offered_qos_profiles=qos,
                count=counts.get(item.name, 0),
            )
        self._topics = merged
        return merged

    def iter_records(self, topic_names: Sequence[str]) -> Iterator[BagRecord]:
        if not topic_names:
            return
        reader = self._reader(topic_names)
        wanted = set(topic_names)
        while reader.has_next():
            topic, payload, timestamp = reader.read_next()
            if topic in wanted:
                yield BagRecord(
                    topic=topic, timestamp_ns=int(timestamp), payload=bytes(payload)
                )


def open_bag(bag_path: str | Path):
    """Open a rosbag2 directory with whichever backend can read it."""
    bag_dir = Path(bag_path).expanduser()
    if bag_dir.is_file():
        bag_dir = bag_dir.parent
    if not bag_dir.is_dir():
        raise BagError('bag directory does not exist: %s' % bag_dir)

    if any(bag_dir.glob('*.db3')):
        return Sqlite3BagSource(bag_dir)
    if any(bag_dir.glob('*.mcap')):
        return Rosbag2PyBagSource(bag_dir, 'mcap')

    info = _read_metadata(bag_dir)
    if info and info.get('storage_identifier'):
        return Rosbag2PyBagSource(bag_dir, str(info['storage_identifier']))
    raise BagError('no recognised rosbag2 storage file under %s' % bag_dir)


def bag_span_ns(records: Sequence[BagRecord]) -> Tuple[int, int]:
    """First and last recorded timestamp of a record sequence."""
    if not records:
        return (0, 0)
    return (records[0].timestamp_ns, records[-1].timestamp_ns)
