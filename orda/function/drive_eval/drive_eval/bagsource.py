"""Read the two bags an evaluation needs, and cache the expensive one.

The source recording is ~20 GB of camera frames wrapped around a few thousand
command and scan messages.  Pulling those out is a single filtered pass, but it
still reads the whole table, so the extracted reference series is cached: an
evaluation is something you re-run after every tuning change, and paying a minute
of disk each time discourages exactly the iteration this tool is for.

The run bag written by ``ros2 bag record`` is small and is read straight through.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterator, List, Optional, Sequence, Tuple
import hashlib
import os
import pickle
import sqlite3


class BagReadError(RuntimeError):
    """Raised when a bag cannot be opened or a needed topic is absent."""


@dataclass(frozen=True)
class RawRecord:
    topic: str
    timestamp_ns: int
    payload: bytes


def _storage_files(bag_dir: Path, suffix: str) -> List[Path]:
    return sorted(bag_dir.glob('*' + suffix))


def _iter_sqlite(bag_dir: Path, topics: Sequence[str]) -> Iterator[RawRecord]:
    wanted = set(topics)
    for path in _storage_files(bag_dir, '.db3'):
        connection = sqlite3.connect(
            'file:%s?mode=ro' % path.as_posix(), uri=True
        )
        try:
            id_to_name = {
                topic_id: name
                for topic_id, name in connection.execute(
                    'SELECT id, name FROM topics'
                )
                if name in wanted
            }
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
                yield RawRecord(id_to_name[topic_id], int(timestamp), bytes(data))
        finally:
            connection.close()


def _iter_rosbag2py(
    bag_dir: Path, topics: Sequence[str], storage_id: str
) -> Iterator[RawRecord]:
    try:
        from rosbag2_py import (
            ConverterOptions,
            SequentialReader,
            StorageFilter,
            StorageOptions,
        )
    except ImportError as exc:  # pragma: no cover - needs a sourced workspace
        raise BagReadError(
            'reading %s storage needs a sourced ROS 2 environment' % storage_id
        ) from exc
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_dir), storage_id=storage_id),
        ConverterOptions('', ''),
    )
    reader.set_filter(StorageFilter(topics=list(topics)))
    wanted = set(topics)
    while reader.has_next():
        topic, payload, timestamp = reader.read_next()
        if topic in wanted:
            yield RawRecord(topic, int(timestamp), bytes(payload))


def topic_types(bag_dir: Path) -> Dict[str, str]:
    """Topic name to message type, without reading any message."""
    if _storage_files(bag_dir, '.db3'):
        merged: Dict[str, str] = {}
        for path in _storage_files(bag_dir, '.db3'):
            connection = sqlite3.connect(
                'file:%s?mode=ro' % path.as_posix(), uri=True
            )
            try:
                for name, type_name in connection.execute(
                    'SELECT name, type FROM topics'
                ):
                    merged.setdefault(name, type_name)
            finally:
                connection.close()
        return merged

    if _storage_files(bag_dir, '.mcap'):
        from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

        reader = SequentialReader()
        reader.open(
            StorageOptions(uri=str(bag_dir), storage_id='mcap'),
            ConverterOptions('', ''),
        )
        return {item.name: item.type for item in reader.get_all_topics_and_types()}

    raise BagReadError('no .db3 or .mcap storage under %s' % bag_dir)


def iter_records(bag_path: str | Path, topics: Sequence[str]) -> Iterator[RawRecord]:
    """Yield the raw records of the named topics, in recorded order."""
    bag_dir = Path(bag_path).expanduser()
    if bag_dir.is_file():
        bag_dir = bag_dir.parent
    if not bag_dir.is_dir():
        raise BagReadError('bag directory does not exist: %s' % bag_dir)
    if not topics:
        return
    if _storage_files(bag_dir, '.db3'):
        yield from _iter_sqlite(bag_dir, topics)
    elif _storage_files(bag_dir, '.mcap'):
        yield from _iter_rosbag2py(bag_dir, topics, 'mcap')
    else:
        raise BagReadError('no .db3 or .mcap storage under %s' % bag_dir)


def decode_records(
    bag_path: str | Path, topics: Sequence[str]
) -> Dict[str, List[Tuple[int, object]]]:
    """Read and deserialize the named topics into ``{topic: [(ns, msg)]}``."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    bag_dir = Path(bag_path).expanduser()
    types = topic_types(bag_dir)
    missing = [topic for topic in topics if topic not in types]
    if missing:
        raise BagReadError(
            'bag %s does not contain: %s' % (bag_dir, ', '.join(sorted(missing)))
        )

    classes = {topic: get_message(types[topic]) for topic in topics}
    collected: Dict[str, List[Tuple[int, object]]] = {topic: [] for topic in topics}
    for record in iter_records(bag_dir, topics):
        collected[record.topic].append(
            (record.timestamp_ns, deserialize_message(record.payload, classes[record.topic]))
        )
    return collected


def _cache_path(bag_dir: Path, topics: Sequence[str]) -> Path:
    fingerprint = hashlib.sha256()
    fingerprint.update('\n'.join(sorted(topics)).encode('utf-8'))
    for path in sorted(bag_dir.iterdir()):
        if path.suffix in ('.db3', '.mcap'):
            stat = path.stat()
            fingerprint.update(('%s:%d' % (path.name, stat.st_size)).encode('utf-8'))
    root = Path(
        os.environ.get('XDG_CACHE_HOME', str(Path.home() / '.cache'))
    ) / 'drive_eval'
    return root / ('%s.%s.pkl' % (bag_dir.name, fingerprint.hexdigest()[:12]))


def load_reference(
    bag_path: str | Path,
    topics: Sequence[str],
    use_cache: bool = True,
    log=None,
) -> Dict[str, List[Tuple[int, object]]]:
    """Decoded reference topics, cached between evaluations of the same bag.

    The cache key covers the topic list and every storage file's size, so a
    re-recorded bag or a different selection rebuilds instead of returning
    something that no longer describes the run.
    """
    bag_dir = Path(bag_path).expanduser()
    cache = _cache_path(bag_dir, topics)

    def announce(message: str) -> None:
        if log is not None:
            log(message)

    if use_cache and cache.is_file():
        try:
            with cache.open('rb') as handle:
                payload = pickle.load(handle)
            announce('reusing cached reference series %s' % cache)
            return payload
        except Exception as exc:
            announce('cached reference unusable (%s); re-reading the bag' % exc)

    announce('reading reference topics from %s (this scans the whole bag)' % bag_dir)
    decoded = decode_records(bag_dir, topics)

    if use_cache:
        try:
            cache.parent.mkdir(parents=True, exist_ok=True)
            with cache.open('wb') as handle:
                pickle.dump(decoded, handle, protocol=4)
            announce('cached reference series at %s' % cache)
        except OSError as exc:
            announce('could not cache reference series (%s)' % exc)

    return decoded
