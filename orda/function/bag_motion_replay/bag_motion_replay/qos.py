"""Rebuild the QoS a recorded publisher offered, from the bag's own metadata.

``topics.offered_qos_profiles`` is written by whichever rosbag2 recorded the bag.
Humble writes the rmw enums as integers, Jazzy writes them as names, so both
spellings have to be understood: this bag was recorded on Humble and is replayed
on Jazzy.

Parsing is pure Python (no yaml, no ROS) so it can be unit-tested anywhere;
:func:`to_qos_profile` is the only part that touches rclpy.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence

HISTORY_NAMES = {0: 'system_default', 1: 'keep_last', 2: 'keep_all', 3: 'unknown'}
RELIABILITY_NAMES = {
    0: 'system_default',
    1: 'reliable',
    2: 'best_effort',
    3: 'unknown',
}
DURABILITY_NAMES = {
    0: 'system_default',
    1: 'transient_local',
    2: 'volatile',
    3: 'unknown',
}


@dataclass(frozen=True)
class QosSpec:
    """The subset of a recorded QoS profile that changes replay behaviour."""

    history: str = 'unknown'
    depth: int = 0
    reliability: str = 'unknown'
    durability: str = 'unknown'

    def merged_with(self, other: 'QosSpec') -> 'QosSpec':
        """Combine two offered profiles into one a replay publisher can offer.

        A RELIABLE publisher satisfies BEST_EFFORT subscribers but not the other
        way round, and TRANSIENT_LOCAL satisfies VOLATILE ones, so the stricter
        side wins whenever the recorded publishers disagreed.
        """
        return QosSpec(
            history=self.history if self.history == other.history else 'unknown',
            depth=max(self.depth, other.depth),
            reliability=_stricter(
                self.reliability, other.reliability, ('reliable', 'best_effort')
            ),
            durability=_stricter(
                self.durability, other.durability, ('transient_local', 'volatile')
            ),
        )


def _stricter(left: str, right: str, order: Sequence[str]) -> str:
    for value in order:
        if left == value or right == value:
            return value
    return left if left == right else 'unknown'


def _coerce(value: str, names: dict) -> str:
    text = value.strip().strip('"\'').lower()
    if not text:
        return 'unknown'
    try:
        return names[int(text)]
    except (ValueError, KeyError):
        pass
    if text in names.values():
        return text
    # Jazzy also writes the RMW spelling, e.g. "RMW_QOS_POLICY_RELIABILITY_RELIABLE".
    for name in names.values():
        if text.endswith(name):
            return name
    return 'unknown'


def parse_offered_qos(text: Optional[str]) -> List[QosSpec]:
    """Parse ``offered_qos_profiles`` into one :class:`QosSpec` per publisher.

    The stored value is a YAML sequence of flat mappings with a couple of nested
    duration mappings.  Only top-level keys matter here, so indentation is enough
    to tell them apart and no yaml parser is needed.
    """
    if not text:
        return []

    entries: List[dict] = []
    base_indent: Optional[int] = None
    current: Optional[dict] = None

    for raw_line in text.replace('\\n', '\n').splitlines():
        line = raw_line.rstrip()
        if not line.strip():
            continue
        indent = len(line) - len(line.lstrip(' '))
        body = line.lstrip(' ')
        if body.startswith('- '):
            current = {}
            entries.append(current)
            body = body[2:]
            indent += 2
            base_indent = indent
        elif current is None:
            # A single profile written without a leading dash.
            current = {}
            entries.append(current)
            base_indent = indent
        if base_indent is None or indent != base_indent:
            continue
        if ':' not in body:
            continue
        key, _, value = body.partition(':')
        current[key.strip()] = value.strip()

    specs = []
    for entry in entries:
        depth_text = entry.get('depth', '0').strip().strip('"\'')
        try:
            depth = int(depth_text)
        except ValueError:
            depth = 0
        specs.append(
            QosSpec(
                history=_coerce(entry.get('history', ''), HISTORY_NAMES),
                depth=depth,
                reliability=_coerce(entry.get('reliability', ''), RELIABILITY_NAMES),
                durability=_coerce(entry.get('durability', ''), DURABILITY_NAMES),
            )
        )
    return specs


def merge_offered_qos(specs: Sequence[QosSpec]) -> QosSpec:
    """Fold every recorded publisher's profile into the one to re-offer."""
    if not specs:
        return QosSpec()
    merged = specs[0]
    for spec in specs[1:]:
        merged = merged.merged_with(spec)
    return merged


def to_qos_profile(spec: QosSpec, default_depth: int = 10):
    """Build an rclpy ``QoSProfile`` from a recorded spec.

    ``history``/``depth`` are deliberately *not* reproduced verbatim.  Recorders
    store ``keep_all``/``depth: 0``, and a replay publisher that keeps every sample
    can stall inside ``publish()`` while the middleware drains a slow subscriber —
    which is exactly the timing error this package exists to avoid.  A bounded
    KEEP_LAST queue keeps ``publish()`` non-blocking; reliability and durability,
    which decide whether a recorded subscriber matches at all, are reproduced.
    """
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )

    reliability = {
        'reliable': ReliabilityPolicy.RELIABLE,
        'best_effort': ReliabilityPolicy.BEST_EFFORT,
        'system_default': ReliabilityPolicy.SYSTEM_DEFAULT,
    }.get(spec.reliability, ReliabilityPolicy.RELIABLE)

    durability = {
        'transient_local': DurabilityPolicy.TRANSIENT_LOCAL,
        'volatile': DurabilityPolicy.VOLATILE,
        'system_default': DurabilityPolicy.SYSTEM_DEFAULT,
    }.get(spec.durability, DurabilityPolicy.VOLATILE)

    depth = spec.depth if spec.depth > 0 else default_depth
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=max(depth, default_depth),
        reliability=reliability,
        durability=durability,
    )


def describe(spec: QosSpec) -> str:
    return 'reliability=%s durability=%s history=%s depth=%d' % (
        spec.reliability,
        spec.durability,
        spec.history,
        spec.depth,
    )
