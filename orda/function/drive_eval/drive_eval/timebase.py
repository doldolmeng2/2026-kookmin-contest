"""Put the run bag and the source bag on the same clock.

The stack is driven by ``ros2 bag play --clock``, so its nodes reason in bag time
while ``ros2 bag record`` stamps what they publish with wall time.  Comparing a
mode transition against the moment the car passed a cone therefore needs the
mapping between the two, and the only honest source for it is the ``/clock``
stream recorded alongside the outputs: each sample is a (wall receipt, bag time)
pair produced by the same playback that fed the sensors.

Pure module — the mapping is arithmetic over integers, so it is unit-testable
without a bag or a ROS environment.
"""

from __future__ import annotations

from bisect import bisect_left
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple


class TimebaseError(ValueError):
    """Raised when a run cannot be placed on the source bag's timeline."""


@dataclass(frozen=True)
class Timebase:
    """Wall-clock to bag-clock conversion, sampled from recorded ``/clock``."""

    wall_ns: Tuple[int, ...]
    sim_ns: Tuple[int, ...]

    @classmethod
    def from_clock_samples(
        cls, samples: Sequence[Tuple[int, int]]
    ) -> 'Timebase':
        """Build from ``(wall receipt ns, published bag time ns)`` pairs.

        Playback publishes ``/clock`` monotonically, but recording can deliver two
        samples with the same wall stamp; duplicates are dropped so the lookup
        keeps a strictly increasing key.
        """
        if len(samples) < 2:
            raise TimebaseError(
                'need at least two /clock samples to place the run on the bag '
                'timeline; got %d. Was the stack played with --clock?' % len(samples)
            )
        ordered = sorted(samples)
        wall: List[int] = []
        sim: List[int] = []
        for wall_ns, sim_ns in ordered:
            if wall and wall_ns == wall[-1]:
                sim[-1] = sim_ns
                continue
            wall.append(int(wall_ns))
            sim.append(int(sim_ns))
        if len(wall) < 2:
            raise TimebaseError('all /clock samples share one wall timestamp')
        return cls(wall_ns=tuple(wall), sim_ns=tuple(sim))

    def sim_at(self, wall_ns: int) -> int:
        """Bag time for a wall timestamp, interpolating between samples.

        Outside the sampled span the nearest segment's slope is continued: the
        first outputs of a run arrive a few milliseconds before the first
        ``/clock`` sample was recorded, and dropping them would silently shorten
        the comparison.
        """
        wall = self.wall_ns
        sim = self.sim_ns
        index = bisect_left(wall, wall_ns)
        if index <= 0:
            low, high = 0, 1
        elif index >= len(wall):
            low, high = len(wall) - 2, len(wall) - 1
        else:
            low, high = index - 1, index
        span = wall[high] - wall[low]
        if span == 0:
            return sim[high]
        ratio = (wall_ns - wall[low]) / span
        return int(round(sim[low] + ratio * (sim[high] - sim[low])))

    def span_ns(self) -> Tuple[int, int]:
        return (self.sim_ns[0], self.sim_ns[-1])

    def rate(self) -> float:
        """Bag seconds per wall second over the whole run.

        A value far from 1.0 means playback could not keep up, which invalidates
        any timing conclusion drawn from the run.
        """
        wall_span = self.wall_ns[-1] - self.wall_ns[0]
        if wall_span == 0:
            return 0.0
        return (self.sim_ns[-1] - self.sim_ns[0]) / wall_span


def relative_seconds(sim_ns: int, bag_epoch_ns: int) -> float:
    """Seconds since the first message of the source bag."""
    return (sim_ns - bag_epoch_ns) / 1e9


@dataclass(frozen=True)
class Series:
    """A time series on the source bag's timeline."""

    times_s: Tuple[float, ...]
    values: Tuple[float, ...]

    def __len__(self) -> int:
        return len(self.times_s)

    def clipped(self, start_s: float, end_s: float) -> 'Series':
        pairs = [
            (t, v)
            for t, v in zip(self.times_s, self.values)
            if start_s <= t <= end_s
        ]
        if not pairs:
            return Series((), ())
        return Series(tuple(p[0] for p in pairs), tuple(p[1] for p in pairs))

    def value_at(self, time_s: float) -> Optional[float]:
        """Most recent value at or before ``time_s`` (zero-order hold)."""
        if not self.times_s:
            return None
        index = bisect_left(self.times_s, time_s)
        if index < len(self.times_s) and self.times_s[index] == time_s:
            return self.values[index]
        if index == 0:
            return None
        return self.values[index - 1]

    def span(self) -> Tuple[float, float]:
        if not self.times_s:
            return (0.0, 0.0)
        return (self.times_s[0], self.times_s[-1])
