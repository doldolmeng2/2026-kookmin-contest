"""Hit a deadline as closely as the machine allows.

``time.sleep`` returns late by a scheduler tick or more, which at 20 Hz is a
visible fraction of the command period.  So the pacer sleeps up to a margin short
of the deadline and then busy-waits the remainder: the sleep gives the CPU back
for nearly the whole wait, and the spin removes the wake-up latency from the part
that actually decides when the message goes out.

The clock and sleep functions are injected so the waiting logic can be tested
against a fake clock instead of against real time.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional
import time

#: Spin for the last 1.5 ms by default.  Long enough to cover a normal Linux
#: wake-up overshoot, short enough that a 50 ms command period still gives the
#: CPU back for 97% of the wait.
DEFAULT_SPIN_MARGIN_NS = 1_500_000

#: Never sleep longer than this in one call, so a stop request is noticed
#: promptly even when the next deadline is far away.
DEFAULT_SLEEP_CHUNK_NS = 20_000_000


@dataclass
class PacerStats:
    """How well the pacer hit its deadlines."""

    waits: int = 0
    already_late: int = 0
    spin_ns: int = 0
    sleep_ns: int = 0


class DeadlinePacer:
    """Wait until an absolute monotonic deadline."""

    def __init__(
        self,
        spin_margin_ns: int = DEFAULT_SPIN_MARGIN_NS,
        sleep_chunk_ns: int = DEFAULT_SLEEP_CHUNK_NS,
        clock: Callable[[], int] = time.monotonic_ns,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self.spin_margin_ns = max(0, int(spin_margin_ns))
        self.sleep_chunk_ns = max(1, int(sleep_chunk_ns))
        self.clock = clock
        self.sleep = sleep
        self.stats = PacerStats()

    def wait_until(
        self,
        deadline_ns: int,
        should_stop: Optional[Callable[[], bool]] = None,
    ) -> int:
        """Block until ``deadline_ns``; return the monotonic time at release.

        Returns early only if ``should_stop`` becomes true, in which case the
        caller decides what to do — the pacer never drops a publish on its own.
        """
        self.stats.waits += 1
        now = self.clock()
        if now >= deadline_ns:
            self.stats.already_late += 1
            return now

        spin_from = deadline_ns - self.spin_margin_ns
        while True:
            now = self.clock()
            if now >= spin_from:
                break
            if should_stop is not None and should_stop():
                return now
            chunk = min(spin_from - now, self.sleep_chunk_ns)
            self.sleep(chunk / 1e9)
            self.stats.sleep_ns += chunk

        while True:
            now = self.clock()
            if now >= deadline_ns:
                self.stats.spin_ns += self.spin_margin_ns
                return now
            if should_stop is not None and should_stop():
                return now


def try_realtime_priority(priority: int = 20) -> str:
    """Ask the kernel for SCHED_FIFO, reporting what happened either way.

    Real-time scheduling is what keeps an unrelated busy process from delaying a
    publish.  It needs ``CAP_SYS_NICE`` (or a matching ``rtprio`` limit), which a
    normal user does not have, so failure is expected and never fatal — the run
    continues on the default scheduler with slightly wider jitter.
    """
    try:
        import ctypes
        import ctypes.util
    except ImportError:  # pragma: no cover - ctypes is always present on Linux
        return 'ctypes unavailable; staying on the default scheduler'

    if not hasattr(ctypes, 'CDLL'):  # pragma: no cover
        return 'ctypes unavailable; staying on the default scheduler'

    try:
        libc = ctypes.CDLL(ctypes.util.find_library('c') or 'libc.so.6', use_errno=True)
    except OSError as exc:  # pragma: no cover - non-glibc systems
        return 'libc unavailable (%s); staying on the default scheduler' % exc

    class SchedParam(ctypes.Structure):
        _fields_ = [('sched_priority', ctypes.c_int)]

    sched_fifo = 1
    param = SchedParam(int(priority))
    result = libc.sched_setscheduler(0, sched_fifo, ctypes.byref(param))
    if result == 0:
        return 'SCHED_FIFO priority %d' % priority
    import os

    errno = ctypes.get_errno()
    return 'SCHED_FIFO refused (%s); staying on the default scheduler' % os.strerror(
        errno
    )


class GcPause:
    """Keep the garbage collector out of the middle of a run.

    Every payload is already in memory before playback starts, so a collection
    during the run can only cost time.  ``freeze()`` moves the loaded cue out of
    the generational sets so a later collection has less to walk.
    """

    def __init__(self) -> None:
        self._was_enabled = False

    def __enter__(self) -> 'GcPause':
        import gc

        self._was_enabled = gc.isenabled()
        gc.collect()
        gc.freeze()
        gc.disable()
        return self

    def __exit__(self, *exc_info) -> None:
        import gc

        gc.unfreeze()
        if self._was_enabled:
            gc.enable()
