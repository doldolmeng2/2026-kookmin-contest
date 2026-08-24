from bag_motion_replay.pacing import DeadlinePacer


class FakeClock:
    """A monotonic clock that only moves when someone sleeps or spins."""

    def __init__(self, start_ns=0, spin_step_ns=10_000):
        self.now_ns = start_ns
        self.spin_step_ns = spin_step_ns
        self.sleeps = []
        self.reads = 0

    def read(self):
        self.reads += 1
        # A busy-wait makes progress even without sleeping.
        value = self.now_ns
        self.now_ns += self.spin_step_ns
        return value

    def sleep(self, seconds):
        self.sleeps.append(seconds)
        self.now_ns += int(seconds * 1e9)


def _pacer(clock, **kwargs):
    return DeadlinePacer(clock=clock.read, sleep=clock.sleep, **kwargs)


def test_waiting_returns_no_earlier_than_the_deadline():
    clock = FakeClock()
    pacer = _pacer(clock, spin_margin_ns=1_000_000)

    released = pacer.wait_until(50_000_000)

    assert released >= 50_000_000


def test_most_of_the_wait_is_slept_not_spun():
    clock = FakeClock()
    pacer = _pacer(clock, spin_margin_ns=1_000_000, sleep_chunk_ns=20_000_000)

    pacer.wait_until(50_000_000)

    slept_ns = sum(seconds * 1e9 for seconds in clock.sleeps)
    assert slept_ns >= 45_000_000


def test_long_waits_are_chopped_so_a_stop_request_is_noticed():
    clock = FakeClock()
    pacer = _pacer(clock, sleep_chunk_ns=20_000_000)

    pacer.wait_until(200_000_000)

    assert len(clock.sleeps) > 1
    assert max(clock.sleeps) <= 0.020 + 1e-9


def test_a_deadline_already_past_returns_at_once_and_is_counted():
    clock = FakeClock(start_ns=100_000_000)
    pacer = _pacer(clock)

    released = pacer.wait_until(50_000_000)

    assert released == 100_000_000
    assert pacer.stats.already_late == 1
    assert clock.sleeps == []


def test_stop_request_breaks_out_of_the_sleep_phase():
    clock = FakeClock()
    pacer = _pacer(clock, sleep_chunk_ns=1_000_000)

    released = pacer.wait_until(10_000_000_000, should_stop=lambda: True)

    assert released < 10_000_000_000


def test_stop_request_breaks_out_of_the_spin_phase():
    clock = FakeClock(spin_step_ns=1)
    pacer = _pacer(clock, spin_margin_ns=1_000_000_000, sleep_chunk_ns=1_000_000)

    released = pacer.wait_until(1_000_000_000, should_stop=lambda: True)

    assert released < 1_000_000_000


def test_stats_count_every_wait():
    clock = FakeClock()
    pacer = _pacer(clock)

    for deadline in (10_000_000, 20_000_000, 30_000_000):
        pacer.wait_until(deadline)

    assert pacer.stats.waits == 3
