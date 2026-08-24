import pytest

from bag_motion_replay.schedule import (
    ScheduleError,
    build_schedule,
    describe_schedule,
    infinite_loop_requested,
)

BASE = 1_786_595_603_561_816_675

# Two topics at 20 Hz, interleaved the way the bag stores them.
RECORDS = [
    (index % 2, BASE + (index // 2) * 50_000_000 + (index % 2) * 1_000, b'x')
    for index in range(20)
]


def test_delays_are_measured_from_the_first_record():
    schedule = build_schedule(RECORDS)

    assert schedule.entries[0].delay_ns == 0
    assert schedule.entries[1].delay_ns == 1_000
    assert schedule.entries[2].delay_ns == 50_000_000


def test_a_late_publish_cannot_push_the_ones_after_it():
    # Every delay is an absolute offset, so consecutive deltas stay exactly the
    # recorded deltas no matter what happens at run time.
    schedule = build_schedule(RECORDS)
    deltas = [
        schedule.entries[i].delay_ns - schedule.entries[i - 1].delay_ns
        for i in range(1, len(schedule.entries))
    ]
    recorded = [
        RECORDS[i][1] - RECORDS[i - 1][1] for i in range(1, len(RECORDS))
    ]

    assert deltas == recorded


def test_rate_scales_the_timeline_without_touching_the_order():
    schedule = build_schedule(RECORDS, rate=2.0)

    assert schedule.entries[2].delay_ns == 25_000_000
    assert [entry.record_index for entry in schedule.entries] == list(range(20))


def test_start_and_end_offsets_cut_a_window():
    schedule = build_schedule(RECORDS, start_offset_s=0.10, end_offset_s=0.20)

    offsets = {entry.bag_timestamp_ns - BASE for entry in schedule.entries}
    assert min(offsets) >= 100_000_000
    assert max(offsets) <= 200_000_000
    assert schedule.entries[0].delay_ns == 0


def test_loops_repeat_the_same_spacing_end_to_end():
    schedule = build_schedule(RECORDS, loop_count=3, loop_gap_s=1.0)

    assert len(schedule.entries) == 3 * len(RECORDS)
    pass_duration = schedule.pass_duration_ns
    second_pass_start = next(
        entry for entry in schedule.entries if entry.loop_index == 1
    )
    assert second_pass_start.delay_ns == pass_duration + 1_000_000_000


def test_every_record_in_the_window_is_scheduled_exactly_once():
    schedule = build_schedule(RECORDS)

    assert sorted(entry.record_index for entry in schedule.entries) == list(range(20))


def test_zero_or_negative_rate_is_rejected():
    for rate in (0.0, -1.0):
        with pytest.raises(ScheduleError):
            build_schedule(RECORDS, rate=rate)


def test_empty_window_is_rejected_rather_than_silently_publishing_nothing():
    with pytest.raises(ScheduleError):
        build_schedule(RECORDS, start_offset_s=5.0)


def test_reversed_window_is_rejected():
    with pytest.raises(ScheduleError):
        build_schedule(RECORDS, start_offset_s=0.2, end_offset_s=0.1)


def test_no_records_is_rejected():
    with pytest.raises(ScheduleError):
        build_schedule([])


def test_negative_loop_means_repeat_until_stopped():
    assert infinite_loop_requested(-1) is True
    assert infinite_loop_requested(1) is False


def test_description_counts_publishes_per_topic():
    text = describe_schedule(build_schedule(RECORDS), ['/a', '/b'])

    assert '/a' in text and '/b' in text
    assert '10 publishes' in text
