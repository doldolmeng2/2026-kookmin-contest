import pytest

from main.mission_types import LaneTarget, ObjectLane, ObjectType
from main.relative_x_fallback import (
    RelativeXObstacleLaneFallback,
    effective_object_lane,
    object_mission_entry_allowed,
)
from main.race_context import RaceContext
from main.race_fsm import Mode, RaceFSM
from main.runtime_adapter import RaceRuntimeAdapter


def feed_relative_x(tracker, samples, *, start=1.0, step=0.1):
    for index, relative_x in enumerate(samples):
        tracker.observe(
            object_type=ObjectType.FIXED,
            box_size=1000.0 + index,
            box_cx=320.0 + relative_x,
            box_cy=200.0 + index,
            confidence=0.80 + index * 0.01,
            received_at=start + index * step,
        )


@pytest.mark.parametrize(
    ("ego_lane", "samples", "expected_lane"),
    [
        (LaneTarget.LANE_TWO, [-66.0, -63.5, -63.5], ObjectLane.LEFT),
        (LaneTarget.LANE_ONE, [43.0, 43.0, 45.0], ObjectLane.LEFT),
        (LaneTarget.LANE_ONE, [80.0, 110.0, 139.5], ObjectLane.RIGHT),
        (LaneTarget.LANE_TWO, [31.0, 30.0, 33.0], ObjectLane.RIGHT),
    ],
)
def test_abcd_entry_proximal_samples_latch_expected_lane(
    ego_lane,
    samples,
    expected_lane,
):
    tracker = RelativeXObstacleLaneFallback(encounter_timeout_s=1.25)

    feed_relative_x(tracker, samples)

    assert tracker.independent_count == 3
    assert tracker.decided is False
    assert tracker.latch_for_entry(ego_lane) is True
    assert tracker.latched_lane is expected_lane


def test_exact_50hz_repetition_counts_as_one_independent_detection():
    tracker = RelativeXObstacleLaneFallback(encounter_timeout_s=1.25)
    kwargs = dict(
        object_type=ObjectType.FIXED,
        box_size=1200.0,
        box_cx=363.0,
        box_cy=203.5,
        confidence=0.85,
    )

    accepted = [
        tracker.observe(received_at=1.0 + index * 0.02, **kwargs)
        for index in range(20)
    ]

    assert accepted.count(True) == 1
    assert tracker.independent_count == 1
    assert tracker.decided is False


def test_latched_lane_does_not_follow_late_box_to_the_other_side():
    tracker = RelativeXObstacleLaneFallback(encounter_timeout_s=1.25)
    feed_relative_x(tracker, [-66.0, -63.5, -63.5])
    assert tracker.latch_for_entry(LaneTarget.LANE_TWO) is True

    feed_relative_x(
        tracker,
        [40.0, 100.0, 200.0],
        start=1.31,
    )

    assert tracker.latched_lane is ObjectLane.LEFT
    assert tracker.independent_count == 3


def test_measured_normal_independent_gap_keeps_candidate_history():
    tracker = RelativeXObstacleLaneFallback(encounter_timeout_s=1.90)
    feed_relative_x(tracker, [43.0], start=1.0)

    counted = tracker.observe(
        object_type=ObjectType.FIXED,
        box_size=1400.0,
        box_cx=338.5,
        box_cy=210.0,
        confidence=0.9,
        received_at=2.80,
    )

    assert counted is True
    assert tracker.independent_count == 2
    assert tracker.decided is False


def test_gap_longer_than_encounter_timeout_starts_a_new_encounter():
    tracker = RelativeXObstacleLaneFallback(encounter_timeout_s=1.90)
    feed_relative_x(tracker, [80.0, 110.0, 139.5])

    counted = tracker.observe(
        object_type=ObjectType.FIXED,
        box_size=2000.0,
        box_cx=363.0,
        box_cy=210.0,
        confidence=0.9,
        received_at=3.11,
    )

    assert counted is True
    assert tracker.independent_count == 1
    assert tracker.latched_lane is ObjectLane.UNKNOWN


def test_fixed_to_moving_type_change_starts_a_new_encounter():
    tracker = RelativeXObstacleLaneFallback(encounter_timeout_s=1.25)
    feed_relative_x(tracker, [80.0, 110.0])
    assert tracker.independent_count == 2

    counted = tracker.observe(
        object_type=ObjectType.MOVING,
        box_size=1800.0,
        box_cx=363.0,
        box_cy=210.0,
        confidence=0.9,
        received_at=1.21,
    )

    assert counted is True
    assert tracker.independent_count == 1
    assert tracker.latched_lane is ObjectLane.UNKNOWN


def test_center_ego_lane_latches_unknown_fallback():
    tracker = RelativeXObstacleLaneFallback(encounter_timeout_s=1.25)

    feed_relative_x(tracker, [80.0, 110.0, 139.5])

    assert tracker.latch_for_entry(LaneTarget.CENTER) is False
    assert tracker.decided is False
    assert tracker.latched_lane is ObjectLane.UNKNOWN


def test_far_distance_c_sequence_does_not_latch_before_entry():
    tracker = RelativeXObstacleLaneFallback(encounter_timeout_s=1.25)

    feed_relative_x(tracker, [5.5, 35.0, 80.5])

    assert tracker.decided is False
    assert tracker.latched_lane is ObjectLane.UNKNOWN


def test_c_latches_lane_two_from_entry_proximal_rolling_samples():
    tracker = RelativeXObstacleLaneFallback(encounter_timeout_s=1.25)
    feed_relative_x(tracker, [5.5, 35.0, 80.5, 96.0, 117.0, 131.0])

    assert tracker.evidence_samples == (96.0, 117.0, 131.0)
    assert tracker.latch_for_entry(LaneTarget.LANE_ONE) is True
    assert tracker.median_relative_x == 117.0
    assert tracker.latched_lane is ObjectLane.RIGHT


def test_runtime_timing_does_not_change_same_entry_proximal_decision():
    fast = RelativeXObstacleLaneFallback(encounter_timeout_s=1.25)
    slow = RelativeXObstacleLaneFallback(encounter_timeout_s=1.25)
    samples = [5.5, 35.0, 80.5, 96.0, 117.0, 131.0]
    feed_relative_x(fast, samples, step=0.10)
    feed_relative_x(slow, samples, step=0.24)

    assert fast.latch_for_entry(LaneTarget.LANE_ONE) is True
    assert slow.latch_for_entry(LaneTarget.LANE_ONE) is True
    assert fast.evidence_samples == slow.evidence_samples
    assert fast.latched_lane is slow.latched_lane is ObjectLane.RIGHT


def test_direct_perception_lane_has_priority_over_conflicting_fallback():
    assert (
        effective_object_lane(ObjectLane.LEFT, ObjectLane.RIGHT)
        is ObjectLane.LEFT
    )
    assert (
        effective_object_lane(ObjectLane.UNKNOWN, ObjectLane.RIGHT)
        is ObjectLane.RIGHT
    )


@pytest.mark.parametrize(
    ("ego_lane", "obstacle_lane", "allowed"),
    [
        (LaneTarget.LANE_ONE, ObjectLane.RIGHT, False),
        (LaneTarget.LANE_TWO, ObjectLane.LEFT, False),
        (LaneTarget.LANE_ONE, ObjectLane.LEFT, True),
        (LaneTarget.LANE_TWO, ObjectLane.RIGHT, True),
        (LaneTarget.CENTER, ObjectLane.LEFT, True),
    ],
)
def test_mission_entry_policy_only_blocks_known_adjacent_obstacles(
    ego_lane,
    obstacle_lane,
    allowed,
):
    assert object_mission_entry_allowed(ego_lane, obstacle_lane) is allowed


def _raw_object(*, lane, object_type, box_cx):
    return [
        0.0,
        float("inf"),
        0.0,
        0.0,
        0.0,
        2200.0,
        box_cx,
        210.0,
        0.0,
        float(lane.value),
        float(object_type.value),
        0.9,
    ]


def test_fallback_entry_lane_is_stored_in_evidence_and_locks_target():
    runtime = RaceRuntimeAdapter(
        fsm=RaceFSM(initial_state=Mode.LANE_DRIVE),
        context=RaceContext(
            state_entered_at=1.0,
            lane_target=LaneTarget.LANE_ONE,
        ),
    )
    assert runtime.record_object_info(
        _raw_object(
            lane=ObjectLane.UNKNOWN,
            object_type=ObjectType.FIXED,
            box_cx=363.0,
        ),
        1.1,
    ).accepted
    snapshot = runtime.latest_object_snapshot
    assert snapshot is not None

    assert runtime.record_object_mission_entry(
        Mode.FIXED_AVOID,
        snapshot,
        1.2,
        ObjectLane.LEFT,
    ).accepted
    entered = runtime.step(1.2)

    assert entered.transition.target is Mode.FIXED_AVOID
    assert runtime.context.lane_target is LaneTarget.LANE_TWO
    assert runtime.lane_action.target_locked is True

    assert runtime.record_object_info(
        _raw_object(
            lane=ObjectLane.RIGHT,
            object_type=ObjectType.FIXED,
            box_cx=500.0,
        ),
        1.3,
    ).accepted
    runtime.step(1.3)

    assert runtime.context.lane_target is LaneTarget.LANE_TWO
    assert runtime.lane_action.target is LaneTarget.LANE_TWO
    assert runtime.lane_action.target_locked is True


def test_direct_entry_lane_wins_over_conflicting_fallback_override():
    runtime = RaceRuntimeAdapter(
        fsm=RaceFSM(initial_state=Mode.LANE_DRIVE),
        context=RaceContext(state_entered_at=1.0),
    )
    assert runtime.record_object_info(
        _raw_object(
            lane=ObjectLane.RIGHT,
            object_type=ObjectType.MOVING,
            box_cx=200.0,
        ),
        1.1,
    ).accepted
    snapshot = runtime.latest_object_snapshot
    assert snapshot is not None

    assert runtime.record_object_mission_entry(
        Mode.OVERTAKE,
        snapshot,
        1.2,
        ObjectLane.LEFT,
    ).accepted
    runtime.step(1.2)

    assert runtime.context.lane_target is LaneTarget.LANE_ONE
