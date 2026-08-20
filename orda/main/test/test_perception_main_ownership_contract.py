"""Executable RED audit for perception/Main ownership boundaries.

This file intentionally states the target architecture.  Passing tests identify
contracts already implemented; assertion failures identify production gaps.
"""

import inspect
import math
import re
from pathlib import Path
from types import SimpleNamespace

import pytest
from sensor_msgs.msg import LaserScan

from main.main import MainNode
from main.main import RELATIVE_X_ENCOUNTER_TIMEOUT_S
from main.mission_types import (
    LaneTarget,
    ObjectLane,
    ObjectType,
    RouteTrafficSignal,
)
from main.overtake import OvertakeGuard
from main.relative_x_fallback import RelativeXObstacleLaneFallback
from main.race_context import RaceContext
from main.race_fsm import Mode, RaceFSM
from main.runtime_adapter import OBJECT_MAX_AGE_S, RaceRuntimeAdapter


class _Logger:
    def info(self, _message):
        pass

    def warning(self, _message):
        pass

    def error(self, _message):
        pass


class _MainMethodHarness:
    """Minimal receiver for invoking the real MainNode callback methods."""

    def __init__(
        self,
        times=(),
        *,
        mode=Mode.LANE_DRIVE,
        state_entered_at=1.0,
        lane_target=LaneTarget.CENTER,
    ):
        self._times = iter(times)
        self.runtime = RaceRuntimeAdapter(
            fsm=RaceFSM(initial_state=mode),
            context=RaceContext(
                state_entered_at=state_entered_at,
                lane_target=lane_target,
            ),
        )
        self.warnings = []
        self._logger = _Logger()
        self._traffic_encounter_active = False
        self.fixed_vehicle_lane = 0
        self.moving_vehicle_lane = 0
        self.traffic_signal = 0

        self.obj_exists = 0.0
        self.object_dist = math.inf
        self.obj_angle = 0.0
        self.obj_span = 0.0
        self.obj_cluster = 0.0
        self.box_size = 0.0
        self.box_cx = 0.0
        self.box_cy = 0.0
        self.box_dx = 0.0
        self.car_lane = 0
        self.object_type = ObjectType.UNKNOWN
        self.object_confidence = 0.0

        self._zone_state = mode
        self._fixed_entry_sent = False
        self._zone_exit_sent = False
        self.overtake = OvertakeGuard()
        self.relative_x_fallback = RelativeXObstacleLaneFallback(
            encounter_timeout_s=RELATIVE_X_ENCOUNTER_TIMEOUT_S,
        )
        self.side_left = math.inf
        self.side_right = math.inf
        self.detected_lane = -1

    def _now_seconds(self):
        return next(self._times)

    def _warn_throttled(self, key, message, now):
        self.warnings.append((key, message, now))

    def _record_traffic_signal(self, value, received_at):
        return MainNode._record_traffic_signal(self, value, received_at)

    def get_logger(self):
        return self._logger


def _runtime(mode, lane_target):
    return RaceRuntimeAdapter(
        fsm=RaceFSM(initial_state=mode),
        context=RaceContext(state_entered_at=1.0, lane_target=lane_target),
    )


def _object_raw(
    object_type,
    *,
    lane=ObjectLane.RIGHT,
    box_size=2090.0,
    confidence=0.9,
):
    return [
        0.0,
        math.inf,
        0.0,
        0.0,
        0.0,
        box_size,
        265.0,
        199.5,
        -14.541839599609375,
        float(lane.value),
        float(object_type.value),
        confidence,
    ]


def _record_object_callback(harness, received_at, object_type, *, lane=ObjectLane.RIGHT):
    MainNode.object_info_raw_callback(
        harness,
        SimpleNamespace(data=_object_raw(object_type, lane=lane)),
    )
    assert harness.runtime.latest_object_snapshot is not None
    assert harness.runtime.latest_object_snapshot.received_at == received_at


def _drive_object_entry_cycle(harness, now):
    assert harness.runtime.record_lane_offset(0, now)
    MainNode._drive_mission_zones(harness, now)
    return harness.runtime.step(now)


def _feed_direction(runtime, object_lane, times):
    for received_at in times:
        result = runtime.record_object_info(
            _object_raw(ObjectType.FIXED, lane=object_lane),
            received_at,
        )
        assert result.accepted
        runtime.step(received_at)


def _scan(*, right_distance=None):
    message = LaserScan()
    message.angle_min = -math.pi
    message.angle_max = math.pi
    message.angle_increment = math.pi / 180.0
    message.range_min = 0.1
    message.range_max = 10.0
    message.ranges = [math.inf] * 361
    if right_distance is not None:
        right_index = round((-math.pi / 2.0 - message.angle_min) / message.angle_increment)
        message.ranges[right_index] = right_distance
    return message


def test_fixed_avoid_exit_retains_lane_two_target():
    runtime = _runtime(Mode.FIXED_AVOID, LaneTarget.LANE_TWO)
    assert runtime.record_fixed_zone_exit(1.1).accepted

    cycle = runtime.step(1.1)

    assert cycle.transition.target is Mode.LANE_DRIVE
    assert runtime.fsm.state is Mode.LANE_DRIVE
    assert runtime.context.lane_target is LaneTarget.LANE_TWO


def test_overtake_exit_retains_lane_one_target():
    runtime = _runtime(Mode.OVERTAKE, LaneTarget.LANE_ONE)
    assert runtime.record_overtake_complete(1.1).accepted

    cycle = runtime.step(1.1)

    assert cycle.transition.target is Mode.LANE_DRIVE
    assert runtime.fsm.state is Mode.LANE_DRIVE
    assert runtime.context.lane_target is LaneTarget.LANE_ONE


@pytest.mark.parametrize(
    "object_type",
    [ObjectType.FIXED, ObjectType.MOVING],
    ids=["fixed", "moving"],
)
def test_pre_session_object_raw_cannot_enter_a_mission(object_type):
    harness = _MainMethodHarness(
        [9.9],
        mode=Mode.LANE_DRIVE,
        state_entered_at=10.0,
    )
    _record_object_callback(harness, 9.9, object_type)

    _drive_object_entry_cycle(harness, 10.1)

    assert harness.runtime.fsm.state is Mode.LANE_DRIVE


@pytest.mark.parametrize(
    "object_type",
    [ObjectType.FIXED, ObjectType.MOVING],
    ids=["fixed", "moving"],
)
def test_object_raw_older_than_main_ttl_cannot_enter_a_mission(object_type):
    received_at = 10.1
    now = received_at + OBJECT_MAX_AGE_S + 0.01
    harness = _MainMethodHarness(
        [received_at],
        mode=Mode.LANE_DRIVE,
        state_entered_at=10.0,
    )
    _record_object_callback(harness, received_at, object_type)

    _drive_object_entry_cycle(harness, now)

    assert harness.runtime.fsm.state is Mode.LANE_DRIVE


def test_stale_scan_time_cannot_finish_side_clearance_timer():
    harness = _MainMethodHarness(
        [1.0, 1.1],
        mode=Mode.OVERTAKE,
        lane_target=LaneTarget.LANE_ONE,
    )
    harness.detected_lane = LaneTarget.LANE_ONE.value
    harness.overtake.enter_zone(0.5)

    MainNode.scan_callback(harness, _scan(right_distance=0.30))
    assert MainNode.is_pass_comp(harness, 1.0) is False
    MainNode.scan_callback(harness, _scan())
    assert MainNode.is_pass_comp(harness, 1.1) is False

    # No new scan arrives.  The cached clear value must not age the timer to done.
    assert harness.runtime.sensor_received_at["scan"] == 1.1
    assert MainNode.is_pass_comp(harness, 3.1) is False


def test_lane_position_cached_before_action_cannot_complete_new_action():
    harness = _MainMethodHarness(
        mode=Mode.FIXED_AVOID,
        lane_target=LaneTarget.CENTER,
    )
    MainNode.lane_position_callback(
        harness,
        SimpleNamespace(data=LaneTarget.LANE_ONE.value),
    )

    _feed_direction(
        harness.runtime,
        ObjectLane.RIGHT,
        (1.1, 1.25, 1.4),
    )

    assert harness.runtime.context.lane_target is LaneTarget.LANE_ONE
    assert harness.runtime.lane_action.pending is True
    assert harness.runtime.lane_action.completed is False


def test_one_stable_green_message_enters_lane_drive_without_main_debounce():
    harness = _MainMethodHarness(
        [1.1],
        mode=Mode.WAIT_GREEN,
        state_entered_at=1.0,
    )
    assert harness.runtime.record_lane_offset(0, 1.1)
    assert harness.runtime.record_scan(1.1)
    MainNode.object_info_callback(
        harness,
        SimpleNamespace(data=[RouteTrafficSignal.STRAIGHT.value, 0, 0]),
    )

    harness.runtime.step(1.1)

    assert harness.runtime.fsm.state is Mode.LANE_DRIVE


def test_cone_entry_ready_true_is_trusted_without_confidence_redebounce():
    runtime = _runtime(Mode.LANE_DRIVE, LaneTarget.CENTER)
    assert runtime.record_lane_offset(0, 1.1)
    assert runtime.record_scan(1.1)
    result = runtime.record_cone_message([0, 0, 10, 1], 1.1)
    assert result.accepted

    runtime.step(1.1)

    assert runtime.fsm.state is Mode.CONE_DRIVE


def test_cone_entry_ready_false_blocks_high_confidence_repetitions():
    runtime = _runtime(Mode.LANE_DRIVE, LaneTarget.CENTER)
    for received_at in (1.1, 1.2, 1.3):
        assert runtime.record_lane_offset(0, received_at)
        assert runtime.record_scan(received_at)
        result = runtime.record_cone_message([0, 0, 100, 0], received_at)
        assert result.accepted
        runtime.step(received_at)

    assert runtime.fsm.state is Mode.LANE_DRIVE


@pytest.mark.parametrize(
    ("object_lane", "expected_target"),
    [
        (ObjectLane.RIGHT, LaneTarget.LANE_ONE),
        (ObjectLane.LEFT, LaneTarget.LANE_TWO),
    ],
    ids=["right-to-lane-one", "left-to-lane-two"],
)
def test_one_stabilized_object_lane_immediately_selects_avoidance_target(
    object_lane,
    expected_target,
):
    runtime = _runtime(Mode.FIXED_AVOID, LaneTarget.CENTER)
    result = runtime.record_object_info(
        _object_raw(ObjectType.FIXED, lane=object_lane),
        1.1,
    )
    assert result.accepted

    runtime.step(1.1)

    assert runtime.context.lane_target is expected_target
    assert runtime.lane_action.target is expected_target


def test_main_scan_callback_only_records_receipt_without_raw_lidar_interpretation():
    source = inspect.getsource(MainNode.scan_callback)

    assert "self.runtime.record_scan" in source
    assert "side_clearance" not in source
    assert "msg.ranges" not in source


def test_invalid_lane_fit_does_not_republish_previous_offset_as_fresh_measurement():
    orda_root = Path(__file__).resolve().parents[2]
    source_path = (
        orda_root
        / "driving"
        / "lane_detection"
        / "src"
        / "lane_detection.cpp"
    )
    policy_path = (
        orda_root
        / "driving"
        / "lane_detection"
        / "include"
        / "lane_detection"
        / "lane_measurement_publication_policy.hpp"
    )
    source = source_path.read_text(encoding="utf-8")
    policy = policy_path.read_text(encoding="utf-8")
    publish_body = source.split(
        "LaneMeasurementPublicationPolicy publishAndDebug", 1
    )[1].split("bool bevLineToFrame", 1)[0]
    image_body = source.split("void imageCallback", 1)[1].split(
        "int classifyLaneFromRatio", 1
    )[0]

    # Invalid geometry may remain available for debug/tracking, but the
    # explicit fit-valid bit must reach the publication policy.
    assert "offset = has_prev_center_fit_ ? prev_offset_ : 0.f;" in image_body
    assert re.search(
        r"validity_msg\.data\s*=\s*valid\s*;\s*"
        r"validity_pub_->publish\(validity_msg\)\s*;",
        image_body,
        re.DOTALL,
    )
    assert re.search(
        r"publishAndDebug\(\s*frame\s*,\s*offset\s*,\s*center_fit\s*,\s*"
        r"valid\s*,\s*show_dbg",
        image_body,
        re.DOTALL,
    )

    # Each measurement publisher is guarded by its own scoped policy bit.
    offset_guard = re.search(
        r"if\s*\(publication_policy\.publish_offset\)\s*\{(?P<body>.*?)"
        r"\}\s*// /lane_fit 발행",
        publish_body,
        re.DOTALL,
    )
    assert offset_guard is not None
    assert "offset_pub_->publish(offset_msg);" in offset_guard.group("body")
    fit_guard = re.search(
        r"if\s*\(publication_policy\.publish_fit\)\s*\{(?P<body>.*?)"
        r"\}\s*// 슬라이딩 윈도우 BEV 디버그 창 표시",
        publish_body,
        re.DOTALL,
    )
    assert fit_guard is not None
    assert "fit_pub_->publish(fit_msg);" in fit_guard.group("body")
    assert re.search(
        r"if\s*\(publication_policy\.publish_lane_position\)\s*\{[^{}]*"
        r"updateAndPublishDetectedLane\(center_fit,\s*true\)\s*;",
        image_body,
        re.DOTALL,
    )

    # {} means all three outputs false for invalid fits. For valid fits,
    # offset/position stay true while only the mapped frame fit is conditional.
    assert re.search(r"if\s*\(!fit_valid\)\s*\{\s*return\s*\{\}\s*;", policy)
    assert "return {true, frame_fit_mapped, true};" in policy
