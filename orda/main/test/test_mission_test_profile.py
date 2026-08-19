from pathlib import Path

import pytest

from main.control_selector import ControlSource
from main.mission_types import LaneTarget, RouteTrafficSignal
from main.race_context import RaceContext
from main.race_fsm import Mode, RaceFSM
from main.runtime_adapter import (
    MissionTestProfile,
    RaceRuntimeAdapter,
    parse_test_profile,
)


ORDA_ROOT = Path(__file__).resolve().parents[2]
BAG_TEST_LAUNCH = ORDA_ROOT / "main" / "launch" / "module_drive_bag_test.py"
PRODUCTION_LAUNCH = ORDA_ROOT / "main" / "launch" / "module_drive.py"

PROFILE_MODES = {
    "wait_green": Mode.WAIT_GREEN,
    "lane": Mode.LANE_DRIVE,
    "lane_one": Mode.LANE_DRIVE,
    "lane_two": Mode.LANE_DRIVE,
    "cone": Mode.CONE_DRIVE,
    "fixed": Mode.FIXED_AVOID,
    "overtake": Mode.OVERTAKE,
    "shortcut": Mode.SHORTCUT,
}

COMPLETION_PROFILES = [
    (
        "fixed",
        "record_fixed_zone_exit",
        Mode.FIXED_AVOID,
        Mode.LANE_DRIVE,
    ),
    (
        "overtake",
        "record_overtake_complete",
        Mode.OVERTAKE,
        Mode.LANE_DRIVE,
    ),
    (
        "shortcut",
        "record_shortcut_complete",
        Mode.SHORTCUT,
        Mode.LANE_DRIVE,
    ),
]


def test_race_profile_is_a_noop_and_preserves_normal_startup_contract():
    fsm = RaceFSM(initial_state=Mode.WAIT_GREEN)
    context = RaceContext()
    runtime = RaceRuntimeAdapter(fsm=fsm, context=context)

    runtime.bootstrap_test_profile("race", started_at=10.0)

    assert runtime.fsm is fsm
    assert runtime.context is context
    assert runtime.fsm.state is Mode.WAIT_GREEN
    assert runtime.context.state_entered_at is None
    assert runtime.step(10.0).transition.changed is False


def test_old_wait_traffic_profile_name_is_only_an_input_alias():
    assert parse_test_profile("wait_traffic") is MissionTestProfile.WAIT_GREEN


@pytest.mark.parametrize("profile", list(PROFILE_MODES))
def test_each_named_profile_starts_in_exact_existing_mode(profile):
    runtime = RaceRuntimeAdapter()

    runtime.bootstrap_test_profile(profile, started_at=10.0)

    assert runtime.fsm.state is PROFILE_MODES[profile]
    assert runtime.context.state_entered_at == 10.0
    expected_lane = {
        "lane_one": LaneTarget.LANE_ONE,
        "lane_two": LaneTarget.LANE_TWO,
    }.get(profile, LaneTarget.CENTER)
    assert runtime.context.lane_target is expected_lane


def test_shortcut_profile_bootstraps_self_consistent_lap_context():
    runtime = RaceRuntimeAdapter()

    runtime.bootstrap_test_profile("shortcut", started_at=10.0)

    assert runtime.context.completed_laps == 1
    assert runtime.context.current_lap == 2
    assert runtime.context.shortcut_lap == 2
    assert runtime.context.shortcut_used is True
    assert runtime.context.on_shortcut_lap is True
    assert runtime.record_shortcut_complete(10.1).accepted is True
    assert runtime.step(10.1).transition.target is Mode.LANE_DRIVE


@pytest.mark.parametrize(
    ("profile", "record_method", "source", "target"),
    COMPLETION_PROFILES,
)
def test_pre_bootstrap_completion_is_discarded_and_fresh_edge_is_required(
    profile,
    record_method,
    source,
    target,
):
    runtime = RaceRuntimeAdapter()
    assert getattr(runtime, record_method)(9.9).accepted is True

    runtime.bootstrap_test_profile(profile, started_at=10.0)
    empty = runtime.step(10.1)

    assert empty.transition.changed is False
    assert runtime.fsm.state is source

    # A completion at the exact state-entry timestamp is not fresh enough.
    assert getattr(runtime, record_method)(10.0).accepted is True
    pre_entry = runtime.step(10.1)
    assert pre_entry.transition.changed is False
    assert runtime.fsm.state is source

    assert getattr(runtime, record_method)(10.2).accepted is True
    fresh = runtime.step(10.2)
    assert fresh.transition.target is target


def test_bootstrap_discards_cached_sensor_perception_and_action_state():
    runtime = RaceRuntimeAdapter()
    runtime.record_scan(9.0)
    runtime.record_lane_offset(4, 9.0)
    runtime.record_cone_message([5, 0, 90], 9.0)
    runtime.record_traffic(True, 9.0)
    runtime.record_route_traffic(
        RouteTrafficSignal.LEFT,
        9.0,
        encounter_started=True,
    )
    runtime.record_lane_change_state([1, 1], 9.0)
    runtime.record_fixed_zone_exit(9.0)

    runtime.bootstrap_test_profile("lane", started_at=10.0)
    cycle = runtime.step(10.1)

    assert cycle.observation.sensor_received_at == {}
    assert cycle.observation.perception_received_at == {}
    assert cycle.observation.cone_message_received_at is None
    assert cycle.observation.traffic_message_received_at is None
    assert cycle.observation.route_traffic_received_at is None
    assert cycle.observation.lane_change_received_at is None
    assert cycle.observation.fixed_zone_exit_received_at is None
    assert runtime.latest_lane_offset is None
    assert runtime.latest_cone_event is None
    assert runtime.latest_object_snapshot is None
    assert runtime.lane_action.pending is False
    assert runtime.traffic_stop_override is False


def test_cone_profile_starts_a_new_exit_handshake_session():
    runtime = RaceRuntimeAdapter()
    runtime.record_cone_message([0, 1, 0], 9.9)

    runtime.bootstrap_test_profile("cone", started_at=10.0)
    assert runtime.context.cone_entered_at == 10.0
    assert runtime.step(10.1).transition.changed is False

    runtime.record_cone_message([0, 1, 0], 10.2)
    unarmed_end = runtime.step(10.2)
    runtime.record_cone_message([0, 0, 90], 10.3)
    armed = runtime.step(10.3)
    runtime.record_cone_message([0, 1, 0], 10.4)
    lane = runtime.step(10.4)

    assert unarmed_end.transition.changed is False
    assert armed.transition.reason == "cone exit session armed"
    assert lane.transition.target is Mode.LANE_DRIVE


def test_profile_start_does_not_weaken_safety_stop_priority():
    runtime = RaceRuntimeAdapter()
    runtime.bootstrap_test_profile("fixed", started_at=10.0)
    runtime.record_fixed_zone_exit(10.1)

    cycle = runtime.step(10.1, fault_reason="synthetic profile fault")

    assert cycle.transition.target is Mode.FIXED_AVOID
    assert cycle.control.source is ControlSource.HOLD
    assert runtime.context.stop_reason == "external fault: synthetic profile fault"


@pytest.mark.parametrize(
    "value",
    ["", "foobar", "FIXED_AVOID", -1, 9, True, None],
)
def test_invalid_or_magic_integer_profile_is_rejected(value):
    with pytest.raises(ValueError):
        parse_test_profile(value)


def test_bag_launch_exposes_profile_without_weakening_motor_isolation():
    bag_source = BAG_TEST_LAUNCH.read_text(encoding="utf-8")
    production_source = PRODUCTION_LAUNCH.read_text(encoding="utf-8")

    assert "DeclareLaunchArgument(\n        'test_profile'" in bag_source
    assert "default_value='2'" in bag_source
    assert "test_profile = LaunchConfiguration('test_profile')" in bag_source
    assert "from launch_ros.parameter_descriptions import ParameterValue" in bag_source
    assert "'test_profile': ParameterValue(test_profile, value_type=str)" in bag_source
    assert "test_profile_arg," in bag_source
    assert "('xycar_motor', '/kmu_main_offline/xycar_motor')" in bag_source
    assert "'live_drive', default_value='false'" in bag_source
    assert "'udp_motor_bridge', default_value='false'" in bag_source
    assert "test_profile" not in production_source


def test_profile_parser_accepts_numeric_contract_and_named_compatibility():
    assert parse_test_profile("fixed") is MissionTestProfile.FIXED
    assert parse_test_profile(" OVERTAKE ") is MissionTestProfile.OVERTAKE
    assert parse_test_profile(3) is MissionTestProfile.LANE_ONE
    assert parse_test_profile("8") is MissionTestProfile.SHORTCUT
