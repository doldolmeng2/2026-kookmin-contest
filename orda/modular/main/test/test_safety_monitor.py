from main.mission_observation import MissionObservation
from main.race_context import RaceContext
from main.race_fsm import Mode
from main.safety_monitor import (
    InputCategory,
    InputRequirement,
    SafetyMonitor,
)


def test_sensor_and_perception_receipt_times_remain_separate():
    observation = MissionObservation(
        now=10.0,
        sensor_received_at={"front": 9.9},
        perception_received_at={"front": 9.7},
    )

    assert observation.last_received_at("sensor", "front") == 9.9
    assert observation.last_received_at("perception", "front") == 9.7


def test_state_specific_stale_reason_contains_input_name():
    monitor = SafetyMonitor(
        {
            Mode.LANE_DRIVE: (
                InputRequirement(InputCategory.PERCEPTION, "lane", 0.2),
            ),
            Mode.CONE_DRIVE: (
                InputRequirement(InputCategory.SENSOR, "lidar", 0.2),
            ),
        }
    )
    observation = MissionObservation(
        now=10.0,
        sensor_received_at={"lidar": 9.0},
        perception_received_at={"lane": 9.9},
    )

    lane_decision = monitor.evaluate(
        Mode.LANE_DRIVE,
        RaceContext(),
        observation,
        motion_enabled=True,
    )
    cone_decision = monitor.evaluate(
        Mode.CONE_DRIVE,
        RaceContext(),
        observation,
        motion_enabled=True,
    )

    assert lane_decision.must_stop is False
    assert lane_decision.inputs_ready is True
    assert cone_decision.must_stop is True
    assert cone_decision.stale_inputs == ("sensor:lidar",)
    assert "sensor:lidar" in cone_decision.reason


def test_motion_disabled_is_not_itself_a_fault():
    monitor = SafetyMonitor(
        {
            Mode.INIT: (
                InputRequirement(InputCategory.SENSOR, "lidar"),
            )
        }
    )

    decision = monitor.evaluate(
        Mode.INIT,
        RaceContext(),
        MissionObservation(now=1.0),
        motion_enabled=False,
    )

    # No fault does not authorize motor output. A command adapter must still
    # enforce motion_enabled=False independently.
    assert decision.must_stop is False
    assert decision.inputs_ready is False
    assert decision.missing_inputs == ("sensor:lidar",)


def test_default_race_timeout_is_240_seconds_even_if_motion_disabled():
    monitor = SafetyMonitor()
    context = RaceContext(race_started_at=10.0)

    decision = monitor.evaluate(
        Mode.LANE_DRIVE,
        context,
        MissionObservation(now=250.0),
        motion_enabled=False,
    )

    assert monitor.race_timeout_s == 240.0
    assert decision.must_stop is True
    assert decision.reason == "race timeout"


def test_default_cone_timeout_only_applies_in_cone_state():
    monitor = SafetyMonitor()
    context = RaceContext(cone_entered_at=10.0)
    observation = MissionObservation(now=70.0)

    cone_decision = monitor.evaluate(
        Mode.CONE_DRIVE,
        context,
        observation,
        motion_enabled=True,
    )
    lane_decision = monitor.evaluate(
        Mode.LANE_DRIVE,
        context,
        observation,
        motion_enabled=True,
    )

    assert monitor.cone_timeout_s == 60.0
    assert cone_decision.must_stop is True
    assert cone_decision.reason == "cone timeout"
    assert lane_decision.must_stop is False
