import pytest

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
            Mode.WAIT_GREEN: (
                InputRequirement(InputCategory.SENSOR, "lidar"),
            )
        }
    )

    decision = monitor.evaluate(
        Mode.WAIT_GREEN,
        RaceContext(),
        MissionObservation(now=1.0),
        motion_enabled=False,
    )

    # No fault does not authorize motor output. A command adapter must still
    # enforce motion_enabled=False independently.
    assert decision.must_stop is False
    assert decision.inputs_ready is False
    assert decision.missing_inputs == ("sensor:lidar",)


def test_official_race_timeout_stops_at_240_seconds_not_before():
    monitor = SafetyMonitor()
    context = RaceContext(race_started_at=10.0)

    before = monitor.evaluate(
        Mode.LANE_DRIVE,
        context,
        MissionObservation(now=249.999),
        motion_enabled=False,
    )
    at_limit = monitor.evaluate(
        Mode.LANE_DRIVE,
        context,
        MissionObservation(now=250.0),
        motion_enabled=False,
    )

    assert monitor.race_timeout_s == 240.0
    assert before.must_stop is False
    assert at_limit.must_stop is True
    assert at_limit.reason == "race timeout"


def test_official_cone_timeout_stops_at_60_seconds_not_before():
    monitor = SafetyMonitor()
    context = RaceContext(cone_entered_at=10.0)

    before = monitor.evaluate(
        Mode.CONE_DRIVE,
        context,
        MissionObservation(now=69.999),
        motion_enabled=True,
    )
    at_limit = monitor.evaluate(
        Mode.CONE_DRIVE,
        context,
        MissionObservation(now=70.0),
        motion_enabled=True,
    )
    lane_decision = monitor.evaluate(
        Mode.LANE_DRIVE,
        context,
        MissionObservation(now=70.0),
        motion_enabled=True,
    )

    assert monitor.cone_timeout_s == 60.0
    assert before.must_stop is False
    assert at_limit.must_stop is True
    assert at_limit.reason == "cone timeout"
    assert lane_decision.must_stop is False


def _cone_monitor(**kwargs):
    return SafetyMonitor(
        {"CONE_DRIVE": (InputRequirement(InputCategory.SENSOR, "scan", 0.5),)},
        **kwargs,
    )


def test_startup_grace_tolerates_inputs_that_never_arrived_yet():
    """센서 워밍업 중 미수신은 정지 사유가 아니다.

    실차 LiDAR는 첫 스캔까지 1~2초가 걸린다. 그 사이 STOP으로 떨어지면
    STOP이 종료 상태라 영원히 복구되지 않았다.
    """
    monitor = _cone_monitor(startup_grace_s=5.0)
    context = RaceContext()

    for now in (0.0, 2.0, 4.9):
        decision = monitor.evaluate(
            "CONE_DRIVE", context, MissionObservation(now=now), motion_enabled=True
        )
        assert decision.must_stop is False
        # 유예 중에도 "준비됨"으로 위장하지는 않는다
        assert decision.inputs_ready is False

    late = monitor.evaluate(
        "CONE_DRIVE", context, MissionObservation(now=5.1), motion_enabled=True
    )
    assert late.must_stop is True
    assert "missing required inputs" in late.reason


def test_startup_grace_does_not_apply_to_inputs_that_went_stale():
    """한 번 받은 뒤 끊긴 입력은 유예 없이 즉시 정지시킨다."""
    monitor = _cone_monitor(startup_grace_s=5.0)
    context = RaceContext()

    fresh = MissionObservation(now=0.0, sensor_received_at={"scan": 0.0})
    assert monitor.evaluate(
        "CONE_DRIVE", context, fresh, motion_enabled=True
    ).must_stop is False

    lost = MissionObservation(now=1.0, sensor_received_at={"scan": 0.0})
    decision = monitor.evaluate("CONE_DRIVE", context, lost, motion_enabled=True)
    assert decision.must_stop is True
    assert "stale required inputs" in decision.reason


def test_startup_grace_zero_restores_immediate_stop():
    monitor = _cone_monitor(startup_grace_s=0.0)
    decision = monitor.evaluate(
        "CONE_DRIVE", RaceContext(), MissionObservation(now=0.0), motion_enabled=True
    )
    assert decision.must_stop is True


def test_startup_grace_rejects_negative():
    with pytest.raises(ValueError):
        SafetyMonitor(startup_grace_s=-1.0)
