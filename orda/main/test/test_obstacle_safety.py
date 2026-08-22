import pytest

from main.control_selector import CommandCandidate, ControlSource, DriveCommand
from main.mission_types import ObjectType
from main.race_context import RaceContext
from main.race_fsm import Mode, RaceFSM
from main.runtime_adapter import RaceRuntimeAdapter, runtime_safety_monitor


def object_payload(object_type):
    return [
        1.0, 1.0, 0.0, 0.0, 1.0, 3000.0, 300.0, 180.0, 0.0, 1.0,
        float(object_type.value), 0.9,
    ]


def adapter(mode):
    return RaceRuntimeAdapter(
        fsm=RaceFSM(initial_state=mode),
        context=RaceContext(state_entered_at=0.0),
        safety_monitor=runtime_safety_monitor(),
    )


def feed(runtime, *, now, lane_at, scan_at, side_at):
    assert runtime.record_lane_offset(0, lane_at)
    assert runtime.record_scan(scan_at)
    assert runtime.record_side_clearance(1.0, 1.0, side_at)
    object_type = (
        ObjectType.FIXED
        if runtime.fsm.state is Mode.FIXED_AVOID
        else ObjectType.MOVING
    )
    assert runtime.record_object_info(object_payload(object_type), now).accepted
    return runtime.step(
        now,
        lane=CommandCandidate(DriveCommand(1.0, 5.0), now),
    )


@pytest.mark.parametrize("mode", [Mode.FIXED_AVOID, Mode.OVERTAKE])
@pytest.mark.parametrize(
    ("stale_name", "lane_at", "scan_at", "side_at"),
    [
        ("perception:lane_offset", 0.0, 2.0, 2.0),
        ("sensor:scan", 2.0, 0.0, 2.0),
        ("perception:side_clearance", 2.0, 2.0, 0.0),
    ],
)
def test_obstacle_mission_stale_input_holds_without_leaving_state(
    mode,
    stale_name,
    lane_at,
    scan_at,
    side_at,
):
    runtime = adapter(mode)
    cycle = feed(
        runtime,
        now=2.0,
        lane_at=lane_at,
        scan_at=scan_at,
        side_at=side_at,
    )
    assert runtime.fsm.state is mode
    assert cycle.transition.changed is False
    assert cycle.control.source is ControlSource.HOLD
    assert cycle.safety.stale_inputs == (stale_name,)
    assert stale_name in cycle.safety.reason


@pytest.mark.parametrize("mode", [Mode.FIXED_AVOID, Mode.OVERTAKE])
def test_obstacle_mission_all_fresh_keeps_existing_lane_control(mode):
    runtime = adapter(mode)
    cycle = feed(
        runtime,
        now=2.0,
        lane_at=2.0,
        scan_at=2.0,
        side_at=2.0,
    )
    assert runtime.fsm.state is mode
    assert cycle.safety.inputs_ready is True
    assert cycle.control.source is ControlSource.LANE
