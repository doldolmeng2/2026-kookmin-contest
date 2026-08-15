from main.mission_observation import MissionObservation
from main.mission_types import RouteTrafficSignal
from main.race_context import RaceContext
from main.race_fsm import Mode, RaceFSM
from main.safety_monitor import SafetyDecision


SAFE = SafetyDecision(inputs_ready=True)


class SyntheticRace:
    def __init__(self):
        self.fsm = RaceFSM(initial_state=Mode.WAIT_GREEN)
        self.context = RaceContext(state_entered_at=0.0)
        self.now = 0.0
        self.trace = [Mode.WAIT_GREEN]
        self.shortcut_used_history = [False]

    def _next_time(self):
        self.now = round(self.now + 0.11, 6)
        return self.now

    def _step(self, observation):
        transition = self.fsm.step(observation, self.context, SAFE)
        if transition.changed:
            self.trace.append(transition.target)
        self.shortcut_used_history.append(self.context.shortcut_used)
        return transition

    def green(self):
        timestamp = self._next_time()
        return self._step(
            MissionObservation(
                now=timestamp,
                green_detected=True,
                traffic_message_received_at=timestamp,
            )
        )

    def route(self, signal):
        timestamp = self._next_time()
        return self._step(
            MissionObservation(
                now=timestamp,
                route_traffic_signal=signal,
                route_traffic_received_at=timestamp,
                traffic_encounter_started=True,
                traffic_encounter_received_at=timestamp,
            )
        )

    def cone(self, *, end_flag=False):
        timestamp = self._next_time()
        confidence = 0 if end_flag else 90
        return self._step(
            MissionObservation(
                now=timestamp,
                cone_detected=not end_flag,
                cone_finished=end_flag,
                cone_confidence=confidence,
                cone_end_flag=end_flag,
                cone_message_received_at=timestamp,
                scan_received_at=timestamp,
            )
        )

    def lane_change_success(self):
        timestamp = self._next_time()
        return self._step(
            MissionObservation(
                now=timestamp,
                lane_change_success=True,
                lane_change_success_edge=True,
                lane_change_received_at=timestamp,
            )
        )

    def fixed_entry(self):
        timestamp = self._next_time()
        return self._step(
            MissionObservation(
                now=timestamp,
                fixed_zone_entered=True,
                fixed_zone_entry_received_at=timestamp,
            )
        )

    def fixed_exit(self):
        timestamp = self._next_time()
        return self._step(
            MissionObservation(
                now=timestamp,
                fixed_zone_exited=True,
                fixed_zone_exit_received_at=timestamp,
            )
        )

    def overtake_entry(self):
        timestamp = self._next_time()
        return self._step(
            MissionObservation(
                now=timestamp,
                overtake_entered=True,
                overtake_entry_received_at=timestamp,
            )
        )

    def completion(self, name):
        timestamp = self._next_time()
        fields = {
            "fixed_avoid_complete": {
                "fixed_avoid_complete": True,
                "fixed_avoid_completed_at": timestamp,
            },
            "overtake_complete": {
                "overtake_complete": True,
                "overtake_complete_received_at": timestamp,
            },
            "shortcut_complete": {
                "shortcut_complete": True,
                "shortcut_complete_received_at": timestamp,
            },
        }
        return self._step(MissionObservation(now=timestamp, **fields[name]))


def start_race(race):
    first = race.green()
    second = race.green()
    started = race.green()

    assert first.changed is False
    assert second.changed is False
    assert started.target is Mode.LANE_DRIVE
    assert race.context.completed_laps == 0
    assert race.context.current_lap == 1


def run_normal_lap(race):
    trace_start = len(race.trace)

    assert race.cone().changed is False
    assert race.cone().changed is False
    assert race.cone().target is Mode.CONE_DRIVE
    assert race.cone().changed is False
    assert race.cone(end_flag=True).target is Mode.LANE_DRIVE

    assert race.fixed_entry().target is Mode.FIXED_AVOID
    assert race.fixed_exit().target is Mode.LANE_DRIVE

    assert race.overtake_entry().target is Mode.OVERTAKE
    assert race.completion("overtake_complete").target is Mode.LANE_DRIVE

    assert race.trace[trace_start:] == [
        Mode.CONE_DRIVE,
        Mode.LANE_DRIVE,
        Mode.FIXED_AVOID,
        Mode.LANE_DRIVE,
        Mode.OVERTAKE,
        Mode.LANE_DRIVE,
    ]


def run_shortcut_lap(race):
    assert race.fsm.state is Mode.SHORTCUT
    assert race.context.on_shortcut_lap is True
    assert race.completion("shortcut_complete").target is Mode.LANE_DRIVE

    # The shortcut lap remains active until the next route encounter. Cone
    # evidence during that interval must not start the normal-route missions.
    for _ in range(3):
        assert race.cone().changed is False
        assert race.fsm.state is Mode.LANE_DRIVE
    assert race.context.on_shortcut_lap is True


def assert_shortcut_was_used_once(race, shortcut_lap):
    assert race.trace.count(Mode.SHORTCUT) == 1
    assert race.context.shortcut_lap == shortcut_lap
    assert race.context.shortcut_used is True

    first_used = race.shortcut_used_history.index(True)
    assert all(race.shortcut_used_history[first_used:])


def finish_race(race, signal=RouteTrafficSignal.STRAIGHT):
    assert race.context.completed_laps == 2
    finished = race.route(signal)
    repeated = race.route(RouteTrafficSignal.LEFT)

    assert finished.target is Mode.FINISH
    assert race.context.completed_laps == 3
    assert repeated.changed is False
    assert race.fsm.state is Mode.FINISH


def test_full_three_lap_race_uses_shortcut_on_lap_two():
    race = SyntheticRace()
    start_race(race)

    run_normal_lap(race)
    lap_two = race.route(RouteTrafficSignal.LEFT)
    assert lap_two.target is Mode.SHORTCUT
    assert race.context.completed_laps == 1
    assert race.context.current_lap == 2
    assert race.context.shortcut_used is True

    run_shortcut_lap(race)
    # A later LEFT cannot select the shortcut again after it has been used.
    lap_three = race.route(RouteTrafficSignal.LEFT)
    assert lap_three.changed is False
    assert race.context.completed_laps == 2
    assert race.context.current_lap == 3

    run_normal_lap(race)
    finish_race(race)
    assert_shortcut_was_used_once(race, shortcut_lap=2)

    assert race.trace == [
        Mode.WAIT_GREEN,
        Mode.LANE_DRIVE,
        Mode.CONE_DRIVE,
        Mode.LANE_DRIVE,
        Mode.FIXED_AVOID,
        Mode.LANE_DRIVE,
        Mode.OVERTAKE,
        Mode.LANE_DRIVE,
        Mode.SHORTCUT,
        Mode.LANE_DRIVE,
        Mode.CONE_DRIVE,
        Mode.LANE_DRIVE,
        Mode.FIXED_AVOID,
        Mode.LANE_DRIVE,
        Mode.OVERTAKE,
        Mode.LANE_DRIVE,
        Mode.FINISH,
    ]


def test_full_three_lap_race_uses_shortcut_on_lap_three():
    race = SyntheticRace()
    start_race(race)

    run_normal_lap(race)
    lap_two = race.route(RouteTrafficSignal.STRAIGHT)
    assert lap_two.changed is False
    assert race.context.completed_laps == 1
    assert race.context.current_lap == 2
    assert race.context.shortcut_used is False

    run_normal_lap(race)
    lap_three = race.route(RouteTrafficSignal.LEFT)
    assert lap_three.target is Mode.SHORTCUT
    assert race.context.completed_laps == 2
    assert race.context.current_lap == 3

    run_shortcut_lap(race)
    finish_race(race, signal=RouteTrafficSignal.LEFT)
    assert_shortcut_was_used_once(race, shortcut_lap=3)

    assert race.trace == [
        Mode.WAIT_GREEN,
        Mode.LANE_DRIVE,
        Mode.CONE_DRIVE,
        Mode.LANE_DRIVE,
        Mode.FIXED_AVOID,
        Mode.LANE_DRIVE,
        Mode.OVERTAKE,
        Mode.LANE_DRIVE,
        Mode.CONE_DRIVE,
        Mode.LANE_DRIVE,
        Mode.FIXED_AVOID,
        Mode.LANE_DRIVE,
        Mode.OVERTAKE,
        Mode.LANE_DRIVE,
        Mode.SHORTCUT,
        Mode.LANE_DRIVE,
        Mode.FINISH,
    ]
