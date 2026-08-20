"""Pure 2026 finals FSM with explicit, receipt-time mission edges."""

from dataclasses import dataclass
from enum import Enum
import math
from typing import Optional

from .cone_entry import ConeEntryConfig, ConeEntryDebouncer, ConeEntryDecision
from .mission_observation import MissionObservation
from .mission_types import RouteTrafficSignal
from .race_context import RaceContext
from .safety_monitor import SafetyDecision


_TIMESTAMP_EPSILON_S = 1e-6


class Mode(str, Enum):
    WAIT_GREEN = "WAIT_GREEN"
    WAIT_TRAFFIC = "WAIT_GREEN"  # compatibility alias for old bags/configs
    LANE_DRIVE = "LANE_DRIVE"
    CONE_DRIVE = "CONE_DRIVE"
    FIXED_AVOID = "FIXED_AVOID"
    OVERTAKE = "OVERTAKE"
    SHORTCUT = "SHORTCUT"
    FINISH = "FINISH"


@dataclass(frozen=True)
class Transition:
    source: Mode
    target: Mode
    reason: str

    @property
    def changed(self) -> bool:
        return self.source is not self.target


class RaceFSM:
    """Apply the race orchestration without ROS side effects."""

    # FINISH만 진짜 종료 상태다. 안전 입력 부족은 FSM 상태가 아니라
    # control arbitration의 HOLD로 처리한다.
    TERMINAL_STATES = frozenset({Mode.FINISH})

    def __init__(
        self,
        initial_state: Mode = Mode.WAIT_GREEN,
        *,
        green_max_age_s: float = 0.5,
        mission_event_max_age_s: float = 0.5,
        cone_entry_config: Optional[ConeEntryConfig] = None,
    ) -> None:
        self.state = initial_state
        if (
            isinstance(green_max_age_s, bool)
            or not isinstance(green_max_age_s, (int, float))
            or not math.isfinite(green_max_age_s)
            or green_max_age_s < 0.0
        ):
            raise ValueError("green_max_age_s must be finite and non-negative")
        self.green_max_age_s = green_max_age_s
        if (
            isinstance(mission_event_max_age_s, bool)
            or not isinstance(mission_event_max_age_s, (int, float))
            or not math.isfinite(mission_event_max_age_s)
            or mission_event_max_age_s < 0.0
        ):
            raise ValueError(
                "mission_event_max_age_s must be finite and non-negative"
            )
        self.mission_event_max_age_s = mission_event_max_age_s
        self._cone_entry = ConeEntryDebouncer(cone_entry_config)
        self._last_cone_drive_message_at: Optional[float] = None
        self._last_live_cone_entry_message_at: Optional[float] = None
        self._last_traffic_message_at: Optional[float] = None
        self._last_mission_message_at: dict[str, float] = {}
        self._cone_exit_armed = False
        self.last_cone_entry_decision: Optional[ConeEntryDecision] = None

    @property
    def cone_entry_config(self) -> ConeEntryConfig:
        return self._cone_entry.config

    @property
    def cone_entry_guard(self) -> ConeEntryDebouncer:
        return self._cone_entry

    @property
    def cone_exit_armed(self) -> bool:
        return self._cone_exit_armed

    def step(
        self,
        observation: MissionObservation,
        context: RaceContext,
        safety: SafetyDecision,
    ) -> Transition:
        self.last_cone_entry_decision = None

        if self.state in self.TERMINAL_STATES:
            return self._stay("terminal state")

        if safety.must_stop:
            reason = safety.reason or "safety hold"
            context.stop_reason = reason
            self._cone_entry.deactivate()
            self._cone_exit_armed = False
            return self._stay(reason)

        context.stop_reason = None

        if self.state is Mode.WAIT_GREEN:
            if not safety.inputs_ready:
                return self._stay("waiting for required inputs")

            if observation.traffic_message_received_at is None:
                return self._stay("waiting for new traffic message")
            if not self._accept_new_traffic_message(observation, context):
                return self._stay("invalid or stale traffic message")

            if observation.green_detected:
                # traffic_node has already classified and debounced this
                # semantic signal. Main owns only receipt/session freshness.
                context.race_started_at = observation.traffic_message_received_at
                return self._change(
                    Mode.LANE_DRIVE,
                    "stable green signal",
                    context,
                    observation.now,
                )
            return self._stay("waiting for stable green")

        if self.state is Mode.LANE_DRIVE:
            route_transition = self._handle_traffic_encounter(
                observation,
                context,
            )
            if route_transition is not None:
                return route_transition

            if (
                not context.on_shortcut_lap
                and observation.overtake_entered is True
                and self._accept_mission_edge(
                    "overtake_entry",
                    observation.overtake_entry_received_at,
                    observation,
                    context,
                )
            ):
                self._cone_entry.deactivate()
                return self._change(
                    Mode.OVERTAKE,
                    "fresh moving-obstacle entry",
                    context,
                    observation.now,
                )

            # Explicit course-zone evidence outranks opportunistic cone
            # perception. Object detection is deliberately not an entry guard.
            if (
                not context.on_shortcut_lap
                and observation.fixed_zone_entered is True
                and self._accept_mission_edge(
                    "fixed_zone_entry",
                    observation.fixed_zone_entry_received_at,
                    observation,
                    context,
                )
            ):
                self._cone_entry.deactivate()
                return self._change(
                    Mode.FIXED_AVOID,
                    "fresh fixed-zone entry",
                    context,
                    observation.now,
                )

            if context.on_shortcut_lap:
                self._cone_entry.deactivate()
                return self._stay("normal-route missions suppressed on shortcut lap")

            if observation.cone_entry_ready is not None:
                self._cone_entry.deactivate()
                if not self._accept_live_cone_entry_message(observation, context):
                    return self._stay("invalid or stale live cone message")
                if observation.cone_entry_ready:
                    context.cone_entered_at = observation.now
                    self._cone_exit_armed = False
                    self._last_cone_drive_message_at = (
                        observation.cone_message_received_at
                    )
                    return self._change(
                        Mode.CONE_DRIVE,
                        "perception-approved cone entry",
                        context,
                        observation.now,
                    )
                return self._stay("cone entry not approved by perception")

            # Explicit legacy replay boundary: only three-field recordings use
            # the former Main-side confidence/count/duration guard.
            decision = self._cone_entry.evaluate(observation)
            self.last_cone_entry_decision = decision
            if decision.triggered:
                context.cone_entered_at = observation.now
                self._cone_exit_armed = False
                self._last_cone_drive_message_at = (
                    observation.cone_message_received_at
                )
                return self._change(
                    Mode.CONE_DRIVE,
                    "cone entry confirmed",
                    context,
                    observation.now,
                )
            return self._stay(decision.reason)

        if self.state is Mode.CONE_DRIVE:
            if not self._accept_new_cone_drive_message(observation, context):
                return self._stay("waiting for fresh cone session evidence")

            if observation.cone_end_flag is False:
                self._cone_exit_armed = True
                return self._stay("cone exit session armed")

            if observation.cone_end_flag is True and self._cone_exit_armed:
                # The entry latch belongs to one cone episode. Rearm it when
                # that episode commits its exit so a later normal lap can
                # qualify independently.
                self._cone_entry.deactivate()
                self._cone_exit_armed = False
                return self._change(
                    Mode.LANE_DRIVE,
                    "fresh cone end flag",
                    context,
                    observation.now,
                )
            return self._stay("cone end flag received before session armed")

        if self.state is Mode.FIXED_AVOID:
            self._cone_entry.deactivate()
            if (
                observation.fixed_zone_exited is True
                and self._accept_mission_edge(
                    "fixed_zone_exit",
                    observation.fixed_zone_exit_received_at,
                    observation,
                    context,
                )
            ):
                # 고정장애물 구간이 끝나면 곧바로 차선 주행으로 돌아간다.
                # OVERTAKE 는 "움직이는 방해차량" 상황용이라 이 경로에서는
                # 거치지 않는다. 진입 계약이 생기면 그때 연결한다.
                return self._change(
                    Mode.LANE_DRIVE,
                    "fresh fixed-zone exit",
                    context,
                    observation.now,
                )
            return self._stay("waiting for fresh fixed-zone exit")

        if self.state is Mode.OVERTAKE:
            self._cone_entry.deactivate()
            if (
                observation.overtake_complete is True
                and self._accept_mission_edge(
                    "overtake_complete",
                    observation.overtake_complete_received_at,
                    observation,
                    context,
                )
            ):
                return self._change(
                    Mode.LANE_DRIVE,
                    "fresh overtake complete",
                    context,
                    observation.now,
                )
            return self._stay("waiting for fresh overtake complete")

        if self.state is Mode.SHORTCUT:
            self._cone_entry.deactivate()
            if (
                observation.shortcut_complete is True
                and self._accept_mission_edge(
                    "shortcut_complete",
                    observation.shortcut_complete_received_at,
                    observation,
                    context,
                )
            ):
                return self._change(
                    Mode.LANE_DRIVE,
                    "fresh shortcut complete",
                    context,
                    observation.now,
                )
            return self._stay("waiting for fresh shortcut complete")

        # Cone-entry evidence must never accumulate outside LANE_DRIVE. Keep a
        # completed episode rearmed for a future LANE_DRIVE visit.
        self._cone_entry.deactivate()

        return self._stay("transition not implemented in this phase")

    def _handle_traffic_encounter(
        self,
        observation: MissionObservation,
        context: RaceContext,
    ) -> Optional[Transition]:
        """Commit one typed route encounter and its deterministic branch."""

        if observation.traffic_encounter_started is not True:
            return None
        if observation.route_traffic_signal not in (
            RouteTrafficSignal.STRAIGHT,
            RouteTrafficSignal.LEFT,
        ):
            return None
        timestamp = observation.traffic_encounter_received_at
        if not self._accept_mission_edge(
            "traffic_encounter",
            timestamp,
            observation,
            context,
        ):
            return None
        if timestamp is None:
            return None
        if (
            context.last_traffic_encounter_at is not None
            and timestamp <= context.last_traffic_encounter_at
        ):
            return None

        context.last_traffic_encounter_at = timestamp
        context.completed_laps = min(
            context.completed_laps + 1,
            context.TOTAL_LAPS,
        )
        if context.completed_laps >= context.TOTAL_LAPS:
            self._cone_entry.deactivate()
            return self._change(
                Mode.FINISH,
                "three traffic encounters completed",
                context,
                observation.now,
            )

        if (
            observation.route_traffic_signal is RouteTrafficSignal.LEFT
            and context.current_lap in (2, 3)
            and context.shortcut_lap is None
        ):
            context.shortcut_lap = context.current_lap
            self._cone_entry.deactivate()
            return self._change(
                Mode.SHORTCUT,
                "left route selected for shortcut lap",
                context,
                observation.now,
            )
        return None

    def _accept_mission_edge(
        self,
        key: str,
        timestamp: Optional[float],
        observation: MissionObservation,
        context: RaceContext,
    ) -> bool:
        """Consume each explicit edge once and qualify only its state session."""

        if not self._valid_timestamp(timestamp):
            return False
        previous = self._last_mission_message_at.get(key)
        if previous is not None and timestamp <= previous:
            return False
        # A unique edge is consumed even if it predates this state or is stale,
        # preventing a sticky value from becoming valid on a later step.
        self._last_mission_message_at[key] = timestamp

        if not self._valid_timestamp(observation.now):
            return False
        if not self._valid_timestamp(context.state_entered_at):
            return False
        age_s = observation.now - timestamp
        return (
            timestamp > context.state_entered_at
            and age_s >= -_TIMESTAMP_EPSILON_S
            and age_s - self.mission_event_max_age_s <= _TIMESTAMP_EPSILON_S
        )

    def _accept_new_traffic_message(
        self,
        observation: MissionObservation,
        context: RaceContext,
    ) -> bool:
        timestamp = observation.traffic_message_received_at
        if not self._valid_timestamp(timestamp):
            return False
        if (
            self._last_traffic_message_at is not None
            and timestamp <= self._last_traffic_message_at
        ):
            return False
        if not self._valid_timestamp(observation.now):
            return False
        if not self._valid_timestamp(context.state_entered_at):
            return False
        age_s = observation.now - timestamp
        accepted = (
            timestamp > context.state_entered_at
            and age_s >= -_TIMESTAMP_EPSILON_S
            and age_s - self.green_max_age_s <= _TIMESTAMP_EPSILON_S
        )
        if accepted:
            self._last_traffic_message_at = timestamp
        return accepted

    def _accept_live_cone_entry_message(
        self,
        observation: MissionObservation,
        context: RaceContext,
    ) -> bool:
        timestamp = observation.cone_message_received_at
        if not self._valid_timestamp(timestamp):
            return False
        if (
            self._last_live_cone_entry_message_at is not None
            and timestamp <= self._last_live_cone_entry_message_at
        ):
            return False
        if not self._valid_timestamp(observation.now):
            return False
        if not self._valid_timestamp(context.state_entered_at):
            return False
        age_s = observation.now - timestamp
        accepted = (
            timestamp > context.state_entered_at
            and age_s >= -_TIMESTAMP_EPSILON_S
            and age_s - self.cone_entry_config.max_cone_age_s
            <= _TIMESTAMP_EPSILON_S
        )
        if accepted:
            self._last_live_cone_entry_message_at = timestamp
        return accepted

    def _accept_new_cone_drive_message(
        self,
        observation: MissionObservation,
        context: RaceContext,
    ) -> bool:
        """Accept one fresh cone receipt edge, regardless of its flag value."""

        timestamp = observation.cone_message_received_at
        if not self._valid_timestamp(timestamp):
            return False

        if (
            self._last_cone_drive_message_at is not None
            and timestamp <= self._last_cone_drive_message_at
        ):
            return False

        # Consume each new receipt edge once even when it is not end evidence.
        self._last_cone_drive_message_at = timestamp

        now = observation.now
        if not self._valid_timestamp(now):
            return False
        state_entered_at = context.state_entered_at
        if not self._valid_timestamp(state_entered_at):
            return False
        age_s = now - timestamp
        return (
            timestamp > state_entered_at
            and age_s >= -_TIMESTAMP_EPSILON_S
            and age_s - self.cone_entry_config.max_cone_age_s
            <= _TIMESTAMP_EPSILON_S
        )

    @staticmethod
    def _valid_timestamp(value: Optional[float]) -> bool:
        return (
            value is not None
            and not isinstance(value, bool)
            and isinstance(value, (int, float))
            and math.isfinite(value)
        )

    def _stay(self, reason: str) -> Transition:
        # A self-transition must not modify context.state_entered_at.
        return Transition(self.state, self.state, reason)

    def _change(
        self,
        target: Mode,
        reason: str,
        context: RaceContext,
        now: float,
    ) -> Transition:
        source = self.state
        if target is source:
            return self._stay(reason)

        self.state = target
        context.state_entered_at = now
        return Transition(source, target, reason)
