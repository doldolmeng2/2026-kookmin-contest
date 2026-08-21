"""Pure 2026 finals FSM with explicit, receipt-time mission edges."""

from dataclasses import dataclass
from enum import Enum
import math
from typing import Optional

from .cone_entry import ConeEntryConfig, ConeEntryDebouncer, ConeEntryDecision
from .lane_validity import (
    LaneValidityConfig,
    LaneValidityDebouncer,
    LaneValidityDecision,
)
from .mission_observation import MissionObservation
from .mission_types import RouteTrafficSignal
from .race_context import RaceContext
from .safety_monitor import SafetyDecision


_TIMESTAMP_EPSILON_S = 1e-6


class Mode(str, Enum):
    INIT = "INIT"
    WAIT_TRAFFIC = "WAIT_TRAFFIC"
    WAIT_GREEN = "WAIT_TRAFFIC"  # compatibility alias
    LANE_DRIVE = "LANE_DRIVE"
    CONE_DRIVE = "CONE_DRIVE"
    REJOIN = "REJOIN"
    FIXED_AVOID = "FIXED_AVOID"
    OVERTAKE = "OVERTAKE"
    SHORTCUT = "SHORTCUT"
    FINISH = "FINISH"
    STOP = "STOP"


@dataclass(frozen=True)
class Transition:
    source: Mode
    target: Mode
    reason: str

    @property
    def changed(self) -> bool:
        return self.source is not self.target


class _GreenDebouncer:
    def __init__(
        self,
        min_consecutive_frames: int,
        min_duration_s: float,
    ) -> None:
        if min_consecutive_frames < 1:
            raise ValueError("min_consecutive_frames must be at least one")
        if min_duration_s < 0.0:
            raise ValueError("min_duration_s must not be negative")

        self._min_consecutive_frames = min_consecutive_frames
        self._min_duration_s = min_duration_s
        self.reset()

    def reset(self) -> None:
        self._consecutive_frames = 0
        self._first_detected_at: Optional[float] = None

    @property
    def first_detected_at(self) -> Optional[float]:
        return self._first_detected_at

    def update(self, detected: bool, now: float) -> bool:
        if not detected:
            self.reset()
            return False

        if self._consecutive_frames == 0:
            self._first_detected_at = now
        self._consecutive_frames += 1

        duration_s = now - self._first_detected_at
        return (
            self._consecutive_frames >= self._min_consecutive_frames
            and duration_s >= self._min_duration_s
        )


class RaceFSM:
    """Apply the complete ten-state orchestration without ROS side effects."""

    # FINISH만 진짜 종료 상태다. STOP은 "복귀 가능한 안전 정지"다.
    #
    # STOP을 종료 상태로 두었더니, 센서가 잠깐만 끊겨도 그대로 주행이 끝났다.
    # 실제로 세 가지 상황에서 같은 증상이 나왔다:
    #   - 실차 기동 시 LiDAR 워밍업 (첫 스캔까지 1~2초)
    #   - bag --loop 재시작 사이의 /scan 공백
    #   - bag 없이 런치했을 때
    # 규정상 "정지 후 1분 내 미재개 시 실격"이라 복귀 경로가 반드시 필요하다
    # (README: 안전 정지는 복귀 가능한 감속을 우선한다).
    TERMINAL_STATES = frozenset({Mode.FINISH})

    # 입력이 회복된 뒤 이만큼 계속 정상이어야 주행을 재개한다. 입력이 깜빡일 때
    # STOP과 주행을 오가며 진동하는 것을 막는다.
    STOP_RECOVERY_HOLD_S = 0.5

    def __init__(
        self,
        initial_state: Mode = Mode.INIT,
        *,
        green_min_consecutive_frames: int = 3,
        green_min_duration_s: float = 0.0,
        green_max_age_s: float = 0.5,
        mission_event_max_age_s: float = 0.5,
        cone_entry_config: Optional[ConeEntryConfig] = None,
        lane_validity_config: Optional[LaneValidityConfig] = None,
    ) -> None:
        self.state = initial_state
        self._green = _GreenDebouncer(
            green_min_consecutive_frames,
            green_min_duration_s,
        )
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
        self._lane_validity = LaneValidityDebouncer(lane_validity_config)
        self._last_cone_drive_message_at: Optional[float] = None
        self._last_traffic_message_at: Optional[float] = None
        self._last_mission_message_at: dict[str, float] = {}
        self._cone_exit_armed = False
        # STOP 복귀용: 어느 모드에서 정지했는지, 입력이 언제부터 정상인지
        self._stop_source: Optional[Mode] = None
        self._stop_healthy_since: Optional[float] = None
        self.last_cone_entry_decision: Optional[ConeEntryDecision] = None
        self.last_lane_validity_decision: Optional[
            LaneValidityDecision
        ] = None

    @property
    def cone_entry_config(self) -> ConeEntryConfig:
        return self._cone_entry.config

    @property
    def cone_entry_guard(self) -> ConeEntryDebouncer:
        return self._cone_entry

    @property
    def cone_exit_armed(self) -> bool:
        return self._cone_exit_armed

    @property
    def lane_validity_guard(self) -> LaneValidityDebouncer:
        return self._lane_validity

    def step(
        self,
        observation: MissionObservation,
        context: RaceContext,
        safety: SafetyDecision,
    ) -> Transition:
        self.last_cone_entry_decision = None
        self.last_lane_validity_decision = None

        if self.state in self.TERMINAL_STATES:
            return self._stay("terminal state")

        if safety.must_stop:
            reason = safety.reason or "safety stop"
            context.stop_reason = reason
            self._green.reset()
            self._cone_entry.deactivate()
            self._lane_validity.deactivate()
            self._cone_exit_armed = False
            self._stop_healthy_since = None
            # 정지 직전 모드를 기억해 두었다가 회복되면 그 구간으로 돌아간다.
            if self.state is not Mode.STOP:
                self._stop_source = self.state
            return self._change(
                Mode.STOP,
                reason,
                context,
                observation.now,
            )

        if self.state is Mode.STOP:
            # 여기까지 왔다면 must_stop이 풀린 상태다. 다만 입력이 깜빡이는
            # 동안 곧바로 재개하면 진동하므로, 연속으로 정상인 시간을 요구한다.
            if not safety.inputs_ready:
                self._stop_healthy_since = None
                return self._stay("waiting for safety inputs to recover")

            if self._stop_healthy_since is None:
                self._stop_healthy_since = observation.now
            held_s = observation.now - self._stop_healthy_since
            if held_s < self.STOP_RECOVERY_HOLD_S:
                return self._stay("holding until safety inputs prove stable")

            target = self._stop_source or Mode.LANE_DRIVE
            self._stop_source = None
            self._stop_healthy_since = None
            context.stop_reason = None
            return self._change(
                target,
                "safety inputs recovered",
                context,
                observation.now,
            )

        if self.state is Mode.INIT:
            if safety.inputs_ready:
                self._green.reset()
                return self._change(
                    Mode.WAIT_GREEN,
                    "required inputs ready",
                    context,
                    observation.now,
                )
            return self._stay("waiting for required inputs")

        if self.state is Mode.WAIT_GREEN:
            if not safety.inputs_ready:
                self._green.reset()
                return self._stay("waiting for required inputs")

            if observation.traffic_message_received_at is None:
                return self._stay("waiting for new traffic message")
            if not self._accept_new_traffic_message(observation):
                self._green.reset()
                return self._stay("invalid or stale traffic message")

            if self._green.update(
                observation.green_detected,
                observation.traffic_message_received_at,
            ):
                # The 2026-07-29 "제9회 경주 진행 방법" p.17 starts the
                # official clock when the signal turns blue, not when the
                # debounce decision commits. Receipt time is the closest
                # available observation of that edge in this clock domain, so
                # retain the first fresh green in the successful sequence while
                # committing the Mode at observation.now.
                context.race_started_at = self._green.first_detected_at
                return self._change(
                    Mode.LANE_DRIVE,
                    "green signal debounced",
                    context,
                    observation.now,
                )
            return self._stay("waiting for debounced green")

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
                self._lane_validity.deactivate()
                self._cone_exit_armed = False
                return self._change(
                    Mode.LANE_DRIVE,
                    "fresh cone end flag",
                    context,
                    observation.now,
                )
            return self._stay("cone end flag received before session armed")

        if self.state is Mode.REJOIN:
            decision = self._lane_validity.evaluate(
                observation,
                context.state_entered_at,
            )
            self.last_lane_validity_decision = decision
            if decision.triggered:
                self._lane_validity.deactivate()
                return self._change(
                    Mode.LANE_DRIVE,
                    "fresh lane validity confirmed",
                    context,
                    observation.now,
                )
            return self._stay(decision.reason)

        if self.state is Mode.FIXED_AVOID:
            self._cone_entry.deactivate()
            self._lane_validity.deactivate()
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
            if (
                observation.fixed_avoid_complete is True
                and self._accept_mission_edge(
                    "fixed_avoid_complete",
                    observation.fixed_avoid_completed_at,
                    observation,
                    context,
                )
            ):
                return self._change(
                    Mode.OVERTAKE,
                    "legacy fixed completion chain",
                    context,
                    observation.now,
                )
            return self._stay("waiting for fresh fixed-zone exit")

        if self.state is Mode.OVERTAKE:
            self._cone_entry.deactivate()
            self._lane_validity.deactivate()
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
            self._lane_validity.deactivate()
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
        self._lane_validity.deactivate()

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
    ) -> bool:
        timestamp = observation.traffic_message_received_at
        if not self._valid_timestamp(timestamp):
            return False
        if (
            self._last_traffic_message_at is not None
            and timestamp <= self._last_traffic_message_at
        ):
            return False
        self._last_traffic_message_at = timestamp
        if not self._valid_timestamp(observation.now):
            return False
        age_s = observation.now - timestamp
        return (
            age_s >= -_TIMESTAMP_EPSILON_S
            and age_s - self.green_max_age_s <= _TIMESTAMP_EPSILON_S
        )

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
