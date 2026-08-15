"""Pure Mode-based control arbitration with no ROS or motor side effects."""

from dataclasses import dataclass
from enum import Enum
import math
from typing import Optional

from .mission_observation import MissionObservation
from .race_context import RaceContext
from .race_fsm import Mode, RaceFSM, Transition
from .safety_monitor import SafetyDecision


_TIMESTAMP_EPSILON_S = 1e-6


class ControlSource(str, Enum):
    STOP = "STOP"
    LANE = "LANE"
    CONE = "CONE"


@dataclass(frozen=True)
class DriveCommand:
    angle: float
    speed: float


@dataclass(frozen=True)
class CommandCandidate:
    command: DriveCommand
    received_at: float


@dataclass(frozen=True)
class ControlDecision:
    source: ControlSource
    command: DriveCommand
    reason: str


@dataclass(frozen=True)
class ControlSelectorConfig:
    """Freshness policy for controller outputs in one shared clock domain.

    이 값은 실측 발행 주기에서 나온다. rosbag2_2026_08_13-09_30_09 (275초)
    에서 ``/lane_offset`` 은 평균 169 ms, p90 358 ms, p99 721 ms 주기로
    나왔다(카메라는 18.8 Hz인데 인지가 5.9 Hz). 예전 기본값 0.25초는 그
    간격의 19%를 "stale"로 분류해, 정상 주행 중에 ControlSource.STOP 이
    끊임없이 튀어나오고 속도가 0으로 리셋됐다.

    라바콘 명령(``/rubbercone_info``)은 LiDAR 전용 노드라 카메라 부하와
    무관하고 10 Hz 로 안정적이므로 0.25초를 유지한다.
    """

    max_lane_age_s: float = 0.8
    max_cone_age_s: float = 0.25

    def __post_init__(self) -> None:
        for name, value in (
            ("max_lane_age_s", self.max_lane_age_s),
            ("max_cone_age_s", self.max_cone_age_s),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value < 0.0
            ):
                raise ValueError(f"{name} must be a finite non-negative number")


@dataclass(frozen=True)
class ControlCycleResult:
    transition: Transition
    control: ControlDecision


class ControlSelector:
    """Select exactly one controller output from the already-committed Mode.

    CONE_DRIVE deliberately has no lane fallback. Rubbercones are placed on
    the lane, so using a lane command when cone control is absent or stale is
    unsafe; the only fallback is a zero STOP command.
    """

    ZERO_COMMAND = DriveCommand(angle=0.0, speed=0.0)

    def __init__(self, config: Optional[ControlSelectorConfig] = None) -> None:
        self.config = config or ControlSelectorConfig()

    def select(
        self,
        mode: Mode,
        now: float,
        *,
        lane: Optional[CommandCandidate] = None,
        cone: Optional[CommandCandidate] = None,
        mission_lane_authorized: bool = False,
        traffic_hold: bool = False,
    ) -> ControlDecision:
        """Select using receipt ages computed only in the caller's clock domain."""

        if not self._valid_number(now):
            return self._stop("invalid selection timestamp")

        if traffic_hold:
            return self._stop("recoverable route-traffic hold")

        if mode in (Mode.LANE_DRIVE, Mode.SHORTCUT):
            valid, reason = self._candidate_is_fresh(
                lane,
                now,
                self.config.max_lane_age_s,
                "lane",
            )
            if valid:
                return ControlDecision(
                    ControlSource.LANE,
                    lane.command,
                    (
                        "fresh shortcut lane command selected"
                        if mode is Mode.SHORTCUT
                        else "fresh lane command selected"
                    ),
                )
            return self._stop(reason)

        if mode is Mode.CONE_DRIVE:
            valid, reason = self._candidate_is_fresh(
                cone,
                now,
                self.config.max_cone_age_s,
                "cone",
            )
            if valid:
                return ControlDecision(
                    ControlSource.CONE,
                    cone.command,
                    "fresh cone command selected",
                )
            # Never inspect lane freshness here. There is intentionally no
            # lane fallback in CONE_DRIVE.
            return self._stop(reason)

        if mode in (Mode.FIXED_AVOID, Mode.OVERTAKE):
            if not mission_lane_authorized:
                return self._stop(f"{mode.value} lane action not authorized")
            valid, reason = self._candidate_is_fresh(
                lane,
                now,
                self.config.max_lane_age_s,
                "lane",
            )
            if valid:
                return ControlDecision(
                    ControlSource.LANE,
                    lane.command,
                    f"fresh {mode.value} lane command selected",
                )
            return self._stop(reason)

        if mode in (Mode.INIT, Mode.WAIT_GREEN, Mode.FINISH, Mode.STOP):
            return self._stop(f"{mode.value} requires stop")

        return self._stop(f"{mode.value} controller not implemented")

    def _candidate_is_fresh(
        self,
        candidate: Optional[CommandCandidate],
        now: float,
        max_age_s: float,
        label: str,
    ) -> tuple[bool, str]:
        if candidate is None:
            return False, f"{label} command missing"
        if not isinstance(candidate, CommandCandidate) or not isinstance(
            candidate.command,
            DriveCommand,
        ):
            return False, f"invalid {label} command candidate"
        if not self._valid_number(candidate.received_at):
            return False, f"invalid {label} command timestamp"
        if not (
            self._valid_number(candidate.command.angle)
            and self._valid_number(candidate.command.speed)
        ):
            return False, f"invalid {label} command value"

        age_s = now - candidate.received_at
        if age_s < -_TIMESTAMP_EPSILON_S:
            return False, f"{label} command timestamp is in the future"
        if age_s - max_age_s > _TIMESTAMP_EPSILON_S:
            return False, f"{label} command stale"
        return True, "fresh"

    @classmethod
    def _stop(cls, reason: str) -> ControlDecision:
        return ControlDecision(ControlSource.STOP, cls.ZERO_COMMAND, reason)

    @staticmethod
    def _valid_number(value: float) -> bool:
        return (
            not isinstance(value, bool)
            and isinstance(value, (int, float))
            and math.isfinite(value)
        )


def commit_fsm_then_select_control(
    fsm: RaceFSM,
    observation: MissionObservation,
    context: RaceContext,
    safety: SafetyDecision,
    selector: ControlSelector,
    *,
    lane: Optional[CommandCandidate] = None,
    cone: Optional[CommandCandidate] = None,
    mission_lane_authorized: bool = False,
    traffic_hold: bool = False,
) -> ControlCycleResult:
    """Commit the FSM first, then arbitrate using its resulting Mode."""

    transition = fsm.step(observation, context, safety)
    control = selector.select(
        fsm.state,
        observation.now,
        lane=lane,
        cone=cone,
        mission_lane_authorized=mission_lane_authorized,
        traffic_hold=traffic_hold,
    )
    return ControlCycleResult(transition=transition, control=control)
