"""ROS-independent safety policy for race observations."""

from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, Iterable, Mapping, Optional, Tuple

from .mission_observation import MissionObservation
from .race_context import RaceContext


class InputCategory(str, Enum):
    SENSOR = "sensor"
    PERCEPTION = "perception"


@dataclass(frozen=True)
class InputRequirement:
    """A named state input and its optional custom freshness limit."""

    category: InputCategory
    name: str
    max_age_s: Optional[float] = None

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError("input requirement name must not be empty")
        if self.max_age_s is not None and self.max_age_s <= 0.0:
            raise ValueError("max_age_s must be positive")

    @property
    def label(self) -> str:
        return f"{self.category.value}:{self.name}"


@dataclass(frozen=True)
class SafetyDecision:
    must_stop: bool = False
    reason: Optional[str] = None
    inputs_ready: bool = True
    missing_inputs: Tuple[str, ...] = ()
    stale_inputs: Tuple[str, ...] = ()


class SafetyMonitor:
    """Evaluate state inputs and competition time limits.

    ``motion_enabled=False`` is not itself a fault. It only prevents missing or
    stale inputs from escalating to ``must_stop`` while motion is disabled.
    Conversely, ``must_stop=False`` is never permission to emit a motor command;
    a future command adapter must enforce motion authorization separately.
    """

    def __init__(
        self,
        required_inputs_by_state: Optional[
            Mapping[Any, Iterable[InputRequirement]]
        ] = None,
        *,
        stale_after_s: float = 0.5,
        race_timeout_s: float = 240.0,
        cone_timeout_s: float = 60.0,
        startup_grace_s: float = 5.0,
    ) -> None:
        # Official limits from "2026 제9회 경주 진행 방법" (2026-07-29):
        # p.37 limits the whole race to four minutes, and p.25 requires the
        # rubber-cone section to be completed within one minute.
        if stale_after_s <= 0.0:
            raise ValueError("stale_after_s must be positive")
        if race_timeout_s <= 0.0 or cone_timeout_s <= 0.0:
            raise ValueError("mission timeouts must be positive")
        if startup_grace_s < 0.0:
            raise ValueError("startup_grace_s must not be negative")

        requirements = required_inputs_by_state or {}
        self._required_inputs_by_state: Dict[
            str, Tuple[InputRequirement, ...]
        ] = {
            self._state_name(state): tuple(state_requirements)
            for state, state_requirements in requirements.items()
        }
        self.stale_after_s = stale_after_s
        self.race_timeout_s = race_timeout_s
        self.cone_timeout_s = cone_timeout_s
        self.startup_grace_s = startup_grace_s
        # 첫 evaluate() 시각. 기동 유예를 재는 기준이다.
        self._first_evaluated_at: Optional[float] = None

    @staticmethod
    def _state_name(state: Any) -> str:
        return str(getattr(state, "value", state))

    def evaluate(
        self,
        state: Any,
        context: RaceContext,
        observation: MissionObservation,
        *,
        motion_enabled: bool,
        fault_reason: Optional[str] = None,
    ) -> SafetyDecision:
        state_name = self._state_name(state)
        missing = []
        stale = []

        for requirement in self._required_inputs_by_state.get(state_name, ()):
            received_at = observation.last_received_at(
                requirement.category.value,
                requirement.name,
            )
            if received_at is None:
                missing.append(requirement.label)
                continue

            max_age_s = (
                self.stale_after_s
                if requirement.max_age_s is None
                else requirement.max_age_s
            )
            age_s = observation.now - received_at
            if age_s < 0.0 or age_s > max_age_s:
                stale.append(requirement.label)

        missing_inputs = tuple(missing)
        stale_inputs = tuple(stale)
        inputs_ready = not missing_inputs and not stale_inputs

        if fault_reason:
            return SafetyDecision(
                True,
                f"external fault: {fault_reason}",
                inputs_ready,
                missing_inputs,
                stale_inputs,
            )

        if (
            context.race_started_at is not None
            and observation.now - context.race_started_at
            >= self.race_timeout_s
        ):
            return SafetyDecision(
                True,
                "race timeout",
                inputs_ready,
                missing_inputs,
                stale_inputs,
            )

        if (
            state_name == "CONE_DRIVE"
            and context.cone_entered_at is not None
            and observation.now - context.cone_entered_at
            >= self.cone_timeout_s
        ):
            return SafetyDecision(
                True,
                "cone timeout",
                inputs_ready,
                missing_inputs,
                stale_inputs,
            )

        # 기동 유예: 아직 한 번도 도착하지 않은 입력(missing)은 센서 워밍업으로
        # 보고 정지 사유에서 제외한다. 실차의 LiDAR는 시리얼을 열고 모터가
        # 회전에 도달해야 첫 스캔이 나와 1~2초가 걸리는데, 그 사이 STOP으로
        # 떨어지면 종료 상태라 영원히 복구되지 않았다.
        #
        # 한 번 받은 뒤 끊긴 입력(stale)은 유예 없이 그대로 정지시킨다. 주행
        # 중 센서 유실은 워밍업과 성격이 다르다.
        #
        # 유예 중에도 inputs_ready는 False로 유지되므로 WAIT_GREEN은 계속
        # 대기하고, ControlSelector는 0 명령을 내보낸다. 즉 유예가 곧
        # "데이터 없이 주행"을 뜻하지는 않는다.
        blocking_missing = missing_inputs
        if self.startup_grace_s > 0.0:
            if self._first_evaluated_at is None:
                self._first_evaluated_at = observation.now
            elapsed = observation.now - self._first_evaluated_at
            if 0.0 <= elapsed < self.startup_grace_s:
                blocking_missing = ()

        if motion_enabled and (blocking_missing or stale_inputs):
            problems = []
            if blocking_missing:
                problems.append(
                    "missing required inputs: " + ", ".join(blocking_missing)
                )
            if stale_inputs:
                problems.append(
                    "stale required inputs: " + ", ".join(stale_inputs)
                )
            return SafetyDecision(
                True,
                "; ".join(problems),
                False,
                missing_inputs,
                stale_inputs,
            )

        return SafetyDecision(
            False,
            None,
            inputs_ready,
            missing_inputs,
            stale_inputs,
        )
