"""ROS-independent runtime adapter for the authoritative race FSM."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum
import math
from typing import Any, Callable, Deque, Optional, Sequence

from .control_selector import (
    CommandCandidate,
    ControlDecision,
    ControlCycleResult,
    ControlSelector,
)
from .mission_observation import MissionObservation
from .mission_types import (
    LaneChangeStateEvent,
    LaneTarget,
    ObjectLane,
    ObjectSnapshot,
    ObjectType,
    RouteTrafficEvent,
    RouteTrafficSignal,
    opposite_lane_target,
)
from .race_context import RaceContext
from .race_fsm import Mode, RaceFSM, Transition
from .safety_monitor import (
    InputCategory,
    InputRequirement,
    SafetyDecision,
    SafetyMonitor,
)


DEFAULT_CONE_EVENT_QUEUE_CAPACITY = 16

# ── 입력 신선도 예산 ────────────────────────────────────────────────────────
#
# 이 값들은 "정상 동작 중 이만큼 늦어질 수 있다"는 실측 기반 예산이지, 목표
# 주기가 아니다. rosbag2_2026_08_13-09_30_09 (275초) 실측 발행 주기:
#
#   토픽            평균     p90     p99     최대
#   /image_raw      53 ms    73 ms   91 ms   264 ms
#   /lane_offset   169 ms   358 ms  721 ms  1243 ms
#   /object_info_raw 248 ms 280 ms  306 ms   348 ms
#
# 예전 값(0.25 / 0.5초)은 이 실측보다 빠듯해서, 정상 주행 중에도 lane_offset이
# 19%(0.25초 기준) / 4%(0.5초 기준) 확률로 "끊김"으로 분류됐다. 그때마다
# LANE_DRIVE에서 입력 부족을 상태 전이로 처리하면 속도가 0으로 리셋돼 차가 앞으로 못 나갔다
# (기록된 모터 출력: 속도 0인 사이클 24.2%, 평균 속도 2.24).
#
# 근본 해결은 인지 파이프라인을 빠르게 하는 것이다(디버그 창 비활성화 등 별도
# 수정). 예산은 그 위에 두는 안전망이므로, 실측 p99 + 지터를 덮되 무한정
# 늘리지는 않는다.
LANE_OFFSET_MAX_AGE_S = 1.0
LANE_POSITION_MAX_AGE_S = LANE_OFFSET_MAX_AGE_S
SCAN_MAX_AGE_S = 0.5
SIDE_CLEARANCE_MAX_AGE_S = SCAN_MAX_AGE_S
TRAFFIC_MAX_AGE_S = 0.5
OBJECT_MAX_AGE_S = 0.6

_MOTION_MODES = frozenset(
    {
        Mode.LANE_DRIVE,
        Mode.CONE_DRIVE,
        Mode.FIXED_AVOID,
        Mode.OVERTAKE,
        Mode.SHORTCUT,
    }
)


class MissionTestProfile(str, Enum):
    """Named bag-test entry points backed by existing production Modes."""

    RACE = "race"
    WAIT_GREEN = "wait_green"
    LANE = "lane"
    LANE_ONE = "lane_one"
    LANE_TWO = "lane_two"
    CONE = "cone"
    FIXED = "fixed"
    OVERTAKE = "overtake"
    SHORTCUT = "shortcut"


_TEST_PROFILE_MODES = {
    MissionTestProfile.WAIT_GREEN: Mode.WAIT_GREEN,
    MissionTestProfile.LANE: Mode.LANE_DRIVE,
    MissionTestProfile.LANE_ONE: Mode.LANE_DRIVE,
    MissionTestProfile.LANE_TWO: Mode.LANE_DRIVE,
    MissionTestProfile.CONE: Mode.CONE_DRIVE,
    MissionTestProfile.FIXED: Mode.FIXED_AVOID,
    MissionTestProfile.OVERTAKE: Mode.OVERTAKE,
    MissionTestProfile.SHORTCUT: Mode.SHORTCUT,
}

_NUMERIC_TEST_PROFILES = {
    0: MissionTestProfile.RACE,
    1: MissionTestProfile.WAIT_GREEN,
    2: MissionTestProfile.LANE,
    3: MissionTestProfile.LANE_ONE,
    4: MissionTestProfile.LANE_TWO,
    5: MissionTestProfile.CONE,
    6: MissionTestProfile.FIXED,
    7: MissionTestProfile.OVERTAKE,
    8: MissionTestProfile.SHORTCUT,
}


def parse_test_profile(value: Any) -> MissionTestProfile:
    """Parse the numeric mission contract, retaining named compatibility."""

    if isinstance(value, MissionTestProfile):
        return value
    if isinstance(value, int) and not isinstance(value, bool):
        try:
            return _NUMERIC_TEST_PROFILES[value]
        except KeyError as exc:
            raise ValueError(f"unknown numeric mission test profile: {value}") from exc
    if not isinstance(value, str):
        raise ValueError(f"invalid mission test profile: {value!r}")
    stripped = value.strip().lower()
    if stripped.isdigit():
        return parse_test_profile(int(stripped))
    if stripped == "wait_traffic":
        stripped = MissionTestProfile.WAIT_GREEN.value
    try:
        return MissionTestProfile(stripped)
    except ValueError as exc:
        raise ValueError(f"unknown mission test profile: {value}") from exc


@dataclass(frozen=True)
class ConeMessageEvent:
    """One validated ``/rubbercone_info`` callback receipt edge."""

    offset: int
    end_flag: bool
    confidence: int
    entry_ready: Optional[bool]
    received_at: float


@dataclass(frozen=True)
class TrafficMessageEvent:
    detected: bool
    received_at: float


@dataclass(frozen=True)
class ConeEnqueueResult:
    accepted: bool
    warning: Optional[str] = None
    dropped_oldest: bool = False


@dataclass(frozen=True)
class InputRecordResult:
    accepted: bool
    warning: Optional[str] = None


@dataclass(frozen=True)
class MissionEdgeEvent:
    received_at: float


@dataclass(frozen=True)
class ObjectEntryEvidence:
    """Stable lane carried by the exact snapshot that requested a mission."""

    expected_state: Mode
    lane: ObjectLane
    object_received_at: float
    mission_edge_received_at: float


@dataclass
class LaneActionStatus:
    """Action-level state; it deliberately does not add an FSM mode."""

    mode: Optional[Mode] = None
    safe_to_drive: bool = False
    pending: bool = False
    completed: bool = False
    target: Optional[LaneTarget] = None
    target_locked: bool = False
    started_at: Optional[float] = None
    completed_at: Optional[float] = None


@dataclass(frozen=True)
class RuntimeCycleResult:
    observation: MissionObservation
    safety: SafetyDecision
    fsm_control: ControlCycleResult
    cone_session_active_command: Optional[bool]
    discarded_pre_phase_events: int = 0

    @property
    def transition(self) -> Transition:
        return self.fsm_control.transition

    @property
    def control(self) -> ControlDecision:
        return self.fsm_control.control


def dispatch_cone_session_state(
    cycle: RuntimeCycleResult,
    publish_state: Callable[[bool], None],
) -> bool:
    """Dispatch one state command for a committed cone-session boundary."""

    if cycle.cone_session_active_command is None:
        return False
    publish_state(cycle.cone_session_active_command)
    return True


def runtime_safety_monitor() -> SafetyMonitor:
    """Build state-specific safety requirements from existing ROS inputs."""

    return SafetyMonitor(
        {
            # WAIT_GREEN is also the startup readiness gate. The vehicle stays
            # stopped until the official object contract, lane controller and
            # LiDAR have each produced a fresh message.
            Mode.WAIT_GREEN: (
                InputRequirement(
                    InputCategory.PERCEPTION,
                    "object_info",
                    TRAFFIC_MAX_AGE_S,
                ),
                InputRequirement(
                    InputCategory.PERCEPTION,
                    "lane_offset",
                    LANE_OFFSET_MAX_AGE_S,
                ),
                InputRequirement(InputCategory.SENSOR, "scan", SCAN_MAX_AGE_S),
            ),
            Mode.LANE_DRIVE: (
                InputRequirement(
                    InputCategory.PERCEPTION,
                    "lane_offset",
                    LANE_OFFSET_MAX_AGE_S,
                ),
            ),
            Mode.SHORTCUT: (
                InputRequirement(
                    InputCategory.PERCEPTION,
                    "lane_offset",
                    LANE_OFFSET_MAX_AGE_S,
                ),
            ),
            # A missing detector reset must leave CONE_DRIVE waiting for a
            # fresh zero, not force an immediate terminal state. Scan loss is
            # still a terminal sensor failure; cone command freshness is
            # enforced independently by ControlSelector.
            Mode.CONE_DRIVE: (
                InputRequirement(InputCategory.SENSOR, "scan", 0.5),
            ),
            Mode.FIXED_AVOID: (
                InputRequirement(
                    InputCategory.PERCEPTION,
                    "lane_offset",
                    LANE_OFFSET_MAX_AGE_S,
                ),
                InputRequirement(InputCategory.SENSOR, "scan", SCAN_MAX_AGE_S),
                InputRequirement(
                    InputCategory.PERCEPTION,
                    "side_clearance",
                    SIDE_CLEARANCE_MAX_AGE_S,
                ),
            ),
            Mode.OVERTAKE: (
                InputRequirement(
                    InputCategory.PERCEPTION,
                    "lane_offset",
                    LANE_OFFSET_MAX_AGE_S,
                ),
                InputRequirement(InputCategory.SENSOR, "scan", SCAN_MAX_AGE_S),
                InputRequirement(
                    InputCategory.PERCEPTION,
                    "side_clearance",
                    SIDE_CLEARANCE_MAX_AGE_S,
                ),
            ),
        },
    )


class RaceRuntimeAdapter:
    """Own the sole runtime ``RaceFSM`` and consume callback edges once."""

    def __init__(
        self,
        *,
        fsm: Optional[RaceFSM] = None,
        context: Optional[RaceContext] = None,
        safety_monitor: Optional[SafetyMonitor] = None,
        selector: Optional[ControlSelector] = None,
        cone_queue_capacity: int = DEFAULT_CONE_EVENT_QUEUE_CAPACITY,
        object_max_age_s: float = OBJECT_MAX_AGE_S,
        lane_change_max_age_s: float = 0.25,
        lane_position_max_age_s: float = LANE_POSITION_MAX_AGE_S,
    ) -> None:
        if (
            isinstance(cone_queue_capacity, bool)
            or not isinstance(cone_queue_capacity, int)
            or cone_queue_capacity < 1
        ):
            raise ValueError("cone_queue_capacity must be a positive integer")
        for name, value in (
            ("object_max_age_s", object_max_age_s),
            ("lane_change_max_age_s", lane_change_max_age_s),
            ("lane_position_max_age_s", lane_position_max_age_s),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value < 0.0
            ):
                raise ValueError(f"{name} must be finite and non-negative")

        self.fsm = fsm if fsm is not None else RaceFSM()
        self.context = context if context is not None else RaceContext()
        self.safety_monitor = (
            safety_monitor if safety_monitor is not None else SafetyMonitor()
        )
        self.selector = selector if selector is not None else ControlSelector()
        self.cone_queue_capacity = cone_queue_capacity
        self.object_max_age_s = float(object_max_age_s)
        self.lane_change_max_age_s = float(lane_change_max_age_s)
        self.lane_position_max_age_s = float(lane_position_max_age_s)

        self._cone_events: Deque[ConeMessageEvent] = deque()
        self._traffic_events: Deque[TrafficMessageEvent] = deque()
        self._lane_change_events: Deque[LaneChangeStateEvent] = deque()
        self._route_traffic_events: Deque[RouteTrafficEvent] = deque()
        self._approved_route_encounters: Deque[RouteTrafficEvent] = deque()
        self._mission_events: dict[str, Deque[MissionEdgeEvent]] = {
            "fixed_zone_entry": deque(),
            "fixed_zone_exit": deque(),
            "overtake_entry": deque(),
            "overtake_complete": deque(),
            "shortcut_complete": deque(),
        }
        self._last_internal_event_at: dict[str, float] = {}
        self.cone_queue_overflow_count = 0

        self.sensor_received_at: dict[str, float] = {}
        self.perception_received_at: dict[str, float] = {}
        self.green_detected = False
        self.latest_lane_offset: Optional[int] = None
        self.latest_lane_received_at: Optional[float] = None
        self.latest_lane_path_preview: Optional[
            tuple[float, float, float, float, float]
        ] = None
        self.lane_path_preview_received_at: Optional[float] = None
        self.latest_lane_guardrail: Optional[tuple[float, float]] = None
        self.lane_guardrail_received_at: Optional[float] = None
        self.latest_cone_event: Optional[ConeMessageEvent] = None
        self.latest_object_snapshot: Optional[ObjectSnapshot] = None
        self._pending_object_entry_evidence: Optional[ObjectEntryEvidence] = None
        self.latest_side_left: float = float("inf")
        self.latest_side_right: float = float("inf")
        self.side_clearance_received_at: Optional[float] = None
        self._last_lane_change_received_at: Optional[float] = None
        self._lane_change_success_active = False
        self.traffic_stop_override = False
        self.lane_action = LaneActionStatus()
        # /lane_position 실측 차선 (-1=미확정). main_node 가 콜백에서 갱신한다.
        self.measured_lane: int = -1
        self.measured_lane_received_at: Optional[float] = None

    def bootstrap_test_profile(
        self,
        profile: MissionTestProfile | str,
        started_at: float,
    ) -> None:
        """Start one bag-test session without changing RaceFSM transitions.

        ``race`` is deliberately a no-op so the production initialization
        path remains identical. Other profiles replace the FSM and context as
        one fresh session and discard every pending or cached input edge.
        """

        typed_profile = parse_test_profile(profile)
        if typed_profile is MissionTestProfile.RACE:
            return
        if not self._valid_timestamp(started_at):
            raise ValueError("test profile start timestamp must be finite")

        initial_mode = _TEST_PROFILE_MODES[typed_profile]
        self.fsm = RaceFSM(
            initial_state=initial_mode,
            mission_event_max_age_s=self.fsm.mission_event_max_age_s,
            cone_entry_config=self.fsm.cone_entry_config,
        )
        context_kwargs = {}
        if typed_profile is MissionTestProfile.CONE:
            context_kwargs["cone_entered_at"] = float(started_at)
        elif typed_profile is MissionTestProfile.SHORTCUT:
            context_kwargs.update(completed_laps=1, shortcut_lap=2)
        if typed_profile is MissionTestProfile.LANE_ONE:
            context_kwargs["lane_target"] = LaneTarget.LANE_ONE
        elif typed_profile is MissionTestProfile.LANE_TWO:
            context_kwargs["lane_target"] = LaneTarget.LANE_TWO
        self.context = RaceContext(
            state_entered_at=float(started_at),
            **context_kwargs,
        )

        self._cone_events.clear()
        self._traffic_events.clear()
        self._lane_change_events.clear()
        self._route_traffic_events.clear()
        self._approved_route_encounters.clear()
        for queue in self._mission_events.values():
            queue.clear()
        self._last_internal_event_at.clear()
        self.cone_queue_overflow_count = 0

        self.sensor_received_at.clear()
        self.perception_received_at.clear()
        self.green_detected = False
        self.latest_lane_offset = None
        self.latest_lane_received_at = None
        self.latest_lane_path_preview = None
        self.lane_path_preview_received_at = None
        self.latest_lane_guardrail = None
        self.lane_guardrail_received_at = None
        self.latest_cone_event = None
        self.latest_object_snapshot = None
        self._pending_object_entry_evidence = None
        self.latest_side_left = float("inf")
        self.latest_side_right = float("inf")
        self.side_clearance_received_at = None
        self._last_lane_change_received_at = None
        self._lane_change_success_active = False
        self.traffic_stop_override = False
        self.lane_action = LaneActionStatus()
        self.measured_lane = -1
        self.measured_lane_received_at = None

    @property
    def pending_cone_event_count(self) -> int:
        return len(self._cone_events)

    def record_scan(self, received_at: float) -> bool:
        if not self._valid_timestamp(received_at):
            return False
        self.sensor_received_at["scan"] = received_at
        return True

    def record_side_clearance(
        self,
        left: float,
        right: float,
        received_at: float,
    ) -> bool:
        """Record one validated semantic side-distance observation."""

        if (
            not self._valid_clearance(left)
            or not self._valid_clearance(right)
            or not self._valid_timestamp(received_at)
        ):
            return False
        if (
            self.side_clearance_received_at is not None
            and received_at <= self.side_clearance_received_at
        ):
            return False
        self.latest_side_left = float(left)
        self.latest_side_right = float(right)
        self.side_clearance_received_at = received_at
        self.perception_received_at["side_clearance"] = received_at
        return True

    def record_lane_offset(self, offset: int, received_at: float) -> bool:
        if not self._valid_int(offset) or not self._valid_timestamp(received_at):
            return False
        self.latest_lane_offset = offset
        self.latest_lane_received_at = received_at
        self.perception_received_at["lane_offset"] = received_at
        return True

    def record_lane_guardrail(
        self,
        left: float,
        right: float,
        received_at: float,
    ) -> bool:
        """Record one /lane_guardrail measurement (BEV px, negative = unseen).

        lane_node publishes this in the same callback as /lane_offset, so the
        two always describe the same frame. Both margins unseen is a valid
        reading -- it means no outer solid line was visible -- and the
        controller decays its term to zero on that.
        """

        if not self._valid_timestamp(received_at):
            return False
        if not self._valid_number(left) or not self._valid_number(right):
            return False
        if (
            self.lane_guardrail_received_at is not None
            and received_at <= self.lane_guardrail_received_at
        ):
            return False
        self.latest_lane_guardrail = (float(left), float(right))
        self.lane_guardrail_received_at = received_at
        self.perception_received_at["lane_guardrail"] = received_at
        return True

    def record_lane_path_preview(
        self,
        target_offset: float,
        curvature: float,
        confidence: float,
        target_y_ratio: float,
        source_lane_offset: float,
        received_at: float,
    ) -> bool:
        """Record one validated far-path preview paired with ``lane_offset``."""

        values = (
            target_offset,
            curvature,
            confidence,
            target_y_ratio,
            source_lane_offset,
        )
        if not self._valid_timestamp(received_at):
            return False
        if any(not self._valid_number(value) for value in values):
            return False
        if not 0.0 <= float(confidence) <= 1.0:
            return False
        if not 0.0 <= float(target_y_ratio) <= 1.0:
            return False
        if (
            self.lane_path_preview_received_at is not None
            and received_at <= self.lane_path_preview_received_at
        ):
            return False
        self.latest_lane_path_preview = tuple(float(value) for value in values)
        self.lane_path_preview_received_at = received_at
        self.perception_received_at["lane_path_preview"] = received_at
        return True

    def record_lane_position(self, value: int, received_at: float) -> bool:
        """Record one validated lane measurement without accepting old receipts."""

        if (
            not self._valid_int(value)
            or value not in (-1, 0, 1, 2)
            or not self._valid_timestamp(received_at)
        ):
            return False
        if (
            self.measured_lane_received_at is not None
            and received_at <= self.measured_lane_received_at
        ):
            return False
        self.measured_lane = value
        self.measured_lane_received_at = received_at
        self.perception_received_at["lane_position"] = received_at
        return True

    def record_traffic(self, detected: bool, received_at: float) -> bool:
        if not isinstance(detected, bool) or not self._valid_timestamp(received_at):
            return False
        if len(self._traffic_events) == self.cone_queue_capacity:
            self._traffic_events.popleft()
        self._traffic_events.append(
            TrafficMessageEvent(detected=detected, received_at=received_at),
        )
        self.green_detected = detected
        self.perception_received_at["object_info"] = received_at
        return True

    def record_object_info(
        self,
        data: Sequence[Any],
        received_at: float,
    ) -> InputRecordResult:
        """Validate the internal object_info_raw payload and retain its subset.

        앞 10필드는 기존 계약이다. 새 publisher는 object_type과 confidence를
        11·12번째에 붙인다. 잠깐 사용했던 side_left/side_right 2필드 layout과
        옛 10필드 bag도 계속 읽을 수 있도록 길이는 10 이상으로 받는다.
        """

        try:
            values = list(data)
        except TypeError:
            values = []
        if len(values) < 10:
            return InputRecordResult(False, "object_info_raw requires at least 10 fields")
        if not self._valid_timestamp(received_at):
            return InputRecordResult(
                False,
                "invalid object_info_raw receipt timestamp",
            )

        # 앞 10필드는 기존 계약이다. 11번째 object_type과 12번째 confidence는
        # 새 YOLO 분류 계약이며, 예전 10필드 bag도 계속 읽을 수 있게 선택값이다.
        object_type = ObjectType.UNKNOWN
        object_confidence = 0.0
        if len(values) >= 11:
            raw_type = values[10]
            looks_like_type = not (
                isinstance(raw_type, bool)
                or not isinstance(raw_type, (int, float))
                or not math.isfinite(float(raw_type))
                or not float(raw_type).is_integer()
            )
            if looks_like_type:
                try:
                    object_type = ObjectType(int(raw_type))
                except ValueError:
                    return InputRecordResult(
                        False,
                        "object_info_raw type must be -1, 0, or 1",
                    )
            elif len(values) < 12:
                return InputRecordResult(
                    False,
                    "object_info_raw type must be an integer",
                )
            # Two old optional side-LiDAR fields were briefly appended to the
            # 10-field payload. A non-integral field 11 identifies that layout;
            # accept and ignore both so recorded bags remain readable.
        if len(values) >= 12 and looks_like_type:
            raw_confidence = values[11]
            if (
                isinstance(raw_confidence, bool)
                or not isinstance(raw_confidence, (int, float))
                or not math.isfinite(float(raw_confidence))
                or not 0.0 <= float(raw_confidence) <= 1.0
            ):
                return InputRecordResult(
                    False,
                    "object_info_raw confidence must be within 0..1",
                )
            object_confidence = float(raw_confidence)

        # 검증은 계약된 앞 10필드에만 적용한다. 뒤에 붙는 선택 필드는 위에서
        # 별도 검증했다.
        values = values[:10]
        if any(
            not isinstance(value, (int, float)) or isinstance(value, bool)
            for value in values
        ):
            return InputRecordResult(
                False,
                "object_info_raw fields must be numbers",
            )

        numeric_values = [float(value) for value in values]
        if any(math.isnan(value) for value in numeric_values):
            return InputRecordResult(
                False,
                "object_info_raw fields must not contain NaN",
            )

        exists_value = numeric_values[0]
        if exists_value not in (0.0, 1.0):
            return InputRecordResult(
                False,
                "object_info_raw exists must be 0 or 1",
            )
        lane_value = numeric_values[9]
        if not math.isfinite(lane_value) or not lane_value.is_integer():
            return InputRecordResult(
                False,
                "object_info_raw lane label must be an integer",
            )
        try:
            lane = ObjectLane(int(lane_value))
        except ValueError:
            return InputRecordResult(
                False,
                "object_info_raw lane label must be 0, 1, or 2",
            )

        # YOLO 박스 면적. 음수는 검출기가 낼 수 없는 값이므로 거른다.
        box_px = numeric_values[5]
        if not math.isfinite(box_px) or box_px < 0.0:
            return InputRecordResult(
                False,
                "object_info_raw box size must be a non-negative number",
            )

        distance = numeric_values[1]
        if exists_value == 1.0:
            if any(not math.isfinite(value) for value in numeric_values):
                return InputRecordResult(
                    False,
                    "detected object_info_raw fields must be finite numbers",
                )
            if distance < 0.0:
                return InputRecordResult(
                    False,
                    "object_info_raw distance must be non-negative",
                )
            snapshot_distance: Optional[float] = distance
            snapshot_lane = lane
        else:
            # The existing detector intentionally emits +inf for min_dist when
            # no LiDAR cluster exists.  Keep the heartbeat/receipt, but do not
            # expose stale bbox/lane or the sentinel as actionable evidence.
            if distance < 0.0 or distance == -math.inf:
                return InputRecordResult(
                    False,
                    "absent object_info_raw distance must be non-negative or +inf",
                )
            if any(not math.isfinite(value) for value in numeric_values[2:9]):
                return InputRecordResult(
                    False,
                    "absent object_info_raw metadata must be finite numbers",
                )
            snapshot_distance = None
            # 차선 라벨은 카메라 산출물이므로 LiDAR 미검출과 무관하게 유지한다.
            # 예전에는 여기서 UNKNOWN 으로 지웠는데, 그러면 LiDAR 사거리 밖에
            # 있는 방해차량은 YOLO가 또렷이 보고 있어도 회피 방향을 정할 수
            # 없었다. 박스가 없을 때 검출기가 lane_label 0(=UNKNOWN)을 내므로
            # 그대로 두어도 없는 정보를 지어내지 않는다.
            snapshot_lane = lane

        self.latest_object_snapshot = ObjectSnapshot(
            exists=bool(exists_value),
            distance=snapshot_distance,
            box_px=box_px,
            lane=snapshot_lane,
            received_at=received_at,
            object_type=object_type,
            confidence=object_confidence,
        )
        self.perception_received_at["object_info_raw"] = received_at
        return InputRecordResult(True)

    def record_lane_change_state(
        self,
        data: Sequence[Any],
        received_at: float,
    ) -> InputRecordResult:
        """Queue one validated feedback sample and deduplicate its success pulse."""

        try:
            values = list(data)
        except TypeError:
            values = []
        if len(values) != 2:
            return InputRecordResult(
                False,
                "lane_change_state requires [changing, success]",
            )
        if not self._valid_timestamp(received_at):
            return InputRecordResult(
                False,
                "invalid lane_change_state receipt timestamp",
            )
        if (
            self._last_lane_change_received_at is not None
            and received_at <= self._last_lane_change_received_at
        ):
            return InputRecordResult(
                False,
                "duplicate or regressed lane_change_state receipt",
            )
        if any(not self._valid_binary_int(value) for value in values):
            return InputRecordResult(
                False,
                "lane_change_state fields must be 0 or 1",
            )

        changing = bool(values[0])
        success = bool(values[1])
        success_edge = success and not self._lane_change_success_active
        self._lane_change_success_active = success
        self._last_lane_change_received_at = received_at
        if len(self._lane_change_events) == self.cone_queue_capacity:
            self._lane_change_events.popleft()
        self._lane_change_events.append(
            LaneChangeStateEvent(
                changing=changing,
                success=success,
                success_edge=success_edge,
                received_at=received_at,
            )
        )
        self.perception_received_at["lane_change_state"] = received_at
        return InputRecordResult(True)

    def lane_change_in_progress(self, now: float) -> bool:
        """True while the newest fresh /lane_change_state says a change is on.

        The guardrail term must stay out of a deliberate lane change: crossing
        a line is the point of the manoeuvre, so a repulsion term would fight
        it. A stale event does not count -- if the topic went quiet we no
        longer know a change is underway.
        """

        if not self._lane_change_events:
            return False
        newest = self._lane_change_events[-1]
        if not newest.changing:
            return False
        age = now - newest.received_at
        return 0.0 <= age <= self.lane_change_max_age_s

    def record_route_traffic(
        self,
        signal: RouteTrafficSignal,
        received_at: float,
        *,
        encounter_started: bool = False,
    ) -> InputRecordResult:
        """Test/integration seam for a route source that production lacks."""

        if isinstance(signal, bool):
            return InputRecordResult(False, "invalid route traffic signal")
        try:
            typed_signal = RouteTrafficSignal(signal)
        except (TypeError, ValueError):
            return InputRecordResult(False, "invalid route traffic signal")
        if not isinstance(encounter_started, bool):
            return InputRecordResult(False, "encounter_started must be boolean")
        if not self._valid_timestamp(received_at):
            return InputRecordResult(False, "invalid route traffic timestamp")
        if len(self._route_traffic_events) == self.cone_queue_capacity:
            self._route_traffic_events.popleft()
        self._route_traffic_events.append(
            RouteTrafficEvent(typed_signal, False, received_at)
        )
        if encounter_started:
            if len(self._approved_route_encounters) == self.cone_queue_capacity:
                self._approved_route_encounters.popleft()
            self._approved_route_encounters.append(
                RouteTrafficEvent(typed_signal, True, received_at)
            )
        return InputRecordResult(True)

    def record_fixed_zone_entry(self, received_at: float) -> InputRecordResult:
        return self._record_mission_edge("fixed_zone_entry", received_at)

    def record_fixed_zone_exit(self, received_at: float) -> InputRecordResult:
        return self._record_mission_edge("fixed_zone_exit", received_at)

    def record_overtake_entry(self, received_at: float) -> InputRecordResult:
        return self._record_mission_edge("overtake_entry", received_at)

    def record_object_mission_entry(
        self,
        expected_state: Mode,
        snapshot: ObjectSnapshot,
        received_at: float,
        effective_lane: Optional[ObjectLane] = None,
    ) -> InputRecordResult:
        """Queue one validated object entry and preserve its lane once.

        ``MainNode`` calls this only after applying the normal LANE_DRIVE
        session, TTL, box-size and object-type gates.  Binding the semantic
        lane to the same mission edge lets the committed state initialize its
        action without pretending that the pre-commit receipt occurred after
        the new state's ``state_entered_at``.
        """

        self._pending_object_entry_evidence = None
        if expected_state not in (Mode.FIXED_AVOID, Mode.OVERTAKE):
            return InputRecordResult(False, "invalid object mission state")
        if snapshot is not self.latest_object_snapshot:
            return InputRecordResult(False, "object entry snapshot is not latest")
        expected_type = (
            ObjectType.FIXED
            if expected_state is Mode.FIXED_AVOID
            else ObjectType.MOVING
        )
        if snapshot.object_type is not expected_type:
            return InputRecordResult(False, "object entry type/state mismatch")
        entry_lane = snapshot.lane
        if (
            entry_lane not in (ObjectLane.LEFT, ObjectLane.RIGHT)
            and effective_lane in (ObjectLane.LEFT, ObjectLane.RIGHT)
        ):
            entry_lane = effective_lane
        if opposite_lane_target(entry_lane) is None:
            return InputRecordResult(
                False,
                "object entry lane must be LEFT or RIGHT",
            )

        event_name = (
            "fixed_zone_entry"
            if expected_state is Mode.FIXED_AVOID
            else "overtake_entry"
        )
        result = self._record_mission_edge(event_name, received_at)
        if not result.accepted:
            return result

        self._pending_object_entry_evidence = ObjectEntryEvidence(
            expected_state=expected_state,
            lane=entry_lane,
            object_received_at=snapshot.received_at,
            mission_edge_received_at=received_at,
        )
        return result

    def record_overtake_complete(self, received_at: float) -> InputRecordResult:
        return self._record_mission_edge("overtake_complete", received_at)

    def record_shortcut_complete(self, received_at: float) -> InputRecordResult:
        return self._record_mission_edge("shortcut_complete", received_at)

    def _record_mission_edge(
        self,
        name: str,
        received_at: float,
    ) -> InputRecordResult:
        """Record a one-shot completion event, never a sticky boolean level."""

        if not self._valid_timestamp(received_at):
            return InputRecordResult(False, f"invalid {name} timestamp")
        previous = self._last_internal_event_at.get(name)
        if previous is not None and received_at <= previous:
            return InputRecordResult(False, f"duplicate or regressed {name} edge")
        self._last_internal_event_at[name] = received_at
        queue = self._mission_events[name]
        if len(queue) == self.cone_queue_capacity:
            queue.popleft()
        queue.append(MissionEdgeEvent(received_at))
        return InputRecordResult(True)

    def record_cone_message(
        self,
        data: Sequence[Any],
        received_at: float,
    ) -> ConeEnqueueResult:
        """Validate and queue one callback without reusing an older event."""

        try:
            values = list(data)
        except TypeError:
            values = []

        if len(values) not in (3, 4):
            return ConeEnqueueResult(
                False,
                "rubbercone_info requires legacy 3 fields or live 4 fields",
            )
        if not self._valid_timestamp(received_at):
            return ConeEnqueueResult(False, "invalid rubbercone receipt timestamp")

        offset, end_flag, confidence = values[:3]
        if not self._valid_int(offset):
            return ConeEnqueueResult(False, "invalid rubbercone offset")
        if not self._valid_int(end_flag) or end_flag not in (0, 1):
            return ConeEnqueueResult(False, "rubbercone end_flag must be 0 or 1")
        if (
            not self._valid_int(confidence)
            or confidence < 0
            or confidence > 100
        ):
            return ConeEnqueueResult(
                False,
                "rubbercone confidence must be an integer from 0 to 100",
            )

        entry_ready: Optional[bool] = None
        if len(values) == 4:
            raw_entry_ready = values[3]
            if (
                not self._valid_int(raw_entry_ready)
                or raw_entry_ready not in (0, 1)
            ):
                return ConeEnqueueResult(
                    False,
                    "rubbercone entry_ready must be 0 or 1",
                )
            if end_flag == 1 and raw_entry_ready == 1:
                return ConeEnqueueResult(
                    False,
                    "rubbercone end_flag and entry_ready cannot both be 1",
                )
            entry_ready = bool(raw_entry_ready)

        event = ConeMessageEvent(
            offset=offset,
            end_flag=bool(end_flag),
            confidence=confidence,
            entry_ready=entry_ready,
            received_at=received_at,
        )
        dropped_oldest = False
        if len(self._cone_events) == self.cone_queue_capacity:
            # Prefer the newest detector output. Losing an older 0 edge can only
            # leave cone exit disarmed, which is the fail-safe outcome.
            self._cone_events.popleft()
            self.cone_queue_overflow_count += 1
            dropped_oldest = True
        self._cone_events.append(event)
        self.latest_cone_event = event
        self.perception_received_at["rubbercone_info"] = received_at

        warning = (
            "rubbercone event queue full; dropped oldest event"
            if dropped_oldest
            else None
        )
        return ConeEnqueueResult(True, warning, dropped_oldest)

    def step(
        self,
        now: float,
        *,
        lane: Optional[CommandCandidate] = None,
        cone: Optional[CommandCandidate] = None,
        fault_reason: Optional[str] = None,
    ) -> RuntimeCycleResult:
        """Consume at most one cone event and run one authoritative FSM cycle."""

        if self.context.state_entered_at is None and self._valid_timestamp(now):
            self.context.state_entered_at = now

        cone_event = self._cone_events.popleft() if self._cone_events else None
        traffic_event = (
            self._traffic_events.popleft()
            if self._traffic_events
            else None
        )
        lane_change_event = (
            self._lane_change_events.popleft()
            if self._lane_change_events
            else None
        )
        route_traffic_event = (
            self._route_traffic_events.popleft()
            if self._route_traffic_events
            else None
        )
        approved_route_encounter = (
            self._approved_route_encounters[0]
            if self._approved_route_encounters
            else None
        )
        mission_events = {
            name: queue.popleft() if queue else None
            for name, queue in self._mission_events.items()
        }
        object_snapshot = self.latest_object_snapshot

        if route_traffic_event is not None and self._event_is_fresh(
            route_traffic_event.received_at,
            now,
            self.fsm.mission_event_max_age_s,
        ):
            if route_traffic_event.signal is RouteTrafficSignal.RED_AMBER:
                self.traffic_stop_override = True
            elif route_traffic_event.signal in (
                RouteTrafficSignal.STRAIGHT,
                RouteTrafficSignal.LEFT,
            ):
                self.traffic_stop_override = False

        observation = MissionObservation(
            now=now,
            sensor_received_at=self.sensor_received_at,
            perception_received_at=self.perception_received_at,
            green_detected=(
                traffic_event.detected if traffic_event is not None else False
            ),
            traffic_message_received_at=(
                traffic_event.received_at if traffic_event is not None else None
            ),
            cone_detected=(
                cone_event is not None
                and not cone_event.end_flag
                and cone_event.confidence > 0
            ),
            cone_finished=(
                cone_event is not None and cone_event.end_flag
            ),
            cone_confidence=(
                cone_event.confidence if cone_event is not None else None
            ),
            cone_end_flag=(
                cone_event.end_flag if cone_event is not None else None
            ),
            cone_entry_ready=(
                cone_event.entry_ready if cone_event is not None else None
            ),
            cone_message_received_at=(
                cone_event.received_at if cone_event is not None else None
            ),
            scan_received_at=self.sensor_received_at.get("scan"),
            object_exists=(
                object_snapshot.exists if object_snapshot is not None else False
            ),
            object_distance=(
                object_snapshot.distance if object_snapshot is not None else None
            ),
            object_lane=(
                object_snapshot.lane
                if object_snapshot is not None
                else ObjectLane.UNKNOWN
            ),
            object_box_px=(
                object_snapshot.box_px if object_snapshot is not None else 0.0
            ),
            object_type=(
                object_snapshot.object_type
                if object_snapshot is not None
                else ObjectType.UNKNOWN
            ),
            object_confidence=(
                object_snapshot.confidence if object_snapshot is not None else 0.0
            ),
            object_received_at=(
                object_snapshot.received_at if object_snapshot is not None else None
            ),
            lane_change_changing=(
                lane_change_event.changing
                if lane_change_event is not None
                else False
            ),
            lane_change_success=(
                lane_change_event.success
                if lane_change_event is not None
                else False
            ),
            lane_change_success_edge=(
                lane_change_event.success_edge
                if lane_change_event is not None
                else False
            ),
            lane_change_received_at=(
                lane_change_event.received_at
                if lane_change_event is not None
                else None
            ),
            fixed_zone_entered=(
                mission_events["fixed_zone_entry"] is not None
            ),
            fixed_zone_entry_received_at=(
                mission_events["fixed_zone_entry"].received_at
                if mission_events["fixed_zone_entry"] is not None
                else None
            ),
            fixed_zone_exited=(
                mission_events["fixed_zone_exit"] is not None
            ),
            fixed_zone_exit_received_at=(
                mission_events["fixed_zone_exit"].received_at
                if mission_events["fixed_zone_exit"] is not None
                else None
            ),
            overtake_entered=mission_events["overtake_entry"] is not None,
            overtake_entry_received_at=(
                mission_events["overtake_entry"].received_at
                if mission_events["overtake_entry"] is not None
                else None
            ),
            overtake_complete=mission_events["overtake_complete"] is not None,
            overtake_complete_received_at=(
                mission_events["overtake_complete"].received_at
                if mission_events["overtake_complete"] is not None
                else None
            ),
            shortcut_complete=mission_events["shortcut_complete"] is not None,
            shortcut_complete_received_at=(
                mission_events["shortcut_complete"].received_at
                if mission_events["shortcut_complete"] is not None
                else None
            ),
            route_traffic_signal=(
                approved_route_encounter.signal
                if approved_route_encounter is not None
                else route_traffic_event.signal
                if route_traffic_event is not None
                else RouteTrafficSignal.UNKNOWN
            ),
            route_traffic_received_at=(
                approved_route_encounter.received_at
                if approved_route_encounter is not None
                else route_traffic_event.received_at
                if route_traffic_event is not None
                else None
            ),
            traffic_encounter_started=(
                approved_route_encounter is not None
            ),
            traffic_encounter_received_at=(
                approved_route_encounter.received_at
                if approved_route_encounter is not None
                else None
            ),
            traffic_encounter_approved=approved_route_encounter is not None,
        )
        safety = self.safety_monitor.evaluate(
            self.fsm.state,
            self.context,
            observation,
            motion_enabled=self.fsm.state in _MOTION_MODES,
            fault_reason=fault_reason,
        )
        transition = self.fsm.step(observation, self.context, safety)
        if not safety.must_stop and approved_route_encounter is not None:
            self._approved_route_encounters.popleft()
        self._update_lane_action(observation, transition)
        control = self.selector.select(
            self.fsm.state,
            observation.now,
            lane=lane,
            cone=cone,
            mission_lane_authorized=self.lane_action.safe_to_drive,
            traffic_hold=self.traffic_stop_override,
            safety_hold=safety.must_stop,
        )
        fsm_control = ControlCycleResult(transition=transition, control=control)
        cone_entry_committed = (
            transition.changed
            and transition.source is Mode.LANE_DRIVE
            and transition.target is Mode.CONE_DRIVE
        )
        cone_exit_committed = (
            transition.changed
            and transition.source is Mode.CONE_DRIVE
            and transition.target is Mode.LANE_DRIVE
            and transition.reason == "fresh cone end flag"
            and observation.cone_end_flag is True
            and observation.cone_message_received_at is not None
        )
        cone_session_active_command: Optional[bool] = None
        if cone_entry_committed:
            cone_session_active_command = True
        elif cone_exit_committed:
            cone_session_active_command = False

        discarded = 0
        if cone_session_active_command is not None:
            # Events queued before either committed phase boundary belong to
            # the previous producer phase and cannot be replayed afterward.
            discarded = len(self._cone_events)
            self._cone_events.clear()
            self.latest_cone_event = None
            self.perception_received_at.pop("rubbercone_info", None)

        return RuntimeCycleResult(
            observation=observation,
            safety=safety,
            fsm_control=fsm_control,
            cone_session_active_command=cone_session_active_command,
            discarded_pre_phase_events=discarded,
        )

    def _update_lane_action(
        self,
        observation: MissionObservation,
        transition: Transition,
    ) -> None:
        entry_evidence = self._consume_object_entry_evidence(
            observation,
            transition,
        )
        mode = self.fsm.state
        action_modes = (Mode.FIXED_AVOID, Mode.OVERTAKE)
        if mode not in action_modes:
            self.lane_action = LaneActionStatus()
            return
        if self.lane_action.mode is not mode or transition.changed:
            self.lane_action = LaneActionStatus(mode=mode)

        action = self.lane_action
        object_fresh = self._event_is_fresh(
            observation.object_received_at,
            observation.now,
            self.object_max_age_s,
        )

        # 인지가 살아 있으면 주행을 인가한다. 회피 방향이 아직 확정되지 않았어도
        # (박스가 안 보이거나, 좌/우가 미확정이거나, 디바운스가 덜 찼어도)
        # 지금 차선을 그대로 유지한 채 계속 가는 편이 장애물 앞에서 멈춰 서는
        # 것보다 안전하다. 예전에는 이 세 경우가 별도 안전 상태라, 방향이 흔들릴 때마다
        # 구간 한복판에서 속도가 0으로 떨어졌다.
        #
        # README: "고정장애물이 2차선에 있으면 1차선으로 회피하고, 그대로
        # 1차선에서 방해차량 구간을 시작한다. 회피 후 2차선으로 되돌아오지
        # 않는다." 중앙 주행은 회피 전 기본값일 뿐 복귀 목표가 아니다.
        action.safe_to_drive = object_fresh

        # object_detection publishes an already stabilized semantic lane.
        # Main validates receipt/session freshness and owns only the action.
        entry_target = (
            opposite_lane_target(entry_evidence.lane)
            if entry_evidence is not None
            else None
        )
        target = entry_target or opposite_lane_target(observation.object_lane)
        target_received_at = (
            entry_evidence.object_received_at
            if entry_evidence is not None
            else observation.object_received_at
        )
        target_fresh = entry_evidence is not None or self._event_is_fresh(
            target_received_at,
            observation.now,
            self.object_max_age_s,
        )
        target_is_authorized = entry_evidence is not None or (
            self.context.state_entered_at is not None
            and observation.object_received_at is not None
            and observation.object_received_at > self.context.state_entered_at
        )
        if (
            not action.target_locked
            and target_fresh
            and target is not None
            and target_is_authorized
        ):
            target_changed = target != self.context.lane_target
            self.context.lane_target = target
            action.target = target
            action.target_locked = True
            if target_changed:
                action.pending = True
                action.completed = False
                action.completed_at = None
                action.started_at = observation.now

        # 차선 변경 완료 판정 ①: 실측 차선(/lane_position)이 목표와 일치.
        #
        # /lane_change_state 의 성공 엣지만 쓰면 완료가 안 잡혔다. 그 판정은
        # "오프셋이 400px 이상 튄 뒤 50px 이내로 8프레임 연속"인데, bag 실측상
        # 스파이크 자체가 안 나거나(최대 217~313px) 스파이크 뒤 안정이 1프레임밖에
        # 이어지지 않아 조건을 못 채웠다. 실측 차선은 5프레임 디바운스를 이미
        # 거친 값이라 오프셋 거동에 의존하지 않는다.
        if (
            action.pending
            and action.target is not None
            and action.started_at is not None
            and self.measured_lane_received_at is not None
            and self.measured_lane_received_at > action.started_at
            and self._event_is_fresh(
                self.measured_lane_received_at,
                observation.now,
                self.lane_position_max_age_s,
            )
            and self.measured_lane == action.target.value
        ):
            action.pending = False
            action.completed = True
            action.completed_at = self.measured_lane_received_at
            return

        # 완료 판정 ②: /lane_change_state 성공 엣지 (기존 경로 유지)
        success_at = observation.lane_change_received_at
        if (
            action.pending
            and observation.lane_change_success_edge
            and action.started_at is not None
            and success_at is not None
            and success_at > action.started_at
            and self._event_is_fresh(
                success_at,
                observation.now,
                self.lane_change_max_age_s,
            )
        ):
            action.pending = False
            action.completed = True
            action.completed_at = success_at

    def _consume_object_entry_evidence(
        self,
        observation: MissionObservation,
        transition: Transition,
    ) -> Optional[ObjectEntryEvidence]:
        """Consume or discard the one snapshot bound to this FSM commit."""

        evidence = self._pending_object_entry_evidence
        self._pending_object_entry_evidence = None
        if evidence is None:
            return None
        if (
            not transition.changed
            or transition.source is not Mode.LANE_DRIVE
            or transition.target is not evidence.expected_state
            or self.fsm.state is not evidence.expected_state
        ):
            return None
        edge_received_at = (
            observation.fixed_zone_entry_received_at
            if evidence.expected_state is Mode.FIXED_AVOID
            else observation.overtake_entry_received_at
        )
        if edge_received_at != evidence.mission_edge_received_at:
            return None
        return evidence

    @staticmethod
    def _event_is_fresh(
        received_at: Optional[float],
        now: float,
        max_age_s: float,
    ) -> bool:
        if not RaceRuntimeAdapter._valid_timestamp(received_at):
            return False
        if not RaceRuntimeAdapter._valid_timestamp(now):
            return False
        age_s = now - received_at
        return 0.0 <= age_s <= max_age_s

    @staticmethod
    def _valid_int(value: Any) -> bool:
        return isinstance(value, int) and not isinstance(value, bool)

    @staticmethod
    def _valid_binary_int(value: Any) -> bool:
        if isinstance(value, bool):
            return False
        return isinstance(value, int) and value in (0, 1)

    @staticmethod
    def _valid_number(value: Any) -> bool:
        return (
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(value)
        )

    @staticmethod
    def _valid_clearance(value: Any) -> bool:
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            return False
        return (math.isfinite(value) and value >= 0.0) or (
            math.isinf(value) and value > 0.0
        )

    @staticmethod
    def _valid_timestamp(value: Any) -> bool:
        return (
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(value)
        )
