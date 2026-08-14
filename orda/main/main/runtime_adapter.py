"""ROS-independent runtime adapter for the authoritative race FSM."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Any, Callable, Deque, Optional, Sequence

from .avoid_direction import (
    AvoidDirectionConfig,
    AvoidDirectionDebouncer,
    AvoidDirectionDecision,
)
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
    RouteTrafficEvent,
    RouteTrafficSignal,
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
#   /object_info   248 ms   280 ms  306 ms   348 ms
#
# 예전 값(0.25 / 0.5초)은 이 실측보다 빠듯해서, 정상 주행 중에도 lane_offset이
# 19%(0.25초 기준) / 4%(0.5초 기준) 확률로 "끊김"으로 분류됐다. 그때마다
# LANE_DRIVE -> STOP 으로 떨어지고 속도가 0으로 리셋돼 차가 앞으로 못 나갔다
# (기록된 모터 출력: 속도 0인 사이클 24.2%, 평균 속도 2.24).
#
# 근본 해결은 인지 파이프라인을 빠르게 하는 것이다(디버그 창 비활성화 등 별도
# 수정). 예산은 그 위에 두는 안전망이므로, 실측 p99 + 지터를 덮되 무한정
# 늘리지는 않는다.
LANE_OFFSET_MAX_AGE_S = 1.0
SCAN_MAX_AGE_S = 0.5
TRAFFIC_MAX_AGE_S = 0.5
OBJECT_MAX_AGE_S = 0.6

# 차선 변경 명령을 한 번 낸 뒤, 측면 LiDAR 추월 완료 판정이 나고도 이만큼
# 더 지나야 다음 명령을 낼 수 있다.
#
# 장애물 옆을 지나는 동안 회피 방향을 다시 확정하면 안 되기 때문이다.
# rosbag2_fixed_obstacles_overtake_2 실측 — 옆을 지나가는 동안 장애물이
# 화면을 가로질러 쓸려나가면서 car_lane 이 1에서 2로 정직하게 반전했고,
# 그 증거로 [5, 1] 명령이 나갔다. 지나가고 있는 장애물 쪽으로 되돌아가라는
# 명령이다:
#
#   t=5.97~6.92  box  1144→ 4590  cx 333→344  dx -150→-125  car_lane=1
#   t=7.13       box  5044        cx 379      dx  -15       car_lane=0
#   t=7.33~7.55  box  8928→14570  cx 486→562  dx +109→+226  car_lane=2
#
# 추월 완료 판정(main.overtake.OvertakeGuard)은 측면 LiDAR 로 장애물이
# 옆을 지나간 것을 확인한 시점이다. 거기서 2초를 더 두면 차체가 완전히
# 빠져나간 뒤에야 다음 명령이 가능해진다.
COMMAND_RELEASE_AFTER_PASS_S = 2.0

_MOTION_MODES = frozenset(
    {
        Mode.LANE_DRIVE,
        Mode.CONE_DRIVE,
        Mode.REJOIN,
        Mode.FIXED_AVOID,
        Mode.OVERTAKE,
        Mode.SHORTCUT,
    }
)


@dataclass(frozen=True)
class ConeMessageEvent:
    """One validated ``/rubbercone_info`` callback receipt edge."""

    offset: int
    end_flag: bool
    confidence: int
    received_at: float


@dataclass(frozen=True)
class LaneValidityEvent:
    """One future explicit lane-validity callback receipt edge."""

    valid: bool
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


@dataclass
class LaneActionStatus:
    """Action-level state; it deliberately does not add an FSM mode."""

    mode: Optional[Mode] = None
    safe_to_drive: bool = False
    pending: bool = False
    completed: bool = False
    target: Optional[LaneTarget] = None
    started_at: Optional[float] = None
    completed_at: Optional[float] = None


@dataclass(frozen=True)
class RuntimeCycleResult:
    observation: MissionObservation
    safety: SafetyDecision
    fsm_control: ControlCycleResult
    publish_cone_reset: bool
    discarded_pre_reset_events: int = 0
    discarded_pre_rejoin_lane_events: int = 0

    @property
    def transition(self) -> Transition:
        return self.fsm_control.transition

    @property
    def control(self) -> ControlDecision:
        return self.fsm_control.control


def dispatch_cone_reset(
    cycle: RuntimeCycleResult,
    publish_reset: Callable[[], None],
) -> bool:
    """Dispatch one reset command only for a committed cone-entry edge."""

    if not cycle.publish_cone_reset:
        return False
    publish_reset()
    return True


def runtime_safety_monitor() -> SafetyMonitor:
    """Build state-specific safety requirements from existing ROS inputs."""

    return SafetyMonitor(
        {
            Mode.INIT: (
                InputRequirement(
                    InputCategory.PERCEPTION,
                    "traffic_detection",
                    TRAFFIC_MAX_AGE_S,
                ),
                InputRequirement(
                    InputCategory.PERCEPTION,
                    "lane_offset",
                    LANE_OFFSET_MAX_AGE_S,
                ),
                InputRequirement(InputCategory.SENSOR, "scan", SCAN_MAX_AGE_S),
            ),
            Mode.WAIT_GREEN: (
                InputRequirement(
                    InputCategory.PERCEPTION,
                    "traffic_detection",
                    TRAFFIC_MAX_AGE_S,
                ),
            ),
            Mode.LANE_DRIVE: (
                InputRequirement(
                    InputCategory.PERCEPTION,
                    "lane_offset",
                    LANE_OFFSET_MAX_AGE_S,
                ),
            ),
            # STOP 복귀 조건. 이 항목이 비어 있으면 SafetyMonitor 가 STOP 에서
            # 검사할 입력이 없어 inputs_ready 가 항상 True 였고, RaceFSM 의
            # 복귀 게이트(STOP_RECOVERY_HOLD_S)가 "입력이 회복됐는지"가 아니라
            # 단순 0.5초 타이머로 동작했다. 실측에서 STOP 구간 길이가 전부
            # 0.50~0.52초로 똑같았던 이유다. 그래서 인지가 죽은 채로 주행을
            # 재개했다가 곧바로 다시 STOP 으로 떨어지기를 반복했다.
            #
            # STOP 은 motion_enabled=False 이므로, 여기 등록해도 must_stop 이
            # 새로 발생하지는 않는다. 오직 복귀 게이트로만 쓰인다.
            Mode.STOP: (
                InputRequirement(
                    InputCategory.PERCEPTION,
                    "lane_offset",
                    LANE_OFFSET_MAX_AGE_S,
                ),
                InputRequirement(InputCategory.SENSOR, "scan", SCAN_MAX_AGE_S),
            ),
            # A missing detector reset must leave CONE_DRIVE waiting for a
            # fresh zero, not force an immediate terminal STOP. Scan loss is
            # still a terminal sensor failure; cone command freshness is
            # enforced independently by ControlSelector.
            Mode.CONE_DRIVE: (
                InputRequirement(InputCategory.SENSOR, "scan", 0.5),
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
        avoid_direction_config: Optional[AvoidDirectionConfig] = None,
        command_release_after_pass_s: float = COMMAND_RELEASE_AFTER_PASS_S,
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
        self.command_release_after_pass_s = float(command_release_after_pass_s)
        # 마지막으로 낸 차선 변경 명령 시각과, 그 뒤의 추월 완료 판정 시각.
        # 모드가 바뀌어도 유지된다 (lane_action 은 구간마다 새로 만들어진다).
        self.last_lane_command_at: Optional[float] = None
        self.pass_completed_at: Optional[float] = None
        self.avoid_direction = AvoidDirectionDebouncer(avoid_direction_config)

        self._cone_events: Deque[ConeMessageEvent] = deque()
        self._lane_validity_events: Deque[LaneValidityEvent] = deque()
        self._traffic_events: Deque[TrafficMessageEvent] = deque()
        self._lane_change_events: Deque[LaneChangeStateEvent] = deque()
        self._route_traffic_events: Deque[RouteTrafficEvent] = deque()
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
        self.latest_cone_event: Optional[ConeMessageEvent] = None
        self.latest_object_snapshot: Optional[ObjectSnapshot] = None
        self._last_lane_change_received_at: Optional[float] = None
        self._lane_change_success_active = False
        self.traffic_stop_override = False
        self.lane_action = LaneActionStatus()
        self.last_avoid_direction: Optional[AvoidDirectionDecision] = None
        # /lane_position 실측 차선 (-1=미확정). main_node 가 콜백에서 갱신한다.
        self.measured_lane: int = -1

    @property
    def pending_cone_event_count(self) -> int:
        return len(self._cone_events)

    def record_scan(self, received_at: float) -> bool:
        if not self._valid_timestamp(received_at):
            return False
        self.sensor_received_at["scan"] = received_at
        return True

    def record_lane_offset(self, offset: int, received_at: float) -> bool:
        if not self._valid_int(offset) or not self._valid_timestamp(received_at):
            return False
        self.latest_lane_offset = offset
        self.latest_lane_received_at = received_at
        self.perception_received_at["lane_offset"] = received_at
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
        self.perception_received_at["traffic_detection"] = received_at
        return True

    def record_lane_validity(self, valid: bool, received_at: float) -> bool:
        """Queue one explicit validity edge for the future lane publisher."""

        if not isinstance(valid, bool) or not self._valid_timestamp(received_at):
            return False
        if len(self._lane_validity_events) == self.cone_queue_capacity:
            self._lane_validity_events.popleft()
        self._lane_validity_events.append(
            LaneValidityEvent(valid=valid, received_at=received_at),
        )
        self.perception_received_at["lane_validity"] = received_at
        return True

    def record_object_info(
        self,
        data: Sequence[Any],
        received_at: float,
    ) -> InputRecordResult:
        """Validate the object_info payload and retain its typed subset.

        README 계약은 11필드다:

            [exists, min_dist, angle, span, cluster_size,
             box_size, box_cx, box_cy, dx, car_lane, is_moving]

        11번째 is_moving 은 회피 규칙을 고른다 (0=고정장애물 → FIXED_AVOID,
        1=방해차량 → OVERTAKE). 검출기가 아직 고정장애물만 학습해 상수 0을
        넣으므로(object_detection.cpp 의 kIsMovingFixedObstacle) 지금은 늘
        고정장애물 경로로 간다. 이동체를 구분하는 모델이 들어오면 이 필드만
        1로 바뀌어도 분기가 살아난다.

        길이를 '정확히 11'이 아니라 '10 이상'으로 보는 이유는 is_moving 이
        생기기 전에 기록한 bag 때문이다 (rosbag2_object1 은 10필드,
        rosbag2_fixed_obstacles_* 는 11필드). 앞 10필드는 두 세대가 같고,
        11번째가 없으면 is_moving 은 None(판단 근거 없음)이 된다.

        측면 LiDAR 거리(side_left, side_right)는 이 토픽에 없다. main_node 가
        /scan 에서 직접 계산한다 (main.py 의 side_clearance 호출).
        """

        try:
            values = list(data)
        except TypeError:
            values = []
        if len(values) < 10:
            return InputRecordResult(False, "object_info requires at least 10 fields")
        if not self._valid_timestamp(received_at):
            return InputRecordResult(False, "invalid object_info receipt timestamp")

        # 11번째 is_moving 은 계약상 0/1 뿐이다. 그 밖의 값은 검출기 버그이고,
        # 회피 규칙을 고르는 값이라 조용히 삼키면 안 된다.
        is_moving: Optional[bool] = None
        if len(values) >= 11:
            flag = values[10]
            if (
                isinstance(flag, bool)
                or not isinstance(flag, (int, float))
                or float(flag) not in (0.0, 1.0)
            ):
                return InputRecordResult(False, "object_info is_moving must be 0 or 1")
            is_moving = float(flag) == 1.0

        # 나머지 검증은 앞 10필드에만 적용한다.
        values = values[:10]
        if any(
            not isinstance(value, (int, float)) or isinstance(value, bool)
            for value in values
        ):
            return InputRecordResult(False, "object_info fields must be numbers")

        numeric_values = [float(value) for value in values]
        if any(math.isnan(value) for value in numeric_values):
            return InputRecordResult(False, "object_info fields must not contain NaN")

        exists_value = numeric_values[0]
        if exists_value not in (0.0, 1.0):
            return InputRecordResult(False, "object_info exists must be 0 or 1")
        lane_value = numeric_values[9]
        if not math.isfinite(lane_value) or not lane_value.is_integer():
            return InputRecordResult(False, "object_info lane label must be an integer")
        try:
            lane = ObjectLane(int(lane_value))
        except ValueError:
            return InputRecordResult(False, "object_info lane label must be 0, 1, or 2")

        # YOLO 박스 면적. 음수는 검출기가 낼 수 없는 값이므로 거른다.
        box_px = numeric_values[5]
        if not math.isfinite(box_px) or box_px < 0.0:
            return InputRecordResult(
                False,
                "object_info box size must be a non-negative number",
            )

        distance = numeric_values[1]
        if exists_value == 1.0:
            if any(not math.isfinite(value) for value in numeric_values):
                return InputRecordResult(
                    False,
                    "detected object_info fields must be finite numbers",
                )
            if distance < 0.0:
                return InputRecordResult(
                    False,
                    "object_info distance must be non-negative",
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
                    "absent object_info distance must be non-negative or +inf",
                )
            if any(not math.isfinite(value) for value in numeric_values[2:9]):
                return InputRecordResult(
                    False,
                    "absent object_info metadata must be finite numbers",
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
            box_cx=numeric_values[6],
            box_dx=numeric_values[8],
            lane=snapshot_lane,
            received_at=received_at,
            is_moving=is_moving,
        )
        self.perception_received_at["object_info"] = received_at
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
            RouteTrafficEvent(typed_signal, encounter_started, received_at)
        )
        return InputRecordResult(True)

    def record_fixed_zone_entry(self, received_at: float) -> InputRecordResult:
        return self._record_mission_edge("fixed_zone_entry", received_at)

    def record_fixed_zone_exit(self, received_at: float) -> InputRecordResult:
        return self._record_mission_edge("fixed_zone_exit", received_at)

    def record_pass_complete(self, now: float) -> None:
        """측면 LiDAR 추월 완료 판정 시각을 기록한다 (명령 잠금 해제 기준)."""

        if self._valid_timestamp(now):
            self.pass_completed_at = now

    def record_overtake_entry(self, received_at: float) -> InputRecordResult:
        return self._record_mission_edge("overtake_entry", received_at)

    def record_overtake_complete(self, received_at: float) -> InputRecordResult:
        return self._record_mission_edge("overtake_complete", received_at)

    def record_shortcut_complete(self, received_at: float) -> InputRecordResult:
        return self._record_mission_edge("shortcut_complete", received_at)

    def _record_mission_edge(
        self,
        name: str,
        received_at: float,
    ) -> InputRecordResult:
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

        if len(values) < 3:
            return ConeEnqueueResult(
                False,
                "rubbercone_info requires [offset, end_flag, confidence]",
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

        event = ConeMessageEvent(
            offset=offset,
            end_flag=bool(end_flag),
            confidence=confidence,
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
        lane_validity_event = (
            self._lane_validity_events.popleft()
            if self._lane_validity_events
            else None
        )
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
            # There is no dedicated lane-validity input in the current graph.
            lane_valid=(
                lane_validity_event.valid
                if lane_validity_event is not None
                else False
            ),
            lane_valid_received_at=(
                lane_validity_event.received_at
                if lane_validity_event is not None
                else None
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
            object_box_cx=(
                object_snapshot.box_cx if object_snapshot is not None else 0.0
            ),
            object_box_dx=(
                object_snapshot.box_dx if object_snapshot is not None else 0.0
            ),
            object_is_moving=(
                object_snapshot.is_moving if object_snapshot is not None else None
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
            fixed_zone_entered=mission_events["fixed_zone_entry"] is not None,
            fixed_zone_entry_received_at=(
                mission_events["fixed_zone_entry"].received_at
                if mission_events["fixed_zone_entry"] is not None
                else None
            ),
            fixed_zone_exited=mission_events["fixed_zone_exit"] is not None,
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
                route_traffic_event.signal
                if route_traffic_event is not None
                else RouteTrafficSignal.UNKNOWN
            ),
            route_traffic_received_at=(
                route_traffic_event.received_at
                if route_traffic_event is not None
                else None
            ),
            traffic_encounter_started=(
                route_traffic_event.encounter_started
                if route_traffic_event is not None
                else False
            ),
            traffic_encounter_received_at=(
                route_traffic_event.received_at
                if route_traffic_event is not None
                and route_traffic_event.encounter_started
                else None
            ),
        )
        safety = self.safety_monitor.evaluate(
            self.fsm.state,
            self.context,
            observation,
            motion_enabled=self.fsm.state in _MOTION_MODES,
            fault_reason=fault_reason,
        )
        transition = self.fsm.step(observation, self.context, safety)
        self._update_lane_action(observation, transition)
        control = self.selector.select(
            self.fsm.state,
            observation.now,
            lane=lane,
            cone=cone,
            mission_lane_authorized=self.lane_action.safe_to_drive,
            traffic_hold=self.traffic_stop_override,
        )
        fsm_control = ControlCycleResult(transition=transition, control=control)
        publish_cone_reset = (
            transition.changed
            and transition.source is Mode.LANE_DRIVE
            and transition.target is Mode.CONE_DRIVE
        )

        discarded = 0
        if publish_cone_reset:
            # Events already queued before the reset publish belong to the old
            # detector session and must never arm the new exit handshake.
            discarded = len(self._cone_events)
            self._cone_events.clear()
            self.latest_cone_event = None
            self.perception_received_at.pop("rubbercone_info", None)

        discarded_lane = 0
        if (
            transition.changed
            and transition.source is Mode.CONE_DRIVE
            and transition.target is Mode.REJOIN
        ):
            discarded_lane = len(self._lane_validity_events)
            self._lane_validity_events.clear()
            self.perception_received_at.pop("lane_validity", None)

        return RuntimeCycleResult(
            observation=observation,
            safety=safety,
            fsm_control=fsm_control,
            publish_cone_reset=publish_cone_reset,
            discarded_pre_reset_events=discarded,
            discarded_pre_rejoin_lane_events=discarded_lane,
        )

    def _update_lane_action(
        self,
        observation: MissionObservation,
        transition: Transition,
    ) -> None:
        # 회피 방향 가드는 구간 밖에서도 계속 돌린다. 방향 판단은 카메라만 보는
        # 인지 필터라 구간 진입을 기다릴 이유가 없고, 미리 데워 두면 진입 직후
        # 곧바로 회피를 시작할 수 있다.
        direction = self.avoid_direction.update(observation)
        self.last_avoid_direction = direction

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
        # 것보다 안전하다. 예전에는 이 세 경우가 STOP 이라, 방향이 흔들릴 때마다
        # 구간 한복판에서 속도가 0으로 떨어졌다.
        #
        # README: "고정장애물이 2차선에 있으면 1차선으로 회피하고, 그대로
        # 1차선에서 방해차량 구간을 시작한다. 회피 후 2차선으로 되돌아오지
        # 않는다." 중앙 주행은 회피 전 기본값일 뿐 복귀 목표가 아니다.
        action.safe_to_drive = object_fresh

        # 회피 방향은 YOLO만으로 정한다. object_box_px 와 object_lane 은
        # 카메라 단독 산출물이고, LiDAR는 추월 완료 확인(main.overtake)에만 쓴다.
        #
        # 예전에는 LiDAR 기반 object_exists 를 요구했다. 구간 진입은
        # YOLO(box_size > FIXED_ENTRY_BOX_PX)로 하면서 방향 결정만 LiDAR에
        # 걸어둔 비대칭이라, 카메라가 방해차량을 또렷이 보는데도 회피를
        # 시작하지 못했다. rosbag2_fixed_obstacles_overtake_1 실측:
        # 방해차량이 대부분 2 m 밖이라 range_max_m(2.0)에 걸려 전방 클러스터가
        # 103 스캔 중 1번만 형성됐고(±10도 최소거리 중앙값 5.08 m),
        # object_exists 가 1810 샘플 내내 0이었다. 같은 구간에서 YOLO는
        # 박스 최대 10549 px^2, car_lane=1을 안정적으로 냈다.
        #
        # 단, 한 프레임짜리 car_lane 반전으로 방향을 확정하지는 않는다.
        # AvoidDirectionDebouncer 가 연속 프레임 합의에 더해 **차선 피팅과
        # 박스가 둘 다 안정할 것**을 요구한다.
        #
        # 명령을 한 번 낸 뒤에는, 그 명령보다 뒤에 나온 추월 완료 판정이
        # command_release_after_pass_s 만큼 묵어야 다음 명령을 낼 수 있다.
        # lane_action 이 아니라 어댑터가 들고 있으므로 구간을 나갔다 들어와도
        # 유지된다 — 방금 지나친 장애물로 재진입해도 명령이 다시 나가지 않는다.
        command_locked = self.last_lane_command_at is not None and not (
            self.pass_completed_at is not None
            and self.pass_completed_at >= self.last_lane_command_at
            and observation.now - self.pass_completed_at
            >= self.command_release_after_pass_s
        )
        target = direction.target
        if (
            object_fresh
            and not command_locked
            and target is not None
            and target != self.context.lane_target
            and self.context.state_entered_at is not None
            and direction.last_sample_at is not None
            and direction.last_sample_at > self.context.state_entered_at
        ):
            # 이미 시작했거나(pending) 끝난(completed) 변경이어도 방향을 고친다.
            # 예전에는 not pending and not completed 를 요구해서, 잘못된
            # 방향으로 한 번 들어가면 반대 증거가 아무리 쌓여도 되돌릴 수
            # 없었다. 실측 bag에서 car_lane=1 이 20프레임 넘게 연속으로
            # 나왔는데도 목표가 1차선(장애물이 있는 쪽)에 고정된 채 충돌했다.
            #
            # 중앙(CENTER) 복귀는 여기서도 일어나지 않는다.
            # opposite_lane_target() 이 좌/우만 돌려주기 때문이다.
            self.context.lane_target = target
            action.target = target
            action.pending = True
            action.completed = False
            action.completed_at = None
            action.started_at = observation.now
            self.last_lane_command_at = observation.now

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
            and self.measured_lane == action.target.value
        ):
            action.pending = False
            action.completed = True
            action.completed_at = observation.now
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
    def _valid_timestamp(value: Any) -> bool:
        return (
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(value)
        )
