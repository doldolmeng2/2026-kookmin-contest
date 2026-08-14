"""Pure debounce guard for the fixed-obstacle avoidance direction.

한 프레임의 ``car_lane`` 을 그대로 회피 방향으로 확정하면 안 된다.
``car_lane`` 은 object_detection 이 YOLO 박스 중심(box_cx)과 ``/lane_fit`` 의
중앙선(x_line)을 비교해 내는 값인데, lane_fit 이 6 Hz 로 흔들리는 동안
x_line 이 프레임 사이에 수십 px 씩 튄다.

rosbag2_2026_08_13-09_30_09 실측 (t=42.3~42.9s, 같은 방해차량):

    box_cx=154.5  dx= -83.7  car_lane=1
    box_cx=155.5  dx= -41.3  car_lane=1
    box_cx=159.0  dx= +30.8  car_lane=2   <- 한 프레임만 반전
    box_cx=190.5  dx= -76.9  car_lane=1
    (이후 20프레임 이상 car_lane=1)

박스는 거의 움직이지 않았는데 x_line 이 약 68 px 점프해 부호가 뒤집혔다.
그 한 프레임으로 회피 목표가 장애물이 있는 쪽으로 확정됐고, 차는 그대로
장애물에 들어갔다. 이 가드는 같은 방향이 연속 ``min_consecutive_frames``
번 관측될 때만 방향을 확정해 그런 단발 반전을 걸러낸다.
"""

from dataclasses import dataclass
import math
from typing import Optional

from .mission_observation import MissionObservation
from .mission_types import LaneTarget, opposite_lane_target


_TIMESTAMP_EPSILON_S = 1e-6


@dataclass(frozen=True)
class AvoidDirectionConfig:
    """Receipt-time contract required to commit one avoidance direction."""

    # 같은 좌/우 판정이 연속으로 이만큼 와야 방향을 확정한다.
    min_consecutive_frames: int = 3
    # 그 연속 구간이 실제로 이만큼의 시간을 덮어야 한다.
    #
    # 프레임 수만 세면 안 된다. object_detection 은 50 Hz 타이머로 /object_info
    # 를 내보내면서 YOLO 결과가 갱신되지 않아도 직전 박스를 그대로 재발행한다.
    # 실측(bag 재생): 메시지 1444개 중 85.7%가 바로 앞 메시지와 완전히 동일한
    # 재발행이었고, 실제로 갱신된 YOLO 결과는 300 ms 간격(p10 147 ms)이었다.
    # 그래서 "연속 3프레임"이 60 ms 만에 채워져 디바운스가 무력해졌다.
    # 실측 t=61.38s 에서 단 한 번의 검출(dx=+254)이 3번 재발행되어 차선 변경
    # 명령 [5, 1] 이 나갔다.
    min_duration_s: float = 0.25
    # 이보다 오래된 /object_info 스냅샷은 증거로 세지 않는다. bag 실측 주기는
    # 평균 248 ms, p99 306 ms, 최대 348 ms 였다.
    max_age_s: float = 0.6
    # 이미 확정한 방향을 반대로 뒤집는 최소 간격. 좌우로 번갈아 흔들리는 것을
    # 막되, 지속적인 반대 증거는 통과시킨다.
    retarget_hold_s: float = 0.5
    # 연속 구간이 성립하려면 **차선 피팅과 박스가 둘 다 안정**해야 한다.
    # 프레임 사이에 이보다 크게 움직이면 그 구간은 끊고 다시 센다.
    #
    # 차선 피팅 기준선은 x_line = box_cx - box_dx 로 복원한다. 방향 판정
    # (dx 의 부호)이 이 선을 기준으로 나오므로, 선이 튀는 동안의 좌/우
    # 판정은 박스가 가만히 있어도 뒤집힌다.
    #
    # 값의 근거 — 고정장애물 bag 6개에서 진입 무렵(box 1000~6000) 연속 YOLO
    # 프레임 간 변화량:
    #   |Δx_line|   중앙 4.2~30.5, p90 12.4~56.3, 최대 76.8
    #   |Δbox_cx|   중앙 0.5~28.0, p90  1.5~38.0, 최대 54.0
    # 60 px 는 여섯 bag 의 p90 을 모두 덮어 정상 접근을 통과시키면서,
    # 방해차량 bag 에서 부호를 뒤집었던 138~258 px 점프는 걸러낸다.
    max_line_jump_px: float = 60.0
    max_box_jump_px: float = 60.0

    def __post_init__(self) -> None:
        if (
            isinstance(self.min_consecutive_frames, bool)
            or not isinstance(self.min_consecutive_frames, int)
            or self.min_consecutive_frames < 1
        ):
            raise ValueError(
                "min_consecutive_frames must be an integer of at least one"
            )
        for name, value in (
            ("min_duration_s", self.min_duration_s),
            ("max_age_s", self.max_age_s),
            ("retarget_hold_s", self.retarget_hold_s),
            ("max_line_jump_px", self.max_line_jump_px),
            ("max_box_jump_px", self.max_box_jump_px),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value < 0.0
            ):
                raise ValueError(f"{name} must be a finite non-negative number")


@dataclass(frozen=True)
class AvoidDirectionDecision:
    """확정된 회피 방향과 그 판단 근거."""

    target: Optional[LaneTarget]
    reason: str
    new_sample: bool = False
    changed: bool = False
    candidate: Optional[LaneTarget] = None
    streak: int = 0
    # 현재 연속 구간이 덮은 시간
    streak_elapsed_s: float = 0.0
    # 확정에 쓰인 가장 최근 표본의 수신 시각. 구간 진입 이후 증거인지
    # 호출자가 따질 수 있도록 노출한다.
    last_sample_at: Optional[float] = None


class AvoidDirectionDebouncer:
    """Confirm one avoidance side only after consecutive agreeing samples.

    이 가드는 구간(FIXED_AVOID/OVERTAKE) 밖에서도 계속 돌린다. 방향 판단은
    카메라만 보는 인지 필터이므로 구간 진입을 기다릴 이유가 없고, 미리 데워
    두면 진입 직후 곧바로 회피를 시작할 수 있다.
    """

    def __init__(self, config: Optional[AvoidDirectionConfig] = None) -> None:
        self.config = config or AvoidDirectionConfig()
        self.confirmed: Optional[LaneTarget] = None
        self.confirmed_at: Optional[float] = None
        self.candidate: Optional[LaneTarget] = None
        self.streak = 0
        self.streak_started_at: Optional[float] = None
        self.last_sample_at: Optional[float] = None
        # 직전 표본의 기준선/박스 위치. 안정성 판정에만 쓴다.
        self.last_x_line: Optional[float] = None
        self.last_box_cx: Optional[float] = None

    def reset(self) -> None:
        self.confirmed = None
        self.confirmed_at = None
        self.candidate = None
        self.streak = 0
        self.streak_started_at = None
        self.last_sample_at = None
        self.last_x_line = None
        self.last_box_cx = None

    def update(self, observation: MissionObservation) -> AvoidDirectionDecision:
        received_at = observation.object_received_at
        now = observation.now

        if not self._valid_timestamp(received_at) or not self._valid_timestamp(now):
            return self._decision("no valid object snapshot")

        # 이미 센 표본은 다시 세지 않는다. 제어 주기(50 Hz)가 카메라(4~6 Hz)보다
        # 훨씬 빨라서, 같은 스냅샷을 매 주기 세면 연속 프레임 조건이 한 프레임
        # 만에 충족되어 디바운스가 통째로 무력해진다.
        if (
            self.last_sample_at is not None
            and received_at <= self.last_sample_at
        ):
            return self._decision("no new object snapshot")

        age_s = now - received_at
        if (
            age_s < -_TIMESTAMP_EPSILON_S
            or age_s - self.config.max_age_s > _TIMESTAMP_EPSILON_S
        ):
            # 표본 사이가 벌어지면 연속성이 끊긴 것이다. 다만 이미 확정한
            # 방향은 유지한다. 회피 도중 카메라가 잠깐 끊겼다고 목표를 버리면
            # 차선 변경이 중간에 멈춘다.
            self._break_streak()
            return self._decision("object snapshot too old", new_sample=False)

        self.last_sample_at = received_at

        if observation.object_box_px <= 0.0:
            # 카메라가 방해차량을 못 본 프레임. 반대 증거가 아니라 증거 없음이다.
            self._break_streak()
            return self._decision("no camera box in this frame", new_sample=True)

        # 차선 피팅과 박스가 둘 다 안정해야 이 표본을 연속 구간에 넣는다.
        box_cx = observation.object_box_cx
        x_line = box_cx - observation.object_box_dx
        unstable = (
            self.last_x_line is not None
            and self.last_box_cx is not None
            and (
                abs(x_line - self.last_x_line) > self.config.max_line_jump_px
                or abs(box_cx - self.last_box_cx) > self.config.max_box_jump_px
            )
        )
        self.last_x_line = x_line
        self.last_box_cx = box_cx
        if unstable:
            # 튄 프레임 자체는 증거로 세지 않고 구간만 끊는다. 다음 프레임부터
            # 다시 쌓이므로, 흔들림이 멈추면 자연히 확정된다.
            self._break_streak()
            return self._decision("lane fit or box jumped", new_sample=True)

        candidate = opposite_lane_target(observation.object_lane)
        if candidate is None:
            # 박스는 있는데 중앙선 기준 좌/우가 미확정(dx가 마진 안)이다.
            # 이것도 반대 증거가 아니므로 확정된 방향을 뒤집지 않는다.
            self._break_streak()
            return self._decision("box has no side label", new_sample=True)

        if candidate is self.candidate:
            self.streak += 1
        else:
            self.candidate = candidate
            self.streak = 1
            self.streak_started_at = received_at

        elapsed_s = received_at - (self.streak_started_at or received_at)
        if (
            self.streak < self.config.min_consecutive_frames
            or elapsed_s + _TIMESTAMP_EPSILON_S < self.config.min_duration_s
        ):
            return self._decision(
                "waiting for consecutive side agreement",
                new_sample=True,
            )

        if candidate is self.confirmed:
            return self._decision("side re-confirmed", new_sample=True)

        if (
            self.confirmed is not None
            and self.confirmed_at is not None
            and now - self.confirmed_at + _TIMESTAMP_EPSILON_S
            < self.config.retarget_hold_s
        ):
            return self._decision(
                "opposite side held back by retarget hold",
                new_sample=True,
            )

        self.confirmed = candidate
        self.confirmed_at = now
        return self._decision("side confirmed", new_sample=True, changed=True)

    def _break_streak(self) -> None:
        self.candidate = None
        self.streak = 0
        self.streak_started_at = None

    def _decision(
        self,
        reason: str,
        *,
        new_sample: bool = False,
        changed: bool = False,
    ) -> AvoidDirectionDecision:
        elapsed_s = 0.0
        if self.streak_started_at is not None and self.last_sample_at is not None:
            elapsed_s = max(0.0, self.last_sample_at - self.streak_started_at)
        return AvoidDirectionDecision(
            target=self.confirmed,
            reason=reason,
            new_sample=new_sample,
            changed=changed,
            candidate=self.candidate,
            streak=self.streak,
            streak_elapsed_s=elapsed_s,
            last_sample_at=self.last_sample_at,
        )

    @staticmethod
    def _valid_timestamp(value: Optional[float]) -> bool:
        return (
            value is not None
            and not isinstance(value, bool)
            and isinstance(value, (int, float))
            and math.isfinite(value)
        )
