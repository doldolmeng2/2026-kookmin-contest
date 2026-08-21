"""Pure camera-based braking guard for a fixed obstacle in our own lane.

rclpy를 import 하지 않고 토픽도 직접 다루지 않으므로, 실차 없이 단위 테스트로
검증할 수 있다 (README: 노드 파일은 main.py 하나뿐이고 나머지는 순수 로직 모듈).

담당하는 판단은 하나다 — **고정장애물이 우리와 같은 차선에 있는가, 그리고
얼마나 가까운가.** 그 둘로 속도 상한을 만든다. 차선 변경 명령이나 구간 전이는
건드리지 않는다.

왜 카메라인가 (LiDAR 클러스터링을 쓰지 않는 이유)
--------------------------------------------------
고정장애물 bag 6개에서 전방 ±10도 클러스터링(object_detection 의 exists/min_dist)
검출률을 실측했다:

    overtake_1  0/103 =  0.0%      pass_1  20/97 = 20.6%
    overtake_2 29/129 = 22.5%      pass_2   3/62 =  4.8%
    stop_1     22/45  = 48.9%      stop_2  22/48 = 45.8%

range_max_m 가 2.0 m 라 접근 단계에서는 거의 잡히지 않는다. 게다가 min_dist 는
"2 m 안에 무언가 있다"만 말할 뿐 **그것이 내 진로인지 옆 차선인지 모른다.**

같은 bag 을 카메라 신호로 재보면 판정의 성격이 다르다 (stop_1):

    고유 YOLO 박스 프레임 124
      car_lane / ego_lane 둘 다 확정 : 72 (58%)
      그중 "같은 차선" 판정          : 71

확정된 72 프레임 중 71 건이 같은 차선으로 나왔고, 옆 차선을 지나가는
overtake_1 에서는 같은 차선 판정이 **0 건**이었다. 즉 한 번 확정되면 거의 틀리지
않고 오검출도 없다. 이것이 LiDAR 가 줄 수 없는 정보다.

미확정 42% 를 어떻게 다루는가
------------------------------
미확정의 원인은 object_detection 의 좌/우 데드밴드(lane_split_margin_px = 45 px)다.

    미확정 프레임의 |dx| 중앙값 : 26.6 px   (데드밴드 미만)
    확정   프레임의 |dx| 중앙값 : 83.5 px

원근 때문에 멀수록 박스 중심과 차선이 화면에서 가까워져 |dx| 가 작아진다.
브레이크 관점에서는 다행이다 — 정작 필요한 근거리에서 확정률이 올라간다
(overtake_1 14% vs stop_1 58%).

데드밴드를 낮추는 대신 **판정을 유지(hold)** 한다. 미확정을 "안전"으로 해석하면
그 42% 가 그대로 구멍이 되기 때문이다. 해제는 반대 증거(다른 차선으로 확정)나
시간 초과로만 한다.

차선 인코딩(README 규약): 0 = 중앙, 1 = 왼쪽(1차선), 2 = 오른쪽(2차선), -1 = 미확정
시각(now)은 초 단위 float이며 호출자가 하나의 클럭 도메인으로 통일해서 넘긴다.
"""

from dataclasses import dataclass
import math
from typing import Optional


@dataclass(frozen=True)
class SameLaneBrakeConfig:
    """같은 차선 감속 프로필. 속도는 **발행 단위**(Controller 출력의 1/2)다."""

    # 이 면적을 넘어서면 감속을 시작한다.
    #
    # 고정장애물 구간 진입 임계값(FIXED_ENTRY_BOX_PX = 1900)과 같은 수준으로
    # 둔다. 같은 차선에 장애물이 있다고 확정된 상태라면, 구간에 들어서는 그
    # 시점부터 이미 속도를 낮출 이유가 있다.
    slow_box_px: float = 2000.0

    # 이 면적 이상이면 완전 정지한다.
    #
    # stop bag 실측(같은 차선으로 판정된 프레임의 box_px 분포):
    #     stop_1  최소 432  중앙 15040  p90 21840  최대 22885
    # 그리고 box 22686 px² 인 프레임의 LiDAR min_dist 가 0.23 m 였다. 즉 2만대는
    # 이미 접촉 직전이라 그때 제동을 시작하면 늦는다. 중앙값(15040)보다 앞에서
    # 멈추도록 12000 을 쓴다.
    #
    # ★ 실차 튜닝 지점: 너무 일찍 서면 올리고, 아슬아슬하면 내린다.
    stop_box_px: float = 12000.0

    # 감속 구간의 시작 속도. slow_box_px 에서 이 값이 되고 stop_box_px 에서 0 이
    # 되도록 선형 보간한다. LANE_DRIVE 순항이 발행 단위로 21.5 이므로, 감속
    # 구간에 들어서는 순간 절반 이하로 떨어진다.
    crawl_speed: float = 10.0

    # "같은 차선" 판정을 유지하는 시간.
    #
    # 확정률이 58% 라 매 프레임 나오지 않는다. 미확정이 몇 프레임 이어져도 곧바로
    # 풀지 않는다. YOLO 갱신 주기 실측이 평균 0.30초(p10 0.15초)이므로 1.0초면
    # 서너 프레임의 공백을 덮는다.
    hold_s: float = 1.0

    def __post_init__(self) -> None:
        for name, value in (
            ("slow_box_px", self.slow_box_px),
            ("stop_box_px", self.stop_box_px),
            ("crawl_speed", self.crawl_speed),
            ("hold_s", self.hold_s),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value < 0.0
            ):
                raise ValueError(f"{name} must be a finite non-negative number")
        if self.stop_box_px <= self.slow_box_px:
            raise ValueError("stop_box_px must be greater than slow_box_px")


@dataclass(frozen=True)
class SameLaneBrakeDecision:
    """update() 한 번의 결과."""

    # 허용 속도 상한(발행 단위). 무한대면 개입하지 않는다.
    speed_limit: float
    # 지금 "같은 차선"으로 보고 있는지 (hold 중이면 True)
    same_lane: bool
    reason: str
    # 판정이 새 증거가 아니라 hold 로 유지되고 있는지 (로그·디버그용)
    holding: bool = False


class SameLaneBrake:
    """같은 차선 판정 + 박스 면적으로 속도 상한을 만든다.

    lane_target 이나 구간 전이는 건드리지 않는다. 이 클래스는 오직 "얼마나
    느리게 갈 것인가"만 답한다.
    """

    def __init__(self, config: Optional[SameLaneBrakeConfig] = None) -> None:
        self.config = config or SameLaneBrakeConfig()
        self.reset()

    def reset(self) -> None:
        """상태 초기화. bag 루프 감지 등에서 호출한다."""

        self.same_lane_until: Optional[float] = None
        self.last_box_px: float = 0.0

    def update(
        self,
        *,
        now: float,
        car_lane: int,
        ego_lane: int,
        box_px: float,
    ) -> SameLaneBrakeDecision:
        """카메라 신호로 속도 상한을 정한다.

        ``car_lane``  : 장애물이 있는 차선 (/object_info 의 lane_label)
        ``ego_lane``  : 자차 실측 차선 (/lane_position)
        ``box_px``    : YOLO 박스 면적. 0 이면 이번 프레임에 박스가 없다.

        둘 다 1/2 로 확정된 프레임에서만 판정을 갱신하고, 그 밖에는 hold 를
        따른다. 반대 증거(서로 다른 차선으로 확정)가 오면 즉시 해제한다.
        """

        if not self._valid_number(now):
            return SameLaneBrakeDecision(
                speed_limit=float("inf"),
                same_lane=False,
                reason="invalid timestamp",
            )

        box = float(box_px) if self._valid_number(box_px) else 0.0
        if box > 0.0:
            self.last_box_px = box

        determined = car_lane in (1, 2) and ego_lane in (0, 1, 2)
        holding = False

        if determined:
            if car_lane == ego_lane:
                # 새 증거로 확정. hold 시계를 다시 감는다.
                self.same_lane_until = now + self.config.hold_s
            else:
                # 반대 증거다. 옆 차선으로 확인됐으므로 즉시 해제한다.
                # 회피에 성공해 차선을 옮긴 직후가 정확히 이 경우이고, 그때는
                # 곧바로 속도를 회복해야 한다.
                self.same_lane_until = None
        elif self.same_lane_until is not None:
            # 미확정 프레임. 유지 시간이 남아 있으면 직전 판정을 이어간다.
            if now >= self.same_lane_until:
                self.same_lane_until = None
            else:
                holding = True

        if self.same_lane_until is None:
            return SameLaneBrakeDecision(
                speed_limit=float("inf"),
                same_lane=False,
                reason="obstacle not confirmed in our lane",
            )

        limit = self._speed_limit(self.last_box_px)
        if not math.isfinite(limit):
            reason = "same lane but still far"
        elif limit <= 0.0:
            reason = "same lane and too close: stop"
        else:
            reason = "same lane: slowing down"
        return SameLaneBrakeDecision(
            speed_limit=limit,
            same_lane=True,
            reason=reason,
            holding=holding,
        )

    def _speed_limit(self, box_px: float) -> float:
        """박스 면적으로 허용 속도 상한을 만든다.

        면적은 거리의 대용값이다(대략 거리 제곱에 반비례). 절대 거리로 환산하지
        않는 이유는 실측에서 환산 상수가 일정하지 않았기 때문이다 — 같은 bag 에서
        box·d² 가 600~1200 사이로 흔들렸다. 그래서 미터로 바꾸는 대신 실측된
        면적 분포에서 직접 임계값을 잡는다.
        """

        cfg = self.config
        if box_px <= cfg.slow_box_px:
            return float("inf")
        if box_px >= cfg.stop_box_px:
            return 0.0
        remaining = (cfg.stop_box_px - box_px) / (cfg.stop_box_px - cfg.slow_box_px)
        return cfg.crawl_speed * remaining

    @staticmethod
    def _valid_number(value) -> bool:
        return (
            not isinstance(value, bool)
            and isinstance(value, (int, float))
            and math.isfinite(value)
        )
