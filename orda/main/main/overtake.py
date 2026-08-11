"""Pure decision logic for interfering-vehicle avoidance and overtake completion.

main.py가 소유하던 회피/추월 판단을 옮겨온 모듈이다. rclpy를 import 하지 않고
토픽도 직접 다루지 않으므로 실차 없이 단위 테스트로 검증할 수 있다.

담당하는 판단 세 가지:
  1) 지금 회피해야 하는가, 그렇다면 어느 차선으로   (begin_avoidance)
  2) 방해차량이 옆으로 지나갔는가, 지나간 지 충분히 되었는가 (update_zone)
  3) 추월이 끝났으면 어느 차선으로 복귀하는가        (take_restore_lane)

차선 인코딩은 main.py와 같다.
  lane_target / detected_lane : 0 = Lane1(왼쪽), 1 = Lane2(오른쪽)
  car_lane (object_detection) : 1 = L1(왼쪽), 2 = L2(오른쪽), 0 = 중앙, -1 = 미정

시각(now)은 초 단위 float이며 호출자가 하나의 클럭 도메인으로 통일해서 넘긴다.
"""

from dataclasses import dataclass
import math
from typing import Optional


@dataclass(frozen=True)
class OvertakeConfig:
    """방해차량 회피/추월 판단 임계값."""

    # 이 면적을 넘는 YOLO 박스를 회피 대상으로 본다. 전방 인식은 종전대로
    # 카메라(YOLO)가 담당하며 LiDAR는 측면 확인에만 쓴다.
    trigger_box_px: float = 1900.0

    # 측면 LiDAR가 이 거리 안이면 옆에 차가 있다고 본다.
    # bag 실측: 방해차량이 옆에 있을 때 0.26~0.46 m, 벽/빈 차선은 0.90 m 이상.
    side_detect_m: float = 0.60

    # 옆에서 인식한 뒤 이만큼 더 지나야 추월 완료로 본다.
    # LiDAR는 차 맨 앞에 있어서 인식 시점엔 앞범퍼만 나란한 상태다. 초음파
    # (차 중간)와 달리 이르게 반응하므로, 차체가 완전히 빠져나갈 시간을 더 준다.
    pass_delay_s: float = 2.0

    # 구간 시간 상한. 방해차량을 못 만난 바퀴에서도 다음 구간으로 넘어가야 한다.
    zone_timeout_s: float = 12.0

    def __post_init__(self) -> None:
        for name, value in (
            ("trigger_box_px", self.trigger_box_px),
            ("side_detect_m", self.side_detect_m),
            ("pass_delay_s", self.pass_delay_s),
            ("zone_timeout_s", self.zone_timeout_s),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value < 0.0
            ):
                raise ValueError(f"{name} must be a finite non-negative number")


@dataclass(frozen=True)
class PassDecision:
    """update_zone() 한 번의 결과."""

    complete: bool
    reason: str
    # 이번 호출에서 처음으로 측면 방해차량을 인식했는지 (로그 1회 출력용)
    side_just_seen: bool = False
    side_distance: float = float("inf")
    timed_out: bool = False


class OvertakeGuard:
    """회피 목표 차선과 추월 완료 시점을 판단하고 복귀 차선을 기억한다."""

    def __init__(self, config: Optional[OvertakeConfig] = None) -> None:
        self.config = config or OvertakeConfig()
        self.reset()

    def reset(self) -> None:
        """상태 초기화. bag 루프 감지 등에서 호출한다."""

        self.lane_before_avoid: Optional[int] = None
        self.side_seen_at: Optional[float] = None
        self.side_seen_distance: float = float("inf")
        self.zone_entered_at: Optional[float] = None

    def enter_zone(self, now: float, current_lane: Optional[int] = None) -> None:
        """장애물 구간에 진입할 때 구간 판정 상태를 초기화한다.

        ``current_lane``을 주면 그 차선을 복귀 대상으로 기억한다. 회피 방향을
        FSM 쪽에서 결정하는 구성(runtime_adapter가 lane_target을 직접 바꾸는
        경우)에서는 begin_avoidance()를 쓰지 않으므로 이 인자로 기억시킨다.

        인자를 주지 않으면 복귀 차선(lane_before_avoid)은 건드리지 않는다.
        회피 → 구간 진입 순서로 불릴 때 방금 기억한 값이 지워지면 안 된다.
        """

        self.zone_entered_at = now
        self.side_seen_at = None
        self.side_seen_distance = float("inf")
        if current_lane is not None:
            self.lane_before_avoid = current_lane

    def zone_elapsed(self, now: float) -> Optional[float]:
        """구간 진입 후 경과 시간(초). 진입 기록이 없으면 None."""

        if self.zone_entered_at is None:
            return None
        return now - self.zone_entered_at

    # ── 회피 판단 ────────────────────────────────────────────────────────

    def obstacle_in_ego_lane(
        self,
        car_lane: int,
        lane_target: int,
        detected_lane: int,
    ) -> bool:
        """검출된 장애물이 우리 차량과 같은 차선에 있는지 판정한다.

        car_lane이 미정(-1)이거나 중앙(0)이면 확정할 수 없으므로 안전 쪽으로
        (같은 차선으로 간주하여) True를 반환한다. 회피를 놓치는 쪽이 불필요하게
        피하는 쪽보다 비용이 크다.

        기준 차선은 실측값(detected_lane)이 있으면 그것을, 없으면 제어 목표
        (lane_target)를 쓴다.
        """

        if car_lane not in (1, 2):
            return True

        obstacle_lane = 0 if car_lane == 1 else 1
        ego_lane = detected_lane if detected_lane in (0, 1) else lane_target
        return obstacle_lane == ego_lane

    def avoid_target_lane(self, car_lane: int, lane_target: int) -> int:
        """회피 목표 차선을 반환한다.

        README "차선 주행 정책": 추월 방향은 시작 차선과 무관하게 **방해차량이
        있는 차선의 반대편**으로 결정한다 (같은 쪽으로 추월하면 차선 이탈).
        car_lane이 미확정(-1)이나 중앙(0)이면 방향을 고를 근거가 없으므로
        현재 차선의 반대편으로 물러난다.
        """

        if car_lane == 1:   # 장애물이 왼쪽 차선 → 오른쪽으로 피한다
            return 1
        if car_lane == 2:   # 장애물이 오른쪽 차선 → 왼쪽으로 피한다
            return 0
        return 1 - lane_target

    def begin_avoidance(
        self,
        *,
        box_size: float,
        car_lane: int,
        lane_target: int,
        detected_lane: int,
    ) -> Optional[int]:
        """회피가 필요하면 목표 차선을, 아니면 None을 반환한다.

        목표 차선을 반환할 때 **직전 차선을 복귀용으로 기억한다.** 즉 이 호출은
        회피를 확정하는 시점이며, 호출자는 반환값을 그대로 lane_target으로
        쓰면 된다.

        이미 회피 차선에 있으면(target == lane_target) None을 반환한다. 여기서
        다시 반대로 토글하면 방금 피한 차선으로 되돌아가 앞차와 충돌한다.
        """

        if box_size <= self.config.trigger_box_px:
            return None
        if not self.obstacle_in_ego_lane(car_lane, lane_target, detected_lane):
            return None

        target = self.avoid_target_lane(car_lane, lane_target)
        if target == lane_target:
            return None

        self.lane_before_avoid = lane_target
        return target

    # ── 추월 완료 판단 ───────────────────────────────────────────────────

    def update_zone(
        self,
        *,
        now: float,
        lane_target: int,
        side_left: float,
        side_right: float,
    ) -> PassDecision:
        """측면 LiDAR로 추월 완료를 판정한다.

        회피로 차선을 옮겼으므로 방해차량은 현재 주행 차선의 반대편에 있다.
          - Lane1(lane_target=0) 주행 중 → 오른쪽(side_right)
          - Lane2(lane_target=1) 주행 중 → 왼쪽(side_left)

        초음파를 쓰지 않는 이유: bag(rosbag2_object1)에서 오른쪽 초음파가 전
        구간 4cm에 고착되어(전체 샘플의 98%) 차선을 바꾸자마자 "추월 완료"가
        나버렸고, 그 오판이 회피를 원위치로 되돌려 충돌로 이어졌다.
        """

        side_distance = side_right if lane_target == 0 else side_left
        just_seen = False

        if self.side_seen_at is None and side_distance < self.config.side_detect_m:
            self.side_seen_at = now
            self.side_seen_distance = side_distance
            just_seen = True

        if self.side_seen_at is not None:
            if now - self.side_seen_at >= self.config.pass_delay_s:
                return PassDecision(
                    complete=True,
                    reason="overtake confirmed after side pass",
                    side_just_seen=just_seen,
                    side_distance=self.side_seen_distance,
                )

        if self.zone_entered_at is not None:
            if now - self.zone_entered_at >= self.config.zone_timeout_s:
                return PassDecision(
                    complete=True,
                    reason="zone timeout",
                    side_just_seen=just_seen,
                    side_distance=self.side_seen_distance,
                    timed_out=True,
                )

        return PassDecision(
            complete=False,
            reason="waiting for side pass" if self.side_seen_at is None
            else "waiting out pass delay",
            side_just_seen=just_seen,
            side_distance=self.side_seen_distance,
        )

    def take_restore_lane(self) -> Optional[int]:
        """복귀할 차선을 반환하고 기억을 비운다. 없으면 None.

        한 번 소비하면 사라지므로, 같은 회피에 대해 두 번 복귀하지 않는다.
        """

        lane = self.lane_before_avoid
        self.lane_before_avoid = None
        return lane
