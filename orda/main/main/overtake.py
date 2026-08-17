"""Pure decision logic for interfering-vehicle overtake completion.

rclpy를 import 하지 않고 토픽도 직접 다루지 않으므로, 실차 없이 단위 테스트로
검증할 수 있다 (README: 노드 파일은 main.py 하나뿐이고 나머지는 순수 로직 모듈).

담당하는 판단은 하나다 — **방해차량이 옆으로 지나갔고, 지나간 지 충분히
되었는가.** 회피 방향 결정과 lane_target 변경(회피·복귀 모두)은
runtime_adapter._update_lane_action() 이 단독으로 담당하므로 여기서 다루지
않는다. 두 주체가 lane_target을 다투면 서로의 명령을 덮어써 진동한다.

차선 인코딩(README 규약): 0 = 중앙, 1 = 왼쪽(1차선), 2 = 오른쪽(2차선), -1 = 미확정
시각(now)은 초 단위 float이며 호출자가 하나의 클럭 도메인으로 통일해서 넘긴다.
"""

from dataclasses import dataclass
import math
from typing import Optional


@dataclass(frozen=True)
class OvertakeConfig:
    """추월 완료 판정 임계값."""

    # 측면 LiDAR가 이 거리 안이면 옆에 방해차량이 있다고 본다.
    #
    # 고정장애물 bag 6개(overtake/pass/stop × 2)에서 감시 측면의 최소거리를
    # 직접 재보면 통과 순간 0.26~0.34 m, 벽/빈 차선은 0.60 m 이상이다.
    # 0.40은 그 사이에 들어간다 (검출 여유 0.06 m, 벽 여유 0.20 m).
    #
    # 0.60이었을 때는 구간 진입 1초 만에 0.59 m로 걸려, 회피 차선 변경이
    # 끝나기도 전에 완료 카운트다운이 시작됐다. 0.40에서는 같은 bag에서
    # 인식이 1.32초로 밀리고 차선 변경 구간이 24 → 35 샘플로 늘었다.
    #
    # 주의: 예전 주석에 있던 "0.26~0.46 m"는 초음파를 쓰던 시절 다른 bag에서
    # 나온 값이라 지금 측면 LiDAR 구성에는 맞지 않는다. 이 값을 다시 만질
    # 때는 위 6개 bag으로 재측정할 것.
    side_detect_m: float = 0.40

    # 옆에서 한 번 인식한 장애물이 사라진 뒤, 이만큼 연속으로 비어 있어야
    # 추월 완료로 본다. 다시 검출되면 clear 타이머를 즉시 초기화한다.
    pass_delay_s: float = 2.0

    # 측면 통과 증거를 못 얻었을 때 운전자/로그에 이상을 알리는 상한.
    # 이 시간만으로 완료 처리하면 장애물 앞에서 차선 주행으로 돌아갈 수 있어
    # fail-closed로 경고만 하고 현재 미션을 유지한다.
    zone_timeout_s: float = 12.0

    def __post_init__(self) -> None:
        for name, value in (
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
    side_just_cleared: bool = False
    side_distance: float = float("inf")
    timed_out: bool = False


class OvertakeGuard:
    """Semantic side distances determine completion; lane_target stays untouched."""

    def __init__(self, config: Optional[OvertakeConfig] = None) -> None:
        self.config = config or OvertakeConfig()
        self.reset()

    def reset(self) -> None:
        """상태 초기화. bag 루프 감지 등에서 호출한다."""

        self.side_seen_at: Optional[float] = None
        self.clear_started_at: Optional[float] = None
        self.side_seen_distance: float = float("inf")
        self.zone_entered_at: Optional[float] = None
        self._timeout_reported = False

    def enter_zone(self, now: float) -> None:
        """장애물 구간에 진입할 때 종료 판정 상태를 초기화한다."""

        self.zone_entered_at = now
        self.side_seen_at = None
        self.clear_started_at = None
        self.side_seen_distance = float("inf")
        self._timeout_reported = False

    def invalidate_clearance(self) -> None:
        """Discard only the continuous-clear interval after semantic data loss.

        A stale clearance message does not erase evidence that the obstacle was seen,
        nor does it restart the zone timeout.  It does mean that elapsed wall
        time can no longer count as continuously observed clearance.
        """

        self.clear_started_at = None

    def zone_elapsed(self, now: float) -> Optional[float]:
        """구간 진입 후 경과 시간(초). 진입 기록이 없으면 None."""

        if self.zone_entered_at is None:
            return None
        return now - self.zone_entered_at

    def update_zone(
        self,
        *,
        now: float,
        lane_target: int,
        side_left: float,
        side_right: float,
        ego_lane: int = -1,
    ) -> PassDecision:
        """Perception-approved side distances determine overtake completion.

        회피로 차선을 옮겼으므로 방해차량은 현재 주행 차선의 반대편에 있다.
          - 1차선(왼쪽, 1) 주행 중 → 오른쪽(side_right)
          - 2차선(오른쪽, 2) 주행 중 → 왼쪽(side_left)

        어느 쪽을 볼지는 **실측 차선**(ego_lane, /lane_position)을 우선 쓰고,
        미확정(-1)이면 제어 목표(lane_target)로 폴백한다. lane_target은 명령이라
        차선 변경이 진행 중이거나 실패했을 때 실제 위치와 어긋나는데, 그 상태로
        반대편을 감시하면 옆으로 지나가는 차를 놓치고 완료 판정이 구간 시간
        상한으로 밀린다.

        초음파를 쓰지 않는 이유: bag(rosbag2_object1)에서 오른쪽 초음파가 전
        구간 4cm에 고착되어(전체 샘플의 98%) 차선을 바꾸자마자 "추월 완료"가
        나버렸고, 그 오판이 회피를 원위치로 되돌려 충돌로 이어졌다.
        """

        # 차선 규약: 0=중앙, 1=왼쪽(1차선), 2=오른쪽(2차선)
        # 왼쪽 차선에 있으면 피해온 장애물은 오른쪽에, 오른쪽 차선이면 왼쪽에 있다.
        # 중앙(0)이면 감시할 반대편이 정해지지 않으므로 판정하지 않는다.
        current_lane = ego_lane if ego_lane in (1, 2) else lane_target
        if current_lane == 1:
            side_distance = side_right
        elif current_lane == 2:
            side_distance = side_left
        else:
            side_distance = float("inf")
        just_seen = False
        just_cleared = False

        if side_distance < self.config.side_detect_m:
            if self.side_seen_at is None:
                self.side_seen_at = now
                self.side_seen_distance = side_distance
                just_seen = True
            else:
                self.side_seen_distance = min(
                    self.side_seen_distance,
                    side_distance,
                )
            self.clear_started_at = None
        elif self.side_seen_at is not None and self.clear_started_at is None:
            self.clear_started_at = now
            just_cleared = True

        if self.clear_started_at is not None:
            if now - self.clear_started_at + 1e-6 >= self.config.pass_delay_s:
                return PassDecision(
                    complete=True,
                    reason="overtake confirmed after continuous side clearance",
                    side_just_seen=just_seen,
                    side_just_cleared=just_cleared,
                    side_distance=self.side_seen_distance,
                )

        if self.zone_entered_at is not None:
            if now - self.zone_entered_at >= self.config.zone_timeout_s:
                report_timeout = not self._timeout_reported
                self._timeout_reported = True
                return PassDecision(
                    complete=False,
                    reason="zone timeout without a confirmed side-clear sequence",
                    side_just_seen=just_seen,
                    side_just_cleared=just_cleared,
                    side_distance=self.side_seen_distance,
                    timed_out=report_timeout,
                )

        return PassDecision(
            complete=False,
            reason=(
                "waiting for side obstacle"
                if self.side_seen_at is None
                else "waiting for obstacle to clear"
                if self.clear_started_at is None
                else "holding continuous side clearance"
            ),
            side_just_seen=just_seen,
            side_just_cleared=just_cleared,
            side_distance=self.side_seen_distance,
        )
