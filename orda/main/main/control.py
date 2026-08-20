# ─────────────────────────────────────────────────────────────────────────────
# control.py
#
# 역할: RaceFSM이 선택한 LANE_DRIVE/CONE_DRIVE 제어값 계산
#
# 모드별 제어 방식:
#   CONE_DRIVE  : 완만한 PD 제어 조향 + 신뢰도 기반 속도
#   LANE_DRIVE  : Pure Pursuit 조향 + 조향각 기반 속도 감속
#   FIXED_AVOID : PD 조향(회피 전용 이득) + 조향각 기반 속도 감속
#
# 상태 전이와 safety hold 판단은 이 모듈의 책임이 아니다. RaceFSM과
# ControlSelector가 최종 모드를 확정한 뒤 해당 제어 프로필만 호출한다.
# ─────────────────────────────────────────────────────────────────────────────

import math
from collections import namedtuple
from typing import Optional

from main.race_fsm import Mode

# ─────────────────────────────────────────────────────────────────────────────
# 파라미터 설정 구역
# 아래 값들을 수정하여 주행 특성을 조정한다.
# ─────────────────────────────────────────────────────────────────────────────

# PD 제어 파라미터: mode → (kp, kd, alpha)
#   kp    : 비례 이득 (오프셋에 비례한 조향)
#   kd    : 미분 이득 (오프셋 변화율에 비례한 감쇠)
#   alpha : 비선형 보정 계수 (0이면 선형 PD)
PD_PARAMS = {
    # LiDAR 경로 오프셋은 10 Hz로 갱신된다. 과한 이득은 프레임 사이의
    # 작은 경계 변화도 최대 조향으로 키우므로 라바콘에서는 완만하게 쓴다.
    Mode.CONE_DRIVE: (1.0, 0.0, 0.0),
    # 고정장애물 회피: 차선을 옮기는 동안 오프셋이 크게 벌어진다. 차선 주행과
    # 같은 이득이면 조향이 과해지므로 kp를 낮추고 kd(감쇠)를 키운다.
    # ★ 실차 튜닝 지점: 회피가 굼뜨면 kp를 올리고, 흔들리면 kd를 올린다.
    Mode.FIXED_AVOID: (0.12, 0.35, 0.0),
}

# 영상 BEV 좌표계 Pure Pursuit 파라미터. LANE_DRIVE 조향에 쓴다.
# /lane_offset을 차량 중심에서 목표 경로까지의 횡방향 거리 x로 사용하고,
# 차량 전방 lookahead_px 위치에 목표점 (x, lookahead_px)이 있다고 본다.
PURE_PURSUIT_PARAMS = {
    'lookahead_px':       120.0,
    'wheelbase_px':        30.0,
    'steering_gain':        1.0,
    'max_steering_angle': 40.0,
}

# 속도 제어 파라미터: mode → (max_speed, min_speed, scale_factor)
#   max_speed    : 최대 속도
#   min_speed    : 최소 속도 (조향각이 커도 이 속도 아래로 떨어지지 않음)
#   scale_factor : |조향각| × scale_factor 만큼 최대 속도에서 감속
SPEED_PARAMS = {
    Mode.LANE_DRIVE: (43.0, 16.0, 0.5),
    # 회피 중에는 낮은 속도로 안정적으로 옮겨간다. 빠르면 차선 변경이 끝나기
    # 전에 장애물에 도달한다.
    # ★ 실차 튜닝 지점: 회피가 늦으면 max_speed를 낮추고, 굼뜨면 올린다.
    Mode.FIXED_AVOID: (30.0, 14.0, 0.6),
}

# 라바콘 속도 제어 파라미터.
# main.py에서 최종 모터 속도는 이 값의 1/2로 발행된다. 조향각으로 속도를
# 크게 깎으면 S자 경로에서 ``느림-빠름``이 반복되므로, 정상적인 추정 상태는
# 좁은 속도 범위로 유지한다. 다만 큰 조향에서는 물리적인 언더스티어를 막기 위해
# 소폭만 감속하고, 경로를 실제로 잃었을 때 충분히 감속한다.
RUBBERCONE_SPEED_PARAMS = {
    'min_speed':              13.5,  # 경로 상실 시 하한 (최종 6.75)
    'cautious_speed':         18.0,  # 큰 조향·한쪽 경계 보수 상태 (최종 9.0)
    'cruise_speed':           22.0,  # 정상 추정 상태 (최종 11.0)
    'cautious_confidence':    35.0,  # 이 신뢰도부터 9.0까지 회복
    'full_speed_confidence':  85.0,  # 이 신뢰도부터 11.0 허용
    'turn_start_angle':       15.0,  # 이 조향각부터만 완만하게 감속
    # bag(cone_11) 측정: 조향 40°에서도 속도가 9.9로 거의 안 줄어 코너에서
    # 밀려났다. 기울기를 올리고, cautious_speed와 별개인 코너 전용 하한을 둔다.
    'turn_slowdown':          0.20,  # 큰 조향 시 감속 기울기 (40°에서 최종 8.5)
    'turn_min_speed':         14.0,  # 코너 감속 하한 (최종 7.0)
    'max_steering_angle':     45.0,  # 라바콘 구간 안전 조향 한계
}

# ─────────────────────────────────────────────────────────────────────────────

# 내부용 파라미터 구조체 정의
SpeedParams = namedtuple('SpeedParams', ['max_speed', 'min_speed', 'scale_factor'])
PDParams    = namedtuple('PDParams',    ['kp', 'kd', 'alpha'])


class Controller:
    def __init__(self):
        """
        Controller 초기화
        딕셔너리로 정의된 파라미터를 namedtuple로 변환하고 상태를 초기화한다.
        """
        # 제어 출력 상태
        self.angle             = 0.0  # 현재 조향각
        self.speed             = 0.0  # 현재 속도
        # 내부 제어 상태
        self.prev_offset       = 0.0  # 이전 오프셋 (PD 미분항 계산용)
        self.prev_mode: Optional[Mode] = None

        # PD / 속도 파라미터를 namedtuple로 변환
        self.pd_params = {
            mode: PDParams(*vals) for mode, vals in PD_PARAMS.items()
        }
        self.speed_params = {
            mode: SpeedParams(*vals) for mode, vals in SPEED_PARAMS.items()
        }

        self.rubbercone_speed_params = RUBBERCONE_SPEED_PARAMS.copy()
        self.pure_pursuit_params = PURE_PURSUIT_PARAMS.copy()

    def update(self, mode: Mode, offset: int, obstacle_dist: float,
               rubbercone_confidence: int = 100):
        """
        메인 업데이트: 모드에 따라 조향각과 속도를 계산한다.
        모드가 바뀌면 내부 상태를 리셋한다.
        """
        # 모드 전환 감지 → 내부 상태 초기화
        if mode != self.prev_mode:
            self.reset()
            self.prev_mode = mode

        if mode is Mode.CONE_DRIVE:
            # 라바콘: 조향 제한과 경로 신뢰도 기반의 좁은 속도 범위를 사용한다.
            self.angle = self._compute_steering_pd(mode, offset)
            max_angle = self.rubbercone_speed_params['max_steering_angle']
            self.angle = max(-max_angle, min(max_angle, self.angle))
            self.speed = self._compute_rubbercone_speed(
                self.angle, rubbercone_confidence)

        elif mode is Mode.LANE_DRIVE:
            # Pure Pursuit 조향 + 조향각 기반 속도 감속
            self.angle = self._compute_steering_pure_pursuit(offset)
            params     = self.speed_params.get(mode)
            self.speed = (
                self._compute_speed_from_angle(self.angle, params)
                if params else 0.0
            )

        elif mode is Mode.FIXED_AVOID:
            # PD 조향(회피 전용 이득) + 조향각 기반 속도 감속
            self.angle = self._compute_steering_pd(mode, offset)
            params     = self.speed_params.get(mode)
            self.speed = (
                self._compute_speed_from_angle(self.angle, params)
                if params else 0.0
            )

        else:
            # 정의되지 않은 모드: 안전 정지
            self.angle, self.speed = 0.0, 0.0

    def _compute_steering_pd(self, mode: Mode, offset: int) -> float:
        """
        PD 제어로 조향각을 계산한다.
          error         : 현재 오프셋 (차선 중심 기준 픽셀 편차)
          diff          : 오프셋 변화량 (미분항)
          effective_kp  : alpha에 의한 비선형 보정 적용 kp
        """
        params = self.pd_params.get(mode)
        if not params:
            return 0.0

        error = float(offset)
        diff  = error - self.prev_offset
        self.prev_offset = error

        # alpha > 0이면 오프셋이 클수록 kp를 증폭 (비선형 제어)
        effective_kp = params.kp * (1.0 + params.alpha * abs(error))
        return effective_kp * error + params.kd * diff

    def _compute_steering_pure_pursuit(self, offset: int) -> float:
        """BEV 목표점으로부터 Pure Pursuit 조향각을 계산한다.

        목표점은 차량 기준 ``(lateral=offset, forward=lookahead_px)``이다.
        반환값의 부호는 기존 PD와 동일하게 양의 offset에 양의 조향을 낸다.
        """
        params = self.pure_pursuit_params
        lookahead = max(1.0, float(params['lookahead_px']))
        # 이 입력은 별도의 경로점이 아니라 횡오차이므로 lookahead보다 큰 값은
        # 목표점 기하가 뒤집히지 않게 경계에 고정한다. 큰 이탈에서 조향이 다시
        # 작아지는 Pure Pursuit의 잘못된 적용을 방지한다.
        lateral = max(-lookahead, min(lookahead, float(offset)))
        wheelbase = max(0.0, float(params['wheelbase_px']))
        distance_squared = lateral * lateral + lookahead * lookahead
        steering_rad = math.atan2(
            2.0 * wheelbase * lateral,
            distance_squared,
        )
        steering_deg = (
            math.degrees(steering_rad) * float(params['steering_gain'])
        )
        limit = abs(float(params['max_steering_angle']))
        return max(-limit, min(limit, steering_deg))

    def _compute_speed_from_angle(
        self,
        angle: float,
        params: SpeedParams,
    ) -> float:
        """
        조향각에 따른 속도를 계산한다.
        각도가 클수록 속도가 감소하며, min_speed 이하로 떨어지지 않는다.
        """
        speed = params.max_speed - abs(angle) * params.scale_factor
        return max(params.min_speed, speed)

    def _compute_rubbercone_speed(self, angle: float, confidence: int) -> float:
        """
        라바콘 경로 신뢰도로 목표 속도를 계산한다.

        정상 추정 시 최종 약 9.0~11.0을 유지하고, 조향각이 커질수록
        최종 8.5(40°)~8.0(45°)까지 감속해 코너에서 밀려나는 것을 막는다.
        경계가 사라져 신뢰도가 낮아질 때는 최종 6.75까지 감속한다.
        """
        params = self.rubbercone_speed_params
        confidence_ratio = max(0.0, min(1.0, float(confidence) / 100.0))
        cautious_confidence = params['cautious_confidence'] / 100.0
        full_confidence = params['full_speed_confidence'] / 100.0

        if confidence_ratio <= cautious_confidence:
            progress = confidence_ratio / max(cautious_confidence, 1e-6)
            confidence_speed = (
                params['min_speed']
                + (params['cautious_speed'] - params['min_speed']) * progress
            )
        else:
            progress = (confidence_ratio - cautious_confidence) / max(
                full_confidence - cautious_confidence, 1e-6)
            confidence_speed = (
                params['cautious_speed']
                + (params['cruise_speed'] - params['cautious_speed'])
                * max(0.0, min(1.0, progress))
            )

        # 코너 감속은 cautious_speed가 아니라 turn_min_speed를 바닥으로 쓴다.
        # cautious_speed를 바닥으로 두면 기울기를 아무리 올려도 그 아래로
        # 내려가지 않아 깊은 코너에서 감속이 걸리지 않는다.
        turn_excess = max(0.0, abs(angle) - params['turn_start_angle'])
        turn_speed = max(
            params['turn_min_speed'],
            params['cruise_speed'] - turn_excess * params['turn_slowdown'],
        )
        return min(confidence_speed, turn_speed)

    def reset(self):
        """내부 제어 상태 초기화 (모드 전환 시 호출)"""
        self.prev_offset = 0.0

    def get_angle(self) -> float:
        """현재 계산된 조향각 반환"""
        return self.angle

    def get_speed(self) -> float:
        """현재 계산된 속도 반환"""
        return self.speed
