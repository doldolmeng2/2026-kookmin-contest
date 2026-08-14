# ─────────────────────────────────────────────────────────────────────────────
# control.py
#
# 역할: RaceFSM이 선택한 LANE_DRIVE/CONE_DRIVE 제어값 계산
#
# 상태 전이와 STOP 판단은 이 모듈의 책임이 아니다. RaceFSM과
# ControlSelector가 최종 모드를 확정한 뒤 해당 제어 프로필만 호출한다.
# ─────────────────────────────────────────────────────────────────────────────

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
    Mode.LANE_DRIVE: (0.145, 0.3, 0.0),
    # 고정장애물 회피: 차선을 옮기는 동안 오프셋이 크게 벌어진다. 차선 주행과
    # 같은 이득이면 조향이 과해지므로 kp를 낮추고 kd(감쇠)를 키운다.
    # ★ 실차 튜닝 지점: 회피가 굼뜨면 kp를 올리고, 흔들리면 kd를 올린다.
    Mode.FIXED_AVOID: (0.12, 0.35, 0.0),
}

# 속도 제어 파라미터: mode → (max_speed, min_speed, scale_factor)
#   max_speed    : 최대 속도
#   min_speed    : 최소 속도 (조향각이 커도 이 속도 아래로 떨어지지 않음)
#   scale_factor : |조향각| × scale_factor 만큼 최대 속도에서 감속
SPEED_PARAMS = {
    Mode.LANE_DRIVE: (43.0, 16.0, 0.5),
}

# 고정장애물 속도 제어 파라미터. CONE_DRIVE와 같은 구조로, 조향각뿐 아니라
# **장애물이 얼마나 가까운지**를 함께 본다. 최종 모터 속도는 main.py에서 이
# 값의 1/2로 발행된다.
#
# 근접도는 YOLO 박스 면적으로 잰다. 면적은 거리의 대용값이고(대략 거리 제곱에
# 반비례), 절대 거리로 환산하지 않는 이유는 실측에서 환산 상수가 일정하지
# 않았기 때문이다 — 같은 bag에서 box·d²가 600~1200 사이로 흔들렸다. LiDAR
# 전방 거리를 쓰지 않는 이유는 rosbag2_fixed_obstacles_overtake_1 실측에서
# 전방 클러스터가 103 스캔 중 1번만 형성됐기 때문이다.
FIXED_AVOID_SPEED_PARAMS = {
    'cruise_speed':     30.0,  # 장애물이 멀 때 (최종 15.0)
    'near_speed':       14.0,  # 근접 하한 (최종 7.0)
    # 이 면적부터 감속을 시작한다. 구간 진입 임계값(FIXED_ENTRY_BOX_PX = 1900)과
    # 같은 수준이라, 구간에 들어서는 시점부터 이미 속도가 떨어진다.
    'slow_box_px':    2000.0,
    # 이 면적에서 near_speed에 도달한다. stop bag 실측(같은 차선으로 판정된
    # 프레임의 box_px 분포)은 최소 432 / 중앙 15040 / 최대 22885 였고, box
    # 22686 px²인 프레임의 LiDAR min_dist가 0.23 m였다. 2만대는 접촉 직전이라
    # 중앙값보다 앞에서 최저 속도에 닿도록 12000을 쓴다.
    'near_box_px':   12000.0,
    # 같은 차선에 장애물이 이만큼 크게 남아 있으면 정지한다. 회피에 성공해
    # 차선을 옮기면 same_lane이 False가 되어 곧바로 풀린다.
    'stop_box_px':   12000.0,
    'turn_start_angle': 10.0,  # 이 조향각부터 감속
    'turn_slowdown':     0.6,  # 큰 조향 시 감속 기울기
    'turn_min_speed':   14.0,  # 조향 감속 하한 (최종 7.0)
    # 박스를 놓친 프레임에서 마지막 판정을 유지하는 시간.
    #
    # 카메라가 매 프레임 박스를 주지는 않는다. 놓친 프레임을 곧바로 "장애물
    # 없음"으로 읽으면 속도가 순항으로 튀어 올랐다가 다음 검출에서 다시
    # 떨어진다. YOLO 갱신 주기 실측이 평균 0.30초(p10 0.15초)라 1.0초면
    # 서너 프레임의 공백을 덮는다. 이 시간이 지나면 순항으로 돌아간다.
    'box_hold_s':        1.0,
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
        # 마지막으로 본 박스와 그 시각 / 같은 차선 판정 (box_hold_s 동안 유지)
        self.held_box_px       = 0.0
        self.held_same_lane    = False
        self.held_box_at: Optional[float] = None

        # PD / 속도 파라미터를 namedtuple로 변환
        self.pd_params = {
            mode: PDParams(*vals) for mode, vals in PD_PARAMS.items()
        }
        self.speed_params = {
            mode: SpeedParams(*vals) for mode, vals in SPEED_PARAMS.items()
        }

        self.rubbercone_speed_params = RUBBERCONE_SPEED_PARAMS.copy()
        self.fixed_avoid_speed_params = FIXED_AVOID_SPEED_PARAMS.copy()

    def update(self, mode: Mode, offset: int,
               rubbercone_confidence: int = 100,
               box_px: float = 0.0, same_lane: bool = False,
               now: Optional[float] = None):
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

        elif mode is Mode.FIXED_AVOID:
            # 고정장애물: 조향각 + 장애물 근접도로 감속한다.
            self.angle = self._compute_steering_pd(mode, offset)
            self.speed = self._compute_fixed_avoid_speed(
                self.angle, box_px, same_lane, now)

        elif mode is Mode.LANE_DRIVE:
            # PD 조향 + 조향각 기반 속도 감속
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

    def _compute_fixed_avoid_speed(
        self,
        angle: float,
        box_px: float,
        same_lane: bool,
        now: Optional[float] = None,
    ) -> float:
        """장애물 근접도와 조향각으로 회피 구간 속도를 만든다.

        근접 감속과 조향 감속 중 **더 느린 쪽**을 쓴다. 같은 차선에 장애물이
        너무 크게 남아 있으면(피하지 못한 상태) 0을 돌려 정지시킨다.

        박스가 없는 프레임에서는 마지막 판정을 box_hold_s 동안 유지한다.
        ``now`` 를 넘기지 않으면 유지 없이 이번 프레임 값만 쓴다.
        """
        params = self.fixed_avoid_speed_params
        box = float(box_px) if box_px and box_px > 0.0 else 0.0

        valid_now = (
            now is not None
            and not isinstance(now, bool)
            and isinstance(now, (int, float))
        )
        if box > 0.0:
            self.held_box_px = box
            self.held_same_lane = same_lane
            self.held_box_at = now if valid_now else None
        elif (
            valid_now
            and self.held_box_at is not None
            and now - self.held_box_at <= params['box_hold_s']
        ):
            # 카메라가 잠깐 놓친 프레임. 마지막 판정을 그대로 쓴다.
            box = self.held_box_px
            same_lane = self.held_same_lane

        if same_lane and box >= params['stop_box_px']:
            return 0.0

        if box <= params['slow_box_px']:
            near_speed = params['cruise_speed']
        elif box >= params['near_box_px']:
            near_speed = params['near_speed']
        else:
            remaining = (params['near_box_px'] - box) / (
                params['near_box_px'] - params['slow_box_px'])
            near_speed = (
                params['near_speed']
                + (params['cruise_speed'] - params['near_speed']) * remaining
            )

        turn_excess = max(0.0, abs(angle) - params['turn_start_angle'])
        turn_speed = max(
            params['turn_min_speed'],
            params['cruise_speed'] - turn_excess * params['turn_slowdown'],
        )
        return min(near_speed, turn_speed)

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
        self.held_box_px = 0.0
        self.held_same_lane = False
        self.held_box_at = None

    def get_angle(self) -> float:
        """현재 계산된 조향각 반환"""
        return self.angle

    def get_speed(self) -> float:
        """현재 계산된 속도 반환"""
        return self.speed
