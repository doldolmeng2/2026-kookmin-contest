# ─────────────────────────────────────────────────────────────────────────────
# control.py
#
# 역할: RaceFSM이 선택한 LANE_DRIVE/CONE_DRIVE 제어값 계산
#
# 모드별 제어 방식:
#   CONE_DRIVE  : 완만한 PD 제어 조향 + 신뢰도 기반 속도
#   LANE_DRIVE  : Pure Pursuit 조향 + 조향각 기반 속도 감속
#                 PD 조향은 지우지 않고 주석으로 남겨 두었다. 되돌리는 방법은
#                 update()의 LANE_DRIVE 분기 주석을 참고한다.
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
    # ── [PD 롤백 테스트용 보관] LANE_DRIVE PD 이득 ────────────────────
    # 현재 LANE_DRIVE는 Pure Pursuit 조향을 쓰므로 이 이득은 쓰이지 않는다.
    # doldolmeng2/2025-kookmin-contest 의 modular/main/main/control.py 에
    # 있던 이득 (0.145, 0.3, 0.0) 을 실차에서 0.105로 낮춰 둔 값이다.
    # PD로 다시 바꿀 때는 아래 한 줄과 update()의 PD 줄을 함께 살린다.
    # Mode.LANE_DRIVE: (0.105, 0.3, 0.0),
    # ──────────────────────────────────────────────────────────────────
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
    'lookahead_px':       180.0,  #늘리면 조향이 완만해지고, 줄이면 조향이 날카로워진다.
    'wheelbase_px':        50.0,  #코너에서 5조향이 모자라면 늘리면 됨.
    'steering_gain':       2.1,  #전체적인 조향이 약할 때 늘리면 됨.
    'max_steering_angle': 100.0,
}

# 가드레일(바깥 실선 반발) 파라미터. LANE_DRIVE 조향에만 더해진다.
#
# 왜 필요한가: Pure Pursuit 은 오프셋이 아무리 커져도 넘을 수 없는 조향 상한이
# 있다. atan2(2·Lw·x, x²+Ld²) 가 x=Ld 에서 꼭짓점을 찍고 그 뒤로는 오히려 줄어드는데,
# _compute_steering_pure_pursuit 의 lateral clamp 가 정확히 그 꼭짓점(lookahead_px)에
# 걸려 있기 때문이다. 그래서 상한은 항상
#     degrees(atan(2·wheelbase_px / (2·lookahead_px))) × steering_gain
# 이고, max_steering_angle=100 은 한 번도 걸리지 않는다. 위 파라미터를 바꾸면 이
# 값도 바뀐다 — 260/61/2.95 일 때 39.0, 180/50/2.1 일 때 32.6 이다. 어느 쪽이든
# xycar 풀락(100)의 3분의 1 수준이라, 중앙선만 보면 코너에서 더 꺾을 방법이 없다.
# (테스트는 이 값을 하드코딩하지 않고 매번 계산해 확인한다.)
#
# bag 실측(carlane1+carlane2, 3601프레임. 측정 당시 PP 상한은 38.9 였다):
#   - 사람은 전체 프레임의 17.1% 에서 38.9 를 넘겨 조향했다.
#   - 바깥 실선까지의 여유가 190px 아래면 절반 이상의 프레임이 38.9 초과를
#     요구했고, 190 이상이면 2.6~10% 로 떨어진다. → margin_px = 190
#   - 여유<190 이면서 사람이 거의 조향하지 않은 259프레임 중 258개가 ±0.3초
#     안에 풀락 조향과 붙어 있었다(조종 펄스 사이의 빈 구간). 헛발동은 1프레임.
#
# ★ 실차 튜닝 지점: gain_deg. bag 의 조향은 RC 조종기 bang-bang 입력이라
#   (0~5 에 72.3%, 70~101 에 14.2%, 그 사이는 1.8%) 이득을 맞출 근거가 못 된다.
#   "언제" 꺾어야 하는지(margin_px)만 실측에서 가져오고, "얼마나"는 0부터 올린다.
GUARDRAIL_PARAMS = {
    'margin_px':   190.0,  # 이 여유(BEV px) 아래부터 반발 시작. 0이면 기능 정지
    'gain_deg':     45.0,  # 여유 0일 때의 반발 조향 (선형 램프)
    'max_deg':      45.0,  # 반발항 자체의 상한
    'rate_deg':      8.0,  # 프레임당 변화 제한 (여유 감소 p95 13px → 3.1°/frame)
    'min_trust_px':  15.0,  # 이보다 가까우면 좌우 판정을 못 믿는다 (아래 참고)
    'hold_frames':     5,  # 레일을 놓쳐도 직전 값을 유지할 프레임 수
    'decay_frames':   10,  # 그 뒤 0까지 선형 감쇠할 프레임 수
}

# min_trust_px 가 왜 필요한가 (carlane1 bag, t=38.4~38.6s 에서 발견):
#   차가 실선 위에 올라타거나 넘어가면 같은 선이 반대쪽에서 보이기 시작한다.
#   실제로 그 구간에서 여유가 (좌 미관측, 우 1px) 에서 (좌 1px, 우 미관측) 으로
#   프레임 사이에 뒤집혔다. 그 상태로 "가까운 쪽에서 밀어낸다"를 적용하면
#   이미 트랙 밖으로 나간 차를 더 밖으로 민다 — 실측에서 오프셋 +235(우조향
#   필요)인데 가드레일이 -44.8 을 더해 복귀를 상쇄하고 있었다.
#   여유가 이 값보다 작으면 좌우 판정을 포기하고 미관측으로 취급한다. 그러면
#   직전 값을 잠깐 유지했다가 감쇠하므로, 부호가 뒤집히는 대신 조향이 매끄럽게
#   풀린다. 램프상 여유 20px 이면 이미 이득의 89% 라 잃는 것도 거의 없다.

# 속도 제어 파라미터: mode → (max_speed, min_speed, scale_factor)
#   max_speed    : 최대 속도
#   min_speed    : 최소 속도 (조향각이 커도 이 속도 아래로 떨어지지 않음)
#   scale_factor : |조향각| × scale_factor 만큼 최대 속도에서 감속
SPEED_PARAMS = {
    Mode.LANE_DRIVE: (31.0, 12.0, 0.5),
    # PD 비교 주행 전, Pure Pursuit 구성에서 쓰던 속도 프로필은 아래와 같다.
    # 지금은 낮은 속도대(31/12)를 그대로 유지한다. 예전 속도로 되돌리려면
    # 위 줄을 주석 처리하고 아래 줄을 살린다.
    # Mode.LANE_DRIVE: (43.0, 25.0, 0.5),
    # 회피 중에는 낮은 속도로 안정적으로 옮겨간다. 빠르면 차선 변경이 끝나기
    # 전에 장애물에 도달한다.
    # ★ 실차 튜닝 지점: 회피가 늦으면 max_speed를 낮추고, 굼뜨면 올린다.
    Mode.FIXED_AVOID: (25.0, 14.0, 0.6),
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
        # 가드레일 상태 (변화율 제한과 레일 상실 시 감쇠에 쓴다)
        self.guardrail_angle          = 0.0
        self.guardrail_missing_frames = 0

        # PD / 속도 파라미터를 namedtuple로 변환
        self.pd_params = {
            mode: PDParams(*vals) for mode, vals in PD_PARAMS.items()
        }
        self.speed_params = {
            mode: SpeedParams(*vals) for mode, vals in SPEED_PARAMS.items()
        }

        self.rubbercone_speed_params = RUBBERCONE_SPEED_PARAMS.copy()
        self.pure_pursuit_params = PURE_PURSUIT_PARAMS.copy()
        self.guardrail_params = GUARDRAIL_PARAMS.copy()

    def update(self, mode: Mode, offset: int, obstacle_dist: float,
               rubbercone_confidence: int = 100,
               guardrail: Optional[tuple[float, float]] = None):
        """
        메인 업데이트: 모드에 따라 조향각과 속도를 계산한다.
        모드가 바뀌면 내부 상태를 리셋한다.

        guardrail: /lane_guardrail 의 (좌 여유, 우 여유). BEV px, 음수는 미관측.
                   None이면 가드레일 항을 쓰지 않는다(호출자가 모드/신선도로 막음).
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
            # ── 조향 방식 선택 ────────────────────────────────────────
            # Pure Pursuit 조향 + 조향각 기반 속도 감속 (현재 구성)
            # PD 비교 주행으로 되돌리려면 아래 PD 줄과 PD_PARAMS의
            # Mode.LANE_DRIVE 이득 줄을 함께 살리고 Pure Pursuit 줄을 주석 처리한다.
            pursuit_angle = self._compute_steering_pure_pursuit(offset)
            # pursuit_angle = self._compute_steering_pd(mode, offset)
            # ──────────────────────────────────────────────────────────
            # 가드레일 항은 Pure Pursuit "뒤에" 더한다. 앞에서 offset을 건드리면
            # lateral clamp와 atan2 포화를 다시 통과해 효과가 잘려 나간다.
            limit = abs(float(self.pure_pursuit_params['max_steering_angle']))
            self.angle = max(-limit, min(
                limit, pursuit_angle + self._compute_guardrail(guardrail)))
            params     = self.speed_params.get(mode)
            # 속도는 가드레일까지 더한 최종 각도로 계산한다. 그래야 "많이 꺾으면
            # 감속" 이라는 기존 결합이 가드레일에도 그대로 적용된다.
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

    def _compute_guardrail(
        self,
        guardrail: Optional[tuple[float, float]],
    ) -> float:
        """바깥 실선에 가까워진 만큼 조향을 밀어내는 반발항을 계산한다.

        좌/우 중 **가까운 쪽만** 쓴다. 그래서 몇 차선에 있는지 알 필요가 없다 —
        2차선이면 왼쪽 실선이 반대 차선 너머라 자연히 멀고, 1차선이면 그 반대다
        (bag 실측: 선택된 클래스가 주행 차선과 98% 일치).

        부호는 /lane_offset 규약과 같다. 오른쪽 실선이 가까우면 좌조향(음),
        왼쪽이 가까우면 우조향(양).
        """
        params = self.guardrail_params
        threshold = float(params['margin_px'])
        if threshold <= 0.0:
            # 기능 정지. 이 경로에서는 조향이 Pure Pursuit 단독과 완전히 같다.
            self.guardrail_angle = 0.0
            self.guardrail_missing_frames = 0
            return 0.0

        target = self._guardrail_target(guardrail, threshold, params)
        if target is None:
            # 레일을 놓쳤다. 곧바로 0으로 떨어뜨리면 코너 한복판에서 조향이 툭
            # 빠지므로, 잠시 유지했다가 서서히 감쇠시킨다.
            self.guardrail_missing_frames += 1
            hold = int(params['hold_frames'])
            decay = max(1, int(params['decay_frames']))
            if self.guardrail_missing_frames <= hold:
                return self.guardrail_angle
            elapsed = self.guardrail_missing_frames - hold
            if elapsed >= decay:
                self.guardrail_angle = 0.0
            else:
                self.guardrail_angle *= 1.0 - (elapsed / decay)
            return self.guardrail_angle

        self.guardrail_missing_frames = 0
        # 변화율 제한: 레일이 갑자기 나타나거나 사라져도 조향이 튀지 않는다.
        rate = abs(float(params['rate_deg']))
        delta = max(-rate, min(rate, target - self.guardrail_angle))
        self.guardrail_angle += delta
        return self.guardrail_angle

    def _guardrail_target(
        self,
        guardrail: Optional[tuple[float, float]],
        threshold: float,
        params: dict,
    ) -> Optional[float]:
        """관측된 여유로부터 이번 프레임의 목표 반발각을 낸다 (없으면 None)."""
        if guardrail is None:
            return None
        left, right = float(guardrail[0]), float(guardrail[1])

        # 음수는 미관측이다. min_trust_px 보다 가까운 값도 좌우 판정을 못 믿으므로
        # 같이 버린다(위 주석 참고). 둘 다 없으면 근거가 없다.
        floor = float(params['min_trust_px'])
        candidates = []
        if left >= floor:
            candidates.append((left, 1.0))    # 왼쪽이 가까움 → 우조향(양)
        if right >= floor:
            candidates.append((right, -1.0))  # 오른쪽이 가까움 → 좌조향(음)
        if not candidates:
            return None

        margin, direction = min(candidates)
        if margin >= threshold:
            # 여유가 충분하다. 항이 정확히 0이므로 직선 주행 거동과 지터는
            # 가드레일을 켜기 전과 완전히 같다.
            return 0.0

        # 선형 램프. 2차 램프는 실제 관측 구간(100~190px)에서 이득의 0.10~0.22
        # 밖에 못 내보내 너무 물렁했다.
        ratio = (threshold - margin) / threshold
        raw = float(params['gain_deg']) * ratio
        cap = abs(float(params['max_deg']))
        return direction * min(raw, cap)

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
        self.guardrail_angle = 0.0
        self.guardrail_missing_frames = 0

    def get_angle(self) -> float:
        """현재 계산된 조향각 반환"""
        return self.angle

    def get_speed(self) -> float:
        """현재 계산된 속도 반환"""
        return self.speed
