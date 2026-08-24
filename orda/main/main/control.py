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
from collections import deque, namedtuple
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
#
# 두 값은 서로 다른 것을 정한다. 하나만 보고 고치면 반드시 다른 하나가 망가진다:
#   상한       = degrees(atan(wheelbase_px / lookahead_px)) x steering_gain
#   중앙 기울기 = degrees(2 x wheelbase_px / lookahead_px^2) x steering_gain
#   clamp     = lookahead_px + offset_deadband_px
#               (이 값을 넘는 offset 은 전부 같은 조향을 낸다. 데드밴드가 입력을
#                먼저 당기므로 평평해지는 지점도 그만큼 뒤로 밀린다)
# 그래서 비(wheelbase/lookahead)가 상한을, 스케일이 해상도와 지터를 정한다.
#
# 실차 계측(main/tools/curve_diag.py, 곡선 구간 20초 397프레임):
#   - /lane_offset 19.8 Hz, valid 100%, 이 코너에서 -180px (IQR 3px)
#   - 조향 명령 단위는 100 = 풀락. ROS1 xycar_motor.py 가 좌조향을 -50 에서
#     자르므로(우조향은 +100) 상한은 50 을 넘기면 안 된다.
#   - 이전 300/35 는 상한 6.65 라 그 코너에서 5.9 밖에 못 냈다.
#   - 180/180 은 비는 맞지만(상한 45) clamp 가 정확히 180px 이라 측정된 코너가
#     clamp 경계에 얹힌다. offset 90px 에서 이미 38.7 이 나와 완만한 코너와
#     급코너를 구분하지 못한다.
# 그래서 비는 그대로 두고 스케일만 3.1배 키웠다. 상한 45 는 유지되고
# offset 90/180/300/450px 이 각각 17/30/40/44 로 벌어진다. 측정된 IQR 3px 가
# 만드는 조향 흔들림은 0.34(풀락 100 기준)라 무시할 수 있다.
PURE_PURSUIT_PARAMS = {
    'lookahead_px':       630.0,  #늘리면 조향이 완만해지고, 줄이면 조향이 날카로워진다.
    'wheelbase_px':       250.0,  #코너에서 조향이 모자라면 늘리면 됨(상한이 올라간다).
    'steering_gain':       1.0,  #전체적인 조향이 약할 때 늘리면 됨.
    'max_steering_angle': 100.0,
    # 아래 둘은 지터 전용이다. 기본 0 이라 켜기 전에는 조향이 위 세 값만으로
    # 정해지던 때와 정확히 같다. 자세한 설명은 STEERING_FILTER_PARAMS 참고.
    'offset_deadband_px':   0.0,
    'offset_lpf_tau_s':     0.0,
    'speed_angle_slew_per_s': 0.0,
    'offset_median_frames':   3,
    'lookahead_corner_px':  315.0,
    'adaptive_offset_start_px': 100.0,
    'adaptive_offset_end_px':   250.0,
}


# main_node 제어 타이머 주기. main.py 의 create_timer(0.02, control_cycle) 와
# 같아야 offset_lpf_tau_s 가 실제 초 단위로 동작한다. 한쪽만 바꾸면 tau 의
# 의미가 조용히 어긋나므로 test_control 이 두 값이 같은지 확인한다.
CONTROL_PERIOD_S = 0.02

# 지터 억제 파라미터. lookahead_px/wheelbase_px 와는 목적이 다르다.
#
# 왜 별도로 필요한가: Pure Pursuit 곡선은 단조 오목한 곡선 하나라, 직선의
# 감도(중앙 기울기)와 코너의 권한(상한)이 같은 곡선 위에 얹혀 있다. 비를
# 유지한 채 스케일만 키우면 둘이 같이 줄어든다 — 566→1132 로 키우면 지터는
# 0.2025→0.1012 로 반이 되지만 offset 180px 조향도 30.0→17.2 로 반이 된다.
# 스케일로는 분리가 안 된다.
#
# 아래 둘은 곡선의 모양을 바꾸지 않고 "무엇을 입력으로 줄지"만 고른다:
#   offset_deadband_px : 크기로 자른다. |offset| 이 이 값 이하면 0 으로 본다.
#                        직선의 미세 떨림만 죽고 코너는 거의 그대로다
#                        (d=12 일 때 offset 180 조향은 30.01 → 28.62).
#   offset_lpf_tau_s   : 속도로 자른다. 1차 저역통과의 시정수(초).
#                        50 Hz 기준 tau=0.10 이면 노이즈가 약 절반이 되고
#                        코너 반응은 100 ms 늦는다.
# 성격이 달라 같이 쓴다. 계속 작게 떠는 것은 데드밴드가, 가끔 크게 한 번
# 튀는 오검출은 저역통과가 잡는다.
#
# ★ 값은 반드시 직선 구간 계측에서 정한다. 직선에서는 차가 차선 중앙에 있으면
#   offset 이 0 이어야 하므로, main/tools/curve_diag.py 가 찍는 offset 산포가 곧
#   노이즈 진폭이다. 커브 데이터로는 못 정한다 — 거기서는 큰 offset 이 정상이라
#   노이즈와 진짜 신호가 구분되지 않는다. d 는 그 산포의 절반쯤에서 시작한다.
#
# speed_angle_slew_per_s 는 앞의 둘과 목적이 다르다. 조향이 아니라 **속도**를
# 지킨다. 속도는 max_speed - |조향각| x scale_factor 로 깎이므로, 차선을 놓쳤다
# 되찾을 때 생기는 계단 점프가 조향과 속도를 동시에 때린다.
#   실측(curve_1503 / nofallback_1514): 조향 프레임간 변화 중앙값 0.10 은 속도를
#   0.05 밖에 못 흔드는데(미세 지터는 무해하다), 계단 점프에서는 속도가 한
#   프레임에 5.9~13.7 튀었다. 속도 범위가 10~20 이라 20 에서 하한 10 으로
#   곤두박질치는 크기다.
# 그래서 서보로 나가는 조향은 그대로 두고, 속도 계산에 쓰는 각도만 초당 변화를
# 제한한다. 조향 응답은 1 도 느려지지 않는다.
#
# ★ 위험: 이 제한은 감속도 같이 늦춘다. 진짜 코너 진입에서 감속이 최대
#   (조향 변화량 / slew) 초 늦는다 — 50/s 면 20 만큼의 변화가 0.4 초다. 진짜
#   코너는 조향이 1 초에 걸쳐 쌓이므로 거의 영향이 없고 한 프레임에 튀는 가짜
#   점프만 눌리지만, 너무 낮게 잡으면 진짜 코너까지 늦어진다. min_speed 하한,
#   가드레일, 곡률 기반 속도 상한은 그대로 살아 있다.
#
# offset_median_frames — 지터의 실제 원인을 치는 항목이다.
#   실측(nodeadband_1556): 조향이 20 이상 왕복한 14회가 **100%** offset 50px+
#   점프와 0.3초 안에 동반했다. 정상 구간의 흔들림은 조향 진폭 0.9(풀락의 1%)에
#   불과해 차를 흔들 수 없다. 즉 지터는 이득 문제가 아니라 입력이 튀는 문제다.
#   그래서 lookahead 를 올려도 안 잡힌다 — 이득을 낮추면 상한도 같이 낮아져
#   스윙 비율이 그대로다(실측 확인).
#   실제 Controller 로 같은 bag 을 통과시키면 3프레임 중앙값이 20 이상 왕복을
#   18회 -> 10회로 줄이고 코너 조향은 그대로였다. 대가는 1프레임(약 42ms).
#   데드밴드는 죽은 구간을 만들어 리밋 사이클을 낳고, 저역통과는 모든 변화를
#   지연시킨다. 중앙값은 튀었다 돌아오는 값만 지우고 지속되는 변화는 통과시킨다.
#
# lookahead_corner_px / adaptive_offset_* — 코너 권한을 여는 항목이다.
#   |offset| 이 클수록 코너가 깊다는 것을 곡률 대용으로 쓴다. /lane_path_preview
#   의 진짜 곡률은 27.8% 프레임만 유효해서, 그걸로 lookahead 를 흔들면 나머지
#   72%에서 값이 튀며 새 진동원이 된다. |offset| 은 100% 가용하다.
#   직선에서는 lookahead_px(완만, 지터 적음), 코너에서는 lookahead_corner_px
#   (상한 높음)로 선형 전환한다. 계단 전환은 문턱에서 조향이 도약하므로
#   (실측: 문턱 150 에서 +21.7) start~end 구간에 걸쳐 섞는다.
#   상한은 atan(wheelbase_px / lookahead_corner_px) 이다. wheelbase_px 를
#   바꾸면 이 값도 같이 봐야 코너 권한이 조용히 줄지 않는다.
STEERING_FILTER_PARAMS = {
    'offset_deadband_px': PURE_PURSUIT_PARAMS['offset_deadband_px'],
    'offset_lpf_tau_s':   PURE_PURSUIT_PARAMS['offset_lpf_tau_s'],
    'speed_angle_slew_per_s':
        PURE_PURSUIT_PARAMS['speed_angle_slew_per_s'],
    'offset_median_frames':
        PURE_PURSUIT_PARAMS['offset_median_frames'],
    'lookahead_corner_px':
        PURE_PURSUIT_PARAMS['lookahead_corner_px'],
    'adaptive_offset_start_px':
        PURE_PURSUIT_PARAMS['adaptive_offset_start_px'],
    'adaptive_offset_end_px':
        PURE_PURSUIT_PARAMS['adaptive_offset_end_px'],
}
STEERING_FILTER_TUNABLES = tuple(STEERING_FILTER_PARAMS)

# 둘 다 음수면 의미가 깨진다. 데드밴드가 음수면 작은 offset 을 오히려 키우고,
# tau 가 음수면 alpha 가 1 을 넘어 필터가 발산한다. 0 은 "기능 정지"라 허용한다.
STEERING_FILTER_PARAM_SPEC = {
    'offset_deadband_px': (float, 0.0),
    'offset_lpf_tau_s':   (float, 0.0),
    'speed_angle_slew_per_s': (float, 0.0),
    # 1 이면 자기 자신이라 필터가 없는 것과 같다. 0 은 창이 비어 의미가 깨진다.
    'offset_median_frames': (int, 1),
    'lookahead_corner_px': (float, 0.0),
    'adaptive_offset_start_px': (float, 0.0),
    'adaptive_offset_end_px': (float, 0.0),
}
STEERING_FILTER_PARAM_HELP = {
    'offset_deadband_px':
        '이 값 이하의 |offset| 은 0 으로 본다(직선 지터 제거). 0 이면 정지',
    'offset_lpf_tau_s':
        'offset 1차 저역통과 시정수(초). 클수록 부드럽고 코너가 늦다. 0 이면 정지',
    'speed_angle_slew_per_s':
        '속도 계산에 쓰는 조향각의 초당 변화 한계. 서보로 나가는 조향은 '
        '건드리지 않는다. 낮출수록 속도가 부드럽고 감속이 늦다. 0 이면 정지',
    'offset_median_frames':
        'offset 중앙값 필터 창(프레임). 튀었다 돌아오는 점프만 지운다. '
        '1 이면 정지',
    'lookahead_corner_px':
        '코너에서 쓸 lookahead_px. 작을수록 상한이 높아 더 꺾는다. 0 이면 '
        '적응형 정지(항상 lookahead_px)',
    'adaptive_offset_start_px':
        '이 |offset| 부터 lookahead 를 코너값 쪽으로 섞기 시작한다',
    'adaptive_offset_end_px':
        '이 |offset| 에서 완전히 lookahead_corner_px 가 된다',
}

# 먼 BEV 경로점 선행 조향 파라미터. lane_path_preview 가 None 이거나 enabled 가
# False 면 아래 로직을 전혀 타지 않아 기존 Pure Pursuit 결과와 정확히 같다.
#
# target_offset_px 는 현재 /lane_offset 보다 먼 지점의 경로 중심이다. 먼 점 하나가
# 튀었을 때 조향이 급변하지 않도록 현재 offset 과의 차이를 먼저 제한하고, 그
# 목표점의 Pure Pursuit 결과를 신뢰도만큼 보수적으로 섞는다. curvature_norm 은
# 좌우 부호 정의가 경로점과 어긋날 가능성이 있어 조향 부호에는 쓰지 않고 곡선
# 감속 강도를 보강하는 용도로만 쓴다.
LANE_PATH_PREVIEW_PARAMS = {
    'enabled':                 True,
    'max_target_delta_px':     90.0,
    'steering_blend':           0.45,
    'curvature_norm_cap':       1.0,
    'speed_slowdown_start_px': 20.0,
    'max_speed_reduction':      8.0,
    'curvature_speed_weight':   0.35,
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
    'margin_px':   250.0,  # 이 여유(BEV px) 아래부터 반발 시작. 0이면 기능 정지
    'gain_deg':     60.0,  # 여유 0일 때의 반발 조향 (선형 램프)
    'max_deg':      55.0,  # 반발항 자체의 상한
    'rate_deg':      20.0,  # 프레임당 변화 제한 (여유 감소 p95 13px → 3.1°/frame)
    'min_trust_px':  15.0,  # 이보다 가까우면 좌우 판정을 못 믿는다 (아래 참고)
    'hold_frames':     5,  # 레일을 놓쳐도 직전 값을 유지할 프레임 수
    'decay_frames':   10,  # 그 뒤 0까지 선형 감쇠할 프레임 수
}

# 위 항목은 전부 main_node 의 ROS 파라미터로도 열려 있다. 이름은 앞에
# 'guardrail_' 을 붙인 것이고(gain_deg → guardrail_gain_deg), 검증은
# validate_guardrail_params 하나가 launch 시점과 ros2 param set 양쪽에서
# 똑같이 돈다. 그래서 실차에서 이득을 찾는 동안은 재빌드도 재실행도 필요 없다:
#     ros2 param set /main_node guardrail_gain_deg 20.0
# 위 GUARDRAIL_PARAMS 값은 그 파라미터들의 기본값 역할을 한다.
GUARDRAIL_TUNABLES = tuple(GUARDRAIL_PARAMS)

# 항목별 (형변환, 하한). 하한은 "그 아래로 가면 의미가 깨지는" 값이다.
#   - gain_deg / max_deg / rate_deg 가 음수면 반발 부호가 뒤집혀 차를 실선
#     쪽으로 민다. margin_px 는 0 이 기능 정지라 0 은 허용하지만, 음수면
#     램프 분모가 음수가 되어 여유가 클수록 더 꺾는 거동이 된다.
#   - decay_frames 는 _compute_guardrail 이 어차피 max(1, ...) 로 막지만,
#     0 을 넣은 사람이 그 사실을 모르는 편이 더 위험해서 여기서 거절한다.
GUARDRAIL_PARAM_SPEC = {
    'margin_px':    (float, 0.0),
    'gain_deg':     (float, 0.0),
    'max_deg':      (float, 0.0),
    'rate_deg':     (float, 0.0),
    'min_trust_px': (float, 0.0),
    'hold_frames':  (int,   0),
    'decay_frames': (int,   1),
}

# 런치 인수 설명. 런치 파일이 이 dict 를 그대로 읽어 DeclareLaunchArgument 를
# 만들기 때문에 항목 설명이 소스 한 곳에만 있다. 런치 파일마다 복사해 두면
# 이득 하나 바꿀 때 세 군데를 고쳐야 한다.
GUARDRAIL_PARAM_HELP = {
    'margin_px':    '이 여유(BEV px) 아래부터 반발 시작. 0이면 가드레일 정지',
    'gain_deg':     '여유 0일 때의 반발 조향(도). 반발이 세면 이 값을 낮춘다',
    'max_deg':      '반발항 자체의 상한(도)',
    'rate_deg':     '프레임당 반발각 변화 제한(도)',
    'min_trust_px': '이보다 가까운 여유는 좌우 판정을 버리고 미관측 취급',
    'hold_frames':  '레일을 놓쳐도 직전 값을 유지할 프레임 수',
    'decay_frames': '그 뒤 0까지 선형 감쇠할 프레임 수',
}


def _validate_named_params(overrides: dict, spec: dict, label: str) -> dict:
    """파라미터 후보를 spec 에 맞춰 검사하고 형변환된 dict 로 돌려준다.

    실패하면 ValueError 를 던진다. 호출자(main_node)는 그 메시지를 그대로
    SetParametersResult.reason 에 실어 보내므로 ros2 param set 을 친 쪽에서
    무엇이 왜 거절됐는지 바로 보인다. 조용히 무시하면 튜닝 중에
    "바꿨는데 왜 그대로지" 를 디버깅하게 된다.
    """
    checked = {}
    for key, value in overrides.items():
        if key not in spec:
            raise ValueError(
                f'unknown {label} parameter: {key} '
                f'(known: {", ".join(spec)})'
            )
        cast, minimum = spec[key]
        if isinstance(value, bool):
            # bool 은 int 의 서브클래스라 cast 를 그냥 통과한다. 파라미터를
            # 잘못 친 것이 분명하므로 여기서 잡는다.
            raise ValueError(
                f'{label} parameter {key} must be {cast.__name__}: {value!r}'
            )
        try:
            converted = cast(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f'{label} parameter {key} must be {cast.__name__}: {value!r}'
            ) from exc
        if cast is float and not math.isfinite(converted):
            raise ValueError(
                f'{label} parameter {key} must be finite: {value!r}'
            )
        if converted < minimum:
            raise ValueError(
                f'{label} parameter {key} must be >= {minimum}: {converted}'
            )
        checked[key] = converted
    return checked


def validate_guardrail_params(overrides: dict) -> dict:
    """가드레일 파라미터 후보를 검사해 형변환된 dict 로 돌려준다."""
    return _validate_named_params(overrides, GUARDRAIL_PARAM_SPEC, 'guardrail')


def validate_steering_filter_params(overrides: dict) -> dict:
    """지터 억제 파라미터 후보를 검사해 형변환된 dict 로 돌려준다."""
    return _validate_named_params(
        overrides, STEERING_FILTER_PARAM_SPEC, 'steering filter')

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
    Mode.LANE_DRIVE: (20.0, 10.0, 0.5),
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
    'min_speed':              6.0,   # 경로 상실 시 하한 (최종 3.0)
    'cautious_speed':         8.0,   # 큰 조향·한쪽 경계 보수 상태 (최종 4.0)
    'cruise_speed':           12.0,  # 정상 추정 상태 (최종 6.0)
    'cautious_confidence':    35.0,  # 이 신뢰도부터 4.0까지 회복
    'full_speed_confidence':  85.0,  # 이 신뢰도부터 6.0 허용
    'turn_start_angle':       15.0,  # 이 조향각부터만 완만하게 감속
    # bag(cone_11) 측정: 조향 40°에서도 속도가 9.9로 거의 안 줄어 코너에서
    # 밀려났다. 기울기를 올리고, cautious_speed와 별개인 코너 전용 하한을 둔다.
    'turn_slowdown':          0.20,  # 큰 조향 시 감속 기울기
    'turn_min_speed':         8.0,   # 코너 감속 하한 (최종 4.0)
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
        # 이전 오프셋 (PD 미분항 계산용). None = 아직 모른다.
        self.prev_offset: Optional[float] = None
        self.prev_mode: Optional[Mode] = None
        # 가드레일 상태 (변화율 제한과 레일 상실 시 감쇠에 쓴다)
        self.guardrail_angle          = 0.0
        self.guardrail_missing_frames = 0
        # offset 저역통과 상태. None 은 "아직 섞을 과거가 없다" 는 뜻이다.
        self.offset_filtered: Optional[float] = None
        # offset 중앙값 필터 창. maxlen 은 파라미터가 바뀔 수 있어 고정하지 않는다.
        self.offset_history: deque = deque()
        # 속도 계산용 조향각(슬루 제한된 값). 서보로 나가는 self.angle 과 다르다.
        self.speed_angle: Optional[float] = None

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
        self.lane_path_preview_params = LANE_PATH_PREVIEW_PARAMS.copy()

    def apply_guardrail_params(self, overrides: dict) -> None:
        """가드레일 파라미터를 주행 중에 갈아 끼운다.

        누적 상태(guardrail_angle)는 일부러 건드리지 않는다. 값을 바꾸는
        시점에 조향을 0 으로 떨구면 코너 한복판에서 튜닝할 때 그 자체가
        외란이 된다. rate_deg 가 다음 프레임부터 새 목표로 끌고 가므로
        전환은 어차피 매끄럽다.
        """
        self.guardrail_params.update(validate_guardrail_params(overrides))

    def apply_steering_filter_params(self, overrides: dict) -> None:
        """지터 억제 파라미터를 주행 중에 갈아 끼운다.

        런타임 값은 pure_pursuit_params 한 곳에만 산다. 필터 상태
        (offset_filtered)는 일부러 유지한다 — 값을 바꿀 때마다 상태를 버리면
        그 프레임에 offset 이 통째로 튀어 튜닝 행위 자체가 외란이 된다.
        """
        self.pure_pursuit_params.update(
            validate_steering_filter_params(overrides))

    def update(self, mode: Mode, offset: int, obstacle_dist: float,
               rubbercone_confidence: int = 100,
               guardrail: Optional[tuple[float, float]] = None,
               lane_path_preview: Optional[
                   tuple[float, float, float]
               ] = None):
        """
        메인 업데이트: 모드에 따라 조향각과 속도를 계산한다.
        모드가 바뀌면 내부 상태를 리셋한다.

        guardrail: /lane_guardrail 의 (좌 여유, 우 여유). BEV px, 음수는 미관측.
                   None이면 가드레일 항을 쓰지 않는다(호출자가 모드/신선도로 막음).
        lane_path_preview: (먼 경로 offset px, 정규화 곡률, 신뢰도 0~1).
                           None/비정상 값이면 기존 제어를 그대로 사용한다.
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
            # 저역통과는 여기서 한 번만 건다(상태가 있으므로). 데드밴드는
            # 상태가 없어 _compute_steering_pure_pursuit 안에 두었고, 그래서
            # preview 목표점도 같은 기준을 지난다.
            lane_offset = self._filter_lane_offset(
                self._median_lane_offset(offset))
            pursuit_angle = self._compute_steering_pure_pursuit(lane_offset)
            pursuit_angle, accepted_preview = self._blend_lane_path_preview(
                pursuit_angle, lane_offset, lane_path_preview)
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
            # 속도만 슬루 제한된 각도로 계산한다. self.angle(서보로 나가는 값)은
            # 그대로다. 가드레일까지 더한 최종 각도를 넣는 것은 유지해서
            # "많이 꺾으면 감속" 이라는 결합 자체는 바뀌지 않는다.
            self.speed = (
                self._compute_speed_from_angle(
                    self._slew_speed_angle(self.angle), params)
                if params else 0.0
            )
            if params and accepted_preview is not None:
                preview_delta, curvature_norm, confidence = accepted_preview
                self.speed = min(
                    self.speed,
                    self._compute_lane_path_preview_speed_cap(
                        params, preview_delta, curvature_norm, confidence),
                )

        elif mode is Mode.FIXED_AVOID:
            # PD 조향(회피 전용 이득) + 조향각 기반 속도 감속
            #
            # 조향 제한을 두는 이유: 다른 모드는 모두 상한이 있는데 여기만 없어서,
            # 큰 오프셋이 그대로 곱해져 나갈 수 있었다. 미분항 킥을 고친 뒤로는
            # 실측 최대가 35° 근처라 이 상한에 닿지 않는다 — 안전망이지 튜닝
            # 지점이 아니다. 차선 주행과 같은 봉투를 쓴다.
            self.angle = self._compute_steering_pd(mode, offset)
            limit = abs(float(self.pure_pursuit_params['max_steering_angle']))
            self.angle = max(-limit, min(limit, self.angle))
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
        # 모드 전환 직후 첫 사이클에는 미분항을 쓰지 않는다.
        #
        # reset() 이 prev_offset 을 0 으로 되돌리므로, 예전에는 새 모드의 첫
        # 계산에서 diff 가 오프셋 전체가 됐다. FIXED_AVOID 는 kp 0.12 · kd 0.35
        # 라서 그 한 사이클만 0.47·e 로 커졌고, 2026-08-13 bag 28.98 초
        # (LANE_DRIVE → FIXED_AVOID 전환 순간) 에 offset -289 px 가 -135.8° 로
        # 나갔다. 그 다음 사이클부터는 -34.7° 근처였으니 오프셋이 아니라 킥이었다.
        #
        # 이전 오프셋을 모른다는 것과 이전 오프셋이 0 이었다는 것은 다르다.
        # 모를 때는 변화량을 0 으로 두는 쪽이 맞다.
        diff = 0.0 if self.prev_offset is None else error - self.prev_offset
        self.prev_offset = error

        # alpha > 0이면 오프셋이 클수록 kp를 증폭 (비선형 제어)
        effective_kp = params.kp * (1.0 + params.alpha * abs(error))
        return effective_kp * error + params.kd * diff

    def _median_lane_offset(self, offset: float) -> float:
        """최근 N 프레임의 중앙값을 낸다 (N<=1 이면 그대로 통과).

        후행 창이라 인과적이다. 지연은 (N-1)/2 프레임 — 3 이면 1프레임(약 42ms).
        저역통과와 달리 지속되는 변화는 그 지연만 지나면 원래 크기로 통과하고,
        한두 프레임 튀었다 돌아오는 값만 사라진다.
        """
        frames = int(self.pure_pursuit_params.get('offset_median_frames', 1))
        value = float(offset)
        if frames <= 1:
            self.offset_history.clear()
            return value
        self.offset_history.append(value)
        while len(self.offset_history) > frames:
            self.offset_history.popleft()
        ordered = sorted(self.offset_history)
        return ordered[len(ordered) // 2]

    def _adaptive_lookahead(self, offset: float) -> float:
        """|offset| 을 곡률 대용으로 써서 lookahead 를 코너값 쪽으로 섞는다.

        직선(작은 offset)에서는 lookahead_px 라 완만하고, 코너로 갈수록
        lookahead_corner_px 에 가까워져 상한이 올라간다. 계단이 아니라 선형으로
        섞는 이유는 문턱에서 조향이 도약하지 않게 하기 위해서다.
        """
        params = self.pure_pursuit_params
        base = max(1.0, float(params['lookahead_px']))
        corner = float(params.get('lookahead_corner_px', 0.0))
        if corner <= 0.0:
            return base
        start = float(params.get('adaptive_offset_start_px', 0.0))
        end = float(params.get('adaptive_offset_end_px', 0.0))
        if end <= start:
            return base
        ratio = max(0.0, min(1.0, (abs(offset) - start) / (end - start)))
        return max(1.0, base + (max(1.0, corner) - base) * ratio)

    def _filter_lane_offset(self, offset: float) -> float:
        """/lane_offset 에 1차 저역통과를 건다 (tau <= 0 이면 그대로 통과).

        alpha 는 CONTROL_PERIOD_S(50 Hz) 기준이다. offset 자체는 약 20 Hz 로
        갱신되는데 이 함수는 50 Hz 로 불리므로, 20 Hz 갱신이 만드는 계단도
        같이 눌린다 — 의도한 효과다.
        """
        value = float(offset)
        if not math.isfinite(value):
            # 비정상 입력에 상태를 오염시키지 않는다. 직전 값을 그대로 쓴다.
            return 0.0 if self.offset_filtered is None else self.offset_filtered
        tau = float(self.pure_pursuit_params.get('offset_lpf_tau_s', 0.0))
        if tau <= 0.0 or self.offset_filtered is None:
            # 첫 프레임은 섞을 과거가 없다. 0 에서 끌어올리면 출발 직후 조향이
            # 비는 구간이 생기므로 관측값으로 초기화한다.
            self.offset_filtered = value
            return value
        alpha = 1.0 - math.exp(-CONTROL_PERIOD_S / tau)
        self.offset_filtered += alpha * (value - self.offset_filtered)
        return self.offset_filtered

    def _slew_speed_angle(self, angle: float) -> float:
        """속도 계산에 쓸 조향각의 변화율을 제한한다 (0 이면 그대로 통과).

        self.angle 은 건드리지 않는다 — 조향은 지금처럼 즉시 나가고 속도만
        부드러워진다. 상태를 따로 두는 이유가 그것이다.
        """
        limit = abs(float(
            self.pure_pursuit_params.get('speed_angle_slew_per_s', 0.0)))
        value = float(angle)
        if limit <= 0.0 or self.speed_angle is None:
            self.speed_angle = value
            return value
        step = limit * CONTROL_PERIOD_S
        self.speed_angle += max(-step, min(step, value - self.speed_angle))
        return self.speed_angle

    def _apply_offset_deadband(self, offset: float) -> float:
        """|offset| 이 데드밴드 이하면 0 으로 만든다 (경계에서 연속).

        빼기(shift)이지 자르기(clip)가 아니다. 경계에서 조향이 툭 튀지 않도록
        데드밴드 바로 바깥은 0 에서 매끄럽게 이어진다.
        """
        deadband = abs(float(
            self.pure_pursuit_params.get('offset_deadband_px', 0.0)))
        if deadband <= 0.0:
            return offset
        return math.copysign(max(0.0, abs(offset) - deadband), offset)

    def _compute_steering_pure_pursuit(self, offset: int) -> float:
        """BEV 목표점으로부터 Pure Pursuit 조향각을 계산한다.

        목표점은 차량 기준 ``(lateral=offset, forward=lookahead_px)``이다.
        반환값의 부호는 기존 PD와 동일하게 양의 offset에 양의 조향을 낸다.
        """
        params = self.pure_pursuit_params
        offset = self._apply_offset_deadband(float(offset))
        # lookahead 는 고정이 아니라 |offset| 에 따라 움직인다. lateral clamp 도
        # 같이 움직이므로 코너에서는 더 작은 값에서 포화한다 — 의도한 거동이다.
        lookahead = self._adaptive_lookahead(offset)
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

    def _blend_lane_path_preview(
        self,
        base_angle: float,
        current_offset: float,
        preview: Optional[tuple[float, float, float]],
    ) -> tuple[float, Optional[tuple[float, float, float]]]:
        """먼 경로점의 Pure Pursuit 조향을 현재 조향에 보수적으로 섞는다.

        두 번째 반환값은 실제 적용된 ``(delta_px, curvature, confidence)``다.
        None 이면 호출자가 기존 속도 계산도 그대로 유지할 수 있다.
        """
        accepted = self._validate_lane_path_preview(preview)
        if accepted is None:
            return base_angle, None

        target_offset, curvature_norm, confidence = accepted
        params = self.lane_path_preview_params
        max_delta = max(0.0, float(params['max_target_delta_px']))
        raw_delta = target_offset - float(current_offset)
        preview_delta = max(-max_delta, min(max_delta, raw_delta))
        clamped_target = float(current_offset) + preview_delta

        preview_angle = self._compute_steering_pure_pursuit(clamped_target)
        blend = max(0.0, min(1.0, float(params['steering_blend'])))
        blend *= confidence
        angle = base_angle + (preview_angle - base_angle) * blend
        return angle, (preview_delta, curvature_norm, confidence)

    def _validate_lane_path_preview(
        self,
        preview: Optional[tuple[float, float, float]],
    ) -> Optional[tuple[float, float, float]]:
        """외부 preview 입력을 유한한 내부 표현으로 바꾼다.

        잘못된 센서 메시지가 제어 루프를 예외로 중단시키지 않도록 모양, 형변환,
        NaN/Inf 를 모두 여기서 거른다. 신뢰도 0은 기능 정지와 같은 의미다.
        """
        params = self.lane_path_preview_params
        if not bool(params.get('enabled', False)) or preview is None:
            return None
        try:
            target_offset, curvature_norm, confidence = preview
            target_offset = float(target_offset)
            curvature_norm = float(curvature_norm)
            confidence = float(confidence)
        except (TypeError, ValueError, OverflowError):
            return None

        if not all(math.isfinite(value) for value in (
                target_offset, curvature_norm, confidence)):
            return None

        confidence = max(0.0, min(1.0, confidence))
        if confidence <= 0.0:
            return None
        curvature_cap = max(
            0.0, float(params.get('curvature_norm_cap', 0.0)))
        curvature_norm = max(
            -curvature_cap, min(curvature_cap, curvature_norm))
        return target_offset, curvature_norm, confidence

    def _compute_lane_path_preview_speed_cap(
        self,
        params: SpeedParams,
        preview_delta: float,
        curvature_norm: float,
        confidence: float,
    ) -> float:
        """앞쪽 경로가 휜 정도에 따른 추가 속도 상한을 계산한다.

        기존의 최종 조향각 감속을 대체하지 않고 ``min`` 상한으로만 결합한다.
        따라서 가드레일 조향 감속은 유지되며, 이 상한도 LANE_DRIVE의 기존
        min_speed 아래로 속도를 내리지 않는다.
        """
        preview_params = self.lane_path_preview_params
        max_delta = max(
            1e-6, abs(float(preview_params['max_target_delta_px'])))
        start = max(0.0, min(
            max_delta, float(preview_params['speed_slowdown_start_px'])))
        delta_range = max(1e-6, max_delta - start)
        delta_strength = max(
            0.0, min(1.0, (abs(preview_delta) - start) / delta_range))

        curvature_cap = max(
            1e-6, abs(float(preview_params['curvature_norm_cap'])))
        curvature_strength = min(1.0, abs(curvature_norm) / curvature_cap)
        curvature_weight = max(0.0, min(
            1.0, float(preview_params['curvature_speed_weight'])))
        curve_strength = max(
            delta_strength, curvature_strength * curvature_weight)
        effective_strength = (
            max(0.0, min(1.0, confidence)) * curve_strength
        )

        reduction = max(
            0.0, float(preview_params['max_speed_reduction']))
        cap = params.max_speed - reduction * effective_strength
        return max(params.min_speed, cap)

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

        정상 추정 시 최종 약 6.0을 유지하고, 조향각이 커질수록
        최종 약 4.0까지 감속해 코너에서 밀려나는 것을 막는다.
        경계가 사라져 신뢰도가 낮아질 때는 최종 3.0까지 감속한다.
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
        # None = "직전 오프셋을 모른다". 0.0 으로 두면 첫 사이클의 미분항이
        # 오프셋 전체가 되어 모드 전환마다 조향이 튄다 (_compute_steering_pd 참고).
        self.prev_offset = None
        self.offset_filtered = None
        self.speed_angle = None
        self.offset_history.clear()
        self.guardrail_angle = 0.0
        self.guardrail_missing_frames = 0

    def get_angle(self) -> float:
        """현재 계산된 조향각 반환"""
        return self.angle

    def get_speed(self) -> float:
        """현재 계산된 속도 반환"""
        return self.speed
