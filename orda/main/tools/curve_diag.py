#!/usr/bin/env python3
"""곡선 구간에서 조향이 왜 부족한지 단계별로 계측한다.

차를 곡선 구간에 둔 채(정지든 주행이든) launch 가 돌아가는 상태에서 실행한다:

    source /opt/ros/jazzy/setup.bash && source ~/xycar_ws/install/setup.bash
    python3 src/orda/main/tools/curve_diag.py --seconds 20

main.control 의 진짜 Controller 를 그대로 불러 쓰므로 여기서 재현한 각도는
차가 실제로 계산하는 값과 같다. 그래서 "PP 상한에서 잘렸는가 / 가드레일이
안 붙었는가 / 인지 주기가 느린가" 를 추정이 아니라 계측으로 가른다.
"""

import argparse
import math
import os
import random
import statistics as st
import sys
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import Bool, Float32MultiArray, Int16

from main.control import (
    CONTROL_PERIOD_S,
    GUARDRAIL_PARAMS,
    LANE_PATH_PREVIEW_PARAMS,
    PURE_PURSUIT_PARAMS,
    SPEED_PARAMS,
    Controller,
)
from main.race_fsm import Mode

# lane_detection.cpp 의 qos_fast 와 같은 프로파일이어야 샘플이 빠지지 않는다.
QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

PAIR_WINDOW_S = 0.05  # 같은 프레임으로 묶을 시간 창


def pct(values, q):
    if not values:
        return float('nan')
    ordered = sorted(values)
    idx = min(len(ordered) - 1, max(0, int(round(q / 100.0 * (len(ordered) - 1)))))
    return ordered[idx]


def describe(name, values, unit='', width=26):
    if not values:
        print(f'  {name:<{width}} (샘플 없음)')
        return
    print(f'  {name:<{width}} n={len(values):<5d} '
          f'min={min(values):8.1f} p25={pct(values,25):8.1f} '
          f'med={pct(values,50):8.1f} p75={pct(values,75):8.1f} '
          f'p95={pct(values,95):8.1f} max={max(values):8.1f} {unit}')


class CurveDiag(Node):
    def __init__(self, seconds):
        super().__init__('curve_diag')
        self.seconds = seconds
        self.offsets = deque()        # (t, value)
        self.guardrails = deque()     # (t, (left, right))
        self.previews = deque()       # (t, (target, curvature, conf))
        self.motors = deque()         # (t, (angle, speed))
        self.valids = deque()         # (t, bool)
        self.diags = deque()          # (t, reason, {key: value})

        self.create_subscription(Int16, '/lane_offset', self._offset, QOS)
        self.create_subscription(
            Float32MultiArray, '/lane_guardrail', self._guardrail, QOS)
        self.create_subscription(
            Float32MultiArray, '/lane_path_preview', self._preview, QOS)
        self.create_subscription(
            Float32MultiArray, '/xycar_motor', self._motor, QOS)
        self.create_subscription(Bool, '/lane_valid', self._valid, QOS)
        self.create_subscription(
            DiagnosticArray, '/lane_detection/pipeline_diagnostics',
            self._diag, QOS)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _offset(self, msg):
        self.offsets.append((self._now(), int(msg.data)))

    def _guardrail(self, msg):
        if len(msg.data) >= 2:
            self.guardrails.append(
                (self._now(), (float(msg.data[0]), float(msg.data[1]))))

    def _preview(self, msg):
        if len(msg.data) >= 3:
            self.previews.append((self._now(), (
                float(msg.data[0]), float(msg.data[1]), float(msg.data[2]))))

    def _motor(self, msg):
        if len(msg.data) >= 2:
            self.motors.append(
                (self._now(), (float(msg.data[0]), float(msg.data[1]))))

    def _valid(self, msg):
        self.valids.append((self._now(), bool(msg.data)))

    def _diag(self, msg):
        for status in msg.status:
            self.diags.append((
                self._now(), status.message,
                {item.key: item.value for item in status.values}))


def rate_of(samples):
    if len(samples) < 2:
        return 0.0
    span = samples[-1][0] - samples[0][0]
    return (len(samples) - 1) / span if span > 0 else 0.0


def nearest(series, t):
    """t 에 가장 가까운 샘플을 PAIR_WINDOW_S 안에서 찾는다."""
    best, best_dt = None, PAIR_WINDOW_S
    for ts, value in series:
        dt = abs(ts - t)
        if dt <= best_dt:
            best, best_dt = value, dt
        elif ts > t + PAIR_WINDOW_S:
            break
    return best


def pp_angle(x, Ld, Lw, gain):
    lat = max(-Ld, min(Ld, x))
    return math.degrees(math.atan2(2.0 * Lw * lat, lat * lat + Ld * Ld)) * gain


def slope_at(x, Ld, Lw, gain, h=0.5):
    """|x| 근처에서 offset 1px 이 만드는 조향 변화(도/px)."""
    return abs(pp_angle(abs(x) + h, Ld, Lw, gain)
               - pp_angle(abs(x) - h, Ld, Lw, gain)) / (2.0 * h)


def lpf_attenuation(tau, lane_hz, samples=4000):
    """측정된 인지 주기와 50 Hz 제어 주기를 그대로 흉내내 감쇠비를 구한다.

    offset 은 lane_hz 로 갱신되고 그 사이에는 같은 값이 유지되는데, 필터는
    CONTROL_PERIOD_S 마다 돈다. 그 계단 유지 덕에 실제 감쇠가 이론식보다
    좋아서, 식 대신 실제 구조로 재는 편이 정확하다.
    """
    if tau <= 0.0 or lane_hz <= 0.0:
        return 1.0
    random.seed(0)
    alpha = 1.0 - math.exp(-CONTROL_PERIOD_S / tau)
    steps = max(1, int(round((1.0 / lane_hz) / CONTROL_PERIOD_S)))
    filtered = 0.0
    raw_values, out_values = [], []
    for _ in range(samples):
        noise = random.gauss(0.0, 1.0)
        for _ in range(steps):
            filtered += alpha * (noise - filtered)
        raw_values.append(noise)
        out_values.append(filtered)
    warm = samples // 5
    return _stdev(out_values[warm:]) / max(1e-9, _stdev(raw_values[warm:]))


def _stdev(values):
    return st.pstdev(values) if len(values) > 1 else 0.0


def report_outliers(raw, sigma, Ld, Lw, gain):
    """드문 큰 점프를 세고 스파이크와 계단으로 가른다.

    둘은 처방이 다르다. 스파이크(튀었다 곧바로 돌아옴)는 저역통과가 지워 주지만,
    계단(튀고 그 자리에 머묾)은 차선을 실제로 다시 잡은 것이라 필터가 지연만
    더한다. sigma 만 보면 둘 다 안 보인다 — p95 는 낮은데 max 만 큰 경우가 그렇다.
    """
    steps = [raw[i] - raw[i - 1] for i in range(1, len(raw))]
    threshold = max(10.0, 5.0 * sigma)
    spikes = walls = 0
    index = 0
    while index < len(steps):
        delta = steps[index]
        if abs(delta) < threshold:
            index += 1
            continue
        following = steps[index + 1] if index + 1 < len(steps) else 0.0
        if following * delta < 0 and abs(following) >= 0.5 * abs(delta):
            # 되돌아오는 다리까지가 스파이크 하나다. 같이 건너뛰지 않으면
            # 복귀 구간이 다시 "계단" 으로 잡혀 한 번을 두 번으로 센다.
            spikes += 1
            index += 2
        else:
            walls += 1
            index += 1
    total = spikes + walls
    print(f'\n  |변화| > {threshold:.0f}px 인 프레임 : {total} / {len(steps)} '
          f'({100.0 * total / max(1, len(steps)):.1f}%)'
          f'  → 스파이크 {spikes} / 계단 {walls}')
    if total:
        biggest = max(abs(delta) for delta in steps)
        print(f'    최대 점프 {biggest:.0f}px = 조향 '
              f'{biggest * slope_at(pct([abs(v) for v in raw], 50), Ld, Lw, gain):.1f} '
              f'만큼의 순간 변화')
        if walls > spikes:
            print('    계단이 우세하다 → 저역통과는 지연만 늘린다. 차선 재탐색'
                  '(fit_reacquire_after_frames) 쪽을 보라.')
        else:
            print('    스파이크가 우세하다 → 저역통과가 바로 듣는다.')


def report_oscillation(raw, sigma, lane_hz):
    """노이즈인지 제어 진동인지 가른다.

    둘 다 "흔들린다" 로 보이지만 처방이 정반대다. 백색잡음에는 저역통과가 듣고,
    폐루프 진동에는 위상 지연이 더해져 오히려 심해진다 — 그때는 이득을 낮춰야 한다.

    추세는 중심 이동평균으로 뺀다(후행 평균을 쓰면 지연분이 잔차에 남아 깨끗한
    노이즈도 저주파 진동처럼 보인다). 진폭은 MAD 로 재서 스파이크 몇 개에
    끌려가지 않게 한다. 잔차가 노이즈 바닥과 구별되지 않으면 아무 말도 하지 않는다.
    """
    window = max(3, int(round(lane_hz * 0.5)))
    if lane_hz <= 0 or len(raw) < 4 * window:
        return
    half = window // 2
    detrended = []
    for index in range(half, len(raw) - half):
        trend = sum(raw[index - half:index + half + 1]) / (2 * half + 1)
        detrended.append(raw[index] - trend)
    centre = pct(detrended, 50)
    amplitude = 1.4826 * pct([abs(v - centre) for v in detrended], 50)
    if amplitude <= 2.5 * max(sigma, 1e-6):
        print(f'  추세 제거 후 잔차 {amplitude:.2f}px 로 노이즈 바닥'
              f'({sigma:.2f}px)과 구별되지 않는다 → 구조적 흔들림 없음.')
        return
    crossings = sum(
        1 for i in range(1, len(detrended))
        if detrended[i - 1] * detrended[i] < 0
    )
    if crossings < 4:
        return
    period_s = (2.0 * len(detrended) / crossings) / lane_hz
    print(f'  구조적 흔들림          = 진폭 {amplitude:.1f}px, 주기 '
          f'{period_s * 1000:.0f} ms ({1.0 / period_s:.1f} Hz)')
    print('    노이즈 바닥보다 크고 주기가 일정하다 → 폐루프 진동일 수 있다.'
          ' 저역통과는 위상 지연을 더해 악화시킨다. 이득 쪽을 먼저 보라.')


def recommend_filters(sigma, mag, Ld, Lw, gain, lane_hz):
    """계측된 노이즈에서 데드밴드와 tau 를 뽑는다."""
    corner = pct(mag, 50)
    deadband = 2.0 * sigma                      # ±2σ 면 노이즈의 약 95%
    plain = abs(pp_angle(corner, Ld, Lw, gain))
    damped = abs(pp_angle(max(0.0, corner - deadband), Ld, Lw, gain))
    cost = 0.0 if plain <= 0 else (plain - damped) / plain * 100.0
    print(f'\n  ── 권장값 ──')
    print(f'  offset_deadband_px = {deadband:.0f}   '
          f'(코너 |offset| {corner:.0f}px 조향 {plain:.1f} -> {damped:.1f}, '
          f'{cost:.1f}% 손해)')
    if cost > 5.0:
        print('     ! 코너 손해가 5% 를 넘는다. 데드밴드를 줄이고 나머지는 '
              'tau 로 넘기는 편이 낫다.')
    # 코너 한복판에서는 offset 이 커서 데드밴드가 안 먹는다. 거기 남는 흔들림은
    # 저역통과로만 줄일 수 있다.
    in_corner = sigma * slope_at(corner, Ld, Lw, gain)
    target = 1.0
    if in_corner <= target:
        print(f'  offset_lpf_tau_s   = 0     '
              f'(코너 중 흔들림이 {in_corner:.2f} 로 이미 {target:.1f} 아래다)')
        return
    need = target / in_corner
    for tau in (0.04, 0.06, 0.08, 0.10, 0.15, 0.20, 0.30):
        if lpf_attenuation(tau, lane_hz) <= need:
            print(f'  offset_lpf_tau_s   = {tau:.2f}  '
                  f'(코너 중 흔들림 {in_corner:.2f} -> '
                  f'{in_corner * lpf_attenuation(tau, lane_hz):.2f}, '
                  f'코너 반응 {tau * 1000:.0f}ms 지연)')
            return
    print(f'  offset_lpf_tau_s   = 0.30+ (노이즈 {in_corner:.2f} 가 커서 '
          f'필터만으로는 부족하다. 인지 쪽을 봐야 한다)')


def collect_live(seconds):
    rclpy.init()
    node = CurveDiag(seconds)
    print(f'수집 중… {seconds:.0f}초. 차를 곡선 구간에 둔 상태를 유지하세요.')
    end = node.get_clock().now().nanoseconds * 1e-9 + seconds
    while rclpy.ok() and node.get_clock().now().nanoseconds * 1e-9 < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    collected = (list(node.offsets), list(node.guardrails), list(node.previews),
                 list(node.motors), list(node.valids), list(node.diags))
    node.destroy_node()
    rclpy.shutdown()
    return collected


def collect_bag(path):
    """기록된 bag 에서 같은 구조로 읽는다.

    ros2 bag play 로 재생해 라이브 수집하는 것보다 정확하다 — 재생 지터나
    QoS 유실이 끼지 않고, 기록 당시의 타임스탬프를 그대로 쓴다. 그래서 같은
    bag 을 몇 번 돌려도 결과가 똑같고 A/B 비교가 의미를 갖는다.
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    import yaml

    storage_id = 'mcap'
    metadata = os.path.join(path, 'metadata.yaml')
    if os.path.isfile(metadata):
        with open(metadata) as handle:
            info = yaml.safe_load(handle) or {}
        storage_id = (info.get('rosbag2_bagfile_information', {})
                      .get('storage_identifier', storage_id))

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id=storage_id),
        rosbag2_py.ConverterOptions('', ''),
    )
    types = {topic.name: topic.type
             for topic in reader.get_all_topics_and_types()}
    offsets, guardrails, previews, motors, valids, diags = [], [], [], [], [], []

    def take_diag(t, m):
        for status in m.status:
            diags.append((t, status.message,
                          {item.key: item.value for item in status.values}))

    sinks = {
        '/lane_detection/pipeline_diagnostics': take_diag,
        '/lane_offset': lambda t, m: offsets.append((t, int(m.data))),
        '/lane_guardrail': lambda t, m: (
            guardrails.append((t, (float(m.data[0]), float(m.data[1]))))
            if len(m.data) >= 2 else None),
        '/lane_path_preview': lambda t, m: (
            previews.append((t, (float(m.data[0]), float(m.data[1]),
                                 float(m.data[2]))))
            if len(m.data) >= 3 else None),
        '/xycar_motor': lambda t, m: (
            motors.append((t, (float(m.data[0]), float(m.data[1]))))
            if len(m.data) >= 2 else None),
        '/lane_valid': lambda t, m: valids.append((t, bool(m.data))),
    }
    print(f'bag 읽는 중… {path}')
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        sink = sinks.get(topic)
        if sink is None:
            continue
        sink(stamp * 1e-9, deserialize_message(data, get_message(types[topic])))
    missing = [name for name in sinks if name not in types]
    if missing:
        print(f'  기록되지 않은 토픽: {", ".join(sorted(missing))}')
    return offsets, guardrails, previews, motors, valids, diags


# 파이프라인이 신호를 흘려보내는 순서. 어느 칸에서 0 이 되는지가 곧 원인이다.
PIPELINE_STAGES = (
    ('input_mask_pixels', '모델 마스크'),
    ('roi_mask_pixels', 'ROI 자른 뒤'),
    ('roi_after_morphology_pixels', '모폴로지 뒤'),
    ('bev_pixels', 'BEV 변환 뒤'),
    ('after_horizontal_pixels', '수평 억제 뒤'),
    ('after_column_pixels', '수직 억제 뒤'),
    ('corridor_pixels', '코리도어 안'),
    ('sliding_points', '슬라이딩 윈도우'),
)


def report_pipeline(diags):
    """왜 차선을 놓치는지 lane_node 의 자체 분류로 집계한다."""
    print('\n' + '=' * 78)
    print('2c. 차선 피팅 실패 원인 (lane_node 파이프라인 진단)')
    print('=' * 78)
    reasons = {}
    for _, reason, _ in diags:
        reasons[reason] = reasons.get(reason, 0) + 1
    total = len(diags)
    failed = total - reasons.get('NONE', 0)
    print(f'  진단 프레임 {total}, 실패 {failed} ({100.0 * failed / total:.1f}%)')
    for reason, count in sorted(reasons.items(), key=lambda kv: -kv[1]):
        if reason == 'NONE':
            continue
        print(f'    {reason:<32} {count:>5}회 ({100.0 * count / total:>4.1f}%)')
    if not failed:
        print('    실패 없음.')
        return

    def numbers(key, want_fail):
        out = []
        for _, reason, values in diags:
            if (reason != 'NONE') != want_fail:
                continue
            try:
                out.append(float(values[key]))
            except (KeyError, ValueError):
                pass
        return out

    print('\n  단계별 남은 화소 (중앙값) — 어디서 신호가 죽는지')
    print(f'    {"단계":<16}{"성공 프레임":>12}{"실패 프레임":>12}')
    for key, label in PIPELINE_STAGES:
        ok_values, bad_values = numbers(key, False), numbers(key, True)
        if not ok_values and not bad_values:
            continue
        ok_median = pct(ok_values, 50) if ok_values else float('nan')
        bad_median = pct(bad_values, 50) if bad_values else float('nan')
        print(f'    {label:<16}{ok_median:>12.0f}{bad_median:>12.0f}')

    # 앵커 리셋은 "문턱을 넘긴 구간" 마다 한 번 일어난다. 프레임 수를 세면
    # 긴 구간 하나가 수십 번으로 부풀려져 원인을 과대평가하게 된다.
    reacq = numbers('fit_reacquire_after_frames', True)
    limit = reacq[0] if reacq else 0.0
    runs, current = [], 0
    for _, reason, _ in diags:
        if reason == 'NONE':
            if current:
                runs.append(current)
            current = 0
        else:
            current += 1
    if current:
        runs.append(current)
    if runs:
        over = [r for r in runs if limit and r >= limit]
        span = diags[-1][0] - diags[0][0]
        print(f'\n  실패 구간 {len(runs)}개, 최장 {max(runs)}프레임 '
              f'(앵커 리셋 문턱 fit_reacquire_after_frames={limit:.0f})')
        if over:
            rate = len(over) / span if span > 0 else 0.0
            print(f'    문턱을 넘긴 구간 {len(over)}개 ({rate:.2f}회/초)'
                  f' → 그때마다 앵커가 ref_x 로 돌아가며 offset 이 크게 튄다')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=20.0,
                    help='라이브 수집 시간(초). --bag 을 주면 무시된다')
    ap.add_argument('--bag', default=None,
                    help='기록된 rosbag2 디렉터리. 주면 라이브 대신 이것을 분석한다')
    args = ap.parse_args()

    if args.bag:
        (offsets, guardrails, previews, motors, valids,
         diags) = collect_bag(args.bag)
    else:
        (offsets, guardrails, previews, motors, valids,
         diags) = collect_live(args.seconds)

    Ld = float(PURE_PURSUIT_PARAMS['lookahead_px'])
    Lw = float(PURE_PURSUIT_PARAMS['wheelbase_px'])
    gain = float(PURE_PURSUIT_PARAMS['steering_gain'])
    pp_ceiling = math.degrees(math.atan(Lw / Ld)) * gain

    print('\n' + '=' * 78)
    print('1. 수신 상태')
    print('=' * 78)
    for label, series in (('/lane_offset', offsets), ('/lane_guardrail', guardrails),
                          ('/lane_path_preview', previews), ('/xycar_motor', motors),
                          ('/lane_valid', valids)):
        print(f'  {label:<22} {rate_of(series):6.2f} Hz   n={len(series)}')
    if valids:
        ok = sum(1 for _, v in valids if v)
        print(f'  lane_valid=true          {100.0*ok/len(valids):5.1f}%')

    if not offsets:
        print('\n/lane_offset 이 하나도 안 들어왔습니다. 차선 인지가 죽어 있습니다.')
        return 1

    print('\n' + '=' * 78)
    print(f'2. /lane_offset 분포  (BEV 폭 1280px, PP lateral clamp = {Ld:.0f}px)')
    print('=' * 78)
    raw = [v for _, v in offsets]
    mag = [abs(v) for v in raw]
    describe('offset (부호 포함)', raw, 'px')
    describe('|offset|', mag, 'px')
    over_clamp = sum(1 for v in mag if v >= Ld)
    over_80 = sum(1 for v in mag if v >= 0.8 * Ld)
    print(f'\n  |offset| >= clamp({Ld:.0f}px)  : {100.0*over_clamp/len(mag):5.1f}%'
          f'   ← 이 비율만큼 코너 깊이 정보가 버려진다')
    print(f'  |offset| >= {0.8*Ld:.0f}px        : {100.0*over_80/len(mag):5.1f}%')

    # ── 노이즈 추정 ──────────────────────────────────────────────────────
    # 직선이 아니어도 노이즈를 뽑을 수 있다. 참값은 프레임 사이에 매끄럽게
    # 변하므로 2차 차분 x[n]-2x[n-1]+x[n-2] 에서는 거의 사라지고 노이즈만
    # 남는다(국소 직선은 정확히 0). 백색잡음이면 2차 차분의 분산이 6*sigma^2
    # 이므로 그 강건 표준편차를 sqrt(6) 으로 나누면 sigma 가 나온다.
    # MAD 를 쓰는 이유는 재탐색 점프 한두 개에 추정이 끌려가지 않게 하기 위해서다.
    second_diff = [raw[i] - 2 * raw[i - 1] + raw[i - 2]
                   for i in range(2, len(raw))]
    sigma = float('nan')
    if len(second_diff) >= 8:
        centre = pct(second_diff, 50)
        mad = pct([abs(v - centre) for v in second_diff], 50)
        sigma = 1.4826 * mad / math.sqrt(6.0)

    print('\n' + '=' * 78)
    print('2b. offset 노이즈 (커브 데이터에서도 유효)')
    print('=' * 78)
    step = [abs(raw[i] - raw[i - 1]) for i in range(1, len(raw))]
    describe('프레임 간 |변화|', step, 'px')
    if math.isnan(sigma):
        print('  샘플이 모자라 노이즈를 추정하지 못했다.')
    else:
        print(f'\n  추정 노이즈 sigma      = {sigma:.2f} px')
        print(f'  조향으로 환산(중앙부)   = {sigma * slope_at(0.0, Ld, Lw, gain):.2f}')
        print(f'  조향으로 환산(|offset| 중앙값 {pct(mag, 50):.0f}px) = '
              f'{sigma * slope_at(pct(mag, 50), Ld, Lw, gain):.2f}')
        report_outliers(raw, sigma, Ld, Lw, gain)
        report_oscillation(raw, sigma, rate_of(offsets))
        recommend_filters(sigma, mag, Ld, Lw, gain, rate_of(offsets))

    print('\n' + '=' * 78)
    print('3. 조향 재현 (실제 Controller 로 프레임 순서대로 계산)')
    print('=' * 78)
    ctrl = Controller()
    pursuit_list, blended_list, grail_list, final_list, preview_gain_list = [], [], [], [], []
    for t, off in offsets:
        gr = nearest(guardrails, t)
        pv = nearest(previews, t)
        pursuit = ctrl._compute_steering_pure_pursuit(off)
        blended, accepted = ctrl._blend_lane_path_preview(pursuit, off, pv)
        grail = ctrl._compute_guardrail(gr)
        limit = abs(float(PURE_PURSUIT_PARAMS['max_steering_angle']))
        final = max(-limit, min(limit, blended + grail))
        pursuit_list.append(pursuit)
        blended_list.append(blended)
        grail_list.append(grail)
        final_list.append(final)
        preview_gain_list.append(blended - pursuit)

    print(f'  PP 이론 상한 = degrees(atan({Lw:.0f}/{Ld:.0f})) x {gain} = {pp_ceiling:.2f}deg\n')
    describe('PP 단독 |각도|', [abs(v) for v in pursuit_list], 'deg')
    describe('preview 기여 |각도|', [abs(v) for v in preview_gain_list], 'deg')
    describe('가드레일 기여 |각도|', [abs(v) for v in grail_list], 'deg')
    describe('최종 |각도| (재현)', [abs(v) for v in final_list], 'deg')
    if motors:
        describe('실제 /xycar_motor |각도|', [abs(a) for _, (a, _) in motors], 'deg')
        describe('실제 /xycar_motor 속도', [s for _, (_, s) in motors], '')

    if previews:
        confs = [c for _, (_, _, c) in previews]
        zero_conf = sum(1 for c in confs if c <= 0.0)
        print()
        describe('preview confidence', confs, '')
        describe('preview target_offset',
                 [t for _, (t, _, _) in previews], 'px')
        print(f'  preview confidence == 0        : '
              f'{100.0*zero_conf/len(confs):5.1f}%'
              f'   ← 100% 면 포화가 아니라 preview 자체가 무효다')
    else:
        zero_conf = 0
        confs = []

    # ROS1 xycar_motor.py 는 angle 을 max(-50, min(angle, 100)) 으로 자른다.
    # 좌우가 비대칭이라, 상한이 50 을 넘길 수 있게 되는 순간 좌커브만 조용히
    # 잘린다. PP 단독일 때는 안 걸리다가 가드레일이 붙으면서 걸리기 시작한다.
    left_clip = sum(1 for v in final_list if v < -50.0)
    right_clip = sum(1 for v in final_list if v > 100.0)
    reachable = pp_ceiling + abs(float(GUARDRAIL_PARAMS['max_deg']))
    print(f'\n  도달 가능 최대 = PP {pp_ceiling:.1f} + 가드레일 '
          f'{abs(float(GUARDRAIL_PARAMS["max_deg"])):.1f} = {reachable:.1f}')
    if reachable > 50.0:
        print(f'  ! ROS1 좌조향 한계 -50 을 넘길 수 있는 구성이다 '
              f'(우조향은 +100 까지 열려 있어 좌우가 비대칭이다)')
    print(f'  좌조향이 -50 아래로 내려간 프레임 : {left_clip} / {len(final_list)} '
          f'({100.0 * left_clip / max(1, len(final_list)):.1f}%)  ← 잘려서 나간다')
    if right_clip:
        print(f'  우조향이 +100 을 넘은 프레임      : {right_clip}')

    # 가드레일이 붙었다 떨어졌다 하면 조향이 그만큼 오르내린다. 코너 조향을
    # 가드레일에 기대는 구성에서는 이 전환 횟수가 곧 체감 지터다.
    engaged = [abs(v) > 0.5 for v in grail_list]
    transitions = sum(1 for i in range(1, len(engaged))
                      if engaged[i] != engaged[i - 1])
    # 몫은 "붙어 있고 조향도 유의미한" 프레임에서만 센다. 최종 조향이 0 근처면
    # 분모가 사라져 몫이 수백 %로 튄다(가드레일과 PP 가 서로 상쇄한 프레임이다).
    share = [abs(g) / abs(f) * 100.0
             for g, f, on in zip(grail_list, final_list, engaged)
             if on and abs(f) > 1.0]
    engaged_ratio = 100.0 * sum(engaged) / max(1, len(engaged))
    print(f'\n  가드레일이 실제로 붙은 프레임 : {engaged_ratio:.1f}%'
          f'   (붙었다 떨어진 전환 {transitions}회)')
    if share:
        print(f'  붙은 프레임에서의 가드레일 몫  : 중앙값 {pct(share, 50):.0f}%, '
              f'p95 {pct(share, 95):.0f}%')
    opposed = sum(1 for g, f, on in zip(grail_list, final_list, engaged)
                  if on and g * f < 0)
    if opposed:
        print(f'  가드레일이 최종 조향과 반대 부호 : {opposed}프레임'
              f' — 이만큼은 PP 를 상쇄한다')
    if engaged_ratio < 5.0:
        print('    거의 안 붙는다 → 코너 조향은 사실상 Pure Pursuit 단독이다.'
              ' 이득을 올려도 발동 조건(margin_px)이 안 맞으면 소용없다.')
    elif transitions > 0.1 * len(engaged):
        print('    전환이 잦다 → 붙었다 떨어질 때마다 조향이 오르내린다.'
              ' 이 자체가 체감 지터다.')

    sat = sum(1 for v in pursuit_list if abs(v) >= 0.85 * pp_ceiling)
    print(f'\n  PP 가 상한의 85% 이상인 프레임 : {100.0*sat/len(pursuit_list):5.1f}%'
          f'   ← 높으면 조향이 이미 벽에 붙어 있다')
    dead_preview = sum(1 for v in preview_gain_list if abs(v) < 0.5)
    print(f'  preview 기여가 0.5deg 미만       : {100.0*dead_preview/len(preview_gain_list):5.1f}%')

    print('\n' + '=' * 78)
    print(f"4. 가드레일 입력  (margin_px={GUARDRAIL_PARAMS['margin_px']:.0f}, "
          f"min_trust_px={GUARDRAIL_PARAMS['min_trust_px']:.0f})")
    print('=' * 78)
    if not guardrails:
        print('  /lane_guardrail 샘플 없음 — 반발항을 쓸 근거가 아예 없다.')
    else:
        left = [g[0] for _, g in guardrails]
        right = [g[1] for _, g in guardrails]
        seen_l = [v for v in left if v >= 0]
        seen_r = [v for v in right if v >= 0]
        print(f'  좌 실선 관측률 {100.0*len(seen_l)/len(left):5.1f}%   '
              f'우 실선 관측률 {100.0*len(seen_r)/len(right):5.1f}%')
        describe('좌 여유(관측분)', seen_l, 'px')
        describe('우 여유(관측분)', seen_r, 'px')
        thr = float(GUARDRAIL_PARAMS['margin_px'])
        floor = float(GUARDRAIL_PARAMS['min_trust_px'])
        usable = [min([v for v in (l, r) if v >= floor], default=None)
                  for l, r in ((g[0], g[1]) for _, g in guardrails)]
        have = [v for v in usable if v is not None]
        print(f'\n  좌우 중 하나라도 신뢰 가능    : {100.0*len(have)/len(usable):5.1f}%'
              f'   ← 낮으면 반발항이 hold/decay 로만 산다')
        if have:
            fired = sum(1 for v in have if v < thr)
            print(f'  그중 margin_px({thr:.0f}) 미만    : {100.0*fired/len(have):5.1f}%'
                  f'   ← 이게 실제로 반발이 걸리는 비율')

    print('\n' + '=' * 78)
    print('5. 속도 결합')
    print('=' * 78)
    mx, mn, sc = SPEED_PARAMS[Mode.LANE_DRIVE]
    knee = (mx - mn) / sc if sc else float('inf')
    print(f'  SPEED_PARAMS[LANE_DRIVE] = max {mx}, min {mn}, scale {sc}')
    print(f'  |각도| {knee:.1f}deg 이상이면 속도가 {mn} 로 고정되어 더 안 줄어든다.')
    beyond = sum(1 for v in final_list if abs(v) >= knee)
    print(f'  최종 각도가 그 무릎을 넘는 프레임: {100.0*beyond/len(final_list):5.1f}%')

    if diags:
        report_pipeline(diags)

    print('\n' + '=' * 78)
    print('6. 판정')
    print('=' * 78)
    verdicts = []
    lane_hz = rate_of(offsets)
    if lane_hz < 12.0:
        verdicts.append(
            f'[인지 주기] /lane_offset {lane_hz:.1f} Hz. 프레임 간격 '
            f'{1000.0/lane_hz if lane_hz else 0:.0f}ms 라 코너 진입 반응이 그만큼 늦다. '
            f'lane_debug:=false 로 imshow 부하부터 뺄 것.')
    if 100.0 * over_clamp / len(mag) > 15.0:
        verdicts.append(
            f'[clamp 포화] |offset| 이 clamp({Ld:.0f}px) 를 넘는 프레임이 '
            f'{100.0*over_clamp/len(mag):.1f}%. 그 구간에서는 코너가 깊어져도 '
            f'조향이 전혀 안 늘어난다. lookahead_px 를 넓혀야 한다.')
    if 100.0 * sat / len(pursuit_list) > 15.0:
        verdicts.append(
            f'[PP 상한] PP 가 상한 {pp_ceiling:.1f} 의 85% 이상인 프레임이 '
            f'{100.0*sat/len(pursuit_list):.1f}%. wheelbase_px/lookahead_px 비를 '
            f'키우지 않으면 더 못 꺾는다.')
    if guardrails and have and 100.0 * len(have) / len(usable) < 50.0:
        verdicts.append(
            f'[가드레일 미관측] 좌우 실선을 신뢰 가능하게 본 프레임이 '
            f'{100.0*len(have)/len(usable):.1f}% 뿐이다. 상한을 넘길 유일한 항이 '
            f'대부분 hold/decay 로만 살아 있다.')
    if confs and 100.0 * zero_conf / len(confs) > 60.0:
        verdicts.append(
            f'[preview 무효] /lane_path_preview confidence 가 0 인 프레임이 '
            f'{100.0*zero_conf/len(confs):.1f}%. 포화가 아니라 lane_node 가 2차 '
            f'피팅에 실패하고 있다 — path_preview_min_windows(7) / '
            f'min_span_ratio(0.45) / max_rmse_px(25) 조건을 보라.')
    elif 100.0 * dead_preview / len(preview_gain_list) > 60.0:
        verdicts.append(
            f'[선행조향 포화] preview 기여가 0.5deg 미만인 프레임이 '
            f'{100.0*dead_preview/len(preview_gain_list):.1f}%. confidence 는 살아 '
            f'있으므로 base 가 상한에 걸려 preview 가 밀려난 것이다.')
    if not verdicts:
        print('  뚜렷한 포화 신호가 없다. 곡선 구간 샘플이 실제로 들어갔는지 '
              '(위 offset 분포가 직선과 다른지) 먼저 확인할 것.')
    for i, v in enumerate(verdicts, 1):
        print(f'  {i}) {v}')
    print()
    return 0


if __name__ == '__main__':
    sys.exit(main())
