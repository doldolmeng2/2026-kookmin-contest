#!/usr/bin/env python3
"""pure_pursuit_tuner.html 이 쓰는 기준값을 실제 Controller 로 다시 만들어 대조한다.

HTML 안의 JS 는 control.py 의 _compute_steering_pure_pursuit() 를 손으로 옮긴
것이라 원본이 바뀌면 조용히 어긋난다. 이 스크립트는 실제 Controller 를 돌려
얻은 기준 벡터를 HTML 에 박힌 것과 비교하고, --update 를 주면 갱신한다.

사용법:
    python3 pure_pursuit_tuner_verify.py            # 대조만
    python3 pure_pursuit_tuner_verify.py --update   # 어긋나면 HTML 갱신

PYTHONPATH 에 main 패키지가 있어야 한다. 워크스페이스 루트에서:
    PYTHONPATH=install/main/lib/python3.12/site-packages \
        python3 src/orda/main/tools/pure_pursuit_tuner_verify.py
"""
import argparse
import json
import math
import pathlib
import re
import sys

HTML = pathlib.Path(__file__).with_name('pure_pursuit_tuner.html')
TAG = re.compile(r'(<script type="application/json" id="d-ref">)(.*?)(</script>)', re.S)

# HTML 의 PRESETS 와 같은 값이어야 한다.
PRESETS = {
    'src':     {'lookahead_px': 80.0,  'wheelbase_px': 20.0,
                'steering_gain': 0.85, 'max_steering_angle': 100.0},
    'install': {'lookahead_px': 120.0, 'wheelbase_px': 30.0,
                'steering_gain': 1.0,  'max_steering_angle': 40.0},
}
PROBE = [-800, -400, -200, -121, -120, -119, -81, -80, -79, -40, -13, 0,
         7, 25, 53, 79, 80, 81, 119, 120, 121, 200, 400, 694, 800]


def build():
    from main.control import Controller

    def ceiling(p):
        atan = math.degrees(math.atan(p['wheelbase_px'] / p['lookahead_px']))
        return min(atan * p['steering_gain'], p['max_steering_angle'])

    vectors = []
    for name, params in PRESETS.items():
        c = Controller()
        c.pure_pursuit_params = dict(params)
        vectors.append({
            'name': name,
            'params': params,
            'ceiling_deg': round(ceiling(params), 6),
            'out': [round(c._compute_steering_pure_pursuit(o), 12) for o in PROBE],
        })
    return {'probe': PROBE, 'vectors': vectors}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--update', action='store_true',
                    help='어긋나면 HTML 의 기준값을 갱신한다')
    args = ap.parse_args()

    try:
        fresh = build()
    except ImportError as exc:
        print(f'main.control 을 import 할 수 없다: {exc}', file=sys.stderr)
        print('PYTHONPATH=install/main/lib/python3.12/site-packages 를 붙여서 실행할 것.',
              file=sys.stderr)
        return 2

    text = HTML.read_text()
    m = TAG.search(text)
    if not m:
        print('HTML 에서 d-ref 블록을 찾지 못했다.', file=sys.stderr)
        return 2
    embedded = json.loads(m.group(2))

    bad = []
    if embedded.get('probe') != fresh['probe']:
        bad.append('probe 목록이 다르다')
    emb = {v['name']: v for v in embedded.get('vectors', [])}
    for v in fresh['vectors']:
        e = emb.get(v['name'])
        if e is None:
            bad.append(f"{v['name']}: HTML 에 없다")
            continue
        if e['params'] != v['params']:
            bad.append(f"{v['name']}: 프리셋 파라미터가 다르다 "
                       f"(HTML {e['params']} vs 실제 {v['params']})")
        for o, a, b in zip(fresh['probe'], e['out'], v['out']):
            if abs(a - b) > 1e-9:
                bad.append(f"{v['name']} offset={o}: HTML {a} vs Controller {b}")

    if not bad:
        n = len(fresh['probe']) * len(fresh['vectors'])
        print(f'일치 — 기준값 {n}개가 실제 Controller 출력과 같다.')
        return 0

    print(f'불일치 {len(bad)}건:', file=sys.stderr)
    for b in bad[:20]:
        print('  -', b, file=sys.stderr)
    if len(bad) > 20:
        print(f'  ... 외 {len(bad)-20}건', file=sys.stderr)

    if args.update:
        HTML.write_text(TAG.sub(
            lambda mm: mm.group(1) + json.dumps(fresh, separators=(',', ':')) + mm.group(3),
            text, count=1))
        print(f'{HTML.name} 의 기준값을 갱신했다.')
        return 0

    print('--update 를 주면 HTML 을 갱신한다.', file=sys.stderr)
    return 1


if __name__ == '__main__':
    sys.exit(main())
