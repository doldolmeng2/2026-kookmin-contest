#!/usr/bin/env python3
"""빨간 방해차량의 라벨 '초안'을 HSV 색 분할로 자동 생성한다.

이건 완성된 라벨이 아니라 초안이다. 반드시 labelImg 등으로 검수해야 한다.
- 근거리(차가 큼): 대체로 정확
- 중거리: 마스크가 조각나므로 팽창 후 병합해서 하나로 합친다
- 원거리(차가 수십 픽셀): 자주 놓친다 → 손으로 추가해야 함

출력은 YOLO 형식 txt (클래스 0, 정규화 cx cy w h).
차가 안 보이는 프레임은 빈 txt 를 만든다. 빈 라벨도 학습에 필요하다
(배경 오검출을 줄이는 negative sample 역할).

사용 예:
    python3 autolabel_red.py --images ~/yolo_raw --labels ~/yolo_raw_labels
    python3 autolabel_red.py --images ~/yolo_raw --preview check.jpg
"""
import argparse
import glob
import os

import cv2
import numpy as np

# 빨강은 HSV 색상환의 양 끝에 걸쳐 있어 구간을 두 개 잡는다.
RED_LO_1, RED_HI_1 = (0, 90, 60), (10, 255, 255)
RED_LO_2, RED_HI_2 = (170, 90, 60), (180, 255, 255)

ROI_TOP_RATIO = 0.35   # 이 비율 위쪽은 무시 (천장/책상 등 잡동사니 제거)
MERGE_KERNEL = 21      # 조각난 마스크를 하나로 붙이는 팽창 커널
MIN_RED_PX = 40        # 성분이 가져야 하는 최소 빨강 화소 수
MIN_SIDE = 6           # 박스 최소 변 길이(px)
PAD_RATIO = 0.12       # 박스를 상하좌우로 넓히는 비율(범퍼/스포일러 포함용)


def red_mask(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m = cv2.bitwise_or(cv2.inRange(hsv, RED_LO_1, RED_HI_1),
                       cv2.inRange(hsv, RED_LO_2, RED_HI_2))
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k)
    m[:int(m.shape[0] * ROI_TOP_RATIO), :] = 0
    return m


def find_boxes(bgr):
    """빨강 성분을 병합해 차량 후보 박스 목록을 돌려준다."""
    H, W = bgr.shape[:2]
    m = red_mask(bgr)

    # 조각을 크게 팽창시켜 붙인 뒤, 성분별로 '원본' 마스크의 화소만 세고
    # 박스도 원본 마스크 기준으로 잡는다(팽창분만큼 커지지 않게).
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (MERGE_KERNEL, MERGE_KERNEL))
    merged = cv2.dilate(m, k)
    n, labels = cv2.connectedComponents(merged)

    boxes = []
    for i in range(1, n):
        sel = (labels == i) & (m > 0)
        cnt = int(sel.sum())
        if cnt < MIN_RED_PX:
            continue
        ys, xs = np.nonzero(sel)
        x0, x1 = int(xs.min()), int(xs.max())
        y0, y1 = int(ys.min()), int(ys.max())
        w, h = x1 - x0 + 1, y1 - y0 + 1
        if w < MIN_SIDE or h < MIN_SIDE:
            continue
        if not (0.4 <= w / h <= 5.0):     # 차 뒷모습은 가로로 넓다
            continue
        px, py = int(w * PAD_RATIO), int(h * PAD_RATIO)
        x0, y0 = max(0, x0 - px), max(0, y0 - py)
        x1, y1 = min(W - 1, x1 + px), min(H - 1, y1 + py)
        boxes.append((x0, y0, x1 - x0 + 1, y1 - y0 + 1, cnt))

    boxes.sort(key=lambda b: b[4], reverse=True)
    return drop_reflections(boxes)


def drop_reflections(boxes):
    """바닥 반사상으로 보이는 박스를 버린다.

    실내 트랙 바닥이 광택이라 빨간 차가 아래로 비친다. 반사상은 (1) 차 박스
    바로 아래에서 시작하고 (2) 가로로 겹치며 (3) 더 작다. 세 조건이 모두
    맞을 때만 버리므로, 앞뒤로 늘어선 진짜 차 두 대는 살아남는다.
    """
    kept = []
    for b in boxes:          # 빨강 화소가 많은 순 = 진짜 차가 먼저 온다
        bx0, by0, bw, bh, bcnt = b
        bx1, by1 = bx0 + bw, by0 + bh
        reflection = False
        for kx0, ky0, kw, kh, kcnt in kept:
            kx1, ky1 = kx0 + kw, ky0 + kh
            if by0 < ky1 - 0.25 * kh:        # 아래쪽에서 시작하지 않으면 반사 아님
                continue
            overlap = min(bx1, kx1) - max(bx0, kx0)
            if overlap < 0.5 * bw:           # 가로로 겹치지 않으면 반사 아님
                continue
            if bcnt < kcnt:                  # 반사상은 원본보다 흐리고 작다
                reflection = True
                break
        if not reflection:
            kept.append(b)
    return kept


def to_yolo(box, W, H):
    x, y, w, h, _ = box
    return f"0 {(x + w / 2) / W:.6f} {(y + h / 2) / H:.6f} {w / W:.6f} {h / H:.6f}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", required=True)
    ap.add_argument("--labels", help="YOLO txt 를 쓸 디렉터리")
    ap.add_argument("--preview", help="검수용 컨택트시트 경로(.jpg)")
    ap.add_argument("--max-boxes", type=int, default=2,
                    help="프레임당 남길 최대 박스 수(방해차량이 2대인 구간 대비)")
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(args.images, "*.png")))
    if not files:
        raise SystemExit(f"이미지가 없습니다: {args.images}")

    if args.labels:
        os.makedirs(args.labels, exist_ok=True)
        # labelImg 는 라벨 디렉터리의 classes.txt 로 클래스 이름을 읽는다.
        with open(os.path.join(args.labels, "classes.txt"), "w") as fh:
            fh.write("obstacle_car\n")

    hit = 0
    for f in files:
        img = cv2.imread(f)
        H, W = img.shape[:2]
        boxes = find_boxes(img)[:args.max_boxes]
        if boxes:
            hit += 1
        if args.labels:
            stem = os.path.splitext(os.path.basename(f))[0]
            with open(os.path.join(args.labels, stem + ".txt"), "w") as fh:
                fh.write("\n".join(to_yolo(b, W, H) for b in boxes))
                if boxes:
                    fh.write("\n")

    print(f"{len(files)}장 중 {hit}장에서 박스 검출 ({100.0 * hit / len(files):.1f}%)")
    if args.labels:
        print(f"라벨 초안 -> {args.labels}")
        print("!! 반드시 labelImg 로 검수하세요. 특히 원거리 프레임은 누락이 많습니다.")

    if args.preview:
        idxs = np.linspace(0, len(files) - 1, 12).astype(int)
        tiles = []
        for i in idxs:
            img = cv2.imread(files[i])
            for x, y, w, h, cnt in find_boxes(img)[:args.max_boxes]:
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(img, str(cnt), (x, max(12, y - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(img, os.path.basename(files[i])[-9:-4], (6, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            tiles.append(img)
        rows = [np.hstack(tiles[r:r + 4]) for r in range(0, 12, 4)]
        cv2.imwrite(args.preview, np.vstack(rows))
        print(f"미리보기 -> {args.preview}")


if __name__ == "__main__":
    main()
