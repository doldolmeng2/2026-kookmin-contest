#!/usr/bin/env python3
"""완성된 YOLO 라벨을 이미지에 그려 컨택트시트로 확인한다.

학습을 돌리기 전 라벨이 실제로 물체에 맞는지 눈으로 검증하는 용도.
박스가 있는 프레임 중에서 균등 간격으로 뽑는다.

사용 예:
    python3 preview_labels.py --images ~/yolo_raw --labels ~/yolo_raw_labels --out /tmp/final.jpg
"""
import argparse
import glob
import os

import cv2
import numpy as np


def load_boxes(path, W, H):
    if not os.path.exists(path):
        return []
    out = []
    with open(path) as fh:
        for line in fh:
            p = line.split()
            if len(p) != 5:
                continue
            cx, cy, w, h = (float(v) for v in p[1:])
            out.append((int((cx - w / 2) * W), int((cy - h / 2) * H),
                        int(w * W), int(h * H)))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", required=True)
    ap.add_argument("--labels", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--count", type=int, default=12)
    ap.add_argument("--empty", action="store_true",
                    help="박스가 있는 프레임 대신 빈 라벨 프레임을 본다")
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(args.images, "*.png")))
    picked = []
    for f in files:
        lbl = os.path.join(args.labels,
                           os.path.splitext(os.path.basename(f))[0] + ".txt")
        has = os.path.exists(lbl) and os.path.getsize(lbl) > 0
        if has != args.empty:
            picked.append((f, lbl))

    if not picked:
        raise SystemExit("해당하는 프레임이 없습니다")

    idxs = np.linspace(0, len(picked) - 1, min(args.count, len(picked))).astype(int)
    tiles = []
    for i in idxs:
        f, lbl = picked[i]
        img = cv2.imread(f)
        H, W = img.shape[:2]
        for x, y, w, h in load_boxes(lbl, W, H):
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(img, os.path.basename(f)[-9:-4], (6, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        tiles.append(img)

    cols = 4
    while len(tiles) % cols:
        tiles.append(np.zeros_like(tiles[0]))
    rows = [np.hstack(tiles[r:r + cols]) for r in range(0, len(tiles), cols)]
    cv2.imwrite(args.out, np.vstack(rows))
    print(f"{len(picked)}장 중 {len(idxs)}장 -> {args.out}")


if __name__ == "__main__":
    main()
