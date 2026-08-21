#!/usr/bin/env python3
"""이미지/라벨을 YOLO 학습용 디렉터리 구조로 분할하고 data.yaml 을 만든다.

핵심: train/val 을 '백 단위'로 나눈다. 같은 백의 연속 프레임은 거의 동일해서
무작위로 섞으면 val 에 train 과 사실상 같은 사진이 들어가고, val mAP 가
부풀려져 실제 성능을 오판하게 된다.

파일명이 '<백이름>_<번호>.png' 라고 가정한다 (extract_frames.py 출력 형식).

사용 예:
    python3 split_dataset.py --images ~/yolo_raw --labels ~/yolo_raw_labels \
        --out ~/yolo_dataset --val-bags rosbag2_fixed_obstacles_pass_2
"""
import argparse
import glob
import os
import shutil


def bag_of(path):
    """'rosbag2_fixed_obstacles_stop_1_00042.png' -> 'rosbag2_fixed_obstacles_stop_1'"""
    return os.path.basename(path).rsplit("_", 1)[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", required=True)
    ap.add_argument("--labels", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--val-bags", nargs="+", required=True,
                    help="검증용으로 통째로 뺄 백 이름들")
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(args.images, "*.png")))
    if not files:
        raise SystemExit(f"이미지가 없습니다: {args.images}")

    bags = sorted({bag_of(f) for f in files})
    unknown = [b for b in args.val_bags if b not in bags]
    if unknown:
        raise SystemExit(f"이런 백이 없습니다: {unknown}\n사용 가능: {bags}")

    for sub in ("images/train", "images/val", "labels/train", "labels/val"):
        os.makedirs(os.path.join(args.out, sub), exist_ok=True)

    counts = {"train": 0, "val": 0}
    missing = 0
    for f in files:
        split = "val" if bag_of(f) in args.val_bags else "train"
        stem = os.path.splitext(os.path.basename(f))[0]
        lbl = os.path.join(args.labels, stem + ".txt")
        if not os.path.exists(lbl):
            missing += 1
            continue
        shutil.copy(f, os.path.join(args.out, "images", split, os.path.basename(f)))
        shutil.copy(lbl, os.path.join(args.out, "labels", split, stem + ".txt"))
        counts[split] += 1

    # classes.txt 가 있으면 그 순서를 그대로 쓴다 (다중 클래스 데이터셋 지원).
    # 없으면 옛 단일 클래스 기본값으로 되돌아간다.
    classes_path = os.path.join(args.labels, "classes.txt")
    if os.path.exists(classes_path):
        with open(classes_path) as fh:
            names = [line.strip() for line in fh if line.strip()]
    else:
        names = ["obstacle_car"]

    yaml_path = os.path.join(args.out, "data.yaml")
    names_yaml = "[" + ", ".join(f'"{n}"' for n in names) + "]"
    with open(yaml_path, "w") as fh:
        fh.write(f"path: {os.path.abspath(args.out)}\n"
                 "train: images/train\n"
                 "val: images/val\n"
                 f"nc: {len(names)}\n"
                 f"names: {names_yaml}\n")

    print(f"train {counts['train']}장 / val {counts['val']}장")
    if missing:
        print(f"라벨이 없어 건너뛴 이미지: {missing}장")
    print(f"검증용 백: {', '.join(args.val_bags)}")
    print(f"data.yaml -> {yaml_path}")


if __name__ == "__main__":
    main()
