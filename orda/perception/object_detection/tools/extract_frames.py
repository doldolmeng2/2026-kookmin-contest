#!/usr/bin/env python3
"""로스백의 /resized_image 를 학습용 이미지로 추출한다.

연속 프레임은 거의 중복이므로 --stride 로 솎아낸다.
출력 파일명은 '<백이름>_<프레임번호>.png' 로, 어느 백에서 왔는지가 남는다.
(train/val 을 백 단위로 나눌 때 이 접두사를 쓴다)

사용 예:
    python3 extract_frames.py --out ~/yolo_raw --stride 4 ~/my_rosbag/rosbag2_fixed_obstacles_*
"""
import argparse
import os

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image


def read_images(bag_dir, topic):
    """로스백에서 Image 메시지를 순서대로 읽어 BGR ndarray 로 돌려준다."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_dir, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))

    while reader.has_next():
        _, data, _ = reader.read_next()
        m = deserialize_message(data, Image)
        arr = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, -1)
        if m.encoding == "rgb8":
            arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        yield arr.copy()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bags", nargs="+")
    ap.add_argument("--out", required=True, help="이미지를 저장할 디렉터리")
    ap.add_argument("--topic", default="/resized_image")
    ap.add_argument("--stride", type=int, default=4, help="N 프레임마다 1장 사용")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    total = 0
    for bag in args.bags:
        name = os.path.basename(bag.rstrip("/"))
        kept = 0
        for i, img in enumerate(read_images(bag, args.topic)):
            if i % args.stride:
                continue
            cv2.imwrite(os.path.join(args.out, f"{name}_{i:05d}.png"), img)
            kept += 1
        print(f"{name}: {kept}장 추출")
        total += kept
    print(f"\n합계 {total}장 -> {args.out}")


if __name__ == "__main__":
    main()
