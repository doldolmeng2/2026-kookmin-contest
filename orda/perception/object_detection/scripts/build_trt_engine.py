#!/usr/bin/env python3
"""YOLO 검출기 ONNX -> TensorRT fp16 엔진 빌드.

object_yolo_node 는 이 엔진이 있으면 GPU(TensorRT)로, 없으면 ONNX Runtime CPU 로
돈다. CPU 경로는 실측 560ms/프레임이라 8코어를 통째로 먹고 카메라까지 굶기므로,
실차에서는 반드시 한 번 만들어 두어야 한다.

    ros2 run object_detection build_trt_engine.py

빌드는 약 7~8분 걸린다. 엔진은 GPU 아키텍처(sm)와 TensorRT 버전에 묶이므로,
보드를 바꾸거나 TensorRT 를 올렸으면 다시 만들어야 한다 — 파일 이름에 둘 다
들어 있어 옛 엔진을 잘못 집는 일은 없다.
"""

import argparse
import subprocess
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from object_detection.trt_runtime import default_engine_path

TRTEXEC = "/usr/src/tensorrt/bin/trtexec"


def main() -> int:
    share = Path(get_package_share_directory("object_detection"))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--onnx",
        default=str(share / "model" / "train10_detector_best.onnx"),
        help="검출기 ONNX 경로",
    )
    parser.add_argument("--output", default="", help="엔진 출력 경로 (기본: 캐시)")
    parser.add_argument(
        "--force", action="store_true", help="이미 있어도 다시 빌드한다"
    )
    args = parser.parse_args()

    onnx = Path(args.onnx)
    if not onnx.is_file():
        print(f"ONNX 없음: {onnx}", file=sys.stderr)
        return 1
    if not Path(TRTEXEC).is_file():
        print(f"trtexec 없음: {TRTEXEC}", file=sys.stderr)
        return 1

    out = Path(args.output) if args.output else default_engine_path(str(onnx))
    if out.is_file() and not args.force:
        print(f"이미 있음 (다시 만들려면 --force): {out}")
        return 0
    out.parent.mkdir(parents=True, exist_ok=True)

    print(f"빌드 시작 (7~8분 걸립니다)\n  {onnx}\n  -> {out}")
    result = subprocess.run(
        [
            TRTEXEC,
            f"--onnx={onnx}",
            "--fp16",
            f"--saveEngine={out}",
            "--skipInference",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    if result.returncode != 0 or not out.is_file():
        print(result.stdout[-3000:], file=sys.stderr)
        print("엔진 빌드 실패", file=sys.stderr)
        return 1
    for line in result.stdout.splitlines():
        if "Engine built in" in line:
            print(line.strip())
    print(f"완료: {out} ({out.stat().st_size / 1e6:.1f} MB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
