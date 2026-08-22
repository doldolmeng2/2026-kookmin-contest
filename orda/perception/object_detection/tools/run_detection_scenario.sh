#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# run_detection_scenario.sh
#
# 역할: object_detection 패키지(resize_node + lane_node + object_yolo_node +
#       object_node)를 CAMERA VIEW 디버그 창 켠 채로 띄운다. rosbag 재생은
#       이 스크립트가 하지 않는다 — 별도 터미널에서 아래 명령으로 직접 실행할 것:
#
#           ros2 bag play <bag_path> --clock --topics /image_raw /scan
#
# CAMERA VIEW에서 보이는 것:
#   초록 박스       : object_info 판정에 실제로 쓰이는 가장 가까운 차량 박스
#                      (+ 차선 dx 텍스트)
#   초록/청록/주황 등: 신호등 박스 (색상별 상태)
#
# 사용법:
#   ./run_detection_scenario.sh
#   (다른 터미널) ros2 bag play ~/my_rosbag/rosbag2_2026_08_13-13_33_23 --clock --topics /image_raw /scan
# ─────────────────────────────────────────────────────────────────────────────
set -euo pipefail

WS_DIR="$HOME/xycar_ws_e8ffd93"

# ROS의 setup.bash 들이 내부적으로 미설정 변수를 참조해서 set -u 와 충돌한다.
# source 하는 동안만 잠깐 끄고 바로 되돌린다.
set +u
source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"
set -u

echo "== object_detection_test.py 실행 (CAMERA VIEW 켬) =="
echo "== 다른 터미널에서 bag을 재생하세요: =="
echo "==   ros2 bag play <bag_path> --clock --topics /image_raw /scan =="

exec ros2 launch main object_detection_test.py enable_gui:=true
