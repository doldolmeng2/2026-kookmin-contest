#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# run_stack_sweep.sh
#
# 역할: 여러 bag 에 대해 run_stack_eval.sh 를 차례로 돌리고, 결과를 한 장의 표로
#       모은다.
#
# 목적은 "이 bag 에서 된다" 가 아니라 "배치가 바뀌어도 된다" 를 보는 것이다.
# 본선에서는 고정장애물 위치·좌회전 신호 타이밍·라바콘 배치가 모두 달라지므로,
# 같은 판정이 여러 녹화에서 유지되는지가 실제로 알고 싶은 값이다.
#
#   ros2 run drive_eval run_stack_sweep.sh
#   BAGS="rosbag2_shortcut_1:1 rosbag2_straight:1" ros2 run drive_eval run_stack_sweep.sh
#
# BAGS 항목 형식은 `<bag 디렉터리 이름>:<main_node 초기 모드>` 다.
#   0 = WAIT_GREEN  전체 레이스 녹화 (출발 신호부터 있는 bag)
#   1 = LANE_DRIVE  구간만 잘라 둔 시나리오 클립 (출발 신호가 없다)
# ─────────────────────────────────────────────────────────────────────────────
set -o pipefail

ROOT="${ROOT:-$HOME/my_rosbag}"
SWEEP="${SWEEP:-$HOME/drive_eval_runs/sweep_$(date +%Y%m%d_%H%M%S)}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUNNER="${RUNNER:-$HERE/run_stack_eval.sh}"

# 기본 목록: 카메라가 충분한 속도로 들어있는 bag 만 고른다. /resized_image 가
# 5 Hz 밖에 없는 녹화는 lane_node 가 굶어서 스택 판단이 아니라 입력 부족을
# 재게 된다.
DEFAULT_BAGS="
rosbag2_2026_08_13-13_33_23:0
manual_20260818_134220:1
rosbag2_2026_08_06-13_27_27:1
rosbag2_carlane_1:1
rosbag2_carlane_2:1
rosbag2_carrlane_middle:1
rosbag2_fixed_obstacles_overtake_1:1
rosbag2_fixed_obstacles_overtake_2:1
rosbag2_fixed_obstacles_pass_1:1
rosbag2_fixed_obstacles_pass_2:1
rosbag2_fixed_obstacles_stop_1:1
rosbag2_fixed_obstacles_stop_2:1
rosbag2_shortcut_1:1
rosbag2_straight:1
rosbag2_traffi_4_2026_07_24-14_42_20:1
"

BAGS="${BAGS:-$DEFAULT_BAGS}"

source /opt/ros/jazzy/setup.bash
source "$HOME/xycar_ws/install/setup.bash"

mkdir -p "$SWEEP"
echo "sweep dir: $SWEEP"

for entry in $BAGS; do
  name="${entry%%:*}"
  mode="${entry##*:}"
  [ "$mode" = "$name" ] && mode=1
  bag="$ROOT/$name"

  if [ ! -d "$bag" ]; then
    echo "!! skipping $name (no such bag)"
    continue
  fi

  echo
  echo "############################################################"
  echo "# $name  (mode=$mode)"
  echo "############################################################"
  BAG="$bag" OUT="$SWEEP/$name" MODE="$mode" PROFILE=0 bash "$RUNNER" \
    > "$SWEEP/$name.log" 2>&1
  status=$?
  echo "$name finished with status $status"
  tail -n 14 "$SWEEP/$name.log"
done

echo
echo "############################################################"
echo "# sweep summary"
echo "############################################################"
ros2 run drive_eval drive_eval summary --runs "$SWEEP" 2>&1 \
  | tee "$SWEEP/summary.txt"
STATUS=${PIPESTATUS[0]}

echo
echo "sweep dir: $SWEEP"
echo "summary:   $SWEEP/summary.txt"
exit "$STATUS"
