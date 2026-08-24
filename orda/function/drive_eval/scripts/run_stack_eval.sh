#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# run_stack_eval.sh
#
# 역할: bag 의 센서 토픽만 재생해 xycar_ws 스택을 돌리고, 스택이 내놓은 조향·모드·
#       라바콘 판단을 별도 bag 으로 기록한 뒤 drive_eval 로 채점한다.
#
# bag 에서 가져오는 것은 상황(센서)뿐이다. 녹화된 조이콘 조향/속도
# (/joy, /xycar_motor, /commands/*) 는 재생하지 않는다 — 그것들은 채점 기준이지
# 입력이 아니다.
#
#   ros2 run drive_eval run_stack_eval.sh
#   DURATION=30 START_OFFSET=12 ros2 run drive_eval run_stack_eval.sh
# ─────────────────────────────────────────────────────────────────────────────
set -o pipefail

BAG="${BAG:-$HOME/my_rosbag/rosbag2_2026_08_13-13_33_23}"
OUT="${OUT:-$HOME/drive_eval_runs/run_$(date +%Y%m%d_%H%M%S)}"
WS="${WS:-$HOME/xycar_ws}"
PROFILE="${PROFILE:-0}"          # 0 = race (production 초기화 경로를 그대로 쓴다)
# main_node 초기 모드. test_profile=0(race) 은 이 값을 그대로 쓴다.
# 0 = WAIT_GREEN: 출발 신호를 WAIT_GREEN 이 소비하므로 랩 카운트가 실전과 같다.
#     module_drive.py(실전)의 기본값도 0 이라 여기서도 0 을 기본으로 둔다.
# 1 = LANE_DRIVE: 첫 신호등 목격이 곧바로 1랩으로 세어져 FINISH 가 한 바퀴 일찍 뜬다.
#     module_drive_bag_test.py 의 기본값이 1 이므로 그냥 띄우면 이 함정에 빠진다.
MODE="${MODE:-0}"
START_OFFSET="${START_OFFSET:-0}"
DURATION="${DURATION:-0}"        # 0 = 끝까지
WARMUP="${WARMUP:-50}"           # PIDNet/YOLO 워밍업 대기 [s]
CLOCK_HZ="${CLOCK_HZ:-200}"
DOMAIN="${ROS_DOMAIN_ID:-77}"
# 런치에 그대로 덧붙일 인자. 인지 파라미터 하나를 바꿔가며 재실행할 때 쓴다.
#   EXTRA_LAUNCH_ARGS="rubbercone_max_corridor_width:=1.10"
EXTRA_LAUNCH_ARGS="${EXTRA_LAUNCH_ARGS:-}"

# bag 이 스택에 넣어 주는 입력. 조향/속도 토픽은 일부러 빠져 있다.
INPUT_TOPICS=(/scan /resized_image /xycar_ultrasonic /image_raw /camera_info)

# 스택이 내놓는 판단. /clock 은 두 bag 의 시간축을 맞추는 데 반드시 필요하다.
OUTPUT_TOPICS=(
  /clock
  /kmu_main_offline/xycar_motor
  /mode_info
  /lane_info
  /rubbercone_info
  /rubbercone_offset
  /rubbercone_session_active
  /lane_offset
  /lane_fit
  /lane_change_state
  /lane_position
  /object_info
  /object_info_raw
  /road_surface
  /internal/lane_command
  /side_clearance
)

# 런치가 띄우는 노드 실행 파일. 이름으로 확실히 죽여야 다음 실행에 겹치지 않는다.
STACK_PROCESSES=(
  module_drive_bag_test
  main_node
  lane_node
  rubbercone_node
  object_node
  object_yolo_node
  pidnet_inference
  road_surface_node
  kmu_preflight
)

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID="$DOMAIN"

mkdir -p "$OUT"
echo "run dir: $OUT"
echo "bag:     $BAG"

# 스택을 확실히 내린다. `ros2 launch` 에 INT 를 보내는 것만으로는 노드가 남을 수
# 있고, 남은 노드는 다음 실행에서 같은 토픽에 같이 발행해 결과를 통째로 망친다
# (실제로 sweep 첫 시도에서 main_node 가 6개까지 쌓였다).
stop_stack() {
  local pattern
  for pattern in "${STACK_PROCESSES[@]}"; do
    pkill -INT -f "$pattern" 2>/dev/null
  done
  local waited=0
  while [ "$waited" -lt 15 ]; do
    if ! pgrep -f main_node > /dev/null 2>&1; then
      return 0
    fi
    sleep 1
    waited=$((waited + 1))
  done
  for pattern in "${STACK_PROCESSES[@]}"; do
    pkill -KILL -f "$pattern" 2>/dev/null
  done
  sleep 2
}

STACK_PID=""
REC_PID=""

# 녹화는 반드시 스스로 끝나게 둔다. SIGKILL 로 잘라내면 mcap 이 마무리되지 않아
# 파일 전체를 못 읽는다 — 200초를 돌려놓고 채점 단계에서 알게 된다.
stop_recorder() {
  [ -z "$REC_PID" ] && return 0
  kill -INT "$REC_PID" 2>/dev/null
  local waited=0
  while [ "$waited" -lt 30 ]; do
    if ! kill -0 "$REC_PID" 2>/dev/null; then
      return 0
    fi
    sleep 1
    waited=$((waited + 1))
  done
  echo "recorder did not exit in 30 s; the run bag may be unreadable" >&2
  kill -TERM "$REC_PID" 2>/dev/null
  sleep 5
}

cleanup() {
  echo "== stopping =="
  stop_recorder
  [ -n "$STACK_PID" ] && kill -INT "$STACK_PID" 2>/dev/null
  stop_stack
  wait 2>/dev/null
}
trap cleanup EXIT

echo "== clearing any stack left over from an earlier run =="
stop_stack

echo "== launching stack (test_profile=$PROFILE, mode=$MODE) =="
ros2 launch main module_drive_bag_test.py \
  test_profile:="$PROFILE" \
  mode:="$MODE" \
  show_debug:=false \
  object_enable_gui:=false \
  rubbercone_enable_gui:=false \
  $EXTRA_LAUNCH_ARGS \
  > "$OUT/stack.log" 2>&1 &
STACK_PID=$!

echo "== waiting ${WARMUP}s for PIDNet/YOLO warm-up =="
sleep "$WARMUP"
ros2 node list --spin-time 3 > "$OUT/nodes.txt" 2>&1
echo "-- nodes --"
cat "$OUT/nodes.txt"

MAIN_COUNT=$(grep -c '^/main_node$' "$OUT/nodes.txt")
if [ "$MAIN_COUNT" -eq 0 ]; then
  echo "main_node did not come up; see $OUT/stack.log" >&2
  exit 1
fi
if [ "$MAIN_COUNT" -gt 1 ]; then
  # 같은 토픽에 두 FSM 이 발행하면 모드 타임라인이 뒤섞여 리포트가 통째로
  # 거짓이 된다. 조용히 진행하느니 여기서 멈춘다.
  echo "$MAIN_COUNT main_node instances are alive; refusing to record a run that" >&2
  echo "would mix two FSMs on the same topics" >&2
  exit 1
fi

echo "== recording stack output =="
ros2 bag record -o "$OUT/outbag" "${OUTPUT_TOPICS[@]}" > "$OUT/record.log" 2>&1 &
REC_PID=$!
sleep 3

PLAY_ARGS=(--clock "$CLOCK_HZ" --topics "${INPUT_TOPICS[@]}")
[ "$START_OFFSET" != "0" ] && PLAY_ARGS+=(--start-offset "$START_OFFSET")
[ "$DURATION" != "0" ] && PLAY_ARGS+=(--playback-duration "$DURATION")

echo "== playing sensors: ${PLAY_ARGS[*]} =="
ros2 bag play "$BAG" "${PLAY_ARGS[@]}" 2>&1 | tee "$OUT/play.log"
echo "== playback finished; flushing recorder =="
sleep 5

cleanup
trap - EXIT

if ! ros2 bag info "$OUT/outbag" > "$OUT/outbag_info.txt" 2>&1; then
  echo "the recorded run bag is unreadable; see $OUT/outbag_info.txt" >&2
  echo "(a recorder cut short before it finalised its file leaves it like this)" >&2
  exit 1
fi

echo "== evaluating =="
ros2 run drive_eval drive_eval evaluate \
  --source-bag "$BAG" \
  --run-bag "$OUT/outbag" \
  --json "$OUT/report.json" 2>&1 | tee "$OUT/report.txt"
STATUS=${PIPESTATUS[0]}

echo
echo "run dir:  $OUT"
echo "report:   $OUT/report.txt"
exit "$STATUS"
