#!/usr/bin/env bash
# Safe, single-command ROS integration test for the KMU rubber-cone bag.
#
# This harness never starts a hardware driver and never replays recorded motor,
# detector, lane, traffic, or diagnostic topics.  The production main node's
# output remains remapped to /bag_test/xycar_motor by module_drive_bag_test.py.

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ORDA_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WORKSPACE_ROOT="$(cd "${ORDA_ROOT}/../.." && pwd)"
DEFAULT_BAG_PATH="${HOME}/bags/kmu_real_lidar/rubbercone_20260725_221245"
BAG_PATH="${1:-${DEFAULT_BAG_PATH}}"

if (( $# > 1 )); then
    echo "usage: $0 [rubbercone_bag_path]" >&2
    exit 2
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "ROS Humble setup not found: /opt/ros/humble/setup.bash" >&2
    exit 2
fi
if [[ ! -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
    echo "workspace setup not found: ${WORKSPACE_ROOT}/install/setup.bash" >&2
    exit 2
fi
if [[ ! -f "${BAG_PATH}/metadata.yaml" ]]; then
    echo "bag metadata not found: ${BAG_PATH}/metadata.yaml" >&2
    exit 2
fi

# shellcheck disable=SC1091
set +u
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u

RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="${KMU_TEST_OUTPUT_DIR:-/tmp/kmu_rubbercone_fsm_bag_test_${RUN_STAMP}}"
mkdir -p "${RUN_DIR}"

LAUNCH_LOG="${RUN_DIR}/launch.log"
LANE_MOCK_LOG="${RUN_DIR}/lane_mock.log"
TRAFFIC_MOCK_LOG="${RUN_DIR}/traffic_mock.log"
SCAN_MOCK_LOG="${RUN_DIR}/scan_mock.log"
CLOCK_MOCK_LOG="${RUN_DIR}/clock_mock.log"
OBJECT_YOLO_MOCK_LOG="${RUN_DIR}/object_yolo_mock.log"
BAG_PLAY_LOG="${RUN_DIR}/bag_play.log"
MOTOR_LOG="${RUN_DIR}/bag_test_motor.log"
CONE_MOTOR_SAMPLE="${RUN_DIR}/cone_drive_motor_sample.log"
POST_CONE_LANE_MOTOR_SAMPLE="${RUN_DIR}/post_cone_lane_motor_sample.log"
RUBBERCONE_LOG="${RUN_DIR}/rubbercone_info.log"
SUMMARY_LOG="${RUN_DIR}/summary.txt"

LAUNCH_PID=""
LANE_MOCK_PID=""
TRAFFIC_MOCK_PID=""
SCAN_MOCK_PID=""
CLOCK_MOCK_PID=""
OBJECT_YOLO_MOCK_PID=""
MOTOR_ECHO_PID=""
RUBBERCONE_ECHO_PID=""
BAG_PLAY_PID=""

stop_process() {
    local pid="$1"
    local attempt
    local state
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
        kill -TERM "${pid}" 2>/dev/null || true
        for attempt in {1..20}; do
            state="$(ps -o stat= -p "${pid}" 2>/dev/null | tr -d ' ' || true)"
            if [[ -z "${state}" || "${state}" == Z* ]]; then
                wait "${pid}" 2>/dev/null || true
                return
            fi
            sleep 0.1
        done
        # ros2 launch normally terminates its children on TERM.  If its parent
        # is still alive, terminate only those exact test children before the
        # final parent fallback; never use a workspace-wide process pattern.
        pkill -TERM -P "${pid}" 2>/dev/null || true
        kill -KILL "${pid}" 2>/dev/null || true
        wait "${pid}" 2>/dev/null || true
    fi
}

wait_for_test_graph_cleanup() {
    # Fast DDS discovery can outlive the already-terminated OS processes for
    # several seconds.  Wait through that lease/cache window so an immediate
    # repeated session does not fail the preflight on stale node names.
    local deadline=$((SECONDS + 15))
    local nodes
    while (( SECONDS < deadline )); do
        nodes="$(ros2 node list 2>/dev/null || true)"
        if ! grep -Eq '^/(main_node|traffic_node|rubbercone_node|resize_node|lane_node|object_yolo_node|object_node|kmu_test_lane_mock|kmu_test_traffic_mock|kmu_test_scan_mock|kmu_test_clock_mock|kmu_test_object_yolo_mock)$' \
            <<<"${nodes}"; then
            return 0
        fi
        sleep 0.2
    done
    return 1
}

cleanup() {
    local exit_code=$?
    trap - EXIT INT TERM
    stop_process "${BAG_PLAY_PID}"
    stop_process "${MOTOR_ECHO_PID}"
    stop_process "${RUBBERCONE_ECHO_PID}"
    stop_process "${LAUNCH_PID}"
    stop_process "${LANE_MOCK_PID}"
    stop_process "${TRAFFIC_MOCK_PID}"
    stop_process "${SCAN_MOCK_PID}"
    stop_process "${CLOCK_MOCK_PID}"
    stop_process "${OBJECT_YOLO_MOCK_PID}"
    if wait_for_test_graph_cleanup; then
        echo "cleanup_graph=PASS" | tee -a "${SUMMARY_LOG}"
    else
        echo "cleanup_graph=TIMEOUT" | tee -a "${SUMMARY_LOG}" >&2
    fi
    echo "logs: ${RUN_DIR}"
    exit "${exit_code}"
}
trap cleanup EXIT INT TERM

publisher_count() {
    local topic="$1"
    local info
    local count
    info="$(ros2 topic info "${topic}" 2>/dev/null || true)"
    count="$(awk '/Publisher count:/ {print $3; exit}' <<<"${info}")"
    printf '%s' "${count:-0}"
}

assert_no_real_motor_publishers() {
    local phase="$1"
    local count
    count="$(publisher_count /xycar_motor)"
    printf '%s /xycar_motor publisher_count=%s\n' "${phase}" "${count}" \
        | tee -a "${SUMMARY_LOG}"
    if [[ "${count}" != "0" ]]; then
        echo "ABORT: /xycar_motor has a publisher during ${phase}" \
            | tee -a "${SUMMARY_LOG}" >&2
        exit 1
    fi
}

wait_for_publisher_count() {
    local topic="$1"
    local expected="$2"
    local timeout_s="$3"
    local deadline=$((SECONDS + timeout_s))
    while (( SECONDS < deadline )); do
        if [[ "$(publisher_count "${topic}")" == "${expected}" ]]; then
            return 0
        fi
        sleep 0.1
    done
    echo "timed out waiting for ${topic} publisher_count=${expected}" >&2
    return 1
}

wait_for_log() {
    local pattern="$1"
    local timeout_s="$2"
    local deadline=$((SECONDS + timeout_s))
    while (( SECONDS < deadline )); do
        if grep -Fq -- "${pattern}" "${LAUNCH_LOG}" 2>/dev/null; then
            return 0
        fi
        if [[ -n "${LAUNCH_PID}" ]] && ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
            echo "launch exited while waiting for: ${pattern}" >&2
            return 1
        fi
        sleep 0.1
    done
    echo "timed out waiting for launch log: ${pattern}" >&2
    return 1
}

if ! ros2 bag info "${BAG_PATH}" \
    | grep -Eq '(^|[[:space:]])Topic: /scan[[:space:]]+\|'; then
    echo "bag does not contain /scan: ${BAG_PATH}" >&2
    exit 2
fi

if ros2 node list 2>/dev/null \
    | grep -Eq '^/(main_node|traffic_node|rubbercone_node|resize_node|lane_node|object_yolo_node|object_node|kmu_test_lane_mock|kmu_test_traffic_mock|kmu_test_scan_mock|kmu_test_clock_mock|kmu_test_object_yolo_mock)$'; then
    echo "a bag-test launch or KMU mock node is already running; stop it before retrying" >&2
    exit 1
fi

{
    echo "KMU rubber-cone FSM bag integration"
    echo "run_dir=${RUN_DIR}"
    echo "bag=${BAG_PATH}"
    echo "branch=$(git -C "${ORDA_ROOT}" branch --show-current)"
    echo "head=$(git -C "${ORDA_ROOT}" rev-parse --short HEAD)"
    echo "playback_topics=/scan"
} >"${SUMMARY_LOG}"

assert_no_real_motor_publishers "before launch"

# Match the production subscriber contracts exactly. WAIT_GREEN is the
# startup gate, so the empty scan and YOLO mocks prove readiness before the
# recorded /scan starts. The launch uses simulated time, so a constant nonzero
# test clock is also required until rosbag takes ownership of /clock.
python3 -c '
import time

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

rclpy.init()
node = Node("kmu_test_clock_mock")
publisher = node.create_publisher(Clock, "/clock", 10)
started = time.monotonic()
try:
    while rclpy.ok():
        stamp = 1.0 + time.monotonic() - started
        message = Clock()
        message.clock.sec = int(stamp)
        message.clock.nanosec = int((stamp - int(stamp)) * 1_000_000_000)
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(0.02)
finally:
    node.destroy_node()
    rclpy.shutdown()
' >"${CLOCK_MOCK_LOG}" 2>&1 &
CLOCK_MOCK_PID=$!

ros2 topic pub -r 10 -p 100 -n kmu_test_lane_mock \
    --qos-history keep_last --qos-depth 10 \
    --qos-reliability best_effort --qos-durability volatile \
    /lane_offset std_msgs/msg/Int16 '{data: 0}' \
    >"${LANE_MOCK_LOG}" 2>&1 &
LANE_MOCK_PID=$!

ros2 topic pub -r 10 -p 100 -n kmu_test_traffic_mock \
    --qos-history keep_last --qos-depth 10 \
    --qos-reliability best_effort --qos-durability volatile \
    /traffic_detection std_msgs/msg/Int32 '{data: 2}' \
    >"${TRAFFIC_MOCK_LOG}" 2>&1 &
TRAFFIC_MOCK_PID=$!

ros2 topic pub -r 10 -p 100 -n kmu_test_object_yolo_mock \
    --qos-history keep_last --qos-depth 1 \
    --qos-reliability best_effort --qos-durability volatile \
    /object_yolo std_msgs/msg/Float32MultiArray \
    '{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}' \
    >"${OBJECT_YOLO_MOCK_LOG}" 2>&1 &
OBJECT_YOLO_MOCK_PID=$!

ros2 topic pub -r 10 -p 100 -n kmu_test_scan_mock \
    --qos-history keep_last --qos-depth 10 \
    --qos-reliability best_effort --qos-durability volatile \
    /scan sensor_msgs/msg/LaserScan \
    '{angle_min: -1.5708, angle_max: 1.5708, angle_increment: 0.01745, range_min: 0.1, range_max: 12.0, ranges: []}' \
    >"${SCAN_MOCK_LOG}" 2>&1 &
SCAN_MOCK_PID=$!

stdbuf -oL -eL ros2 launch main module_drive_bag_test.py \
    mode:=0 show_debug:=false rubbercone_enable_gui:=false \
    >"${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID=$!

wait_for_publisher_count /bag_test/xycar_motor 1 15
wait_for_log "FSM WAIT_GREEN -> LANE_DRIVE: green signal debounced" 15

stop_process "${SCAN_MOCK_PID}"
SCAN_MOCK_PID=""

ros2 topic echo /bag_test/xycar_motor std_msgs/msg/Float32MultiArray \
    --field data >"${MOTOR_LOG}" 2>&1 &
MOTOR_ECHO_PID=$!
ros2 topic echo /rubbercone_info std_msgs/msg/Int32MultiArray \
    --field data >"${RUBBERCONE_LOG}" 2>&1 &
RUBBERCONE_ECHO_PID=$!

# Mandatory final interlock immediately before playback.
assert_no_real_motor_publishers "before bag playback"

stop_process "${CLOCK_MOCK_PID}"
CLOCK_MOCK_PID=""

echo "command=ros2 bag play ${BAG_PATH} --disable-keyboard-controls --clock --topics /scan" \
    | tee -a "${SUMMARY_LOG}"
ros2 bag play "${BAG_PATH}" --disable-keyboard-controls --clock --topics /scan \
    >"${BAG_PLAY_LOG}" 2>&1 &
BAG_PLAY_PID=$!

wait_for_log "FSM LANE_DRIVE -> CONE_DRIVE: cone entry confirmed" 20

# Capture a non-zero command only after the FSM has committed CONE_DRIVE.
timeout 5s ros2 topic echo /bag_test/xycar_motor \
    std_msgs/msg/Float32MultiArray --field data --once \
    --filter 'len(m) >= 2 and (abs(float(m[0])) > 1e-6 or abs(float(m[1])) > 1e-6)' \
    >"${CONE_MOTOR_SAMPLE}" 2>&1

wait_for_log "FSM CONE_DRIVE -> LANE_DRIVE: fresh cone end flag" 20

# A committed transition is not enough by itself: prove that the continuing
# fresh lane-offset mock is selected after the direct lane-drive return.
timeout 5s ros2 topic echo /bag_test/xycar_motor \
    std_msgs/msg/Float32MultiArray --field data --once \
    --filter 'len(m) >= 2 and (abs(float(m[0])) > 1e-6 or abs(float(m[1])) > 1e-6)' \
    >"${POST_CONE_LANE_MOTOR_SAMPLE}" 2>&1

if ! wait "${BAG_PLAY_PID}"; then
    BAG_PLAY_PID=""
    echo "bag playback failed" >&2
    exit 1
fi
BAG_PLAY_PID=""
sleep 1

assert_no_real_motor_publishers "after bag playback"

if grep -Fq -- "-> STOP:" "${LAUNCH_LOG}"; then
    echo "unexpected terminal STOP transition" | tee -a "${SUMMARY_LOG}" >&2
    exit 1
fi
if [[ ! -s "${CONE_MOTOR_SAMPLE}" ]]; then
    echo "no non-zero isolated motor sample captured in CONE_DRIVE" \
        | tee -a "${SUMMARY_LOG}" >&2
    exit 1
fi
if [[ ! -s "${POST_CONE_LANE_MOTOR_SAMPLE}" ]]; then
    echo "no non-zero isolated lane motor sample captured after cone exit" \
        | tee -a "${SUMMARY_LOG}" >&2
    exit 1
fi

{
    echo "wait_green_to_lane=PASS"
    echo "lane_to_cone=PASS"
    echo "cone_drive_nonzero_bag_test_motor=PASS"
    echo "cone_to_lane=PASS"
    echo "post_cone_lane_nonzero_bag_test_motor=PASS"
    echo "real_xycar_motor_isolated=PASS"
    if grep -Fq "Rubber-cone exit detection armed" "${LAUNCH_LOG}"; then
        echo "rubbercone_exit_armed=PASS"
    else
        echo "rubbercone_exit_armed=NOT_OBSERVED"
    fi
    if grep -Fq "Rubber-cone end latched" "${LAUNCH_LOG}"; then
        echo "rubbercone_end_latched=PASS"
    else
        echo "rubbercone_end_latched=NOT_OBSERVED"
    fi
    echo "result=PASS"
} | tee -a "${SUMMARY_LOG}"
