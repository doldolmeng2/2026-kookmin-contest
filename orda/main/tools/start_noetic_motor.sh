#!/usr/bin/env bash
set -euo pipefail

CONTAINER=xycar-noetic-motor
HOST_DEVICE=/dev/ttyACM0
RUNTIME=/ws/runtime
CHAIN_LOG=/tmp/xycar_motor_chain.log
UDP_LOG=/tmp/xycar_udp_rx.log

fail() { echo "ERROR: $*" >&2; exit 1; }
ros1() {
  docker exec "$CONTAINER" bash -lc '
    set -euo pipefail
    export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
    set +u
    source /opt/ros/noetic/setup.bash
    source /ws/devel/setup.bash
    set -u
    "$@"
  ' bash "$@"
}
nodes() { ros1 timeout 3 rosnode list 2>/dev/null || true; }
has_node() { nodes | grep -Fxq "$1"; }
chain_count() {
  local current; current=$(nodes)
  local count=0 node
  for node in /vesc_driver /ackermann_to_vesc /xycar_motor; do
    grep -Fxq "$node" <<<"$current" && ((count+=1)) || true
  done
  printf '%s\n' "$count"
}
send_zero() {
  python3 - <<'PY'
import socket, struct
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
for _ in range(3): sock.sendto(struct.pack('!ff', 0.0, 0.0), ('127.0.0.1', 39001))
sock.close()
PY
}
telemetry() { ros1 timeout 5 rostopic echo -n 1 /sensors/core; }

docker inspect "$CONTAINER" >/dev/null 2>&1 || fail "missing container $CONTAINER"
[[ -r "$HOST_DEVICE" && -w "$HOST_DEVICE" ]] || fail "$HOST_DEVICE must be readable and writable"
if [[ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER")" != true ]]; then docker start "$CONTAINER" >/dev/null; fi
docker exec "$CONTAINER" test -d /ws || fail "container /ws mount is missing"
docker exec "$CONTAINER" test -x "$RUNTIME/ros1_xycar_udp_rx.py" || fail "UDP receiver is missing"
docker exec "$CONTAINER" ln -sfn /dev/ttyACM0 /dev/ttyMOTOR

count=$(chain_count)
if (( count > 0 && count < 3 )); then
  echo "partial motor chain detected; resetting dedicated container" >&2
  send_zero
  docker stop "$CONTAINER" >/dev/null
  docker start "$CONTAINER" >/dev/null
  docker exec "$CONTAINER" ln -sfn /dev/ttyACM0 /dev/ttyMOTOR
  count=0
fi
if (( count != 3 )); then
  docker exec -d "$CONTAINER" bash -lc '
    set -euo pipefail
    export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
    set +u
    source /opt/ros/noetic/setup.bash
    source /ws/devel/setup.bash
    set -u
    exec roslaunch xycar_motor xycar_motor.launch > /tmp/xycar_motor_chain.log 2>&1
  ' >/dev/null
fi
for _ in $(seq 1 20); do [[ "$(chain_count)" == 3 ]] && break; sleep 1; done
[[ "$(chain_count)" == 3 ]] || { docker exec "$CONTAINER" cat "$CHAIN_LOG" || true; fail "motor chain nodes are not healthy"; }

if ! has_node /xycar_udp_receiver; then
  docker exec -d "$CONTAINER" bash -lc '
    set -euo pipefail
    export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
    set +u
    source /opt/ros/noetic/setup.bash
    source /ws/devel/setup.bash
    set -u
    exec python3 /ws/runtime/ros1_xycar_udp_rx.py > /tmp/xycar_udp_rx.log 2>&1
  ' >/dev/null
fi
for _ in $(seq 1 10); do has_node /xycar_udp_receiver && break; sleep 1; done
has_node /xycar_udp_receiver || { docker exec "$CONTAINER" cat "$UDP_LOG" || true; fail "UDP receiver node is not healthy"; }

TELEMETRY=$(telemetry) || fail "telemetry unavailable"
echo "$TELEMETRY"
echo "$TELEMETRY" | grep -Eq 'speed:' || fail "VESC speed telemetry is missing"
echo "$TELEMETRY" | grep -Eq 'fault_code: 0' || fail "VESC fault_code is not zero"
echo "$TELEMETRY" | grep -Eq 'voltage_input: (9\.|1[0-1]\.)' && echo "WARNING: low battery voltage; zero-only verification only" || true
echo "Noetic motor runtime ready; no motor command was published by this script."
