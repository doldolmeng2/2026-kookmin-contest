#!/usr/bin/env bash
set -euo pipefail

CONTAINER=xycar-noetic-motor
python3 - <<'PY'
import socket
import struct
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
for _ in range(3):
    sock.sendto(struct.pack('!ff', 0.0, 0.0), ('127.0.0.1', 39001))
sock.close()
PY
sleep 0.5
if ! docker inspect "$CONTAINER" >/dev/null 2>&1; then exit 0; fi
if [[ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER")" != true ]]; then exit 0; fi
docker exec "$CONTAINER" bash -lc '
  set -euo pipefail
  export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
  set +u
  source /opt/ros/noetic/setup.bash
  source /ws/devel/setup.bash
  set -u
  rm -f /ws/runtime/.xycar_udp_rx.pid /ws/runtime/.xycar_motor_chain.pid
'
docker stop "$CONTAINER" >/dev/null
echo "Noetic container stopped after zero UDP commands."
