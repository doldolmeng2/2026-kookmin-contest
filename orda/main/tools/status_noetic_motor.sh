#!/usr/bin/env bash
set -euo pipefail

CONTAINER=xycar-noetic-motor
docker inspect "$CONTAINER" --format 'container={{.State.Status}} image={{.Config.Image}}'
if [[ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER")" != true ]]; then exit 0; fi
docker exec "$CONTAINER" bash -lc '
  set -euo pipefail
  export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
  set +u
  source /opt/ros/noetic/setup.bash
  source /ws/devel/setup.bash
  set -u
  printf "ttyMOTOR="; readlink -f /dev/ttyMOTOR || true
  echo "required motor nodes:"
  timeout 3 rosnode list 2>/dev/null | grep -Fx -e /vesc_driver -e /ackermann_to_vesc -e /xycar_motor -e /xycar_udp_receiver || true
  rostopic info /xycar_motor || true
  timeout 5 rostopic echo -n 1 /sensors/core || true
'
