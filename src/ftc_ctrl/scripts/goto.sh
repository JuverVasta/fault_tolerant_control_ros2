#!/bin/bash
# goto.sh <x> <y> <z> [mav_name] — sends a waypoint to the controller.
if [ $# -lt 3 ]; then
  echo "Usage: $0 <x> <y> <z> [mav_name]"
  exit 1
fi
NS=${4:-hummingbird}
ros2 topic pub --once /$NS/reference_pos geometry_msgs/msg/Point "{x: $1, y: $2, z: $3}"
echo "[goto] waypoint ($1, $2, $3) sent to $NS"
