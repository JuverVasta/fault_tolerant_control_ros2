#!/bin/bash
# set_yaw.sh <yaw_rad> [mav_name] — sets desired heading (nominal mode only).
if [ $# -lt 1 ]; then
  echo "Usage: $0 <yaw_radians> [mav_name]"
  exit 1
fi
NS=${2:-hummingbird}
ros2 topic pub --once /$NS/heading_design std_msgs/msg/Float64 "{data: $1}"
echo "[set_yaw] heading $1 rad sent to $NS"
