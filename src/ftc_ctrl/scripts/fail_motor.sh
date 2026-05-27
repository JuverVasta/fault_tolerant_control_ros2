#!/bin/bash
# fail_motor.sh [mav_name] — triggers the motor-failure mode.
# Which motor fails is set by the 'failed_prop' parameter in the YAML config.
NS=${1:-hummingbird}
echo "[fail_motor] triggering motor failure on $NS"
ros2 topic pub --once /$NS/stop_rotors std_msgs/msg/Empty "{}"
echo "[fail_motor] failure mode active. The drone should start spinning."
