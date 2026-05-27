#!/bin/bash
# takeoff.sh [mav_name] — arms the controller and sends an initial hover waypoint.
NS=${1:-hummingbird}
echo "[takeoff] arming controller ($NS)..."
ros2 topic pub --once /$NS/start_rotors std_msgs/msg/Empty "{}"
sleep 1
echo "[takeoff] sending hover waypoint (0, 0, 1.0)"
ros2 topic pub --once /$NS/reference_pos geometry_msgs/msg/Point "{x: 0.0, y: 0.0, z: 1.0}"
echo "[takeoff] done."
