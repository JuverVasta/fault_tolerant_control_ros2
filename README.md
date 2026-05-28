# Fault-Tolerant Control ROS 2 Humble

Original project for ROS 1 [uzh-rpg/fault_tolerant_control](https://github.com/uzh-rpg/fault_tolerant_control)

```


# 1. Build
cd ftc_ros2_ws
colcon build --symlink-install
source install/setup.bash

# 2. Run the simulation
ros2 launch ftc_bringup simulation.launch.py

# 3. Takeoff in a new terminal
source install/setup.bash
ros2 run ftc_ctrl takeoff.sh

# 4. Waypoint flight
ros2 run ftc_ctrl goto.sh 1.0 0.0 1.5

# 5. Motor failure
ros2 run ftc_ctrl fail_motor.sh
```


## License

The controller and messages inherit GPL-3.0 from the original project
uzh-rpg/fault_tolerant_control. New packages (`ftc_gazebo_plugin`,
`ftc_description`, `ftc_bringup`) are GPL-3.0.


