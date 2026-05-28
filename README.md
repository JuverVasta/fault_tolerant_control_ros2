# Fault-Tolerant Control ROS 2 Humble

Original project for ROS 1 [uzh-rpg/fault_tolerant_control](https://github.com/uzh-rpg/fault_tolerant_control)

```
# Install dependencies
sudo apt install -y python3-colcon-common-extensions libeigen3-dev \
  ros-humble-eigen3-cmake-module ros-humble-joy \
  ros-humble-gazebo-ros-pkgs ros-humble-xacro \
  ros-humble-robot-state-publisher

# Clone and build
git clone https://github.com/JuverVasta/fault_tolerant_control_ros2.git
cd fault_tolerant_control_ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# Run the simulation
ros2 launch ftc_bringup simulation.launch.py

# New terminal
source ~/fault_tolerant_control_ros2/install/setup.bash
ros2 run ftc_ctrl takeoff.sh

# Waypoint flight
ros2 run ftc_ctrl goto.sh 1.0 0.0 1.5

# Motor failure
ros2 run ftc_ctrl fail_motor.sh
```


## License

The controller and messages inherit GPL-3.0 from the original project
uzh-rpg/fault_tolerant_control. New packages (`ftc_gazebo_plugin`,
`ftc_description`, `ftc_bringup`) are GPL-3.0.


