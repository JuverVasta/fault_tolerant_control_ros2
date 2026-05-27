"""
simulation.launch.py — single-command launch of the whole simulation.

Starts, in order:
  1. Gazebo Classic with an empty world (gzserver + gzclient).
  2. robot_state_publisher with the quadrotor URDF.
  3. The 'spawn_entity' service call that drops the quadrotor into Gazebo.
  4. The state_interface node (Gazebo odometry <-> controller messages).
  5. The fault-tolerant controller node (ftc_ctrl).

Everything runs inside the 'hummingbird' namespace so the topic names match
what the controller and the Gazebo plugin expect.

Run:
    ros2 launch ftc_bringup simulation.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    GroupAction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_bringup = get_package_share_directory('ftc_bringup')
    pkg_description = get_package_share_directory('ftc_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world_file = os.path.join(pkg_bringup, 'worlds', 'empty.world')
    urdf_xacro = os.path.join(pkg_description, 'urdf', 'hummingbird.urdf.xacro')
    controller_yaml = os.path.join(pkg_bringup, 'config', 'controller.yaml')

    mav_name = LaunchConfiguration('mav_name')

    # ---- 1. Gazebo ----
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file, 'verbose': 'true'}.items(),
    )

    # ---- 2. robot_state_publisher (renders the URDF) ----
    robot_description = Command(['xacro ', urdf_xacro])
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str)}],
    )

    # ---- 3. Spawn the quadrotor 0.1 m above the ground ----
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_quadrotor',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'hummingbird',
            '-z', '0.1',
        ],
    )

    # ---- 4 + 5. Controller + interface, inside the namespace ----
    nodes = GroupAction([
        PushRosNamespace(mav_name),
        Node(
            package='ftc_ctrl',
            executable='state_interface_node',
            name='state_interface',
            output='screen',
            parameters=[controller_yaml],
        ),
        Node(
            package='ftc_ctrl',
            executable='ftc_ctrl_node',
            name='ftc_ctrl',
            output='screen',
            parameters=[controller_yaml],
        ),
    ])

    # Delay the controller a few seconds so Gazebo and the model are ready.
    delayed_nodes = TimerAction(period=5.0, actions=[nodes])

    return LaunchDescription([
        DeclareLaunchArgument('mav_name', default_value='hummingbird'),
        gazebo,
        rsp,
        spawn,
        delayed_nodes,
    ])
