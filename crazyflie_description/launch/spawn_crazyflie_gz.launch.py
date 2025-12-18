#!/usr/bin/env python3
"""
Launch Gazebo Harmonic with the Crazyflie quadrotor model,
camera auto-tracking, and ROS2 ↔ Gazebo bridge.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    # === Paths ===
    pkg_share = get_package_share_directory("crazyflie_description")
    xacro_file = os.path.join(pkg_share, "urdf", "crazyflie_body.xacro")
    world_file = os.path.join(pkg_share, "worlds", "empty_quadcopter.world")
    bridge_config = os.path.join(pkg_share, "config", "ros_gz_crazyflie_bridge.yaml")

    # === Process Xacro → URDF ===
    robot_description_config = xacro.process_file(xacro_file).toxml()

    # === Launch arguments ===
    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="crazyflie",
        description="Name of the Crazyflie robot instance"
    )

    # === Gazebo (Harmonic) launch ===
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            # Use the tuned world with camera tracking
            "gz_args": f"-v 4 {world_file}"
        }.items(),
    )

    # === Spawn the Crazyflie ===
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", LaunchConfiguration("robot_name"),
            "-string", robot_description_config,
            "-allow_renaming", "true",
            "-z", "0.00",  # spawn slightly above ground
        ],
        output="screen",
    )

    # === ROS ↔ Gazebo bridge ===
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        parameters=[{"config_file": bridge_config}],
        output="screen",
    )

    return LaunchDescription([
        robot_name_arg,
        gazebo_launch,
        spawn_robot,
        bridge,
    ])
