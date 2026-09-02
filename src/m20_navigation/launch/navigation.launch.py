"""M20 Navigation System Launch

Launches:
  - navigation_node: Terrain + Hybrid A* + DWA/LinePlanner + DrDDS bridge
  - occ_grid_bridge: (optional) lightning-lm occ_grid -> /GRID_MAP

Requires separate SLAM or localization nodes to provide /ODOM, TF and /lio_pose.
Usage:
  ros2 launch m20_navigation navigation.launch.py enable_motion_output:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory("m20_navigation")
    config_dir = os.path.join(pkg_dir, "config")

    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=os.path.join(
            config_dir, "native_navigation.yaml"),
            description="Configuration file for navigation_node"),
        DeclareLaunchArgument(
            "enable_motion_output", default_value="false",
            description="Set to true to publish /NAV_CMD (requires DrDDS SDK or ROS fallback)"),
        DeclareLaunchArgument(
            "enable_occ_grid_bridge", default_value="false",
            description="Start occ_grid_bridge to publish lightning-lm grid as /GRID_MAP"),
        DeclareLaunchArgument(
            "grid_path", default_value="",
            description="Path to occ_grid.pgm from lightning-lm mapping"),
        DeclareLaunchArgument("log_level", default_value="info"),

        Node(
            package="m20_navigation",
            executable="navigation_node",
            name="navigation_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                LaunchConfiguration("config"),
                {"navigation.enable_motion_output": LaunchConfiguration("enable_motion_output")},
            ],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),

        Node(
            package="m20_navigation",
            executable="occ_grid_bridge",
            name="occ_grid_bridge",
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_occ_grid_bridge")),
            emulate_tty=True,
            parameters=[{
                "grid_path": LaunchConfiguration("grid_path"),
                "frame_id": "map",
            }],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
    ])