"""Launch M20 in Gazebo Classic with ros2_control, sensors and controllers.

Args:
  world:  scene name (corridor | plaza | office) or absolute path to an SDF
  gui:    start gzclient (needs X; WSL is headless -> default false)
  spawn_x / spawn_y / spawn_z: robot spawn pose (z = base_link height)
  hipy / knee: standing pose values passed to stand_node
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler,
                            SetEnvironmentVariable, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _resolve_world(context, *args, **kwargs):
    pkg = get_package_share_directory("m20_description")
    scene = context.launch_configurations.get("world", "corridor")
    if os.path.isabs(scene) or scene.endswith(".sdf"):
        context.launch_configurations["world_path"] = scene
    else:
        context.launch_configurations["world_path"] = os.path.join(
            pkg, "worlds", f"{scene}.sdf")
    return []


def generate_launch_description():
    pkg = get_package_share_directory("m20_description")
    with open(os.path.join(pkg, "urdf", "m20.urdf")) as f:
        urdf_str = f.read()
    robot_description = ParameterValue(urdf_str, value_type=str)
    urdf_path = os.path.join(pkg, "urdf", "m20.urdf")

    gazebo_launch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("gazebo_ros"),
                     "launch", "gzserver.launch.py"))

    # ros2_control lives inside gzserver (gazebo_ros2_control plugin hosts its
    # own controller_manager). Spawners run strictly sequentially: concurrent
    # load/configure/activate requests race inside controller_manager 2.54 and
    # end with "can not be configured from 'active' state".
    # Controller params are passed explicitly: the controller nodes inside the
    # gzserver-hosted controller_manager do not inherit the plugin's context
    # parameters.
    ctrl_yaml = os.path.join(pkg, "config", "gazebo_ros2_control.yaml")
    spawn_jsb = Node(
        package="controller_manager", executable="spawner",
        arguments=["--param-file", ctrl_yaml, "joint_state_broadcaster"],
        output="screen")
    spawn_dd = Node(
        package="controller_manager", executable="spawner",
        arguments=["--param-file", ctrl_yaml, "diff_drive_controller"],
        output="screen")

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="corridor"),
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("spawn_x", default_value="0.0"),
        DeclareLaunchArgument("spawn_y", default_value="0.0"),
        DeclareLaunchArgument("spawn_z", default_value="0.6"),
        DeclareLaunchArgument("hipy", default_value="-0.7"),
        DeclareLaunchArgument("knee", default_value="1.4"),

        SetEnvironmentVariable(
            name="GAZEBO_PLUGIN_PATH",
            value=":".join([
                os.path.join(get_package_share_directory("gazebo_ros2_control"),
                             "..", "..", "lib"),
                os.environ.get("GAZEBO_PLUGIN_PATH", ""),
                "/opt/ros/humble/lib",
            ])),

        OpaqueFunction(function=_resolve_world),

        # Gazebo server (gzserver.launch.py sets GAZEBO_* paths + ros plugins)
        IncludeLaunchDescription(
            gazebo_launch,
            launch_arguments={
                "world": LaunchConfiguration("world_path"),
                "verbose": "true",
            }.items(),
        ),

        # Spawn the robot
        Node(
            package="gazebo_ros", executable="spawn_entity.py",
            arguments=[
                "-file", urdf_path,
                "-entity", "m20",
                "-x", LaunchConfiguration("spawn_x"),
                "-y", LaunchConfiguration("spawn_y"),
                "-z", LaunchConfiguration("spawn_z"),
            ],
            output="screen",
        ),

        # State publishers
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description,
                         "use_sim_time": True}],
            output="screen",
        ),
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            arguments=["0", "0", "0.20", "0", "0", "0", "base_link", "lidar_link"],
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "base_link", "imu_link"],
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),

        # Controllers (sequential spawn)
        spawn_jsb,
        RegisterEventHandler(
            OnProcessExit(target_action=spawn_jsb, on_exit=[spawn_dd])),

        # ring field completion for the ray-sensor point cloud
        Node(
            package="ring_fill_node",
            executable=os.path.join(
                get_package_share_directory("ring_fill_node"),
                "..", "..", "bin", "ring_fill_node"),
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),

        # Standing pose (used only when leg joints are re-enabled in
        # adapt_urdf.py; the fixed-leg fallback has no leg controllers)
        TimerAction(
            period=15.0,
            actions=[Node(
                package="m20_description", executable="stand_node.py",
                parameters=[{"hipy": LaunchConfiguration("hipy"),
                             "knee": LaunchConfiguration("knee"),
                             "use_sim_time": True}],
                output="screen",
            )],
        ),
        LogInfo(msg=["world_path=", LaunchConfiguration("world_path")]),
    ])
