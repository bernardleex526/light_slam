"""Start mapping or localization from an installed, robot-specific config."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    """Resolve paths and start the selected SLAM executable."""
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ('mapping', 'localization'):
        raise ValueError('mode must be mapping or localization')
    config = os.path.abspath(LaunchConfiguration('config').perform(context))
    directory = os.path.abspath(LaunchConfiguration('work_dir').perform(context))
    if not os.path.isfile(config):
        raise FileNotFoundError(config)
    os.makedirs(directory, exist_ok=True)
    executable = 'run_slam_online' if mode == 'mapping' else 'run_loc_online'
    return [Node(package='lightning', executable=executable,
                 arguments=['--config', config], cwd=directory, output='screen')]


def generate_launch_description():
    """Declare the mode, configuration and writable map directory."""
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping'),
        DeclareLaunchArgument('config', default_value=os.path.join(
            get_package_share_directory('lightning'), 'config', 'robots', 'm20pro.yaml')),
        DeclareLaunchArgument('work_dir', default_value=os.path.expanduser('~/light_slam_maps')),
        OpaqueFunction(function=start),
    ])
