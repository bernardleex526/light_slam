"""Nav2 with light_slam localization as the map->odom TF authority."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the Nav2 launch description without an AMCL transform authority."""
    sim = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Absolute path to saved occ_grid.yaml'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            get_package_share_directory('m20_navigation'), 'config', 'nav2_light_slam.yaml')),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[{'yaml_filename': LaunchConfiguration('map'), 'use_sim_time': sim}],
             output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_map',
             parameters=[{'use_sim_time': sim, 'autostart': True, 'node_names': ['map_server']}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
            launch_arguments={'use_sim_time': sim,
                              'params_file': LaunchConfiguration('params_file'),
                              'autostart': 'true', 'use_composition': 'False'}.items()),
    ])
