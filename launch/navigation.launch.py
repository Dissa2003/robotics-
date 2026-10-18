"""Navigate on a saved map; do NOT run SLAM Toolbox at the same time."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('ie4060_maze')
    tb3_nav = get_package_share_directory('turtlebot3_navigation2')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.join(share, 'maps', 'maze.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            share, 'config', 'nav2_params.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_nav, 'launch', 'navigation2.launch.py')),
            launch_arguments={
                'use_sim_time': 'True',
                'map': LaunchConfiguration('map'),
                'params_file': LaunchConfiguration('params_file'),
            }.items(),
        ),
    ])
