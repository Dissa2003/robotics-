import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('maze_world')
    world_file = os.path.join(pkg_share, 'worlds', 'my_maze.world')
    turtlebot_share = get_package_share_directory('turtlebot3_gazebo')
    use_sim_time = LaunchConfiguration('use_sim_time')

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -v 2 ', world_file],
            'on_exit_shutdown': 'true',
        }.items()
    )

    robot_model = os.path.join(
        turtlebot_share, 'models', 'turtlebot3_burger', 'model.sdf')
    robot_urdf = os.path.join(
        turtlebot_share, 'urdf', 'turtlebot3_burger.urdf')
    bridge_config = os.path.join(pkg_share, 'config', 'burger_bridge.yaml')

    with open(robot_urdf, encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'burger',
            '-file', robot_model,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
        ],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
        output='screen',
    )

    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the Gazebo simulation clock'),
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(turtlebot_share, 'models')),
        gz_sim_launch,
        spawn_robot,
        bridge,
        state_publisher,
    ])
