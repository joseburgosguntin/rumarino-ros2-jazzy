from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    mission_name = LaunchConfiguration('mission_name')
    env_file_name = LaunchConfiguration('env_file_name')

    return LaunchDescription([
        Node(
            package='mission_executor',
            executable='mission_executor',
            parameters=[{
                'mission_name': mission_name,
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('stonefish_ros2'),
                    'launch',
                    'stonefish_simulator.launch.py'
                ])
            ]),
            launch_arguments={
                'simulation_data': PathJoinSubstitution([FindPackageShare('controller_stonefish'), 'data']),
                'scenario_desc': PathJoinSubstitution([FindPackageShare('controller_stonefish'), 'data', 'scenarios', env_file_name]),
                'simulation_rate': '300.0',
                'window_res_x': '1920',
                'window_res_y': '1080',
                'rendering_quality': 'high'
            }.items()
        ),
    ])

