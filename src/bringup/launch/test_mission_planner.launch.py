from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mission_planner',
            executable='mission_planner_node',
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
                'scenario_desc': PathJoinSubstitution([FindPackageShare('controller_stonefish'), 'data', 'scenarios', 'hydrus_env.scn']),
                'simulation_rate': '300.0',
                'window_res_x': '1920',
                'window_res_y': '1080',
                'rendering_quality': 'high'
            }.items()
        ),
        # one-off publisher
        Node(
            package='bringup',  # where OneShotPublisher lives
            executable='oneshot_map_node',
            output='screen'
        )
    ])

