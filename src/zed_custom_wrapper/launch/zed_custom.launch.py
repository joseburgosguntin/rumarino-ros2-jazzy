from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    onnx_model_path = LaunchConfiguration('onnx_model_path')
    svo_path = LaunchConfiguration('svo_path')

    return LaunchDescription([
        DeclareLaunchArgument(
            'onnx_model_path',
            default_value='',
            description='Path to the ONNX model file'
        ),
        DeclareLaunchArgument(
            'svo_path',
            default_value='',
            description='Path to the SVO file'
        ),
        Node(
            package='zed_custom_wrapper',
            executable='zed_custom_node',
            name='zed_custom_node',
            output='screen',
            parameters=[
                {'resolution': 'HD720'},
                {'fps': 30},
                {'mapping_enabled': True},
                {'onnx_model_path': onnx_model_path},
                {'svo_path': svo_path},
                # Object tracking parameters
                {'matching_distance_threshold': 1.0},  # meters - objects within 1m considered same
                # Expected object counts: [class_id, count, class_id, count, ...]
                # Example: [0, 2, 56, 1, 62, 3] = 2 persons, 1 chair, 3 TVs
                {'expected_object_counts': [
                    56, 4,   # 4 chairs
                ]} # Configure based on your scene
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='zed_base_link_to_camera',
            arguments=['0', '0', '0', '0', '0', '0', 'zed_base_link', 'zed_left_camera_frame']
        )
    ])
