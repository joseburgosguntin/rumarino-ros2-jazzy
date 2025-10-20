from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os

def generate_launch_description():
    """
    ORB-SLAM3 standalone launch file with integrated USB camera.
    
    Launches:
    - USB camera node (usb_cam)
    - ORB-SLAM3 node (monocular or RGB-D mode)
    
    Supports both MONOCULAR and RGB-D modes via the use_depth parameter.
    
    Usage:
        # Monocular mode (default)
        ros2 launch orb_slam3_ros2 mono_webcam.launch.py
        
        # RGB-D mode (requires depth source)
        ros2 launch orb_slam3_ros2 mono_webcam.launch.py use_depth:=true depth_topic:=/camera/depth
        
        # Custom camera settings
        ros2 launch orb_slam3_ros2 mono_webcam.launch.py \
            camera_device:=/dev/video2 \
            image_width:=1280 \
            image_height:=720
    """
    # Get the package directory
    pkg_dir = os.path.dirname(os.path.dirname(__file__))
    
    # Declare launch arguments
    vocabulary_arg = DeclareLaunchArgument(
        'vocabulary_path',
        default_value='/home/cesar/Projects/auv/third_party/ORBvoc.txt',
        description='Path to ORB vocabulary file'
    )
    
    settings_arg = DeclareLaunchArgument(
        'settings_path',
        default_value=os.path.join(pkg_dir, 'config', 'webcam.yaml'),
        description='Path to ORB-SLAM3 camera settings file'
    )
    
    use_viewer_arg = DeclareLaunchArgument(
        'use_viewer',
        default_value='false',
        description='Enable ORB-SLAM3 visualization viewer'
    )
    
    use_depth_arg = DeclareLaunchArgument(
        'use_depth',
        default_value='false',
        description='Enable RGB-D mode (requires depth_topic and depth source)'
    )
    
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='Input RGB/mono image topic'
    )
    
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/camera/depth/image_raw',
        description='Input depth image topic (only used if use_depth=true)'
    )
    
    pose_topic_arg = DeclareLaunchArgument(
        'pose_topic',
        default_value='orb_slam3/camera_pose',
        description='Output camera pose topic'
    )
    
    path_topic_arg = DeclareLaunchArgument(
        'path_topic',
        default_value='orb_slam3/camera_path',
        description='Output camera trajectory path topic'
    )
    
    world_frame_arg = DeclareLaunchArgument(
        'world_frame_id',
        default_value='world',
        description='TF world frame ID'
    )
    
    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame_id',
        default_value='camera',
        description='TF camera frame ID'
    )
    
    queue_size_arg = DeclareLaunchArgument(
        'queue_size',
        default_value='10',
        description='ROS subscription queue size'
    )
    
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Publish TF transforms'
    )
    
    # USB Camera arguments
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video0',
        description='USB camera device path'
    )
    
    image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='640',
        description='Camera image width'
    )
    
    image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='480',
        description='Camera image height'
    )
    
    framerate_arg = DeclareLaunchArgument(
        'framerate',
        default_value='30.0',
        description='Camera framerate'
    )
    
    pixel_format_arg = DeclareLaunchArgument(
        'pixel_format',
        default_value='yuyv',
        description='Camera pixel format (yuyv, mjpeg, etc.)'
    )
    
    # Depth model path
    depth_model_path_arg = DeclareLaunchArgument(
        'depth_model_path',
        default_value='/home/cesar/Projects/auv/weights/dav2_s.pt',
        description='Path to DepthAnything model weights (only used if use_depth=true)'
    )
    
    depth_publish_rate_arg = DeclareLaunchArgument(
        'depth_publish_rate',
        default_value='10.0',
        description='Depth estimation rate in Hz (only used if use_depth=true)'
    )
    
    # USB Camera Node
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('camera_device'),
            'image_width': ParameterValue(LaunchConfiguration('image_width'), value_type=int),
            'image_height': ParameterValue(LaunchConfiguration('image_height'), value_type=int),
            'framerate': ParameterValue(LaunchConfiguration('framerate'), value_type=float),
            'pixel_format': LaunchConfiguration('pixel_format'),
            'camera_frame_id': LaunchConfiguration('camera_frame_id'),
            'io_method': 'mmap',
        }],
        remappings=[
            ('/image_raw', LaunchConfiguration('image_topic')),
        ]
    )

    # Depth Publisher Node (DepthAnything V2) - only if use_depth=true
    depth_publisher_node = Node(
        package='hydrus_cv',
        executable='depth_publisher',
        name='depth_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_depth')),
        parameters=[{
            'rgb_topic': LaunchConfiguration('image_topic'),
            'depth_output_topic': LaunchConfiguration('depth_topic'),
            'depth_model_path': LaunchConfiguration('depth_model_path'),
            'publish_rate': ParameterValue(LaunchConfiguration('depth_publish_rate'), value_type=float),
        }]
    )

    # ORB-SLAM3 node with all parameters
    orb_slam3_node = Node(
        package='orb_slam3_ros2',
        executable='mono',
        name='orb_slam3_mono',
        output='screen',
        parameters=[{
            'vocabulary_path': LaunchConfiguration('vocabulary_path'),
            'settings_path': LaunchConfiguration('settings_path'),
            'use_viewer': ParameterValue(LaunchConfiguration('use_viewer'), value_type=bool),
            'use_depth': ParameterValue(LaunchConfiguration('use_depth'), value_type=bool),
            'image_topic': LaunchConfiguration('image_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'pose_topic': LaunchConfiguration('pose_topic'),
            'path_topic': LaunchConfiguration('path_topic'),
            'world_frame_id': LaunchConfiguration('world_frame_id'),
            'camera_frame_id': LaunchConfiguration('camera_frame_id'),
            'queue_size': ParameterValue(LaunchConfiguration('queue_size'), value_type=int),
            'publish_tf': ParameterValue(LaunchConfiguration('publish_tf'), value_type=bool),
        }]
    )

    return LaunchDescription([
        # ORB-SLAM arguments
        vocabulary_arg,
        settings_arg,
        use_viewer_arg,
        use_depth_arg,
        image_topic_arg,
        depth_topic_arg,
        pose_topic_arg,
        path_topic_arg,
        world_frame_arg,
        camera_frame_arg,
        queue_size_arg,
        publish_tf_arg,
        # Camera arguments
        camera_device_arg,
        image_width_arg,
        image_height_arg,
        framerate_arg,
        pixel_format_arg,
        # Depth arguments
        depth_model_path_arg,
        depth_publish_rate_arg,
        # Nodes - staged launching for RGB-D mode
        usb_cam_node,
        depth_publisher_node,  # Starts immediately if use_depth=true
        # Delay ORB-SLAM3 by 5 seconds to allow depth publisher to initialize
        TimerAction(
            period=5.0,
            actions=[orb_slam3_node]
        ),
    ])
