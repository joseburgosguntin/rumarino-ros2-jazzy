from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os


def generate_launch_description():
    """
    Combined launch file for ORB-SLAM3 and HydrusCV pipeline.
    
    This launch file starts:
    1. USB camera node (usb_cam)
    2. ORB-SLAM3 monocular SLAM node
    3. Depth publisher (DepthAnything V2)
    4. CV publisher (YOLO object detection)
    
    The system provides:
    - Visual SLAM and camera localization via ORB-SLAM3
    - Object detection and mapping via HydrusCV
    - Depth estimation for 3D object positioning
    
    Usage:
        ros2 launch hydrus_cv slam_cv_pipeline.launch.py
        
    Or with custom parameters:
        ros2 launch hydrus_cv slam_cv_pipeline.launch.py \
            camera_device:=/dev/video2 \
            image_width:=1280 \
            image_height:=720 \
            use_orb_viewer:=false
    """
    
    # Get absolute paths
    weights_dir = os.path.join(os.path.expanduser('~'), 'Projects', 'auv', 'weights')
    orb_slam_pkg_dir = os.path.join(os.path.expanduser('~'), 'Projects', 'auv', 'src', 'orb_slam')
    orb_slam3_dir = os.path.join(os.path.expanduser('~'), 'Projects', 'ORB_SLAM3')
    
    # ========== Launch Arguments ==========
    
    # Camera arguments
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
    
    # Topic names
    rgb_topic_arg = DeclareLaunchArgument(
        'rgb_topic',
        default_value='/camera1/image_raw',
        description='Topic name for RGB images (shared between ORB-SLAM and CV)'
    )
    
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/depth_anything/depth',
        description='Topic name for depth images consumed by ORB-SLAM and CV pipeline'
    )
    
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera1/camera_info',
        description='Topic name for camera info'
    )
    
    # ORB-SLAM3 arguments
    orb_vocabulary_arg = DeclareLaunchArgument(
        'orb_vocabulary',
        default_value=os.path.join(orb_slam3_dir, 'Vocabulary', 'ORBvoc.txt'),
        description='Path to ORB-SLAM3 vocabulary file'
    )
    
    orb_settings_arg = DeclareLaunchArgument(
        'orb_settings',
        default_value=os.path.join(orb_slam_pkg_dir, 'config', 'webcam.yaml'),
        description='Path to ORB-SLAM3 camera settings file'
    )
    
    use_orb_viewer_arg = DeclareLaunchArgument(
        'use_orb_viewer',
        default_value='true',
        description='Enable ORB-SLAM3 viewer'
    )

    orb_use_depth_arg = DeclareLaunchArgument(
        'orb_use_depth',
        default_value='true',
        description='Enable RGB-D mode in ORB-SLAM3'
    )

    orb_pose_topic_arg = DeclareLaunchArgument(
        'orb_pose_topic',
        default_value='orb_slam3/camera_pose',
        description='Topic name for ORB-SLAM3 pose output'
    )

    orb_path_topic_arg = DeclareLaunchArgument(
        'orb_path_topic',
        default_value='orb_slam3/camera_path',
        description='Topic name for ORB-SLAM3 path output'
    )

    orb_world_frame_arg = DeclareLaunchArgument(
        'orb_world_frame',
        default_value='world',
        description='TF frame id for ORB-SLAM3 world frame'
    )

    orb_camera_frame_arg = DeclareLaunchArgument(
        'orb_camera_frame',
        default_value='camera',
        description='TF frame id for ORB-SLAM3 camera frame'
    )

    orb_queue_size_arg = DeclareLaunchArgument(
        'orb_queue_size',
        default_value='10',
        description='Queue size for ORB-SLAM3 subscriptions and publishers'
    )

    orb_publish_tf_arg = DeclareLaunchArgument(
        'orb_publish_tf',
        default_value='true',
        description='Publish TF transforms for ORB-SLAM3 pose'
    )
    
    # CV Model paths
    depth_model_path_arg = DeclareLaunchArgument(
        'depth_model_path',
        default_value=os.path.join(weights_dir, 'dav2_s.pt'),
        description='Path to DepthAnything model weights'
    )
    
    yolo_model_path_arg = DeclareLaunchArgument(
        'yolo_model_path',
        default_value=os.path.join(weights_dir, 'yolov8.pt'),
        description='Path to YOLO model weights'
    )
    
    # Processing rates
    depth_publish_rate_arg = DeclareLaunchArgument(
        'depth_publish_rate',
        default_value='10.0',
        description='Depth estimation rate in Hz'
    )
    
    # ========== Nodes ==========
    
    # 1. USB Camera Node
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
            'camera_frame_id': 'camera',
            'io_method': 'mmap',
        }],
        remappings=[
            ('/image_raw', LaunchConfiguration('rgb_topic')),
            ('/camera_info', LaunchConfiguration('camera_info_topic')),
        ]
    )
    
    # 2. ORB-SLAM3 Monocular Node
    orb_slam3_node = Node(
        package='orb_slam3_ros2',
        executable='mono',
        name='orb_slam3_mono',
        output='screen',
        parameters=[{
            'vocabulary_path': LaunchConfiguration('orb_vocabulary'),
            'settings_path': LaunchConfiguration('orb_settings'),
            'use_viewer': ParameterValue(LaunchConfiguration('use_orb_viewer'), value_type=bool),
            'use_depth': ParameterValue(LaunchConfiguration('orb_use_depth'), value_type=bool),
            'image_topic': LaunchConfiguration('rgb_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'pose_topic': LaunchConfiguration('orb_pose_topic'),
            'path_topic': LaunchConfiguration('orb_path_topic'),
            'world_frame_id': LaunchConfiguration('orb_world_frame'),
            'camera_frame_id': LaunchConfiguration('orb_camera_frame'),
            'queue_size': ParameterValue(LaunchConfiguration('orb_queue_size'), value_type=int),
            'publish_tf': ParameterValue(LaunchConfiguration('orb_publish_tf'), value_type=bool),
        }],
        remappings=[
            ('/camera/image_raw', LaunchConfiguration('rgb_topic')),
            ('/camera/depth/image_raw', LaunchConfiguration('depth_topic')),
        ]
    )
    
    # 3. Depth Publisher Node (DepthAnything V2) - Only run if depth is enabled
    depth_publisher_node = Node(
        package='hydrus_cv',
        executable='depth_publisher',
        name='depth_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('orb_use_depth')),
        parameters=[{
            'rgb_topic': LaunchConfiguration('rgb_topic'),
            'depth_output_topic': LaunchConfiguration('depth_topic'),
            'depth_model_path': LaunchConfiguration('depth_model_path'),
            'publish_rate': ParameterValue(LaunchConfiguration('depth_publish_rate'), value_type=float),
        }]
    )
    
    # 4. CV Publisher Node (YOLO + Processing)
    cv_publisher_node = Node(
        package='hydrus_cv',
        executable='cv_publisher',
        name='cv_publisher',
        output='screen',
        parameters=[{
            'rgb_topic': LaunchConfiguration('rgb_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'imu_topic': LaunchConfiguration('orb_pose_topic'),
            'yolo_model_path': LaunchConfiguration('yolo_model_path'),
            'depth_model_path': LaunchConfiguration('depth_model_path'),
        }]
    )
    
    # 5. Map Visualizer Node (RViz markers)
    map_visualizer_node = Node(
        package='hydrus_cv',
        executable='map_visualizer',
        name='map_visualizer',
        output='screen',
        parameters=[{
            'map_topic': 'detections',
            'marker_topic': 'object_markers',
            'world_frame': LaunchConfiguration('orb_world_frame'),
            'marker_lifetime': 1.0,
        }]
    )
    
    # ========== Launch Description ==========
    
    return LaunchDescription([
        # Launch arguments
        camera_device_arg,
        image_width_arg,
        image_height_arg,
        framerate_arg,
        pixel_format_arg,
        rgb_topic_arg,
        depth_topic_arg,
        camera_info_topic_arg,
        orb_vocabulary_arg,
        orb_settings_arg,
        use_orb_viewer_arg,
        orb_use_depth_arg,
        orb_pose_topic_arg,
        orb_path_topic_arg,
        orb_world_frame_arg,
        orb_camera_frame_arg,
        orb_queue_size_arg,
        orb_publish_tf_arg,
        depth_model_path_arg,
        yolo_model_path_arg,
        depth_publish_rate_arg,
        
        # Nodes - Launch order optimized
        # 1. Start camera and depth publisher first
        usb_cam_node,
        depth_publisher_node,
        
        # 2. Delay ORB-SLAM3 to allow depth publisher to initialize (5 seconds)
        TimerAction(
            period=5.0,
            actions=[orb_slam3_node]
        ),
        
        # 3. Start CV publisher after ORB-SLAM is ready
        TimerAction(
            period=7.0,
            actions=[cv_publisher_node]
        ),
        
        # 4. Start map visualizer immediately for RViz
        map_visualizer_node,
    ])