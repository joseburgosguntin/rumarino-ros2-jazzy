
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import glob 
import os
import sys


from .detection_core import DepthAnythingManager


class DepthPublisher(Node):
    """
    ROS2 Node that subscribes to RGB images and publishes depth maps
    using the DepthAnything V2 model.
    """
    
    def __init__(self):
        super().__init__('depth_publisher')
        
        # Declare parameters
        self.declare_parameter('rgb_topic', '/webcam_rgb')
        self.declare_parameter('depth_output_topic', '/depth_anything/depth')
        # Use absolute path to weights directory - using dav2_s.pt (small model) which matches the 'vits' encoder
        default_weights_path = os.path.join(os.path.expanduser('~'), 'Projects', 'auv', 'weights', 'dav2_s.pt')
        self.declare_parameter('depth_model_path', default_weights_path)
        self.declare_parameter('publish_rate', 10.0)  # Hz
        
        # Get parameters
        rgb_topic = self.get_parameter('rgb_topic').get_parameter_value().string_value
        depth_output_topic = self.get_parameter('depth_output_topic').get_parameter_value().string_value
        depth_model_path = self.get_parameter('depth_model_path').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        # Initialize CV Bridge for ROS-OpenCV conversion
        self.bridge = CvBridge()
        
        # Initialize DepthAnything model
        self.get_logger().info(f'Loading DepthAnything model from: {depth_model_path}')
        
        if not os.path.exists(depth_model_path):
            # Try to find available depth models in the project weights directory
            weights_dir = os.path.join(os.path.expanduser('~'), 'Projects', 'auv', 'weights')
            depth_anything_paths = glob.glob(os.path.join(weights_dir, "dav*.pt"))
            exception_message = f"The path {depth_model_path} was not found.\n"
            exception_message += "Available depth models:\n"
            for idx, path in enumerate(depth_anything_paths):
                exception_message += f"{idx}) {path}\n"
            raise Exception(exception_message)
        
        self.depth_model = DepthAnythingManager(depth_model_path)
        self.get_logger().info('DepthAnything model loaded successfully')
        
        # Subscribe to RGB images
        self.rgb_sub = self.create_subscription(
            Image,
            rgb_topic,
            self.rgb_callback,
            10
        )
        
        # Publisher for depth images
        self.depth_pub = self.create_publisher(Image, depth_output_topic, 10)
        
        # Store last received RGB image
        self.last_rgb_image = None
        self.last_rgb_header = None
        
        # Create timer for processing and publishing
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.process_and_publish)
        
        self.get_logger().info(f'DepthPublisher initialized')
        self.get_logger().info(f'  Subscribing to: {rgb_topic}')
        self.get_logger().info(f'  Publishing depth to: {depth_output_topic}')
        self.get_logger().info(f'  Publish rate: {publish_rate} Hz')
    
    def rgb_callback(self, msg: Image):
        """
        Callback for RGB image messages.
        Stores the latest image for processing.
        """
        try:
            # Convert ROS Image to OpenCV format
            self.last_rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.last_rgb_header = msg.header
        except Exception as e:
            self.get_logger().error(f'Error converting RGB image: {e}')
    
    def process_and_publish(self):
        """
        Process the latest RGB image and publish depth map.
        """
        if self.last_rgb_image is None:
            self.get_logger().warn('No RGB image received yet', throttle_duration_sec=5.0)
            return
        
        try:
            # Run depth estimation
            depth_map = self.depth_model.detect(self.last_rgb_image)
            
            # Normalize depth map for visualization (optional)
            # The model outputs relative depth, normalize to 0-1 range
            if depth_map is not None and depth_map.size > 0:
                # Keep original depth values for accurate measurements
                # Convert to 32-bit float for better precision
                depth_map_32f = depth_map.astype(np.float32)
                
                # Create ROS Image message
                depth_msg = self.bridge.cv2_to_imgmsg(depth_map_32f, encoding='32FC1')
                
                # Use the same header as the RGB image for proper synchronization
                if self.last_rgb_header is not None:
                    depth_msg.header = self.last_rgb_header
                
                # Publish
                self.depth_pub.publish(depth_msg)
                
                self.get_logger().debug(
                    f'Published depth map - shape: {depth_map.shape}, '
                    f'range: [{np.min(depth_map):.3f}, {np.max(depth_map):.3f}]'
                )
            else:
                self.get_logger().error('Depth detection returned invalid result')
                
        except Exception as e:
            self.get_logger().error(f'Error processing depth: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DepthPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in depth publisher: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
