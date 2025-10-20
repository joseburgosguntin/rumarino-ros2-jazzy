import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from interfaces.msg import Map, MapObject
from vision_msgs.msg import BoundingBox3D
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3 
from message_filters import Subscriber, ApproximateTimeSynchronizer
import numpy as np
import glob 
import os
import sys

# Add the parent 'src' directory to the Python path to access pyCV
src_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
if src_path not in sys.path:
    sys.path.insert(0, src_path)

from hydrus_cv.detection_core import DepthAnythingManager, YOLOModelManager, map_objects 
from hydrus_cv.custom_types import CameraIntrinsics, Point3D, Rotation3D, DepthImage
from hydrus_cv.custom_types import MapObject as pyMapObject
from hydrus_cv.custom_types import MapState as pyMapState

depth_anything_paths = glob.glob("weights/da*.pt")
yolo_model_paths = glob.glob("weights/yolo*.pt")


class ComputerVisionPublisher(Node):
    def __init__(self):
        super().__init__('ComputerVisionPublisher')


        self.declare_parameter('rgb_topic', '/webcam_rgb')
        self.declare_parameter("depth_topic", "/webcam_depth")
        self.declare_parameter("camera_info_topic", "/webcam_intrinsics")
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("yolo_model_path", "../../weights/yolov8.pt")
        self.declare_parameter('depth_model_path',"../../weights/dav2_b.pt")
        rgb_topic = self.get_parameter('rgb_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value
        cam_intrinsic_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        imu_topic = self.get_parameter("imu_topic").get_parameter_value().string_value

        depth_anything_path : str = self.get_parameter("depth_model_path").get_parameter_value().string_value
        yolo_model_path : str  = self.get_parameter("yolo_model_path").get_parameter_value().string_value

        self.rgb_sub = Subscriber(self, Image, rgb_topic)
        self.info_sub = Subscriber(self, CameraInfo, cam_intrinsic_topic)
        self.depth_sub = Subscriber(self, Image, depth_topic)
        self.imu_sub = Subscriber(self, Pose, imu_topic)
        self.ts = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.info_sub, self.depth_sub, self.imu_sub],
            queue_size=10,
            slop=0.1)
        self.ts.registerCallback(self.sync_inputs)


        self.publisher = self.create_publisher( MapObject, 'detections', 10)
        self.cv_timer = self.create_timer(0.5, self.ros_map_objects)


        self.last_rgb : np.ndarray | None = None
        self.last_depth: np.ndarray | None = None
        self.last_cam_intrinsic : CameraIntrinsics | None = None
        self.last_position:  Point3D | None = None
        self.last_rotation: Rotation3D | None = None
        
        self.depth_anything_model : DepthAnythingManager | None = None
        self.map_state : pyMapState = pyMapState()

        if not os.path.exists(yolo_model_path):
            exception_message   = ""
            exception_message  += f"The path {yolo_model_path} was not found.\n "
            for idx,path in enumerate(yolo_model_paths):
                exception_message += f"{idx}) {path} \n"
            raise Exception(exception_message)
        self.yoloManager = YOLOModelManager(yolo_model_path)
        
        # Initialize class names for YOLO detection (standard underwater objects)
        self.class_name = ["gate", "buoy", "shark", "swordfish", "person", "bottle", "boat"]
        
        # Initialize bounding box dimensions (width, height, depth) for each class
        self.box_map = {
            "gate": (2.0, 3.0, 0.5),
            "buoy": (0.3, 0.3, 0.3), 
            "shark": (2.0, 0.5, 0.3),
            "swordfish": (1.0, 0.2, 0.2),
            "person": (0.6, 1.8, 0.3),
            "bottle": (0.1, 0.3, 0.1),
            "boat": (5.0, 2.0, 1.5)
        }

        topics = self.get_topic_names_and_types()
        topic_names = [name for name, _ in topics]
        if depth_topic not in topic_names:
            self.get_logger().info(f"/webcam_depth is not available using automaticallydepth anything model for depth perception")
            if not os.path.exists(depth_anything_path):
                exception_message   = ""
                exception_message  += f"The path {yolo_model_path} was not found.\n "
                for idx,path in enumerate(depth_anything_paths):
                    exception_message += f"{idx}) {path} \n"
                raise Exception(exception_message)               
            self.depth_anything = DepthAnythingManager(depth_anything_path)




    def ros_img_to_cv2(self ,msg, encoding="bgr8") -> np.ndarray:
        """
        Convert a ROS sensor_msgs/Image message to an OpenCV numpy array.
        :param msg: ROS Image message
        :param encoding: Desired encoding ("bgr8", "mono8", "mono16", "32FC1")
        :return: OpenCV numpy array
        """
        if encoding not in ["bgr8", "mono8", "mono16", "32FC1"]:
            raise ValueError(f"Unsupported encoding: {encoding}")

        dtype_map = {
            "bgr8": np.uint8,
            "mono8": np.uint8,
            "mono16": np.uint16,
            "32FC1": np.float32,
        }

        dtype = dtype_map[encoding]

        # Calculate expected array size based on encoding and dimensions
        channels = 3 if encoding == "bgr8" else 1
        expected_size = msg.height * msg.width * channels
        actual_size = len(msg.data)

        if actual_size != expected_size:
            # Try to infer correct dimensions based on actual data size
            if encoding == "bgr8" and actual_size % 3 == 0:
                total_pixels = actual_size // 3
                # Try to determine if width is correct but height is wrong
                if total_pixels % msg.width == 0:
                    corrected_height = total_pixels // msg.width
                    img_array = np.frombuffer(msg.data, dtype=dtype).reshape(
                        (corrected_height, msg.width, 3)
                    )
                    return img_array

        # Convert the byte data to a NumPy array
        img_array = np.frombuffer(msg.data, dtype=dtype)

        try:
            # Reshape based on image dimensions and encoding
            if encoding == "bgr8":
                # Use step value if available for proper alignment
                if msg.step > 0 and msg.step >= msg.width * 3:
                    img_array = img_array.reshape((msg.height, msg.width, 3))
                else:
                    img_array = np.reshape(img_array, (msg.height, msg.width, 3))
            else:
                # Single-channel
                img_array = np.reshape(img_array, (msg.height, msg.width))

            return img_array

        except Exception as e:
            raise ValueError(
                f"Reshape failed: {e}. Image info: height={msg.height}, width={msg.width}, step={msg.step}, encoding={msg.encoding}"
            )



    

    def sync_inputs(self,rgb : Image,depth : Image, cam_intrinsic: CameraInfo, imu: Pose):
        self.last_rgb = self.ros_img_to_cv2(rgb)
        self.last_depth = self.ros_img_to_cv2(depth)
        self.last_cam_intrinsic = tuple([cam_intrinsic.d, cam_intrinsic.r, cam_intrinsic.k, cam_intrinsic.p]) 
        self.last_position = Point3D(imu.position.x, imu.position.y, imu.position.z)
        self.last_rotation = Rotation3D(imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w)


    def ros_map_objects(self):
        # Check if we have all required data before processing
        if self.last_rgb is None or self.last_cam_intrinsic is None or self.last_position is None or self.last_rotation is None:
            self.get_logger().debug("Waiting for all sensor inputs to be available...")
            return
            
        # Use DepthAnything model if available, otherwise use subscribed depth
        if self.depth_anything_model and self.last_rgb is not None:
            self.last_depth: DepthImage = self.depth_anything_model.detect(self.last_rgb)
        
        # Final check that we have depth data
        if self.last_depth is None:
            self.get_logger().debug("No depth data available, skipping object mapping...")
            return
 
        map_objects(self.map_state,self.class_name, self.box_map, self.yoloManager,self.last_rgb, 
                    self.last_cam_intrinsic, self.last_depth, self.last_position, self.last_rotation)
        objects = []
        for p_object in self.map_state.objects:
            ros_object = MapObject()
            ros_object.cls = p_object.cls
            bbox = BoundingBox3D()
            bbox.center = Pose()
            bbox.center.orientation = Quaternion(
                    p_object.bbox_3d.rotation.x, p_object.bbox_3d.rotation.y,
                    p_object.bbox_3d.rotation.z, p_object.bbox_3d.w
                    )
            bbox.center.position = Point(p_object.point.x , p_object.point.y, p_object.point.z )

            bbox.size = Vector3(p_object.bbox_3d.height, p_object.bbox_3d.width, p_object.bbox_3d.length)
            ros_object.bbox = bbox
            objects.append(ros_object)

        self.publisher.publish(objects)

def main(args=None):
    rclpy.init(args=args)
    node = ComputerVisionPublisher()
    rclpy.spin(node) 
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

