import numpy as np
from ultralytics import YOLO
from .custom_types import BoundingBox3D, CameraIntrinsics, DepthImage, Detection,MapState, MapObject, Rotation3D, Point3D
import torch
import glob
import logging
import math
import os
import sys



# Get the absolute path to the project root
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, '..', '..', '..', '..'))
third_party_path = os.path.join(project_root, 'third_party')

# Add third_party to sys.path so we can import from it
if third_party_path not in sys.path:
    sys.path.insert(0, third_party_path)

# Check if depth_anything_v2 exists
depth_anything_available = os.path.exists(os.path.join(third_party_path, 'depth_anything_v2'))

# This is optional because its not the only way we can  calculate Depth Images. So I dont want to force people to install it if they are not using it.
if depth_anything_available:
    from depth_anything_v2.depth_anything_v2.dpt import DepthAnythingV2
    class DepthAnythingManager:
        def __init__(self, model_path: str):
            self.device ='cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'
            self.model_config ={'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]}
            self.model = DepthAnythingV2(**self.model_config)
            self.model.load_state_dict(torch.load(model_path, map_location="cpu"))
            self.model.to(self.device).eval()

        def detect(self, image: np.ndarray) -> DepthImage:
            return self.model.infer_image(image)




class YOLOModelManager:
    def __init__(self, model_path: str ):
        self.model : YOLO  = YOLO()
    def detect(self, image: np.ndarray) -> list[Detection]:
        """
        Run YOLO object detection on an image.
        :param image: Input image as numpy array
        :return: List of Detection objects
        """
        result_list = []
        results = self.model(image)
        for result in results:
            if hasattr(result, "boxes"):
                for box in result.boxes:
                    x1, y1, x2, y2 = box.xyxy.cpu().numpy()[0]
                    conf = float(box.conf.cpu().numpy()[0])
                    cls_w = int(box.cls.cpu().numpy()[0])
                    result_list.append(
                        Detection(x1, y1, x2, y2, cls_w, conf, 0, None)
                    )
        return result_list

def calculate_point_3d(
    detections: list[Detection],
    depth_image: DepthImage,
    camera_intrinsic: CameraIntrinsics,
):
    """
    Calculate 3D points for detections using depth information.
    :param detections: List of Detection objects to update
    :param depth_image: Depth image as numpy array
    :param camera_intrinsic: Camera intrinsic parameters (fx, fy, cx, cy)
    """
    for detection in detections:
        x_min, y_min, x_max, y_max = (
            detection._x1,
            detection._y1,
            detection._x2,
            detection._y2,
        )
        if depth_image is not None:
            x_min_int = int(x_min)
            x_max_int = int(x_max)
            y_min_int = int(y_min)
            y_max_int = int(y_max)

            # Extract the depth values within the bounding box
            bbox_depth = depth_image[y_min_int:y_max_int, x_min_int:x_max_int]
            if bbox_depth.size > 0:
                mean_depth = float(np.nanmean(bbox_depth))  # type: ignore
                if not np.isnan(mean_depth):
                    fx, fy, cx, cy = camera_intrinsic
                    z = mean_depth
                    detection.depth = z

                    x_center = (x_min + x_max) / 2.0
                    y_center = (y_min + y_max) / 2.0
                    x = (x_center - cx) * z / fx
                    y = (y_center - cy) * z / fy

                    detection.point = Point3D(x=x, y=y, z=z)
                else:
                    detection.point = Point3D(x=0, y=0, z=0)
                    detection._distance = 0
            else:
                detection.point = Point3D(x=0, y=0, z=0)
                detection._distance = 0


def quaternion_to_transform_matrix(rotation: Rotation3D) -> np.ndarray:
    """
    Convert quaternion rotation to 4x4 transformation matrix.
    :param rotation: Rotation3D object with quaternion components
    :return: 4x4 transformation matrix
    """
    w, x, y, z = rotation.w, rotation.x, rotation.y, rotation.z
    rotation_matrix = np.array(
        [
            [
                1 - 2 * y**2 - 2 * z**2,
                2 * x * y - 2 * z * w,
                2 * x * z + 2 * y * w,
            ],
            [
                2 * x * y + 2 * z * w,
                1 - 2 * x**2 - 2 * z**2,
                2 * y * z - 2 * x * w,
            ],
            [
                2 * x * z - 2 * y * w,
                2 * y * z + 2 * x * w,
                1 - 2 * x**2 - 2 * y**2,
            ],
        ]
    )

    transform_matrix = np.eye(4)
    transform_matrix[:3, :3] = rotation_matrix
    return transform_matrix


def transform_to_global(
    detections: list[Detection],
    imu_point: Point3D,
    imu_rotation: Rotation3D,
):
    """
    Transform detection points from camera frame to global frame.
    :param detections: List of Detection objects to transform
    :param imu_point: IMU position in global frame
    :param imu_rotation: IMU orientation as quaternion
    """
    transform_matrix = quaternion_to_transform_matrix(imu_rotation)
    transform_matrix[0:3, 3] = [imu_point.x, imu_point.y, imu_point.z]

    for detection in detections:
        if detection._point is not None:
            point_homogeneous = np.array(
                [detection._point.x, detection._point.y, detection._point.z, 1.0]
            )
            point_global = np.dot(transform_matrix, point_homogeneous)
            detection._point = Point3D(
                x=point_global[0], y=point_global[1], z=point_global[2]
            )

def map_3d_bounding_box(detections: list[Detection], box_map: dict[str, tuple[float , float , float]]):
    for detection in detections:
        assert not detection._point, "Detection didnt had filled the 3d point attribute."
        cls = str(detection._cls)
        bbox3d = BoundingBox3D(Rotation3D(0,0,0,0), box_map[cls][0], box_map[cls][1], box_map[cls][2])
        detection._bbox_3d = bbox3d        




def calculate_distance(p1: Point3D, p2: Point3D) -> float:
    return math.dist((p1.x, p1.y, p1.z), (p2.x, p2.y, p2.z))

def map_objects(

                state: MapState,
                class_name: list[str],
                box_map: dict[str, tuple[float , float , float]],
                yolo_manager: YOLOModelManager,
                image: np.ndarray,
                camera_intrinsic: CameraIntrinsics,
                depth_image: DepthImage | None = None,
                imu_point: Point3D | None = None,
                imu_rotation: Rotation3D | None = None,
                **kwargs
                ) :
    # Check for threshold also add a  kalman filter
    detections : list[Detection] = yolo_manager.detect(image)
    assert  depth_image.any() , "Depth Image was not specified the pipeline needs a DepthImage"
    assert  imu_point, "IMU point informatio was not provided"
    assert  imu_rotation, "IMU rotation information was not provided"
    calculate_point_3d(detections,depth_image,camera_intrinsic)
    transform_to_global(detections, imu_point, imu_rotation)
    map_3d_bounding_box(detections, box_map)

    #First time seeing the object, update the state.
    for detection in detections:
        if state.obj_frequencies[class_name[detection._cls]] == 0:
            new_object = MapObject(state.id_counter,detection._cls, detection._cls, detection._conf,detection._bbox_3d ) 
            state.map_id_objects[state.id_counter] = new_object
            state.obj_frequencies[class_name[detection._cls]] = 1
            state.id_counter += 1

   # Check for thresholds
    new_object_theshold = 0.5
    for id, object in state.map_id_objects.items():
        best_object_match_distace = math.inf
        best_detection: Detection | None = None
        best_object_match_id = 0
        for detection in detections:
            object_detection_distance = calculate_distance(object.point, detection._point) 
            # For the object founds  lets get the detection and see if it is a different object, my measuring the distance.
            if object_detection_distance > new_object_theshold and object.cls == detection._cls:
                new_object = MapObject(state.id_counter,detection._cls, detection._cls, detection._conf,detection._bbox_3d )
                state.id_counter += 1
                state.obj_frequencies[class_name[detection._cls]] += 1
                state.map_id_objects[state.id_counter] = new_object
           # If its less than the threshold then its the same object. We check in here which of all the objects it is
            elif object.cls == detection._cls and object_detection_distance < best_object_match_distace:
                best_object_match_distace = object_detection_distance
                best_detection = detection
                best_object_match_id = object.track_id

        #Update the objects position if needed:
        state.map_id_objects[best_object_match_id].conf =best_detection._conf
        state.map_id_objects[best_object_match_id].point = best_detection._point
        state.map_id_objects[best_object_match_id].bbox_3d = best_detection._bbox_3d
        # Add a Kalman Filter Logic somewhere overthere




