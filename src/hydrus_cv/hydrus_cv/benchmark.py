# This python file should test with a small video replay the performance of the algorithms in terms
# of latency.

import cv2
import numpy as np
import time
import tracemalloc
from typing import Callable
from .custom_types import CameraIntrinsics, DepthImage, Detection, MapState, Point3D, Rotation3D
from .detection_core import DepthAnythingManager, YOLOModelManager,  calculate_point_3d, map_objects
import random
import copy
class MockVideo:
    def __init__(self,height: int , width: int, channels: int,frames: int, seed: int, cameras:int = 1):
        self.height : int = height
        self.width: int = width
        self.channels: int  = channels
        self.frames: int = frames
        self.cameras: int = cameras
        self.frame_counter: int = cameras
        np.random.seed(seed)

    def read(self)-> tuple[bool, np.ndarray] :
        self.frame_counter += 1
        image = np.random.randn(self.height, self.width, self.channels)
        ret = True
        if self.frame_counter > self.frames:
            ret = False
        return (ret , image)

    def get_frames(self) ->list[np.ndarray]:
        return []

class MockCamera:
    def __init__(self,height:int , width: int,channels: int, seed: int,cameras:int):
        self.cameras: int = cameras
        self.height : int = height
        self.width   :int = width
        self.channels:int     = channels
        self.seed   : int= seed
        np.random.seed(seed)


    def get_frames(self):
        return [np.random.randint(low = 0, high = 255, size =(self.height,self.width,self.channels), dtype= np.uint8)
                for _ in range(self.cameras)]
class MockIMU:
    def __init__(self):
        self.start_point: Point3D = Point3D(0,0,0)
        self.start_rotation: Rotation3D = Rotation3D(0,0,0,0)
        self.point_trajectory: list[Point3D] = [self.start_point]
        self.rotation_trajectory: list[Rotation3D] = [self.start_rotation]


    def generate_straight_line(self,axis : int, step: float):
        new_point = copy.copy(self.point_trajectory[-1])
        if axis == 0:
            new_point.x += step
        elif axis == 1:
            new_point.y += step
        elif axis == 2:
            new_point.z += step
        else:
            raise Exception(f"axis should be 0,  1 or 2 and you give {axis}") 

    def get_last_orientation(self)-> tuple[Point3D, Rotation3D]:
        return self.point_trajectory[-1], self.rotation_trajectory[-1]


class MockDetection:
    def __init__(self, cls_number: int, detection_number: int ):
        self.cls_range = range(0,cls_number)

    def generate_detection(self,cls:bool, bbox : bool,  point3d: bool, bbox3d: bool):
        #TODO
        pass


class OpenCVManager:
    def __init__(self):
        self.cameras = []
        count = 0
        try:
            while count < 10:
                self.cameras.append(cv2.VideoCapture(count))
                count +=1
        except:
            print(f"A total of {count} cameras")
    def get_frames(self)-> list[np.ndarray]:
        for idx, camera in enumerate(self.cameras):
            if not camera.isOpened():
                print(f"error opening camera {idx}")
        frames = [camera.read()[1] for camera in self.cameras]
        return frames
    def release(self):
        for camera in self.cameras:
            camera.release()



def log_performance(func: Callable, *args ):
    tracemalloc.start()
    start = time.perf_counter()
    func(*args)
    end = time.perf_counter()
    current, peak = tracemalloc.get_traced_memory()
    final_time = end - start
    return final_time, current, peak


def benchmark_yolo(yolo_model :str, frame_limit: int, video_path: str | None = None ,mock_data: bool  = False) -> list[float]:
    
   yolo_manager = YOLOModelManager(yolo_model)
   if video_path:
       cap = cv2.VideoCapture(video_path)
       if not cap.isOpened():
           print("Error: Cannot open video file")
           exit()
   if mock_data:
       cap = MockVideo(1080, 720, 3, frame_limit, 0)
   result = []
   current_frame = 0
   while True:
       current_frame += 1 
       ret, frame = cap.read()
       if not ret or current_frame > frame_limit :
           break    
       bench_result = log_performance(yolo_manager.detect, frame) 
       result.append(bench_result) 
   return result


def benchmark_calculate_point_3d(mock_data: bool, detections: list[Detection], depth_frame: DepthImage, camera_intrinsic: list[float], frame_limit: int) :
    """
    This function expect a Detection object that have been filled with the ultralytics
    """
    if mock_data:
        detection_generator = MockDetection(10, 20)
        mock_video = MockVideo(720,480, 1,300,0)
        detections = detection_generator.generate_detection()
    while True:
        current_frame += 1
        ret, depth_frame = cap.read()
        camera_intrinsic = [random.random() for _ in range(4)]
        if not ret or current_frame > frame_limit :
            break    
        bench_result = log_performance(calculate_point_3d, detections, depth_frame, camera_intrinsic) 
        result.append(bench_result)


def camera_map_object_visualization():
    #TODO: 
    pass
def calibrate_camera(calibration_image_path: str )-> CameraIntrinsics:
    import os
    CHECKERBOARD = (9, 6)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
    objp[:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

    objpoints = [] # 3D points
    imgpoints = [] # 2D points

    images = os.scandir(calibration_image_path)  # folder with your calibration images
    for image_dir in images:
        if image_dir.name.endswith(".png") or image_dir.name.endswith(".jpg"):
            img = cv2.imread(image_dir.name)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
            if ret:
                objpoints.append(objp)
                corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
                imgpoints.append(corners2)
                cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
                cv2.imshow('img', img)
                cv2.waitKey(200)

    cv2.destroyAllWindows()
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    return CameraIntrinsics((mtx, dist, rvecs, tvecs))


def camera_simulation():
    cv_manager = MockCamera(1280,720,3, 0,1)
    # cv_manager = OpenCVManager()
    yolo_model = YOLOModelManager("./yolov11.pt")
    class_names = ["gate", "buoy", "shark", "swordfish"]
    box_map = {"gate": (2.0,3.0,4.0)}
    camera_intrinsics = (1.0,2.0,3.0,4.0)
    # camera_intrinsics = calibrate_camera("./calibration_images")
    depth_anything = DepthAnythingManager("../weights/dav2_s.pt")
    imu_mock = MockIMU()
    map_state = MapState()
    
    while True:
        frames = cv_manager.get_frames()
        for idx, frame in enumerate(frames):
            cv2.imshow(f"Camera {idx}", frame)

        imu_mock.generate_straight_line(axis = 1, step = 0.3)
        point, rotation = imu_mock.get_last_orientation()
        depth_image = depth_anything.detect(frame)
        map_objects(map_state, class_names, box_map, yolo_model, frames[0], camera_intrinsics,
                    imu_point= point, imu_rotation= rotation, depth_image = depth_image)
        print(map_state)
        if cv2.waitKey(1) & 0xff == ord('q'):
            break


if __name__ == "__main__":
    video_path = "no video"
    yolo_path  = "./yolo11n.pt"
    camera_simulation()
    #benchmark_yolo(video_path= video_path, yolo_model=yolo_path,frame_limit=40, mock_data=True)

    
    
