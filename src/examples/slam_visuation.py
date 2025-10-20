#!/usr/bin/env python3
"""
Monocular SLAM implementation using ORB features with 3D visualization
Standalone version without ROS dependencies
"""

import cv2
import matplotlib
import numpy as np

# Set matplotlib backend to Agg to avoid GUI thread issues
matplotlib.use("Agg")
import argparse
import os
import queue
import threading
import time
from collections import deque

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Force use of X11 instead of Wayland to avoid Qt plugin issues
os.environ["QT_QPA_PLATFORM"] = "xcb"


class MonocularSLAM:
    """
    Implements a monocular SLAM system using ORB features and provides 3D visualization
    """

    def __init__(self, camera_matrix=None, dist_coeffs=None):
        # Camera parameters (default values, can be overridden)
        if camera_matrix is None:
            # Default camera matrix for a typical webcam
            self.focal_length = 800.0  # focal length in pixels
            self.pp = (320.0, 240.0)  # principal point (cx, cy) for 640x480 resolution
            self.camera_matrix = np.array(
                [
                    [self.focal_length, 0, self.pp[0]],
                    [0, self.focal_length, self.pp[1]],
                    [0, 0, 1],
                ]
            )
        else:
            self.camera_matrix = camera_matrix
            # Extract focal length and principal point from provided matrix
            self.focal_length = camera_matrix[0, 0]
            self.pp = (camera_matrix[0, 2], camera_matrix[1, 2])

        self.dist_coeffs = np.zeros((4, 1)) if dist_coeffs is None else dist_coeffs

        # ORB detector
        self.orb = cv2.ORB_create(
            nfeatures=3500
        )  # Reduced feature count to improve performance

        # FLANN parameters for feature matching
        FLANN_INDEX_LSH = 6
        index_params = dict(
            algorithm=FLANN_INDEX_LSH, table_number=6, key_size=12, multi_probe_level=1
        )
        search_params = dict(checks=50)
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)

        # Data for SLAM
        self.prev_frame = None
        self.prev_kp = None
        self.prev_des = None
        self.frame_count = 0

        # 3D visualization data
        self.camera_positions = []  # List of camera positions [x, y, z]
        self.camera_orientations = []  # List of camera orientations (rotation matrices)
        self.map_points = []  # 3D points in the map
        self.colors = []  # Colors for the 3D points

        # Track history for visualization
        self.max_trajectory_len = (
            500  # Reduced trajectory length to avoid memory issues
        )
        self.trajectory_history = deque(maxlen=self.max_trajectory_len)

        # Absolute scale (for trajectory estimation)
        self.abs_scale = 1.0

        # Current camera pose (R|t)
        self.current_R = np.eye(3)
        self.current_t = np.zeros((3, 1))

        # Use image-based visualization instead of 3D plot
        self.trajectory_image = np.ones((600, 800, 3), dtype=np.uint8) * 255
        self.map_points_image = np.ones((600, 800, 3), dtype=np.uint8) * 255
        self.scale_factor = 10  # Scale factor for 2D trajectory visualization
        self.center_x, self.center_y = 400, 300  # Center of visualization

        # For controlling visualization
        self.show_2d = True
        self.show_3d_image = True

        # Store original frame for feature display
        self.current_frame = None

        print("Monocular SLAM initialized with 2D trajectory visualization")

    def detect_features(self, frame):
        """Detect ORB features in a frame"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        kp, des = self.orb.detectAndCompute(gray, None)
        return gray, kp, des

    def match_features(self, des1, des2, kp1, kp2):
        """Match features between two frames"""
        if des1 is None or des2 is None:
            return np.array([]), np.array([]), []

        # Try to match using knnMatch, but fall back to older methods if needed
        try:
            matches = self.flann.knnMatch(des1, des2, k=2)

            # Apply Lowe's ratio test
            good_matches = []
            pts1 = []
            pts2 = []

            # Handle case where knnMatch doesn't return enough matches
            for match in matches:
                if len(match) == 2:  # Sometimes less than 2 matches are returned
                    m, n = match
                    if m.distance < 0.7 * n.distance:  # Ratio test
                        good_matches.append(m)
                        pts1.append(kp1[m.queryIdx].pt)
                        pts2.append(kp2[m.trainIdx].pt)

            return (
                np.array(pts1, dtype=np.float32),
                np.array(pts2, dtype=np.float32),
                good_matches,
            )
        except cv2.error as e:
            print(f"OpenCV error in match_features: {e}")
            return np.array([]), np.array([]), []

    def estimate_pose(self, pts1, pts2):
        """Estimate camera pose from matched feature points"""
        if pts1.shape[0] < 5 or pts2.shape[0] < 5:
            print("Not enough points for pose estimation")
            return None, None, None

        # Essential matrix calculation
        try:
            E, mask = cv2.findEssentialMat(
                pts1,
                pts2,
                self.camera_matrix,
                method=cv2.RANSAC,
                prob=0.999,
                threshold=1.0,
            )

            if E is None or E.shape != (3, 3):
                print(
                    f"Failed to compute valid essential matrix, shape: {E.shape if E is not None else 'None'}"
                )
                return None, None, None

            # Recover pose from essential matrix
            try:
                _, R, t, mask = cv2.recoverPose(
                    E, pts1, pts2, self.camera_matrix, mask=mask
                )
                return R, t, mask
            except cv2.error as e:
                print(f"Error in recoverPose: {e}")
                return None, None, None

        except cv2.error as e:
            print(f"Error in findEssentialMat: {e}")
            return None, None, None

    def triangulate_points(self, pts1, pts2, R, t):
        """Triangulate 3D points from matched features"""
        # Projection matrices
        P0 = np.hstack((np.eye(3), np.zeros((3, 1))))
        P0 = self.camera_matrix @ P0

        P1 = np.hstack((R, t))
        P1 = self.camera_matrix @ P1

        # Triangulate
        points_4d = cv2.triangulatePoints(P0, P1, pts1.T, pts2.T)

        # Convert to 3D points
        points_3d = cv2.convertPointsFromHomogeneous(points_4d.T)

        return points_3d

    def update_trajectory_image(self):
        """Update the 2D trajectory visualization image"""
        # Create a fresh image for trajectory visualization
        self.trajectory_image = np.ones((600, 800, 3), dtype=np.uint8) * 255

        # Draw coordinate axes
        cv2.line(
            self.trajectory_image,
            (self.center_x, self.center_y),
            (self.center_x + 50, self.center_y),
            (0, 0, 255),
            2,
        )  # X-axis (red)
        cv2.line(
            self.trajectory_image,
            (self.center_x, self.center_y),
            (self.center_x, self.center_y - 50),
            (0, 255, 0),
            2,
        )  # Y-axis (green)

        # Add text labels for axes
        cv2.putText(
            self.trajectory_image,
            "X",
            (self.center_x + 55, self.center_y + 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 255),
            1,
        )
        cv2.putText(
            self.trajectory_image,
            "Y",
            (self.center_x - 15, self.center_y - 55),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
        )

        # Draw the camera trajectory
        if len(self.trajectory_history) > 1:
            points = []
            for pos in self.trajectory_history:
                x = int(self.center_x + pos[0] * self.scale_factor)
                y = int(
                    self.center_y - pos[1] * self.scale_factor
                )  # Note: Y is flipped in image coordinates
                points.append((x, y))

            # Draw the trajectory line
            for i in range(1, len(points)):
                cv2.line(
                    self.trajectory_image, points[i - 1], points[i], (255, 0, 0), 2
                )

            # Draw the current position
            curr_pos = points[-1]
            cv2.circle(self.trajectory_image, curr_pos, 5, (0, 0, 255), -1)

        # Add text with position info
        if self.trajectory_history:
            x, y, z = self.trajectory_history[-1]
            pos_text = f"Position: X: {x:.2f}, Y: {y:.2f}, Z: {z:.2f}"
            cv2.putText(
                self.trajectory_image,
                pos_text,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 0),
                2,
            )

        # Add scale info
        scale_text = f"Scale: 1 grid = {1/self.scale_factor:.1f} m"
        cv2.putText(
            self.trajectory_image,
            scale_text,
            (10, 580),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 0),
            1,
        )

        # Draw grid for reference
        grid_step = 50
        for x in range(0, 800, grid_step):
            cv2.line(self.trajectory_image, (x, 0), (x, 600), (200, 200, 200), 1)
        for y in range(0, 600, grid_step):
            cv2.line(self.trajectory_image, (0, y), (800, y), (200, 200, 200), 1)

    def update_map_points_image(self):
        """Update the 3D map points visualization image (top-down view)"""
        self.map_points_image = np.ones((600, 800, 3), dtype=np.uint8) * 255

        # Draw coordinate axes
        cv2.line(
            self.map_points_image,
            (self.center_x, self.center_y),
            (self.center_x + 50, self.center_y),
            (0, 0, 255),
            2,
        )  # X-axis (red)
        cv2.line(
            self.map_points_image,
            (self.center_x, self.center_y),
            (self.center_x, self.center_y - 50),
            (0, 255, 0),
            2,
        )  # Y-axis (green)

        # Draw the camera trajectory (same as in trajectory image)
        if len(self.trajectory_history) > 1:
            points = []
            for pos in self.trajectory_history:
                x = int(self.center_x + pos[0] * self.scale_factor)
                y = int(
                    self.center_y - pos[1] * self.scale_factor
                )  # Note: Y is flipped in image coordinates
                points.append((x, y))

            # Draw the trajectory line
            for i in range(1, len(points)):
                cv2.line(
                    self.map_points_image, points[i - 1], points[i], (255, 0, 0), 1
                )

        # Draw map points (limit to a reasonable number)
        max_points = min(2000, len(self.map_points))
        if max_points > 0:
            step = max(1, len(self.map_points) // max_points)

            for i in range(0, len(self.map_points), step):
                point = self.map_points[i]
                color = self.colors[i] if i < len(self.colors) else [0, 255, 0]

                # Get X and Y coordinates (ignore Z for top-down view)
                x = int(self.center_x + point[0] * self.scale_factor)
                y = int(self.center_y - point[1] * self.scale_factor)

                # Check if point is within image bounds
                if 0 <= x < 800 and 0 <= y < 600:
                    # Draw the point with its color
                    cv2.circle(self.map_points_image, (x, y), 1, color, -1)

        # Add legend
        cv2.putText(
            self.map_points_image,
            "Map Points (Top-Down View)",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 0),
            2,
        )
        cv2.putText(
            self.map_points_image,
            f"Points: {len(self.map_points)}",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 0),
            2,
        )

        # Add grid for reference (same as trajectory image)
        grid_step = 50
        for x in range(0, 800, grid_step):
            cv2.line(self.map_points_image, (x, 0), (x, 600), (200, 200, 200), 1)
        for y in range(0, 600, grid_step):
            cv2.line(self.map_points_image, (0, y), (800, y), (200, 200, 200), 1)

    def process_frame(self, frame):
        """Process a new camera frame"""
        # Save current frame for visualization
        self.current_frame = frame.copy()

        # Detect features
        gray, kp, des = self.detect_features(frame)

        # Create an image with features for visualization
        if self.show_2d:
            feature_img = cv2.drawKeypoints(
                frame,
                kp,
                None,
                color=(0, 255, 0),
                flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
            )
            cv2.imshow("ORB Features", feature_img)

        self.frame_count += 1

        # If this is the first frame, just save the features
        if self.prev_frame is None:
            self.prev_frame = gray
            self.prev_kp = kp
            self.prev_des = des

            # Initialize camera position
            self.camera_positions.append([0, 0, 0])
            self.camera_orientations.append(self.current_R.copy())
            self.trajectory_history.append([0, 0, 0])
            return frame

        # Match features with previous frame
        if self.prev_des is not None and des is not None:
            pts1, pts2, matches = self.match_features(
                self.prev_des, des, self.prev_kp, kp
            )

            # Draw matches for visualization
            if self.show_2d and len(matches) > 0:
                img_matches = cv2.drawMatches(
                    self.prev_frame,
                    self.prev_kp,
                    gray,
                    kp,
                    matches[:50],
                    None,
                    flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS,
                )
                cv2.imshow("Feature Matches", img_matches)

            if len(pts1) > 0 and len(pts2) > 0:
                # Estimate pose
                R, t, mask = self.estimate_pose(pts1, pts2)

                if R is not None and t is not None:
                    # Update camera pose
                    self.current_t = (
                        self.current_t + self.abs_scale * self.current_R @ t
                    )
                    self.current_R = R @ self.current_R

                    # Save camera position and orientation
                    current_pos = self.current_t.flatten().tolist()
                    self.camera_positions.append(current_pos)
                    self.camera_orientations.append(self.current_R.copy())
                    self.trajectory_history.append(current_pos)

                    # Triangulate 3D points
                    points_3d = self.triangulate_points(pts1, pts2, R, t)

                    # Add valid 3D points to the map (with a limit to avoid memory issues)
                    max_points = 3000
                    if len(self.map_points) < max_points:
                        for i, pt in enumerate(points_3d):
                            if i < len(mask) and mask[i]:
                                pt_global = self.current_R @ pt.T + self.current_t
                                self.map_points.append(pt_global.flatten().tolist())

                                # Add color based on original image
                                x, y = int(pts1[i][0]), int(pts1[i][1])
                                if 0 <= x < frame.shape[1] and 0 <= y < frame.shape[0]:
                                    color = frame[y, x].tolist()
                                    self.colors.append(color)
                                else:
                                    self.colors.append([0, 255, 0])  # Default to green

                    # Update 2D visualization
                    if self.frame_count % 5 == 0 and self.show_3d_image:
                        self.update_trajectory_image()
                        self.update_map_points_image()
                        cv2.imshow("Camera Trajectory", self.trajectory_image)
                        cv2.imshow("Map Points", self.map_points_image)

                    # Add text with position info
                    x, y, z = current_pos
                    pos_text = f"Position: X: {x:.2f}, Y: {y:.2f}, Z: {z:.2f}"
                    cv2.putText(
                        frame,
                        pos_text,
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2,
                    )

                    # Add feature count
                    features_text = f"Features: {len(kp)}, Matches: {len(matches)}"
                    cv2.putText(
                        frame,
                        features_text,
                        (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2,
                    )

        # Update previous frame data
        self.prev_frame = gray
        self.prev_kp = kp
        self.prev_des = des

        return frame

    def run_from_camera(self, camera_id=0):
        """Run SLAM from a camera"""
        cap = cv2.VideoCapture(camera_id)

        if not cap.isOpened():
            print(f"Error: Could not open camera {camera_id}")
            return

        while True:
            ret, frame = cap.read()

            if not ret:
                print("Failed to grab frame from camera")
                break

            # Process the frame
            processed_frame = self.process_frame(frame)

            # Display the processed frame
            cv2.imshow("Monocular SLAM", processed_frame)

            # Check for keyboard input
            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord("q"):  # ESC or 'q' key
                break
            elif key == ord("s"):  # Toggle 3D visualization
                self.show_3d_image = not self.show_3d_image
                print(f"3D visualization: {'ON' if self.show_3d_image else 'OFF'}")
            elif key == ord("d"):  # Toggle 2D visualization
                self.show_2d = not self.show_2d
                print(f"2D feature visualization: {'ON' if self.show_2d else 'OFF'}")
            elif key == ord("+") or key == ord("="):  # Increase scale
                self.abs_scale *= 1.1
                print(f"Scale factor: {self.abs_scale:.2f}")
            elif key == ord("-"):  # Decrease scale
                self.abs_scale /= 1.1
                print(f"Scale factor: {self.abs_scale:.2f}")

        # Clean up
        cap.release()
        cv2.destroyAllWindows()

    def run_from_video(self, video_path):
        """Run SLAM on a video file"""
        cap = cv2.VideoCapture(video_path)

        if not cap.isOpened():
            print(f"Error: Could not open video file {video_path}")
            return

        while True:
            ret, frame = cap.read()

            if not ret:
                print("End of video file")
                break

            # Process the frame
            processed_frame = self.process_frame(frame)

            # Display the processed frame
            cv2.imshow("Monocular SLAM", processed_frame)

            # Check for keyboard input
            key = cv2.waitKey(30) & 0xFF  # Delay to simulate video fps
            if key == 27 or key == ord("q"):  # ESC or 'q' key
                break
            elif key == ord("s"):  # Toggle 3D visualization
                self.show_3d_image = not self.show_3d_image
                print(f"3D visualization: {'ON' if self.show_3d_image else 'OFF'}")
            elif key == ord("d"):  # Toggle 2D visualization
                self.show_2d = not self.show_2d
                print(f"2D feature visualization: {'ON' if self.show_2d else 'OFF'}")
            elif key == ord("+") or key == ord("="):  # Increase scale
                self.abs_scale *= 1.1
                print(f"Scale factor: {self.abs_scale:.2f}")
            elif key == ord("-"):  # Decrease scale
                self.abs_scale /= 1.1
                print(f"Scale factor: {self.abs_scale:.2f}")
            elif key == ord("p"):  # Pause
                while True:
                    key2 = cv2.waitKey(0) & 0xFF
                    if key2 == ord("p") or key2 == ord("c"):  # Resume with 'p' or 'c'
                        break
                    elif key2 == 27 or key2 == ord("q"):  # Quit with ESC or 'q'
                        break

        # Clean up
        cap.release()
        cv2.destroyAllWindows()


def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description="Monocular SLAM with ORB features")
    parser.add_argument("-v", "--video", type=str, help="Path to video file")
    parser.add_argument("-c", "--camera", type=int, default=0, help="Camera ID")
    parser.add_argument(
        "-s", "--scale", type=float, default=1.0, help="Scale factor for trajectory"
    )
    parser.add_argument(
        "--no2d", action="store_true", help="Disable 2D feature visualization"
    )
    parser.add_argument("--fx", type=float, help="Focal length x (pixels)")
    parser.add_argument("--fy", type=float, help="Focal length y (pixels)")
    parser.add_argument("--cx", type=float, help="Principal point x (pixels)")
    parser.add_argument("--cy", type=float, help="Principal point y (pixels)")
    return parser.parse_args()


def main():
    """Main function to start the SLAM system"""
    args = parse_args()

    # Setup camera matrix if parameters are provided
    camera_matrix = None
    if all(param is not None for param in [args.fx, args.fy, args.cx, args.cy]):
        camera_matrix = np.array(
            [[args.fx, 0, args.cx], [0, args.fy, args.cy], [0, 0, 1]]
        )
        print(f"Using provided camera matrix: \n{camera_matrix}")

    # Initialize the SLAM system
    slam = MonocularSLAM(camera_matrix=camera_matrix)
    slam.show_2d = not args.no2d
    slam.abs_scale = args.scale

    # Run from video file or camera
    try:
        if args.video:
            print(f"Running SLAM on video: {args.video}")
            slam.run_from_video(args.video)
        else:
            print(f"Running SLAM from camera #{args.camera}")
            slam.run_from_camera(args.camera)
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        cv2.destroyAllWindows()
        print("SLAM system stopped")


if __name__ == "__main__":
    main()
