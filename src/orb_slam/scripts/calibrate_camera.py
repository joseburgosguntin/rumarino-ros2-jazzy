#!/usr/bin/env python3
"""
Camera Calibration Script for ORB-SLAM3
Uses OpenCV to calibrate a camera with a chessboard pattern and outputs
the calibration parameters in ORB-SLAM3 YAML format.

Usage:
    python3 calibrate_camera.py --input <video_or_images> --pattern <cols>x<rows> --square_size <size_in_meters>
    
Example:
    python3 calibrate_camera.py --input 0  # Use webcam
    python3 calibrate_camera.py --input calibration_video.mp4
    python3 calibrate_camera.py --input "images/*.jpg" --pattern 9x6 --square_size 0.025
"""

import cv2
import numpy as np
import yaml
import argparse
import glob
import os
from pathlib import Path


class CameraCalibrator:
    def __init__(self, pattern_size=(9, 6), square_size=0.025):
        """
        Initialize camera calibrator
        
        Args:
            pattern_size: (cols, rows) - number of inner corners in chessboard
            square_size: size of each square in meters (e.g., 0.025 = 25mm)
        """
        self.pattern_size = pattern_size
        self.square_size = square_size
        
        # Prepare object points (0,0,0), (1,0,0), (2,0,0) ..., (cols-1,rows-1,0)
        self.objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
        self.objp *= square_size
        
        # Arrays to store object points and image points
        self.objpoints = []  # 3D points in real world space
        self.imgpoints = []  # 2D points in image plane
        
        self.img_shape = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.rvecs = None
        self.tvecs = None
        self.rms_error = None
        
    def find_chessboard(self, image, show=True):
        """
        Find chessboard corners in an image
        
        Returns:
            success (bool), image with drawn corners
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Find the chessboard corners
        ret, corners = cv2.findChessboardCorners(
            gray, 
            self.pattern_size,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        )
        
        display_img = image.copy()
        
        if ret:
            # Refine corner positions
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            
            # Store points
            self.objpoints.append(self.objp)
            self.imgpoints.append(corners_refined)
            
            # Draw corners
            cv2.drawChessboardCorners(display_img, self.pattern_size, corners_refined, ret)
            
            if show:
                cv2.putText(display_img, f"Pattern found! Total: {len(self.objpoints)}", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            if show:
                cv2.putText(display_img, "Pattern not found", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        return ret, display_img
    
    def calibrate_from_video(self, video_source=0, num_frames=20, skip_frames=10):
        """
        Calibrate camera from video feed or video file
        
        Args:
            video_source: 0 for webcam, or path to video file
            num_frames: number of successful calibration frames to collect
            skip_frames: number of frames to skip between captures
        """
        cap = cv2.VideoCapture(video_source)
        
        if not cap.isOpened():
            print(f"Error: Could not open video source {video_source}")
            return False
        
        print(f"\nCalibration Instructions:")
        print(f"- Pattern size: {self.pattern_size[0]}x{self.pattern_size[1]} (cols x rows)")
        print(f"- Square size: {self.square_size * 1000:.1f}mm")
        print(f"- Need {num_frames} successful detections")
        print(f"- Press SPACE to capture a frame")
        print(f"- Press 'q' to quit")
        print(f"- Press 'c' to calibrate with current images")
        print("\nTips:")
        print("- Move the chessboard to different positions and angles")
        print("- Cover different parts of the image")
        print("- Keep the pattern flat and well-lit\n")
        
        frame_count = 0
        capture_count = 0
        
        while capture_count < num_frames:
            ret, frame = cap.read()
            if not ret:
                print("End of video or error reading frame")
                break
            
            if self.img_shape is None:
                self.img_shape = frame.shape[:2]
            
            frame_count += 1
            
            # Try to find chessboard
            found, display_img = self.find_chessboard(frame, show=False)
            
            # Add instructions
            cv2.putText(display_img, f"Captures: {len(self.objpoints)}/{num_frames}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            cv2.putText(display_img, "SPACE: capture | C: calibrate | Q: quit", 
                       (10, display_img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            if found:
                cv2.putText(display_img, "Pattern detected!", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            cv2.imshow('Camera Calibration', display_img)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                print("Calibration cancelled by user")
                break
            elif key == ord(' ') and found:
                capture_count += 1
                print(f"✓ Captured frame {capture_count}/{num_frames}")
            elif key == ord('c'):
                if len(self.objpoints) >= 3:
                    print(f"\nCalibrating with {len(self.objpoints)} images...")
                    break
                else:
                    print(f"Need at least 3 images, currently have {len(self.objpoints)}")
        
        cap.release()
        cv2.destroyAllWindows()
        
        if len(self.objpoints) < 3:
            print(f"Not enough calibration images. Got {len(self.objpoints)}, need at least 3")
            return False
        
        # Perform calibration
        return self.calibrate()
    
    def calibrate_from_images(self, image_pattern):
        """
        Calibrate camera from a set of images
        
        Args:
            image_pattern: glob pattern for images (e.g., "calibration/*.jpg")
        """
        images = glob.glob(image_pattern)
        
        if not images:
            print(f"No images found matching pattern: {image_pattern}")
            return False
        
        print(f"Found {len(images)} images")
        
        for idx, fname in enumerate(images):
            img = cv2.imread(fname)
            if img is None:
                print(f"Could not read image: {fname}")
                continue
            
            if self.img_shape is None:
                self.img_shape = img.shape[:2]
            
            ret, display_img = self.find_chessboard(img, show=False)
            
            if ret:
                print(f"✓ [{idx+1}/{len(images)}] {os.path.basename(fname)}")
            else:
                print(f"✗ [{idx+1}/{len(images)}] {os.path.basename(fname)} - Pattern not found")
            
            # Show image briefly
            cv2.imshow('Calibration', display_img)
            cv2.waitKey(100)
        
        cv2.destroyAllWindows()
        
        if len(self.objpoints) < 3:
            print(f"Not enough calibration images. Got {len(self.objpoints)}, need at least 3")
            return False
        
        # Perform calibration
        return self.calibrate()
    
    def calibrate(self):
        """
        Perform camera calibration using collected points
        """
        print(f"\nCalibrating with {len(self.objpoints)} images...")
        
        ret, self.camera_matrix, self.dist_coeffs, self.rvecs, self.tvecs = cv2.calibrateCamera(
            self.objpoints, 
            self.imgpoints, 
            self.img_shape[::-1],
            None, 
            None
        )
        
        if not ret:
            print("Calibration failed!")
            return False
        
        # Calculate reprojection error
        total_error = 0
        for i in range(len(self.objpoints)):
            imgpoints2, _ = cv2.projectPoints(
                self.objpoints[i], 
                self.rvecs[i], 
                self.tvecs[i], 
                self.camera_matrix, 
                self.dist_coeffs
            )
            error = cv2.norm(self.imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
            total_error += error
        
        self.rms_error = total_error / len(self.objpoints)
        
        print("\n" + "="*60)
        print("CALIBRATION SUCCESSFUL!")
        print("="*60)
        print(f"RMS Reprojection Error: {self.rms_error:.4f} pixels")
        print(f"Image size: {self.img_shape[1]} x {self.img_shape[0]}")
        print(f"\nCamera Matrix (K):")
        print(self.camera_matrix)
        print(f"\nDistortion Coefficients:")
        print(f"k1={self.dist_coeffs[0][0]:.6f}, k2={self.dist_coeffs[0][1]:.6f}, "
              f"p1={self.dist_coeffs[0][2]:.6f}, p2={self.dist_coeffs[0][3]:.6f}, "
              f"k3={self.dist_coeffs[0][4]:.6f}")
        print("="*60 + "\n")
        
        return True
    
    def save_to_orbslam_yaml(self, output_file, camera_fps=30, camera_rgb=1):
        """
        Save calibration results to ORB-SLAM3 YAML format
        """
        if self.camera_matrix is None:
            print("No calibration data available. Run calibration first.")
            return False
        
        # Extract parameters
        fx = float(self.camera_matrix[0, 0])
        fy = float(self.camera_matrix[1, 1])
        cx = float(self.camera_matrix[0, 2])
        cy = float(self.camera_matrix[1, 2])
        
        k1 = float(self.dist_coeffs[0][0])
        k2 = float(self.dist_coeffs[0][1])
        p1 = float(self.dist_coeffs[0][2])
        p2 = float(self.dist_coeffs[0][3])
        k3 = float(self.dist_coeffs[0][4]) if len(self.dist_coeffs[0]) > 4 else 0.0
        
        width = int(self.img_shape[1])
        height = int(self.img_shape[0])
        
        # Create YAML content
        yaml_content = f"""%YAML:1.0

#--------------------------------------------------------------------------------------------
# Camera Parameters. Adjust them!
#--------------------------------------------------------------------------------------------
File.version: "1.0"

Camera.type: "PinHole"

# Camera calibration and distortion parameters (OpenCV)
Camera1.fx: {fx:.6f}
Camera1.fy: {fy:.6f}
Camera1.cx: {cx:.6f}
Camera1.cy: {cy:.6f}

# distortion parameters
Camera1.k1: {k1:.6f}
Camera1.k2: {k2:.6f}
Camera1.p1: {p1:.6f}
Camera1.p2: {p2:.6f}

# Camera resolution
Camera.width: {width}
Camera.height: {height}

# Camera frames per second 
Camera.fps: {camera_fps}

# Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)
Camera.RGB: {camera_rgb}

#--------------------------------------------------------------------------------------------
# ORB Parameters
#--------------------------------------------------------------------------------------------
# ORB Extractor: Number of features per image
ORBextractor.nFeatures: 1250

# ORB Extractor: Scale factor between levels in the scale pyramid 	
ORBextractor.scaleFactor: 1.2

# ORB Extractor: Number of levels in the scale pyramid	
ORBextractor.nLevels: 8

# ORB Extractor: Fast threshold
# Image is divided in a grid. At each cell FAST are extracted imposing a minimum response.
# Firstly we impose iniThFAST. If no corners are detected we impose a lower value minThFAST
# You can lower these values if your images have low contrast			
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7

#--------------------------------------------------------------------------------------------
# Viewer Parameters
#--------------------------------------------------------------------------------------------
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -3.5
Viewer.ViewpointF: 500.0
"""
        
        # Write to file
        with open(output_file, 'w') as f:
            f.write(yaml_content)
        
        print(f"✓ Calibration saved to: {output_file}")
        print(f"  RMS Error: {self.rms_error:.4f} pixels")
        return True


def main():
    parser = argparse.ArgumentParser(
        description='Camera calibration for ORB-SLAM3',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Calibrate from webcam (default)
  python3 calibrate_camera.py
  
  # Calibrate from video file
  python3 calibrate_camera.py --input calibration.mp4
  
  # Calibrate from images
  python3 calibrate_camera.py --input "images/*.jpg"
  
  # Custom chessboard pattern and square size
  python3 calibrate_camera.py --pattern 7x5 --square_size 0.03
  
  # Specify output file and camera parameters
  python3 calibrate_camera.py --output my_camera.yaml --fps 60 --rgb 0
"""
    )
    
    parser.add_argument('--input', type=str, default='0',
                       help='Input source: 0 for webcam, video file path, or image pattern (e.g., "images/*.jpg")')
    parser.add_argument('--pattern', type=str, default='9x6',
                       help='Chessboard pattern size as COLSxROWS (default: 9x6)')
    parser.add_argument('--square_size', type=float, default=0.025,
                       help='Size of chessboard squares in meters (default: 0.025 = 25mm)')
    parser.add_argument('--output', type=str, default='camera_calibration.yaml',
                       help='Output YAML file path (default: camera_calibration.yaml)')
    parser.add_argument('--num_frames', type=int, default=20,
                       help='Number of calibration frames to collect from video (default: 20)')
    parser.add_argument('--fps', type=int, default=30,
                       help='Camera FPS for YAML output (default: 30)')
    parser.add_argument('--rgb', type=int, choices=[0, 1], default=1,
                       help='Color order: 0=BGR, 1=RGB (default: 1)')
    
    args = parser.parse_args()
    
    # Parse pattern size
    try:
        cols, rows = map(int, args.pattern.lower().split('x'))
        pattern_size = (cols, rows)
    except:
        print(f"Error: Invalid pattern format '{args.pattern}'. Use format like '9x6'")
        return 1
    
    print("\n" + "="*60)
    print("ORB-SLAM3 Camera Calibration Tool")
    print("="*60)
    
    # Create calibrator
    calibrator = CameraCalibrator(pattern_size=pattern_size, square_size=args.square_size)
    
    # Determine input type
    if args.input == '0':
        # Webcam
        print(f"Using webcam for calibration...")
        success = calibrator.calibrate_from_video(0, num_frames=args.num_frames)
    elif '*' in args.input or '?' in args.input:
        # Image pattern
        print(f"Using image pattern: {args.input}")
        success = calibrator.calibrate_from_images(args.input)
    elif os.path.isfile(args.input):
        # Video file
        print(f"Using video file: {args.input}")
        success = calibrator.calibrate_from_video(args.input, num_frames=args.num_frames)
    else:
        print(f"Error: Input '{args.input}' not found")
        return 1
    
    if success:
        # Save results
        calibrator.save_to_orbslam_yaml(args.output, camera_fps=args.fps, camera_rgb=args.rgb)
        print(f"\n✓ Calibration complete! You can now use '{args.output}' with ORB-SLAM3")
        return 0
    else:
        print("\n✗ Calibration failed")
        return 1


if __name__ == '__main__':
    exit(main())
