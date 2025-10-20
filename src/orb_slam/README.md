# ORB_SLAM3 ROS2 Package

This document provides complete step-by-step instructions to build and run the ORB_SLAM3 ROS2 package.

## Prerequisites

### System Requirements
- Ubuntu 22.04 or later
- ROS2 Jazzy installed
- CMake 3.5 or higher
- GCC 7.0 or higher
- Eigen3
- OpenCV 4.x
- Pangolin (available through ROS2 installation)

### ROS2 Dependencies
```bash
sudo apt update
sudo apt install ros-jazzy-desktop-full
sudo apt install ros-jazzy-cv-bridge ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs
sudo apt install ros-jazzy-sensor-msgs ros-jazzy-geometry-msgs ros-jazzy-nav-msgs
```

### Development Tools
```bash
sudo apt install build-essential cmake git
sudo apt install libeigen3-dev libopencv-dev
```

## Building Instructions

### Step 1: Navigate to Project Directory
```bash
cd /home/cesar/Projects/ORB_SLAM3
```

### Step 2: Clean Previous Builds (if any)
```bash
# Remove any previous build artifacts
rm -rf build/ lib/
rm -rf ros2_ws/build/ ros2_ws/install/ ros2_ws/log/

# Clean third-party builds
sudo rm -rf Thirdparty/*/build/

# Remove any local Pangolin directory (we use system Pangolin)
rm -rf Pangolin/
```

### Step 3: Setup Environment for System Pangolin
```bash
# Set CMAKE_PREFIX_PATH to find system Pangolin from ROS2
export CMAKE_PREFIX_PATH="/opt/ros/jazzy:$CMAKE_PREFIX_PATH"
```

### Step 4: Build ORB_SLAM3 Main Library
```bash
# Ensure build script won't download Pangolin (already modified)
# The build.sh has been modified to use system Pangolin instead of downloading

# Build the main ORB_SLAM3 library
./build.sh
```

**Expected Output:** 
- Build should complete successfully with some warnings
- `libORB_SLAM3.so` should be created in the `lib/` directory
- Third-party libraries (DBoW2, g2o) will be built automatically

### Step 5: Extract Vocabulary File (if needed)
```bash
cd Vocabulary/
# Check if vocabulary file exists
if [ ! -f "ORBvoc.txt" ] || [ ! -s "ORBvoc.txt" ]; then
    echo "Extracting vocabulary file..."
    tar -xzf ORBvoc.txt.tar.gz
fi
cd ..
```

### Step 6: Build ROS2 Package
```bash
cd ros2_ws/

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash

# Build the ROS2 package
colcon build --packages-select orb_slam3_ros2

# Source the built package
source install/setup.bash
```

**Expected Output:**
- Package should build successfully with warnings (deprecation warnings are normal)
- Build summary should show "1 package finished"

## Running Instructions

### Step 1: Setup Environment
```bash
cd /home/cesar/Projects/ORB_SLAM3/ros2_ws

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash

# Source the built package
source install/setup.bash

# Set library path for ORB_SLAM3
export LD_LIBRARY_PATH=/home/cesar/Projects/ORB_SLAM3/lib:$LD_LIBRARY_PATH
```

### Step 2: Run the SLAM Node

**Option 1: Monocular mode (default):**
```bash
ros2 run orb_slam3_ros2 mono --ros-args --params-file /home/cesar/Projects/ORB_SLAM3/ROS2/config/params.yaml
```

**Option 2: RGB-D mode (with depth):**
```bash
ros2 run orb_slam3_ros2 mono --ros-args --params-file /home/cesar/Projects/ORB_SLAM3/ROS2/config/rgbd_params.yaml
```

**Option 3: Manual parameter override:**
```bash
# Enable RGB-D mode on the fly
ros2 run orb_slam3_ros2 mono --ros-args --params-file /home/cesar/Projects/ORB_SLAM3/ROS2/config/params.yaml -p use_depth:=true
```

**With custom camera (USB camera example):**
```bash
ros2 run orb_slam3_ros2 mono --ros-args --params-file /home/cesar/Projects/ORB_SLAM3/ROS2/config/usb_cam_params.yaml
```

**With custom camera (RealSense example):**
```bash
ros2 run orb_slam3_ros2 mono --ros-args --params-file /home/cesar/Projects/ORB_SLAM3/ROS2/config/realsense_params.yaml
```

**Override specific parameters:**
```bash
ros2 run orb_slam3_ros2 mono --ros-args \
    --params-file /home/cesar/Projects/ORB_SLAM3/ROS2/config/params.yaml \
    -p image_topic:=/my_camera/image_raw \
    -p use_viewer:=false
```

### Step 3: Verify Node is Running
The node should start successfully and display:
```
[INFO] [timestamp] [orb_slam3_mono]: === ORB-SLAM3 Configuration ===
[INFO] [timestamp] [orb_slam3_mono]: Vocabulary: /home/cesar/Projects/ORB_SLAM3/Vocabulary/ORBvoc.txt
[INFO] [timestamp] [orb_slam3_mono]: Settings: /home/cesar/Projects/ORB_SLAM3/ROS2/config/webcam.yaml
[INFO] [timestamp] [orb_slam3_mono]: === ROS2 Parameters ===
[INFO] [timestamp] [orb_slam3_mono]: Image topic: /camera1/image_raw
[INFO] [timestamp] [orb_slam3_mono]: ORB-SLAM3 initialized!
[INFO] [timestamp] [orb_slam3_mono]: Monocular SLAM node ready!
Starting the Viewer
```

## Configuration

### Sensor Mode Configuration

The node supports **two sensor modes** configured via the `use_depth` parameter:

#### 1. **Monocular Mode** (`use_depth: false`) - Default
- Uses only camera images
- Single camera required
- Scale is relative (unknown absolute scale)
- **Best for:** Single camera setups, outdoor environments, cost-effective solutions

#### 2. **RGB-D Mode** (`use_depth: true`)
- Uses camera images + depth data
- Requires RGB-D camera (RealSense D435/D455, Kinect, etc.)
- Provides known absolute scale
- More accurate 3D reconstruction
- **Best for:** Indoor robotics, manipulation tasks, when depth sensor is available

### ROS2 Parameters File

All configuration is done through ROS2 parameter files in `/home/cesar/Projects/ORB_SLAM3/ROS2/config/`:

**Available parameter files:**
- `params.yaml` - **Monocular mode** configuration (default)
- `rgbd_params.yaml` - **RGB-D mode** configuration  
- `usb_cam_params.yaml` - USB camera configuration
- `realsense_params.yaml` - Intel RealSense configuration

**Parameters explained:**

```yaml
orb_slam3_node:
  ros__parameters:
    # File paths
    vocabulary_path: "/path/to/ORBvoc.txt"        # ORB vocabulary file
    settings_path: "/path/to/camera_config.yaml"  # Camera calibration file
    use_viewer: true                               # Enable/disable Pangolin viewer
    
    # Sensor mode selection
    use_depth: false                               # true=RGB-D mode, false=Monocular mode
    
    # Topic configuration
    image_topic: "/camera/image_raw"              # Input RGB/grayscale image
    depth_topic: "/camera/depth/image_raw"        # Input depth (only if use_depth=true)
    pose_topic: "orb_slam3/camera_pose"           # Output pose topic
    path_topic: "orb_slam3/camera_path"           # Output path topic
    
    # Frame IDs
    world_frame_id: "world"                       # World/map frame
    camera_frame_id: "camera"                     # Camera frame
    
    # Settings
    queue_size: 10                                # Subscriber queue size
    publish_tf: true                              # Publish TF transforms
```

### Customizing for Your Camera

1. **Copy an existing parameter file:**
   ```bash
   cp /home/cesar/Projects/ORB_SLAM3/ROS2/config/params.yaml \
      /home/cesar/Projects/ORB_SLAM3/ROS2/config/my_camera_params.yaml
   ```

2. **Edit the image topic to match your camera:**
   ```yaml
   image_topic: "/my_camera/image_raw"  # Change this to your camera's topic
   ```

3. **Run with your custom parameters:**
   ```bash
   ros2 run orb_slam3_ros2 mono --ros-args \
       --params-file /home/cesar/Projects/ORB_SLAM3/ROS2/config/my_camera_params.yaml
   ```

### Camera Calibration Settings

Edit the camera calibration YAML files (e.g., `webcam.yaml`) to modify:
- Camera intrinsic parameters (fx, fy, cx, cy)
- Distortion coefficients (k1, k2, p1, p2, k3)
- Image resolution
- ORB feature extraction parameters

## Configuration

### Camera Configuration
Edit `/home/cesar/Projects/ORB_SLAM3/ROS2/config/webcam.yaml` or `params.yaml` to modify:
- Camera intrinsic parameters (fx, fy, cx, cy)
- Distortion coefficients
- Image resolution
- ORB feature extraction parameters

### ROS2 Topics
The node subscribes to:
- `/camera1/image_raw` (sensor_msgs/Image) - Camera images

The node publishes:
- `orb_slam3/camera_pose` (geometry_msgs/PoseStamped) - Camera pose
- `orb_slam3/camera_path` (nav_msgs/Path) - Camera trajectory

## Testing with Camera Data

### Option 1: Use USB Camera
```bash
# In a new terminal, install and run usb_cam
sudo apt install ros-jazzy-usb-cam
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0 -p image_topic:=/camera1/image_raw
```

### Option 2: Publish Test Images
```bash
# Install image_tools for testing
sudo apt install ros-jazzy-image-tools

# Publish test images (change topic name to match)
ros2 run image_tools cam2image --ros-args -p topic:=/camera1/image_raw
```

### Option 3: Use Rosbag
```bash
# Play a recorded rosbag with camera data
ros2 bag play your_camera_data.bag --remap /original_topic:=/camera1/image_raw
```

## Troubleshooting

### Build Issues

**Problem: "Pangolin not found"**
```bash
# Ensure ROS2 environment is sourced
source /opt/ros/jazzy/setup.bash
export CMAKE_PREFIX_PATH="/opt/ros/jazzy:$CMAKE_PREFIX_PATH"
```

**Problem: "libORB_SLAM3.so not found"**
```bash
# Add library path
export LD_LIBRARY_PATH=/home/cesar/Projects/ORB_SLAM3/lib:$LD_LIBRARY_PATH
```

**Problem: CMake configuration errors**
```bash
# Clean and rebuild
rm -rf build/ lib/ ros2_ws/build/ ros2_ws/install/
# Then follow build steps again
```

### Runtime Issues

**Problem: "Package 'orb_slam3_ros2' not found"**
```bash
# Make sure to source the built package
cd /home/cesar/Projects/ORB_SLAM3/ros2_ws
source install/setup.bash
```

**Problem: Node starts but no SLAM tracking**
- Verify camera images are being published: `ros2 topic echo /camera1/image_raw`
- Check camera calibration parameters in config file
- Ensure sufficient lighting and texture in camera view

### Performance Tips
- Use a good camera with stable framerate
- Ensure proper camera calibration
- Provide sufficient texture and lighting for feature extraction
- Start with slow, smooth camera movements for initialization

## File Structure
```
ORB_SLAM3/
├── build.sh                    # Main build script (modified for system Pangolin)
├── CMakeLists.txt             # Main CMake configuration
├── lib/                       # Built libraries
│   └── libORB_SLAM3.so
├── Vocabulary/                # ORB vocabulary
│   └── ORBvoc.txt
├── ROS2/                      # ROS2 package source
│   ├── CMakeLists.txt         # ROS2 package CMake (modified)
│   ├── package.xml
│   ├── config/
│   │   ├── params.yaml
│   │   └── webcam.yaml
│   └── src/
│       └── monocular_node.cpp
└── ros2_ws/                   # ROS2 workspace
    ├── build/
    ├── install/
    └── src/
        └── orb_slam3_ros2 -> ../ROS2/  # Symlink to ROS2 package
```

## Quick Reference

### Complete Build Process
```bash
# 1. Clean and setup
cd /home/cesar/Projects/ORB_SLAM3
rm -rf build/ lib/ ros2_ws/build/ ros2_ws/install/ ros2_ws/log/
sudo rm -rf Thirdparty/*/build/
rm -rf Pangolin/
source /opt/ros/jazzy/setup.bash
export CMAKE_PREFIX_PATH="/opt/ros/jazzy:$CMAKE_PREFIX_PATH"

# 2. Build main library
./build.sh

# 3. Extract vocabulary (if needed)
cd Vocabulary/
if [ ! -f "ORBvoc.txt" ] || [ ! -s "ORBvoc.txt" ]; then
    tar -xzf ORBvoc.txt.tar.gz
fi
cd ..

# 4. Build ROS2 package
cd ros2_ws/
source /opt/ros/jazzy/setup.bash
colcon build --packages-select orb_slam3_ros2
```

### Run the Node
```bash
cd /home/cesar/Projects/ORB_SLAM3/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=/home/cesar/Projects/ORB_SLAM3/lib:$LD_LIBRARY_PATH

# Run with default parameters
ros2 run orb_slam3_ros2 mono --ros-args \
    --params-file /home/cesar/Projects/ORB_SLAM3/ROS2/config/params.yaml

# Or with custom camera settings
ros2 run orb_slam3_ros2 mono --ros-args \
    --params-file /home/cesar/Projects/ORB_SLAM3/ROS2/config/usb_cam_params.yaml
```

## Example: Testing with Webcam

1. **Build everything:**
   ```bash
   cd ~/Projects/ORB_SLAM3
   ./build.sh
   
   cd ~/ros2_ws
   colcon build --packages-select orb_slam3_ros2
   source install/setup.bash
   ```

2. **Launch:**
   ```bash
   ros2 launch orb_slam3_ros2 mono_webcam.launch.py
   ```

3. **Visualize in RViz2 (optional):**
   ```bash
   # In another terminal
   ros2 run rviz2 rviz2
   # Add Path display with topic /orb_slam3/camera_path
   ```

4. **Move your camera** slowly to initialize tracking

5. **Stop with Ctrl+C** - trajectories will be saved

## Troubleshooting

### No images received
```bash
# Check if camera is publishing
ros2 topic list
ros2 topic hz /camera/image_raw
ros2 topic echo /camera/image_raw --no-arr
```

### Camera not found
```bash
# List available cameras
ls /dev/video*

# Try different camera ID
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video1
```

### Build errors
```bash
# Make sure ORB-SLAM3 library exists
ls ~/Projects/ORB_SLAM3/lib/libORB_SLAM3.so

# Source ROS2 properly
source /opt/ros/humble/setup.bash
```

### Poor tracking
- Calibrate your camera properly
- Ensure good lighting
- Move camera slowly during initialization
- Check that feature count is appropriate in config file

## Advanced Usage

### Custom Topics via Parameters

Edit `config/params.yaml` to customize topic names:

```yaml
orb_slam3_mono:
  ros__parameters:
    image_topic: "/my_robot/camera/image"
    camera_info_topic: "/my_robot/camera/info"
    pose_topic: "/slam/pose"
    path_topic: "/slam/path"
    world_frame_id: "map"
    camera_frame_id: "camera_optical_frame"
```

Or override at launch time:
```bash
ros2 run orb_slam3_ros2 mono \
  <vocab> <settings> true \
  --ros-args \
  -p image_topic:=/my_camera/image \
  -p camera_info_topic:=/my_camera/info \
  -p world_frame_id:=map \
  -p publish_tf:=false
```

### Disable TF Publishing

To disable TF broadcasting (useful if you have your own localization):
```yaml
publish_tf: false
```

### Disable Viewer

Set the viewer argument to `false` for headless operation:
```bash
ros2 run orb_slam3_ros2 mono <vocab> <settings> false
```

### Record a Bag File

```bash
# Record camera images
ros2 bag record /camera/image_raw

# Play back
ros2 bag play <bagfile>
```

## Integration with Your Robot

To integrate with your robot:

1. **Match topic names** in your launch file or use remapping
2. **Calibrate your camera** and create a config YAML
3. **Test with recorded data** first (bag files)
4. **Tune ORB parameters** for your environment
5. **Use published pose** for navigation or other tasks

## Code Structure

```
ROS2/
├── src/
│   └── monocular_node.cpp    # Monocular SLAM node
├── launch/
│   └── mono_webcam.launch.py # Launch file for webcam
├── config/
│   └── webcam.yaml           # Camera configuration
├── CMakeLists.txt            # Build configuration
├── package.xml               # ROS2 package manifest
└── README.md                 # This file
```

## References

- [ORB-SLAM3 Paper](https://arxiv.org/abs/2007.11898)
- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [cv_bridge Tutorial](https://docs.ros.org/en/humble/p/cv_bridge/)
