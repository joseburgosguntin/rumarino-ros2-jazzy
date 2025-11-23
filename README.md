
## Simulation:


### Clone and go in repo
```sh
git clone --recurisve git@github.com:joseburgosguntin/rumarino-ros2-jazzy.git
cd ./rumarino-ros2-jazzy
```


## System Dependencies

### Required Tools
- Python 3
- C++ compiler (GCC)
- Rust

### Fedora:
```sh
# Install essential build tools
sudo dnf install python3 python3-pip gcc gcc-c++ rust cargo

# Install ROS 2
sudo dnf copr enable tavie/ros2
sudo dnf install ros-jazzy-desktop
sudo dnf install ros-jazzy-vision-msgs
sudo dnf install freetype-devel
sudo dnf install SDL2-devel
sudo dnf install glm-devel
sudo dnf install eigen3-devel
sudo dnf install ogre-devel
sudo dnf install opencv-devel
sudo dnf install openssl-devel
sudo dnf install boost-devel
sudo dnf install libepoxy-devel
python3 -m pip install wheel
```
### Ubuntu:
```bash
# Install essential build tools
sudo apt update
sudo apt install -y python3 python3-pip python3-venv build-essential curl

# Install Clang/LLVM (required for Rust ROS 2 bindings)
sudo apt install -y libclang-dev llvm-dev clang

# Add ROS 2 repository (if not already added)
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Install ROS 2 and dependencies
sudo apt update
sudo apt install -y ros-jazzy-desktop \
    ros-jazzy-vision-msgs \
    libfreetype6-dev \
    libsdl2-dev \
    libglm-dev \
    libeigen3-dev \
    libogre-1.9-dev \
    libopencv-dev \
    libssl-dev \
    libboost-all-dev \
    libepoxy-dev
```

### Install Stonefish Simulator
```sh
cd ./vendor/stonefish
mkdir build
cd build
cmake ..
make -j16 # (where X is the number of threads)
sudo make install
cd ../../../../../
```

## Test mission\_executor

```sh
# Navigate to the workspace
cd ~/ros2_ws/rumarino-ros2-jazzy

# Source ROS 2 environment
# Fedora:
source /usr/lib64/ros2-jazzy/setup.zsh
# Ubuntu:
source /opt/ros/jazzy/setup.bash


# Build packages
colcon build --packages-select interfaces bringup Stonefish stonefish_ros2 controller_stonefish mission_executor 

# Source the workspace
#Fedora
source install/setup.sh
#Ubuntu
source install/setup.bash

ros2 launch bringup test_mission_executor.launch.py mission_name:=prequalify env_file_name:=hydrus_env.scn
```


## Computer Vision

### Dependencies
  - [ZED-SDK 5.1](https://www.stereolabs.com/developers/release)
  - [Cuda 12.8](https://developer.nvidia.com/cuda-12-8-0-download-archive)

### Run ZED Custom Wrapper
```sh
colcon build --packages-select zed_custom_wrapper && source ./install/setup.bash && ros2 launch zed_custom_wrapper zed_custom.launch.py onnx_model_path:=./src/zed_custom_wrapper/yolov8n.onnx
```
