## Precualify

- [X] find gate (cv)
- [ ] determine dimensions gate
- [ ] pass in gate (hardcode)
- [x] find object (cv)
- [ ] determine dimensions object
- [x] can we move? (controller)
- [ ] move around? (hardcode)


## Setup 

### System Dependencies

```sh
sudo dnf copr enable tavie/ros2

sudo dnf install ros-jazzy-desktop
sudo dnf install ros-jazzy-vision-msgs

# cv
sudo dnf install ros-jazzy-geometry-msgs
sudo dnf install ros-jazzy-rviz2
sudo dnf install ros-jazzy-usb-cam
sudo dnf install ros-jazzy-pangolin

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

### Clone and go in repo
```sh
git clone --recurisve git@github.com:joseburgosguntin/rumarino-ros2-jazzy.git
cd ./rumarino-ros2-jazzy
```

### Install ORB_SLAM3
#### Install Pangolin
```sh
cd ./vendor/Pangolin
mkdir -p build
cd build
cmake ..
make -j16
sudo make install
cd ../../../../../
```
#### Actual ORB_SLAM3
```sh
cd ./vendor/ORB_SLAM3
mkdir -p build && cd ./build
cmake .. -DCMAKE_BUILD_TYPE=Debug \
         -DCMAKE_EXPORT_TYPE=ON
make -j4
sudo make install
cd ../../../../
```

### Install Stonefish
```sh
cd ./vendor/stonefish
mkdir build
cd build
cmake ..
make -j16 # (where X is the number of threads)
sudo make install
cd ../../../../../
```

## Build

```sh
source /usr/lib64/ros2-jazzy/setup.zsh
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.sh
```

## Topics
/hydrus/thrusters std_msgs/Float64MultiArray
/hydrus/odometry  nav_msgs/Odometry
/hydrus/map       interfaces/Map

## Test

Currently in order to test `mission_planner` we are gonna:
- `controller_stonefish` for /hydrus/thrusters
- `controller_stonefish` for /hydrus/odometry
- `just manualy publish a single Map msg` /hydrus/map Map
