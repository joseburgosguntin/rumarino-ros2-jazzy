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

## Test mission\_executor

```sh
source /usr/lib64/ros2-jazzy/setup.zsh
colcon build --packages-select interfaces bringup Stonefish stonefish_ros2 controller_stonefish mission_executor
source ./install/setup.sh
ros2 launch bringup test_mission_executor.launch.py mission_name:=prequalify env_file_name:=prequalify_env.scn
```

To test individually each component (to debug mission-executor), instead of bringup do:
```sh
ros2 launch controller_stonefish hydrussim.launch.py
```

In another terminal:
```sh
ros2 run bringup oneshot_map_node
```

And launch the mission\_executor. To simply execute it:
```sh
ros2 run mission_executor mission_executor --ros-args -p "mission_name:=prequalify"
```

To debug the binary in gdb:
```sh
gdb --args ./target/debug/mission_executor --ros-args -p mission_name:=prequalify
```

You can also run the Debug target in VS code for the same effect.

## Topics
/hydrus/thrusters std_msgs/Float64MultiArray
/hydrus/odometry  nav_msgs/Odometry
/hydrus/map       interfaces/Map
