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
sudo dnf install ros-jazzy-desktop
sudo dnf install ros-jazzy-vision-msgs

sudo dnf install freetype-devel
sudo dnf install SDL2-devel
sudo dnf install glm-devel
sudo dnf install eigen3-devel
```

### Clone and go in repo
```sh
git clone --recurisve git@github.com:joseburgosguntin/rumarino-ros2-jazzy.git
cd ./rumarino-ros2-jazzy
```

### Install Stonefish
```sh
cd ./vendor/stonefish
mkdir build
cd build
cmake ..
make -jX (where X is the number of threads)
sudo make install
```

## Build

```sh
source /usr/lib64/ros2-jazzy/setup.zsh
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.sh
```

## Topics
/hydrus/thrustors std_msgs/Float64MultiArray
/hydrus/odometry  nav_msgs/Odometry
/hydrus/map       interfaces/Map

## Test

Currently in order to test `mission_planner` we are gonna:
- `controller_stonefish` for /hydrus/thrustors
- `controller_stonefish` for /hydrus/odometry
- `just manualy publish a single Map msg` /hydrus/map Map
