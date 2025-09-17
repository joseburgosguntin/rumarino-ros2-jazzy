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

sudo dnf install freetype-devel
sudo dnf install SDL2-devel
sudo dnf install glm-devel
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

## Test

Currently in order to test `mission_planner` we are gonna:
- use `controller_stonefish` to mock `controller_arduino`,
- use `??? something in stonefish` to mock `zed2i stuff`
- use `??? poop a complete map` to mock `omniscience stuff`
