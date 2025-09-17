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

In order to only test `mission_planner` we are gonna asume the rumarino 
already saw all the objects, actually instead its just gonna get bombarded with 
"/detector/box_detection" in the start-up time

So each test case will have:
- objects positions
- map bounding box

So test runner is a ros2-jazzy node that: 
- poops out "/detector/box_detection"
- visual thingy with raylib (wink wink) (if `--visual` flag is passed)
- fails when hit wall (colision) (think about gate)
