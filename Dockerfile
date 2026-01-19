FROM ros:jazzy-ros-base

# Install dependencies matching README Ubuntu instructions
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-venv \
    build-essential \
    curl \
    cmake \
    git \
    # Clang/LLVM (required for Rust ROS 2 bindings)
    libclang-dev \
    llvm-dev \
    clang \
    # ROS 2 build tools
    python3-colcon-common-extensions \
    python3-rosdep \
    # ROS 2 packages needed by stonefish_ros2
    ros-jazzy-vision-msgs \
    ros-jazzy-image-transport \
    ros-jazzy-pcl-conversions \
    ros-jazzy-pcl-ros \
    # Stonefish dependencies (from README)
    libfreetype6-dev \
    libsdl2-dev \
    libglm-dev \
    libeigen3-dev \
    libogre-1.9-dev \
    libopencv-dev \
    libssl-dev \
    libboost-all-dev \
    libepoxy-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Rust
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"
ENV CARGO_TARGET_DIR=/ros2_ws/target

# Create workspace
WORKDIR /ros2_ws
COPY vendor/stonefish ./vendor/stonefish

# Build Stonefish library
RUN cd vendor/stonefish && \
    mkdir -p build && cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig

# Copy only package.xml files for dependency caching
COPY src/interfaces/package.xml ./src/interfaces/package.xml
COPY src/bringup/package.xml ./src/bringup/package.xml
COPY src/controller_stonefish/package.xml ./src/controller_stonefish/package.xml
COPY src/mission_executor/package.xml ./src/mission_executor/package.xml
COPY vendor/stonefish_ros2/package.xml ./src/stonefish_ros2/package.xml

RUN rosdep init || true && \
    rosdep update

RUN bash -c "source /opt/ros/jazzy/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y || true"

# build interfaces first
COPY src/interfaces ./src/interfaces
RUN bash -lc "source /opt/ros/jazzy/setup.bash && \
    colcon build --packages-select interfaces --cmake-args -DCMAKE_BUILD_TYPE=Release"

# copy mission_executor files except actual src/mission_executor/src/*
COPY Cargo.toml Cargo.lock ./
COPY src/mission_executor/Cargo.toml ./src/mission_executor/Cargo.toml
COPY src/mission_executor/Cargo.lock ./src/mission_executor/Cargo.lock
COPY src/mission_executor/CMakeLists.txt ./src/mission_executor/CMakeLists.txt
COPY src/mission_executor/r2r_cargo.cmake ./src/mission_executor/r2r_cargo.cmake
COPY src/mission_executor/dummy.c ./src/mission_executor/dummy.c

# add dummy main.rs to src/mission_executor/src/ so cargo allows building mission_executor
RUN mkdir -p src/mission_executor/src && \ 
    printf "fn main() {}" > src/mission_executor/src/main.rs

# fetch and build dependencies for mission_executor
RUN bash -lc "source /opt/ros/jazzy/setup.bash && \
    source install/setup.bash && \
    colcon build --packages-select mission_executor --cmake-args -DCMAKE_BUILD_TYPE=Release"

COPY src/bringup ./src/bringup
COPY src/controller_stonefish ./src/controller_stonefish
COPY vendor/stonefish_ros2 ./src/stonefish_ros2

# build ROS 2 workspace except interfaces and mission_executor
RUN bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build \
    --packages-select stonefish_ros2 controller_stonefish bringup \
    --cmake-args -DCMAKE_BUILD_TYPE=Release"

# build mission_executor with actual source code
COPY src/mission_executor/src ./src/mission_executor/src
RUN ln -sf /ros2_ws/src/mission_executor/target /ros2_ws/target && \
    bash -c "source /opt/ros/jazzy/setup.bash && \
    source install/setup.bash && \
    colcon build \
    --packages-select mission_executor \
    --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Set environment variables
ENV ROS_DOMAIN_ID=0

