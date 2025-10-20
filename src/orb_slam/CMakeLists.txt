cmake_minimum_required(VERSION 3.8)
project(orb_slam3_ros2)

# ==============================
#  Compiler and build settings
# ==============================
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==============================
#  ROS 2 and external dependencies
# ==============================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)

find_package(OpenCV 4.4 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

# ORB_SLAM3 - use direct library path instead of broken find_package
set(ORB_SLAM3_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../vendor/ORB_SLAM3)
set(ORB_SLAM3_INCLUDE_DIRS
  ${ORB_SLAM3_ROOT_DIR}
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/include
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o/include
)
set(ORB_SLAM3_LIBRARY ${ORB_SLAM3_ROOT_DIR}/lib/libORB_SLAM3.so)
set(DBOW2_LIBRARY ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so)
set(G2O_LIBRARY ${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o/lib/libg2o.so)

# ==============================
#  Dependency list for ament
# ==============================
set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  image_transport
  message_filters
)

# ==============================
#  Build executable node
# ==============================
add_executable(mono src/monocular_node.cpp)

# Make sure ROS2 dependencies and include paths are applied
ament_target_dependencies(mono ${dependencies})

# Link with ORB_SLAM3 and external libs
target_link_libraries(mono
  ${ORB_SLAM3_LIBRARY}            # Direct library path
  ${DBOW2_LIBRARY}                # DBoW2 library
  ${G2O_LIBRARY}                  # g2o library
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  cv_bridge::cv_bridge            # ✅ fixes missing header issue
  -lboost_serialization
  -lcrypto
)

# ==============================
#  Include directories (using direct paths)
# ==============================
target_include_directories(mono PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${ORB_SLAM3_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
)

# ==============================
# Alternative approaches for manual include paths (if needed):
# ==============================
# Option 1: Use CMAKE_INSTALL_PREFIX (respects custom install locations)
# target_include_directories(mono PRIVATE 
#   ${CMAKE_INSTALL_PREFIX}/include/ORB_SLAM3
#   ${CMAKE_INSTALL_PREFIX}/include/ORB_SLAM3/CameraModels
# )
#
# Option 2: Use pkg-config variables (if ORB_SLAM3 provides .pc file)
# find_package(PkgConfig REQUIRED)
# pkg_check_modules(ORB_SLAM3_PC REQUIRED ORB_SLAM3)
# target_include_directories(mono PRIVATE ${ORB_SLAM3_PC_INCLUDE_DIRS})
#
# Option 3: Use environment variable for custom installs
# target_include_directories(mono PRIVATE 
#   $ENV{ORB_SLAM3_ROOT}/include/ORB_SLAM3
#   $ENV{ORB_SLAM3_ROOT}/include/ORB_SLAM3/CameraModels
# )
#
# Option 4: Use find_path to locate headers dynamically
# find_path(ORB_SLAM3_INCLUDE_DIR 
#   NAMES System.h
#   PATHS /usr/local/include/ORB_SLAM3 /usr/include/ORB_SLAM3 /opt/ORB_SLAM3/include
#   PATH_SUFFIXES ORB_SLAM3
# )
# if(ORB_SLAM3_INCLUDE_DIR)
#   target_include_directories(mono PRIVATE ${ORB_SLAM3_INCLUDE_DIR})
# endif()
# ==============================
# ==============================
#  Install targets and resources
# ==============================
install(TARGETS
  mono
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# ==============================
#  Package export for ament
# ==============================
ament_package()

