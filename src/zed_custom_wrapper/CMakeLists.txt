cmake_minimum_required(VERSION 3.8)
project(zed_custom_wrapper)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(zed_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ZED REQUIRED)

include_directories(
  include
  ${ZED_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)

link_directories(
  ${ZED_LIBRARY_DIR}
  ${CUDA_LIBRARY_DIRS}
)

add_executable(zed_custom_node src/zed_custom_node.cpp)
ament_target_dependencies(zed_custom_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  image_transport
  tf2_ros
  tf2_geometry_msgs
  zed_msgs
  interfaces
  vision_msgs
  visualization_msgs
)

target_link_libraries(zed_custom_node
  ${ZED_LIBRARIES}
  ${CUDA_LIBRARIES}
  ${CUDA_CUDA_LIBRARY}
  ${CUDA_CUDART_LIBRARY}
)

install(TARGETS zed_custom_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
