cmake_minimum_required(VERSION 3.16)
project(controller_arduino VERSION 0.0.1 LANGUAGES CXX)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interfaces REQUIRED)

# find_package(rosidl_default_generators REQUIRED)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   "action/Navigate.action"
#   DEPENDENCIES geometry_msgs std_msgs
# )

# Include headers
include_directories(include)

# Node executable
add_executable(controller_node src/main.cc)
ament_target_dependencies(controller_node 
  rclcpp 
  std_msgs 
  rclcpp_action
  interfaces
)

# Install the executable
install(TARGETS
  controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers (if you have any)
install(DIRECTORY include/
  DESTINATION include/
)

# Ament package
ament_package()
