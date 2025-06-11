cmake_minimum_required(VERSION 3.5)
project(nav2_cpp_simple_commander)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# Include directories
include_directories(
  include
)

# Add the executable
add_executable(robot_navigator
  src/robot_navigator.cpp
  src/navigator.cpp
)

# Link dependencies to the executable
ament_target_dependencies(robot_navigator
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  lifecycle_msgs
  action_msgs
)

# Install the executable
install(TARGETS robot_navigator
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Package declaration
ament_package()