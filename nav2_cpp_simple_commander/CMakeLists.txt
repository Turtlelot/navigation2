cmake_minimum_required(VERSION 3.5)
project(nav2_cpp_simple_commander)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# Create include directories
include_directories(include)

# Declare the executable
add_executable(robot_navigator src/robot_navigator.cpp)

# Specify libraries to link a library or executable target against
ament_target_dependencies(
  robot_navigator
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  lifecycle_msgs
  action_msgs
)

install(TARGETS
robot_navigator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
