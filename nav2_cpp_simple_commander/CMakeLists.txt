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
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# Include header files
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
  geographic_msgs
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

## Add the executable for examples 
add_executable(example_nav_to_pose
  examples/example_nav_to_pose.cpp
  src/navigator.cpp
)

# Link dependencies to the executable
ament_target_dependencies(example_nav_to_pose
  rclcpp
  rclcpp_action
  nav2_msgs
  geographic_msgs
  geometry_msgs
  lifecycle_msgs
  action_msgs
)


# Install the test node
install(TARGETS example_nav_to_pose
  DESTINATION lib/${PROJECT_NAME}
)

## Add the executable for examples 
add_executable(go_through_poses_demo
  examples/example_nav_through_poses.cpp
  src/navigator.cpp
)

# Link dependencies to the executable
ament_target_dependencies(go_through_poses_demo
  rclcpp
  rclcpp_action
  nav2_msgs
  geographic_msgs
  geometry_msgs
  lifecycle_msgs
  action_msgs
)

# Install the test node
install(TARGETS go_through_poses_demo
  DESTINATION lib/${PROJECT_NAME}
)
# Package declaration
ament_package()