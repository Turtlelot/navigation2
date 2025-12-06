cmake_minimum_required(VERSION 3.14)
project(nav2_cpp_simple_commander)

# ========= Language / Flags =========
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(ENABLE_ASAN "Enable AddressSanitizer" OFF)
if(ENABLE_ASAN)
  add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address)
endif()

# ========= Dependencies =========
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# Optional linters
find_package(ament_lint_auto QUIET)
find_package(ament_cmake_clang_format QUIET)
find_package(ament_cmake_cppcheck QUIET)
find_package(ament_cmake_cpplint QUIET)

# ========= Includes =========
include_directories(
  include
)

# ========= Main Library =========
add_library(${PROJECT_NAME} SHARED
  src/navigator.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  nav2_msgs
  geographic_msgs
  geometry_msgs
  lifecycle_msgs
  action_msgs
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ========= Executables =========
## Example 1: navigate to pose
add_executable(example_nav_to_pose
  examples/example_nav_to_pose.cpp
)

target_link_libraries(example_nav_to_pose
  ${PROJECT_NAME}
)

ament_target_dependencies(example_nav_to_pose
  rclcpp
)

## Example 2: navigate through poses
add_executable(go_through_poses_demo
  examples/example_nav_through_poses.cpp
)

target_link_libraries(go_through_poses_demo
  ${PROJECT_NAME}
)

ament_target_dependencies(go_through_poses_demo
  rclcpp
)
## Example 3: follow waypoints
add_executable(follow_waypoints_demo
  examples/example_waypoint_follower.cpp
)

target_link_libraries(follow_waypoints_demo
  ${PROJECT_NAME}
)

ament_target_dependencies(follow_waypoints_demo
  rclcpp
)
## Example 4: inspection waypoints demo
add_executable(inspection_waypoints_demo
  examples/demo_inspection.cpp
)

target_link_libraries(inspection_waypoints_demo
  ${PROJECT_NAME}
)

ament_target_dependencies(inspection_waypoints_demo
  rclcpp
)

# ========= Install =========
install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS
    example_nav_to_pose
    go_through_poses_demo
    follow_waypoints_demo
    inspection_waypoints_demo
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

# ========= Exports =========
ament_export_dependencies(
  rclcpp
  rclcpp_action
  nav2_msgs
  geographic_msgs
  geometry_msgs
  lifecycle_msgs
  action_msgs
)
ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)

# ========= Linters (optional) =========
if(ament_lint_auto_FOUND)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()