// Requires implementations of the following functions:
// setInitialPose() – set robot’s start pose

// waitUntilNav2Active() – wait for Nav2 to be ready

// goToPose() – send goal pose

// isTaskComplete() – check if navigation finished

// getFeedback() – get progress updates

// cancelTask() – cancel the goal if needed

// getResult() – get final result status

// lifecycleShutdown() – shutdown Nav2 nodes

// get_clock() – get current ROS time

#include <chrono>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_cpp_simple_commander/navigator.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace Nav2SimpleCommander;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("example_nav_to_pose");
  Navigator navigator(node);

  // Set initial pose
  geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
  initial_pose.header.frame_id = "map";
  initial_pose.header.stamp = node->now();
  initial_pose.pose.pose.position.x = -2.0;
  initial_pose.pose.pose.position.y = 0.0;
  initial_pose.pose.pose.orientation.w = 1.0;

  // navigator.setInitialPose(initial_pose);

  // navigator.lifecycleStartup();
  // // Wait for nav2 stack to become active
  // navigator.waitUntilNav2Active();

  // // Define goal pose
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.header.stamp = node->now();
  goal_pose.pose.position.x = -2.0;
  goal_pose.pose.position.y = -0.5;
  goal_pose.pose.orientation.w = 1.0;

  // // Send goal
  navigator.goToPose(goal_pose);

  rclcpp::sleep_for(std::chrono::seconds(3));
  navigator.cancelTask();

  // navigator.lifecycleShutdown();

  rclcpp::shutdown();
  return 0;
}
