#include <chrono>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_cpp_simple_commander/navigator.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace nav2_simple_commander;

/**
 * @brief Basic navigation demo to follow a given path after smoothing.
 *
 * C++ implementation of Nav2 Simple Commander `example_follow_path.py`.
 *
 * Flow:
 *  - Compute path to goal
 *  - Smooth the path
 *  - Follow the smoothed path
 *  - Print FollowPath feedback (distance_to_goal, speed)
 *  - Print final TaskResult
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("example_follow_path");

  // Scope ensures Navigator is destroyed BEFORE rclcpp::shutdown().
  // Destroy Navigator before rclcpp::shutdown() to avoid ROS2 context errors.
  {
    Navigator navigator(node);

    // (Optional) Set initial robot pose in map

    geometry_msgs::msg::PoseStamped initial_pose;  // Tested in turtlebot3_world / house world
    initial_pose.header.frame_id = "map";
    initial_pose.header.stamp = node->now();
    initial_pose.pose.position.x = 4.0;
    initial_pose.pose.position.y = 2.0;
    initial_pose.pose.orientation.x = 0.0;
    initial_pose.pose.orientation.y = 0.0;
    initial_pose.pose.orientation.z = 1.0;
    initial_pose.pose.orientation.w = 0.0;

    // navigator.setInitialPose(initial_pose);

    // (Optional) Startup Nav2 lifecycle manager (if not autostarted)
    // navigator.lifecycleStartup();

    // Wait for Nav2 to activate fully
    // navigator.waitUntilNav2Active();

    // Define goal pose
    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.frame_id = "map";
    goal_pose.header.stamp = node->now();
    goal_pose.pose.position.x = 0.5;
    goal_pose.pose.position.y = -2.0;
    goal_pose.pose.orientation.w = 1.0;

    // Compute global path
    auto maybe_path = navigator.getPath(initial_pose, goal_pose);
    if (!maybe_path) {
      RCLCPP_ERROR(node->get_logger(), "Failed to compute a path to the goal.");
      rclcpp::shutdown();
      return 1;
    }
    nav_msgs::msg::Path path = *maybe_path;

    // Smooth the path
    auto maybe_smoothed = navigator.smoothPath(path);
    if (!maybe_smoothed) {
      RCLCPP_ERROR(node->get_logger(), "Failed to smooth the computed path.");
      rclcpp::shutdown();
      return 1;
    }
    nav_msgs::msg::Path smoothed_path = *maybe_smoothed;

    // Follow the smoothed path
    if (!navigator.followPath(smoothed_path)) {
      RCLCPP_ERROR(node->get_logger(), "Failed to send FollowPath goal.");
      rclcpp::shutdown();
      return 1;
    }

    // Monitor FollowPath feedback
    int i = 0;
    while (rclcpp::ok() && !navigator.isTaskComplete()) {
      auto feedback = navigator.getFeedback<Navigator::FollowPath>();
      if (feedback && i++ % 5 == 0) {
        RCLCPP_INFO(
          node->get_logger(), "Distance remaining: %.3f m | Speed: %.3f m/s",
          feedback->distance_to_goal, feedback->speed);
      }
      rclcpp::sleep_for(200ms);
    }

    // Handle result of FollowPath
    switch (navigator.getTaskResult()) {
      case TaskResult::kSucceeded:
        RCLCPP_INFO(node->get_logger(), "Result :: Goal succeeded!");
        break;
      case TaskResult::kCanceled:
        RCLCPP_WARN(node->get_logger(), "Result :: Goal was canceled!");
        break;
      case TaskResult::kFailed:
        RCLCPP_ERROR(node->get_logger(), "Result :: Goal failed!");
        break;
      default:
        RCLCPP_ERROR(node->get_logger(), "Result :: Goal returned unknown status.");
        break;
    }
  }

  rclcpp::shutdown();
  return 0;
}
