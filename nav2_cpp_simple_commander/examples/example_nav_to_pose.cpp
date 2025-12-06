#include <chrono>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_cpp_simple_commander/navigator.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace nav2_simple_commander;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("example_nav_to_pose");
  // Scope ensures Navigator is destroyed BEFORE rclcpp::shutdown().
  // Destroy Navigator before rclcpp::shutdown() to avoid ROS2 context errors.
  {
    Navigator navigator(node);

    // (Optional) Set initial robot pose in map
    // tested in turtlebot3_world
    geometry_msgs::msg::PoseStamped initial_pose;
    initial_pose.header.frame_id = "map";
    initial_pose.header.stamp = node->now();
    initial_pose.pose.position.x = -2.0;
    ;
    initial_pose.pose.orientation.x = 0.0;
    initial_pose.pose.orientation.y = 0.0;
    initial_pose.pose.orientation.z = 0.0;
    initial_pose.pose.orientation.w = 1.0;

    navigator.setInitialPose(initial_pose);

    //(Optional) Startup Nav2 lifecycle manager (if not autostarted)
    navigator.lifecycleStartup();

    // (Optional) Wait for Nav2 to activate fully
    navigator.waitUntilNav2Active();

    // Define goal
    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.frame_id = "map";
    goal_pose.header.stamp = node->now();
    goal_pose.pose.position.x = -2.0;
    goal_pose.pose.position.y = -0.5;
    goal_pose.pose.orientation.w = 1.0;

    // Send goal
    if (!navigator.goToPose(goal_pose)) {
      RCLCPP_ERROR(node->get_logger(), "Failed to send goal.");
      rclcpp::shutdown();
      return 1;
    }

    // Monitor navigation feedback
    int i = 0;
    while (!navigator.isTaskComplete()) {
      auto feedback = navigator.getFeedback<Navigator::NavigateToPose>();
      if (feedback && i++ % 5 == 0) {
        RCLCPP_INFO(
          node->get_logger(), "ETA: %.0f seconds",
          rclcpp::Duration(feedback->estimated_time_remaining).seconds());
        RCLCPP_INFO(
          node->get_logger(), "Distance remaining: %.2f meters", feedback->distance_remaining);

        // Cancel if navigation takes too long (demo)
        //decrease navigation timeout to demo cancellation
        if (rclcpp::Duration(feedback->navigation_time) > rclcpp::Duration(80.0s)) {
          navigator.cancelTask();
          break;
        }

        // Preempt the current goal after 8 seconds (demo)
        if (rclcpp::Duration(feedback->navigation_time) > rclcpp::Duration(8.0s)) {
          RCLCPP_WARN(
            node->get_logger(), "Preempting with new goal at nav_time = %.2f",
            rclcpp::Duration(feedback->navigation_time).seconds());
          navigator.cancelTask();           // Cancel old goal
          goal_pose.pose.position.y = 0.5;  // Adjust target
          navigator.goToPose(goal_pose);    // Send new goal
          // After preemption, restart the monitoring loop
          i = 0;
          continue;
        }
      }
      rclcpp::sleep_for(500ms);
    }

    // Handle result of navigation
    switch (navigator.getTaskResult()) {
      case TaskResult::kSucceeded:
        RCLCPP_INFO(node->get_logger(), "Result :: Goal succeeded!");
        break;
      case TaskResult::kCanceled:
        RCLCPP_WARN(node->get_logger(), "Result ::Goal was canceled!");
        break;
      case TaskResult::kFailed:
        RCLCPP_ERROR(node->get_logger(), "Result ::Goal failed!");
        break;
      default:
        RCLCPP_ERROR(node->get_logger(), "Result ::Goal returned unknown status.");
        break;
    }
  }

  // Shutdown Nav2
  // navigator.lifecycleShutdown();

  rclcpp::shutdown();
  return 0;
}
