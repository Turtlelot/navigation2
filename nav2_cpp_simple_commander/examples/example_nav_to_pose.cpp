#include <chrono>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
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
  initial_pose.pose.pose.position.x = 3.45;
  initial_pose.pose.pose.position.y = 2.15;
  initial_pose.pose.pose.orientation.z = 1.0;
  initial_pose.pose.pose.orientation.w = 0.0;
  // navigator.setInitialPose(initial_pose);

  // Startup lifecycle (if not autostarted)
  // navigator.lifecycleStartup();

  // Wait for Nav2 to become active
  // navigator.waitUntilNav2Active();

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

  // Monitor task
  int i = 0;
  while (!navigator.isTaskComplete()) {
    auto feedback = navigator.getFeedback<Navigator::NavigateToPose>();
    if (feedback && i++ % 5 == 0) {
      RCLCPP_INFO(
        node->get_logger(), "ETA: %.0f seconds",
        rclcpp::Duration(feedback->estimated_time_remaining).seconds());

      // Cancel if taking too long
      //Some navigation timeout to demo cancellation
      if (rclcpp::Duration(feedback->navigation_time) > rclcpp::Duration(20.0s)) {
        navigator.cancelTask();
        break;
      }

      // Preempt after 18 seconds
      // Some navigation request change to demo preemption
      //it enters here very LATE
      if (rclcpp::Duration(feedback->navigation_time) > rclcpp::Duration(18.0s)) {
        RCLCPP_WARN(
          node->get_logger(), "Preempting with new goal at nav_time = %.2f",
          rclcpp::Duration(feedback->navigation_time).seconds());
        goal_pose.pose.position.y = 0.5;
        navigator.goToPose(goal_pose);
      }
    }
    rclcpp::sleep_for(500ms);
  }

  // Result handling
  switch (navigator.getTaskResult()) {
    case Navigator::TaskResult::SUCCEEDED:
      RCLCPP_INFO(node->get_logger(), "Result :: Goal succeeded!");
      break;
    case Navigator::TaskResult::CANCELED:
      RCLCPP_WARN(node->get_logger(), "Result ::Goal was canceled!");
      break;
    case Navigator::TaskResult::FAILED:
      RCLCPP_ERROR(node->get_logger(), "Result ::Goal failed!");
      break;
    default:
      RCLCPP_ERROR(node->get_logger(), "Result ::Goal returned unknown status.");
      break;
  }

  // Shutdown Nav2
  // navigator.lifecycleShutdown();

  rclcpp::shutdown();
  return 0;
}
