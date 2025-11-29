#include <chrono>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_cpp_simple_commander/navigator.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace nav2_simple_commander;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("go_through_poses_demo");
  // Scope ensures Navigator is destroyed BEFORE rclcpp::shutdown().
  // Destroy Navigator before rclcpp::shutdown() to avoid ROS2 context errors.
  {
    Navigator navigator(node);

    // Set initial pose (optional for this demo)
    geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
    initial_pose.header.frame_id = "map";
    initial_pose.header.stamp = node->now();
    initial_pose.pose.pose.position.x = 3.45;
    initial_pose.pose.pose.position.y = 2.15;
    initial_pose.pose.pose.orientation.z = 1.0;
    initial_pose.pose.pose.orientation.w = 0.0;
    //   navigator.setInitialPose(initial_pose);

    // Wait until navigation is active
    //   navigator.waitUntilNav2Active();

    // Define goal poses for NavigateThroughPoses
    std::vector<geometry_msgs::msg::PoseStamped> goal_poses;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";

    pose.pose.position.x = 0;
    pose.pose.position.y = -1.5;
    pose.header.stamp = node->now();
    pose.pose.orientation.w = 0.707;
    pose.pose.orientation.z = 0.707;
    goal_poses.push_back(pose);

    pose.pose.position.x = -1;
    pose.pose.position.y = -2;
    pose.header.stamp = node->now();
    goal_poses.push_back(pose);

    pose.pose.position.x = -2;
    pose.pose.position.y = 0;
    pose.header.stamp = node->now();
    goal_poses.push_back(pose);

    //Send multi-pose navigation request
    navigator.goThroughPoses(goal_poses);

    int i = 0;
    while (!navigator.isTaskComplete()) {
      auto feedback = navigator.getFeedback<Navigator::NavigateThroughPoses>();
      if (feedback && i++ % 5 == 0) {
        RCLCPP_INFO(
          node->get_logger(), "ETA: %.0f seconds",
          rclcpp::Duration(feedback->estimated_time_remaining).seconds());

        RCLCPP_INFO(
          node->get_logger(), "Number of poses remaining: %u", feedback->number_of_poses_remaining);

        // Timeout safeguard (demo)
        if (rclcpp::Duration(feedback->navigation_time) > rclcpp::Duration(500.0s)) {
          navigator.cancelTask();
        }
        //Preemption demo: replace current task with a new one

        if (rclcpp::Duration(feedback->navigation_time) > rclcpp::Duration(35.0s)) {
          geometry_msgs::msg::PoseStamped new_goal;
          new_goal.header.frame_id = "map";
          new_goal.header.stamp = node->now();
          new_goal.pose.position.x = -5.0;
          new_goal.pose.position.y = -4.75;
          new_goal.pose.orientation.w = 0.707;
          new_goal.pose.orientation.z = 0.707;
          navigator.cancelTask();                //   Cancel current goal
          navigator.goThroughPoses({new_goal});  //  sSend new navigation goal

          // After preemption, restart the monitoring loop cleanly
          i = 0;
          continue;
        }
      }
      rclcpp::sleep_for(500ms);
    }

    // Evaluate result
    // Result handling
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
  //   navigator.lifecycleShutdown();
  rclcpp::shutdown();
  return 0;
}
