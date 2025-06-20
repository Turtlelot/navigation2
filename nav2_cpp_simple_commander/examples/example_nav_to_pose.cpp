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


#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_cpp_simple_commander/navigator.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Create a ROS 2 node
  auto node = rclcpp::Node::make_shared("example_nav_to_pose");

  // Create the Navigator instance
  Nav2SimpleCommander::Navigator navigator(node);

  // Create and configure an initial pose message
  geometry_msgs::msg::PoseWithCovarianceStamped init_pose;
  init_pose.header.frame_id = "map";
  init_pose.header.stamp = node->now();
  init_pose.pose.pose.position.x = -2.0;
  init_pose.pose.pose.position.y = 0.0;
  init_pose.pose.pose.orientation.w = 1.0;  


  // Set and publish the initial pose
  RCLCPP_INFO(node->get_logger(), "Waiting 3 seconds before setting initial pose...");
  rclcpp::sleep_for(std::chrono::seconds(3));
  RCLCPP_INFO(node->get_logger(), "Setting initial pose...");
  navigator.setInitialPose(init_pose);

  // Wait for AMCL to confirm reception
  rclcpp::Rate rate(1.0);  // 1 Hz
  int attempt = 0;
  const int max_attempts = 10;

  while (rclcpp::ok() && attempt < max_attempts)
  {
    rclcpp::spin_some(node);
    if (navigator.isInitialPoseReceived())
    {
      RCLCPP_INFO(node->get_logger(), "Initial pose successfully received from AMCL.");
      break;
    }

    RCLCPP_INFO(node->get_logger(), "Waiting for AMCL to acknowledge initial pose...");
    rate.sleep();
    attempt++;
  }

  if (!navigator.isInitialPoseReceived())
  {
    RCLCPP_ERROR(node->get_logger(), "Timed out waiting for initial pose acknowledgment from AMCL.");
  }

  rclcpp::shutdown();
  return 0;
}
