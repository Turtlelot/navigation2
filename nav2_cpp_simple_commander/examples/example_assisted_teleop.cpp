
#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "nav2_cpp_simple_commander/navigator.hpp"

using namespace std::chrono_literals;

// Basic navigation demo to run assisted teleoperation.

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Create a ROS 2 node
  auto node = std::make_shared<rclcpp::Node>("assisted_teleop_demo");
  {
    // Create Navigator (C++ equivalent of BasicNavigator)
    nav2_simple_commander::Navigator navigator(node);

    // ------------------------------------------------------------
    // Set initial pose (same as Python demo)
    // ------------------------------------------------------------
    geometry_msgs::msg::PoseStamped initial_pose;
    initial_pose.header.frame_id = "map";
    initial_pose.header.stamp = node->get_clock()->now();
    initial_pose.pose.position.x = 3.45;
    initial_pose.pose.position.y = 2.15;
    initial_pose.pose.orientation.z = 1.0;
    initial_pose.pose.orientation.w = 0.0;

    // navigator.setInitialPose(initial_pose);

    // ------------------------------------------------------------
    // Wait for Nav2 to become fully active
    // ------------------------------------------------------------
    // navigator.waitUntilNav2Active();

    // ------------------------------------------------------------
    // Run Assisted Teleoperation
    // ------------------------------------------------------------
    navigator.assistedTeleop(20.0);

    // Wait until assisted teleop task is complete
    while (rclcpp::ok() && !navigator.isTaskComplete()) {
      //Publish twist commands to be filtered by the assisted teleop action
      std::this_thread::sleep_for(200ms);

      // Spin to process action feedback / result
      rclcpp::spin_some(node);
    }
  }  // Navigator destroyed here before rclcpp::shutdown()
  // ------------------------------------------------------------
  // Shutdown Nav2 lifecycle nodes
  // ------------------------------------------------------------
  // navigator.lifecycleShutdown();

  rclcpp::shutdown();
  return 0;
}
