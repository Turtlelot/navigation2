#include <algorithm>
#include <chrono>
#include <memory>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_cpp_simple_commander/navigator.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace nav2_simple_commander;

/**
 * @brief Basic security route patrol demo. In this demonstration, the expectation
is that there are security cameras mounted on the robots recording or being
watched live by security staff.
 *
 * This demo simulates a security patrol workflow:
 *  - Define a fixed patrol route (list of XY waypoints)
 *  - Set initial pose
 *  - Wait for Nav2 activation
 *  - Continuously patrol the route using NavigateThroughPoses (goThroughPoses)
 *  - Print ETA feedback periodically
 *  - Cancel if navigation time exceeds a timeout (stuck / failure mode)
 *  - Reverse the route at the end to patrol back the other direction
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("security_demo");

  {
    Navigator navigator(node);

    // ------------------------------------------------------------
    // Security route patrol

    // Security route, probably read in from a file for a real application
    // from either a map or drive and repeat.
    // ------------------------------------------------------------
    std::vector<std::pair<double, double>> security_route = {
      {3.0, 2.0}, {3.0, 5.0}, {5.0, 5.0}, {5.0, 2.0},
    };

    // ------------------------------------------------------------
    // Set initial pose
    // ------------------------------------------------------------
    geometry_msgs::msg::PoseStamped initial_pose;
    initial_pose.header.frame_id = "map";
    initial_pose.header.stamp = node->now();
    initial_pose.pose.position.x = 4.0;  //in house world
    initial_pose.pose.position.y = 2.15;
    initial_pose.pose.orientation.z = 1.0;
    initial_pose.pose.orientation.w = 0.0;
    // navigator.setInitialPose(initial_pose);

    // ------------------------------------------------------------
    // Wait for Nav2 to become active
    // ------------------------------------------------------------
    // navigator.waitUntilNav2Active();

    // ------------------------------------------------------------
    // Do security route(patrolling) until shutdown
    // ------------------------------------------------------------
    while (rclcpp::ok()) {
      // Build route poses from waypoints
      std::vector<geometry_msgs::msg::PoseStamped> route_poses;
      route_poses.reserve(security_route.size());

      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.orientation.w = 1.0;

      for (const auto & pt : security_route) {
        pose.header.stamp = node->now();
        pose.pose.position.x = pt.first;
        pose.pose.position.y = pt.second;
        route_poses.push_back(pose);
      }

      // Send route (NavigateThroughPoses)
      if (!navigator.goThroughPoses(route_poses)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to send goThroughPoses goal.");
        break;
      }
      // Do something during our route (e.x. AI detection on camera images for anomalies)

      // Print ETA feedback periodically + timeout cancel
      int i = 0;
      while (rclcpp::ok() && !navigator.isTaskComplete()) {
        auto feedback = navigator.getFeedback<Navigator::NavigateThroughPoses>();
        if (feedback && (++i % 5 == 0)) {
          const double eta_s = rclcpp::Duration(feedback->estimated_time_remaining).seconds();
          RCLCPP_INFO(
            node->get_logger(), "Estimated time to complete current route: %.0f seconds.", eta_s);

          // Some failure mode: cancel if robot clearly stuck (navigation_time > 180s)
          if (rclcpp::Duration(feedback->navigation_time) > rclcpp::Duration(180s)) {
            RCLCPP_WARN(
              node->get_logger(), "Navigation has exceeded timeout of 180s, canceling request.");
            navigator.cancelTask();
          }
        }

        rclcpp::sleep_for(200ms);
      }

      // If at end of route, reverse the route to restart

      std::reverse(security_route.begin(), security_route.end());

      // Handle result
      switch (navigator.getTaskResult()) {
        case TaskResult::kSucceeded:
          RCLCPP_INFO(node->get_logger(), "Route complete! Restarting...");
          break;
        case TaskResult::kCanceled:
          RCLCPP_ERROR(node->get_logger(), "Security route was canceled, exiting.");
          rclcpp::shutdown();
          return 1;
        case TaskResult::kFailed:
          RCLCPP_WARN(node->get_logger(), "Security route failed! Restarting from other side...");
          break;
        default:
          RCLCPP_WARN(node->get_logger(), "Unknown result status! Restarting...");
          break;
      }
    }

    // navigator.lifecycleShutdown();
  }

  rclcpp::shutdown();
  return 0;
}
