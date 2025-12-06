#include <chrono>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_cpp_simple_commander/navigator.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace nav2_simple_commander;
/**
 * @brief Basic stock inspection demo.
 *
 * This demonstration simulates a stock-inspection workflow where the robot
 * follows a predefined sequence of waypoints inside a warehouse or storage area.
 *
 * In a real deployment, the robot would carry cameras, barcode/RFID scanners,
 * or other perception sensors to collect information about shelf inventory,
 * stock quantity, and item locations while navigating through the inspection route.
 *
 * This example:
 *  - Sets an initial pose
 *  - Waits for Nav2 activation
 *  - Sends a list of inspection waypoints using FollowWaypoints
 *  - Prints feedback for the current waypoint being executed
 *  - Returns the robot to the starting pose after completing the route
 */

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("inspection_waypoints_demo");

  // Scope ensures Navigator is destroyed before rclcpp::shutdown()
  {
    Navigator navigator(node);

    // Initial Pose
    geometry_msgs::msg::PoseStamped initial_pose;
    initial_pose.header.frame_id = "map";
    initial_pose.header.stamp = node->now();
    initial_pose.pose.position.x = 4.0;  //in house world
    ;
    initial_pose.pose.position.y = 2.0;
    initial_pose.pose.orientation.z = 1.0;
    initial_pose.pose.orientation.w = 0.0;

    // navigator.setInitialPose(initial_pose);

    // navigator.waitUntilNav2Active();

    // Waypoint Route
    std::vector<std::pair<double, double>> inspection_route = {{0.5, -2.0}, {0.5, 0.0}, {1.0, 4.0},
                                                               {2.0, 4.5},  {5.0, 5.0}, {6.0, 4.0},
                                                               {6.0, 2.0},  {3.0, 2.0}};

    std::vector<geometry_msgs::msg::PoseStamped> inspection_points;

    geometry_msgs::msg::PoseStamped wp;
    wp.header.frame_id = "map";
    wp.pose.orientation.z = 1.0;
    wp.pose.orientation.w = 0.0;

    for (auto & pt : inspection_route) {
      wp.header.stamp = node->now();
      wp.pose.position.x = pt.first;
      wp.pose.position.y = pt.second;
      inspection_points.push_back(wp);
    }

    //  Send Waypoint Task
    navigator.followWaypoints(inspection_points);

    // Monitor Feedback
    int counter = 0;

    // Record start time for timeout-based cancellation
    rclcpp::Time start_time = node->now();

    while (!navigator.isTaskComplete()) {
      auto feedback = navigator.getFeedback<Navigator::FollowWaypoints>();
      if (feedback && counter++ % 5 == 0) {
        RCLCPP_INFO(
          node->get_logger(), "Executing waypoint %u / %zu", feedback->current_waypoint + 1,
          inspection_points.size());
      }

      //not implemented in python exaple
      if ((node->now() - start_time).seconds() > 300.0) {
        RCLCPP_WARN(node->get_logger(), "Inspection timeout — cancelling task...");
        navigator.cancelTask();
        break;
      }
      rclcpp::sleep_for(200ms);
    }
    //ToDO: add per-waypoint success/failure logging 
    // missed waypoints in the followwaypoints ation is not accessible in the current API

    //Check Result
    auto result = navigator.getTaskResult();
    switch (result) {
      case TaskResult::kSucceeded:
        RCLCPP_INFO(node->get_logger(), "Inspection complete! Returning to start...");
        break;
      case TaskResult::kCanceled:
        RCLCPP_WARN(node->get_logger(), "Inspection was canceled. Returning to start...");
        break;
      case TaskResult::kFailed:
        RCLCPP_ERROR(node->get_logger(), "Inspection failed! Returning to start...");
        break;
      default:
        RCLCPP_ERROR(node->get_logger(), "Unknown result status!");
        break;
    }

    // Return to Start
    initial_pose.header.stamp = node->now();
    navigator.goToPose(initial_pose);

    while (!navigator.isTaskComplete()) {
      rclcpp::sleep_for(200ms);
    }
  }

  rclcpp::shutdown();
  return 0;
}
