#include <chrono>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_cpp_simple_commander/navigator.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace nav2_simple_commander;
/**
 * @brief Basic follow-waypoints navigation demo.
 *
 * This example is a direct C++ version of the `example_waypoint_follower.py` demo from Nav2 Simple Commander. 
 * Basic navigation demo to go to poses.
 * This example :
 *  - Sets an initial localization pose
 *  - Waits for the Nav2 stack to fully activate
 *  - Sends a sequence of waypoints for the robot to follow
 *  - Receives and prints waypoint-execution feedback
 *  - Cancels a long-running navigation task (timeout example)
 *  - Preempts the running waypoint task by sending a new waypoint list
 
 */

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("follow_waypoints_demo");

  {
    Navigator navigator(node);  // Scoped to avoid shutdown crash

    // Set initial pose
    geometry_msgs::msg::PoseStamped initial_pose;
    initial_pose.header.frame_id = "map";
    initial_pose.header.stamp = node->now();
    initial_pose.pose.position.x = -2.0;
    ;
    initial_pose.pose.position.y = 0.0;
    initial_pose.pose.orientation.z = 0.0;
    initial_pose.pose.orientation.w = 1.0;
    // navigator.setInitialPose(initial_pose);

    //(Optional) Startup Nav2 lifecycle manager (if not autostarted)
    // navigator.lifecycleStartup();

    // (Optional) Wait for Nav2 to activate fully
    // navigator.waitUntilNav2Active();
    // If desired, you can change or load the map as well
    // navigator.changeMap("/path/to/map.yaml");

    // You may use the navigator to clear or obtain costmaps
    // navigator.clearAllCostmaps();    // also have clearLocalCostmap() and clearGlobalCostmap()

    // Retrieve costmaps (optional)
    // auto global_costmap = navigator.getGlobalCostmap();
    // auto local_costmap  = navigator.getLocalCostmap();

    // Build waypoint list
    std::vector<geometry_msgs::msg::PoseStamped> waypoints;

    auto make_pose = [&](double x, double y) {
      geometry_msgs::msg::PoseStamped p;
      p.header.frame_id = "map";
      p.header.stamp = node->now();
      p.pose.position.x = x;
      p.pose.position.y = y;
      p.pose.orientation.z = 0.707;
      p.pose.orientation.w = 0.707;
      return p;
    };

    waypoints.push_back(make_pose(0.0, 7.0));
    //additional goals can be appended here
    waypoints.push_back(make_pose(2.5, 4.5));
    waypoints.push_back(make_pose(5.0, 4.5));
    // Sanity check: ensure a valid path exists before executing navigation
    auto maybe_path = navigator.getPath(initial_pose, waypoints[0]);

    if (!maybe_path) {
      RCLCPP_ERROR(node->get_logger(), "No valid path exists between start and goal!");

    } else {
      RCLCPP_INFO(node->get_logger(), "Valid path found with %zu poses", maybe_path->poses.size());
    }
    // Send initial waypoint list
    // navigator.followWaypoints(waypoints);
    // Send goal
    if (!navigator.followWaypoints(waypoints)) {
      RCLCPP_ERROR(node->get_logger(), "Failed to send goal.");
      rclcpp::shutdown();
      return 1;
    }

    rclcpp::Time nav_start = node->now();
    int counter = 0;

    // Monitoring loop with feedback
    while (!navigator.isTaskComplete()) {
      auto feedback = navigator.getFeedback<Navigator::FollowWaypoints>();
      counter++;

      if (feedback && counter % 5 == 0) {
        RCLCPP_INFO(
          node->get_logger(), "Executing waypoint %d / %zu", feedback->current_waypoint + 1,
          waypoints.size());

        auto now = node->now();

        // Timeout demo
        if ((now - nav_start).seconds() > 600.0) {
          RCLCPP_WARN(node->get_logger(), "Timeout reached — cancelling task");
          navigator.cancelTask();
        }

        // Preemption demo (35 seconds)
        if ((now - nav_start).seconds() > 35.0) {
          RCLCPP_WARN(node->get_logger(), "Preempting with new waypoint");

          auto new_pose = make_pose(3.0, 2.0);

          navigator.cancelTask();  // cancel previous goal
          navigator.followWaypoints({new_pose});

          nav_start = now;  // reset timer
          counter = 0;      // reset feedback timer
          continue;
        }
      }

      rclcpp::sleep_for(500ms);
    }
    // TODO: Add per-waypoint diagnostics.
    // Track changes in `current_waypoint` to log which waypoint succeeded,
    // and detect failures if the task stops before reaching the next waypoint.
    // Handle result
    switch (navigator.getTaskResult()) {
      case TaskResult::kSucceeded:
        RCLCPP_INFO(node->get_logger(), "Goal succeeded!");
        break;
      case TaskResult::kCanceled:
        RCLCPP_WARN(node->get_logger(), "Goal was canceled!");
        break;
      case TaskResult::kFailed:
        RCLCPP_ERROR(node->get_logger(), "Goal failed!");
        break;
      default:
        RCLCPP_ERROR(node->get_logger(), "Unknown result status!");
        break;
    }
  }

  // Shutdown Nav2
  // navigator.lifecycleShutdown();
  rclcpp::shutdown();
  return 0;
}
