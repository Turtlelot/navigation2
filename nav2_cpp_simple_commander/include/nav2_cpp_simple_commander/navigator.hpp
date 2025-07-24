
#ifndef NAV2_SIMPLE_COMMANDER_CPP__NAVIGATOR_HPP_
#define NAV2_SIMPLE_COMMANDER_CPP__NAVIGATOR_HPP_

#include <any>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "geographic_msgs/msg/geo_pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_cpp_simple_commander/action_handle.hpp"
#include "nav2_msgs/action/assisted_teleop.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/drive_on_heading.hpp"
#include "nav2_msgs/action/follow_gps_waypoints.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/smooth_path.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace Nav2SimpleCommander
{
class Navigator
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using FollowGPSWaypoints = nav2_msgs::action::FollowGPSWaypoints;
  using BackUp = nav2_msgs::action::BackUp;
  using Spin = nav2_msgs::action::Spin;
  using DriveOnHeading = nav2_msgs::action::DriveOnHeading;
  using AssistedTeleop = nav2_msgs::action::AssistedTeleop;
  using FollowPath = nav2_msgs::action::FollowPath;
  using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
  using ComputePathThroughPoses = nav2_msgs::action::ComputePathThroughPoses;
  using SmoothPath = nav2_msgs::action::SmoothPath;

  using LoadMap = nav2_msgs::srv::LoadMap;
  using ClearEntireCostmap = nav2_msgs::srv::ClearEntireCostmap;
  using GetCostmap = nav2_msgs::srv::GetCostmap;

  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using Path = nav_msgs::msg::Path;
  using GeoPose = geographic_msgs::msg::GeoPose;
  using Costmap = nav2_msgs::msg::Costmap;

  explicit Navigator(rclcpp::Node::SharedPtr node);

  /// Send a NavigateToPose goal. Returns true on success.
  bool goToPose(const PoseStamped & pose, const std::string & behavior_tree = "");

  /// Send a NavigateThroughPoses goal. Returns true on success.
  bool goThroughPoses(
    const std::vector<PoseStamped> & poses, const std::string & behavior_tree = "");

  /// Send a FollowWaypoints goal. Returns true on success.
  bool followWaypoints(const std::vector<PoseStamped> & poses);

  /// Send a FollowGpsWaypoints goal. Returns true on success.
  bool followGpsWaypoints(const std::vector<GeoPose> & poses);

  /// Spin the robot for a given distance. Returns true on success.
  bool spin(double spin_dist = 1.57, double time_allowance = 10.0);

  /// Backup the robot for a given distance. Returns true on success.
  bool backup(double backup_dist = 0.15, double backup_speed = 0.025, double time_allowance = 10.0);

  /// Drive the robot on a heading for a given distance. Returns true on success.
  bool driveOnHeading(double dist = 0.15, double speed = 0.025, double time_allowance = 10.0);

  /// Perform assisted teleoperation. Returns true on success.
  bool assistedTeleop(double time_allowance = 30.0);

  /// Follow a path. Returns true on success.
  bool followPath(
    const Path & path, const std::string & controller_id = "",
    const std::string & goal_checker_id = "");

  /// Get a path from the planner. Returns true on success.
  bool getPath(
    const PoseStamped & start, const PoseStamped & end, const std::string & planner_id = "",
    bool use_start = false);

  /// Get a path through poses from the planner. Returns true on success.
  bool getPathThroughPoses(
    const PoseStamped & start, const std::vector<PoseStamped> goals,
    const std::string & planner_id = "", bool use_start = false);

  /// Smooth a path. Returns true on success.
  bool smoothPath(
    const Path & path, const std::string & smoother_id = "", double max_duration = 2.0,
    bool check_for_collision = false);

  // Set the initial pose and publish it to the localization system
  void setInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose);

  // Waits until the navigation system is fully active.
  // Blocks until the specified localizer and navigator nodes are active.
  // If the localizer is "amcl", it waits for initial pose to be received.
  void waitUntilNav2Active(
    const std::string & navigator = "bt_navigator", const std::string & localizer = "amcl");

  //Startup nav2 lifecycle system.
  void lifecycleStartup();

  // Shutdown nav2 lifecycle-managed nodes
  void lifecycleShutdown();

  bool isTaskComplete();

  // Cancel the currently active task (goal)
  void cancelTask();

  // Get the latest feedback for the currently running action
  template <typename ActionT>
  std::shared_ptr<const typename ActionT::Feedback> getFeedback();

  // // Get the result for the currently running action
  template <typename ActionT>
  typename rclcpp_action::ClientGoalHandle<ActionT>::Result::SharedPtr getResult();

  enum class TaskResult { UNKNOWN = 0, SUCCEEDED = 1, CANCELED = 2, FAILED = 3 };
  TaskResult getTaskResult();

  /// Load a map from a file.
  void changeMap(const std::string & map_filepath);

  /// Clear the entire costmap.
  void clearallCostmaps();

  /// Clear the local costmap.
  void clearLocalCostmap();

  /// Clear the global costmap.
  void clearGlobalCostmap();

  /// Get local costmap.
  Costmap getLocalCostmap();

  /// Get global costmap.
  Costmap getGlobalCostmap();

private:
  rclcpp::Node::SharedPtr node_;

  // Latest initial pose to publish
  geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_;

  // Flag to indicate if initial pose was received by AMCL
  bool initial_pose_received_ = false;

  // Publisher to send initial pose
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;

  // Subscriber to listen for pose feedback from AMCL
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;

  // Last time feedback was processed
  rclcpp::Time last_feedback_time_;

  // Handle to the currently running action
  std::shared_ptr<IActionHandle> action_handle_;

  // Helper to publish the stored initial pose
  void publishInitialPose();

  // Callback for receiving pose from AMCL
  // Called when AMCL publishes its pose -> confirms that it accepted our initial pose
  void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  // Waits until AMCL receives and processes the initial pose
  void waitForInitialPose();

  // Wait until a lifecycle node becomes active
  void waitForNodeToActivate(const std::string & node_name);

  /// The single templated runner: waits for server, sends goal, spins for
  /// result.
  template <typename ActionT>
  bool runAction(
    const std::string & action_name, const typename ActionT::Goal & goal,
    std::function<void(const std::shared_ptr<const typename ActionT::Feedback>)> feedback_cb =
      nullptr);
};

void publishInitialPose();

// -----------------------------
// Templated implementations
// -----------------------------

template <typename ActionT>
std::shared_ptr<const typename ActionT::Feedback> Navigator::getFeedback()
{
  if (!action_handle_) {
    RCLCPP_WARN(node_->get_logger(), "No active action handle to get feedback.");
    return nullptr;
  }

  auto handle = std::dynamic_pointer_cast<ActionHandleImpl<ActionT>>(action_handle_);
  if (!handle) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to cast action handle to expected type.");
    return nullptr;
  }

  return handle->getFeedback();
}

template <typename ActionT>
typename rclcpp_action::ClientGoalHandle<ActionT>::Result::SharedPtr Navigator::getResult()
{
  if (!action_handle_) {
    RCLCPP_WARN(node_->get_logger(), "No active action handle to get result.");
    return nullptr;
  }

  auto handle = std::dynamic_pointer_cast<ActionHandleImpl<ActionT>>(action_handle_);
  if (!handle) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to cast action handle to expected type.");
    return nullptr;
  }

  return handle->getResult(node_);
}

}  // namespace Nav2SimpleCommander

#endif  // NAV2_SIMPLE_COMMANDER_CPP__NAVIGATOR_HPP_
