
#ifndef NAV2_SIMPLE_COMMANDER_CPP__NAVIGATOR_HPP_
#define NAV2_SIMPLE_COMMANDER_CPP__NAVIGATOR_HPP_

/// C++ standard library
#include <chrono>
#include <memory>
#include <string>
#include <vector>

/// Third-party headers
#include <geographic_msgs/msg/geo_pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

/// Project headers
#include "nav2_cpp_simple_commander/action_handle.hpp"
#include "nav2_msgs/action/assisted_teleop.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
// #include "nav2_msgs/action/dock_robot.hpp"
#include "nav2_msgs/action/drive_on_heading.hpp"
// #include "nav2_msgs/action/follow_gps_waypoints.hpp"
// #include "nav2_msgs/action/follow_object.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/smooth_path.hpp"
#include "nav2_msgs/action/spin.hpp"
// #include "nav2_msgs/action/undock_robot.hpp"
// #include "nav2_msgs/srv/clear_costmap_around_pose.hpp"
// #include "nav2_msgs/srv/clear_costmap_except_region.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
// #include "nav2_msgs/srv/toggle.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav2_simple_commander
{

/**
 * @brief Outcome of a navigation Task.
 *
 * This enumeration mirrors the Python BasicNavigator TaskResult semantics and
 * is used by Navigator::getTaskResult() to report the final result of an
 * action-based navigation task.
 */
enum class TaskResult { kUnknown = 0, kSucceeded = 1, kCanceled = 2, kFailed = 3 };

/**
 * @brief Configuration values controlling Navigator timeouts and lifecycle behavior.
 *
 * @details Use this structure to provide timeouts for service and action server
 * waits and to control automatic cancellation of active goals when the
 * Navigator object is destroyed.
 */
struct NavigatorConfig
{
  /**
   * @brief Timeout used when waiting for ROS services to become available.
   *
   * Default: 5 seconds.
   */
  std::chrono::seconds service_timeout{5};

  /**
   * @brief Timeout used when waiting for action servers to become available.
   *
   * Default: 5 seconds.
   */
  std::chrono::seconds action_server_timeout{5};

  /**
   * @brief If true, automatically cancel any active action when the Navigator
   * object is destroyed.
   */
  bool auto_cancel_on_destroy{true};
};

/**
 * @class Navigator
 * @brief Main Navigator class providing "navigation as a library" capability
 *
 * This class provides a C++ interface matching the Python BasicNavigator API.
 * All navigation methods (GoToPose, GoThroughPoses, etc.) are non-blocking.
 * Use IsTaskComplete() to poll for completion.
 *
 * Example usage:
 * @code
 *   auto node = std::make_shared<rclcpp::Node>("my_navigator");
 *   Navigator nav(node);
 *
 *   nav.SetInitialPose(initial_pose);
 *   nav.WaitUntilNav2Active();
 *
 *   if (nav.goToPose(goal_pose)) {
 *     while (!nav.isTaskComplete()) {
 *       auto feedback = nav.getFeedback<NavigateToPose>();
 *       // Process feedback...
 *       rclcpp::spin_some(node);
 *     }
 *
 *     if (nav.GetTaskResult() == TaskResult::kSucceeded) {
 *       // Goal reached!
 *     }
 *   }
 * @endcode
 */
class Navigator
{
public:
  // Action type aliases
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  // using FollowGPSWaypoints = nav2_msgs::action::FollowGPSWaypoints;
  using BackUp = nav2_msgs::action::BackUp;
  using Spin = nav2_msgs::action::Spin;
  using DriveOnHeading = nav2_msgs::action::DriveOnHeading;
  using AssistedTeleop = nav2_msgs::action::AssistedTeleop;
  using FollowPath = nav2_msgs::action::FollowPath;
  using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
  using ComputePathThroughPoses = nav2_msgs::action::ComputePathThroughPoses;
  using SmoothPath = nav2_msgs::action::SmoothPath;
  // using DockRobot = nav2_msgs::action::DockRobot;
  // using UndockRobot = nav2_msgs::action::UndockRobot;
  // using FollowObject = nav2_msgs::action::FollowObject;

  // Service type aliases
  using LoadMap = nav2_msgs::srv::LoadMap;
  using ClearEntireCostmap = nav2_msgs::srv::ClearEntireCostmap;
  // using ClearCostmapAroundPose = nav2_msgs::srv::ClearCostmapAroundPose;
  // using ClearCostmapExceptRegion = nav2_msgs::srv::ClearCostmapExceptRegion;
  using GetCostmap = nav2_msgs::srv::GetCostmap;
  // using Toggle = nav2_msgs::srv::Toggle;

  // Message type aliases
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using Path = nav_msgs::msg::Path;
  using GeoPose = geographic_msgs::msg::GeoPose;
  using Costmap = nav2_msgs::msg::Costmap;

  /**
   * @brief Construct a new Navigator object.
   *
   * @param node Shared pointer to an rclcpp::Node used to create action/service
   * clients, publishers and subscriptions. The node must remain valid for the
   * lifetime of the Navigator instance.
   * @param config Navigator configuration values (timeouts, auto-cancel behavior).
   *
   * @details The Navigator class provides a lightweight C++ wrapper around
   * Nav2 action and service interfaces, exposing a friendlier API similar to
   * the Python BasicNavigator. Methods are non-blocking by default for
   * action-based operations; use the provided helpers to poll for completion
   * and retrieve feedback/results.
   */
  explicit Navigator(rclcpp::Node::SharedPtr node, NavigatorConfig config = NavigatorConfig{});

  /**
   * @brief Destroy the Navigator object.
   *
   * @details If `config.auto_cancel_on_destroy` is true and there is an active
   * action goal, the destructor will attempt to cancel the goal gracefully.
   */
  ~Navigator();

  // Delete copy operations
  Navigator(const Navigator &) = delete;
  Navigator & operator=(const Navigator &) = delete;

  // Allow move operations
  Navigator(Navigator &&) = default;
  Navigator & operator=(Navigator &&) = default;

  // ============================================================================
  // Navigation Action Methods (Non-blocking)
  // ============================================================================

  /**
   * @brief Send a NavigateToPose action goal to move the robot to the given pose.
   *
   * @param pose Target pose in the map frame. The pose must be expressed in a frame
   * known to the navigation stack.
   * @param behavior_tree Optional behavior tree file path or XML string to override
   * the planner/controller behavior for this goal.
   * @return true if the goal was accepted by the action server, false otherwise.
   *
   * @details This method is non-blocking: it will send the action goal and return
   * after the server accepts it. Use `isTaskComplete()` and `getTaskResult()` to
   * monitor completion and outcome. Use `getFeedback<NavigateToPose>()` to access
   * runtime feedback.
   */
  bool goToPose(const PoseStamped & pose, const std::string & behavior_tree = "");

  /**
   * @brief Send a NavigateThroughPoses action goal to visit multiple waypoints.
   *
   * @param poses Vector of poses representing waypoints to visit in order.
   * @param behavior_tree Optional behavior tree to customize execution for this request.
   * @return true if the action server accepted the goal, false otherwise.
   *
   * @details Non-blocking; monitor progress via `getFeedback<NavigateThroughPoses>()`,
   * and completion via `isTaskComplete()` and `getTaskResult()`.
   */
  bool goThroughPoses(
    const std::vector<PoseStamped> & poses, const std::string & behavior_tree = "");

  /**
   * @brief Send a FollowWaypoints action goal to follow a list of poses using
   * the task-executor behavior.
   *
   * @param poses Vector of PoseStamped waypoints in the map frame.
   * @return true if the goal was accepted by the server.
   */
  bool followWaypoints(const std::vector<PoseStamped> & poses);

  /**
   * @brief Follow GPS waypoints (if supported by the navigation stack).
   *
   * @param gps_poses Vector of GeoPose (latitude/longitude) waypoints.
   * @return true if the action/server accepted the request.
   *
   * @note Availability depends on Nav2 components compiled into the running system.
   */
  bool followGpsWaypoints(const std::vector<GeoPose> & gps_poses);

  /**
   * @brief Rotate the robot in place using the Spin action.
   *
   * @param spin_dist Rotation angle in radians (positive is counter-clockwise).
   * @param time_allowance Maximum allowed time in seconds for completing the spin.
   * @return true if the spin goal was accepted by the action server.
   */
  bool spin(double spin_dist = 1.57, double time_allowance = 10.0);

  /**
   * @brief Back the robot up by a specified distance.
   *
   * @param backup_dist Distance in meters to back up.
   * @param backup_speed Linear speed in m/s while backing up.
   * @param time_allowance Maximum time in seconds to allow for the maneuver.
   * @return true if the action goal was accepted.
   */
  bool backup(double backup_dist = 0.15, double backup_speed = 0.025, double time_allowance = 10.0);

  /**
   * @brief Drive forward on a fixed heading for a specified distance.
   *
   * @param dist Distance in meters to travel.
   * @param speed Forward speed in m/s.
   * @param time_allowance Maximum time in seconds for the action to complete.
   * @return true if accepted by the action server.
   */
  bool driveOnHeading(double dist = 0.15, double speed = 0.025, double time_allowance = 10.0);

  /**
   * @brief Run assisted teleoperation for a limited duration.
   *
   * @param time_allowance Maximum duration in seconds for assisted teleop.
   * @return true if the action server accepted the teleop request.
   */
  bool assistedTeleop(double time_allowance = 30.0);

  /**
   * @brief Follow a previously computed path using a controller plugin.
   *
   * @param path Path message containing a sequence of poses.
   * @param controller_id Optional controller plugin id (empty = default).
   * @param goal_checker_id Optional goal checker plugin id (empty = default).
   * @return true when the goal is accepted by the FollowPath action server.
   */
  bool followPath(
    const Path & path, const std::string & controller_id = "",
    const std::string & goal_checker_id = "");

  // ============================================================================
  // Path Planning Methods (Blocking - returns when planning is done)
  // ============================================================================

  /**
   * @brief Compute a path from a start pose to a goal pose.
   *
   * @param start Starting pose of the path.
   * @param goal Goal pose of the path.
   * @param planner_id Optional planner to use for computation.
   * @param use_start If true, use the start pose as the starting point of
   * the path; otherwise, use the robot's current pose.
   *
   * @return The computed path on success, std::nullopt on failure.
   */
  std::optional<Path> getPath(
    const PoseStamped & start, const PoseStamped & goal, const std::string & planner_id = "",
    bool use_start = false);

  /**
   * @brief Compute a path that visits all the poses in the given list.
   *
   * @param start Starting pose of the path.
   * @param goals List of goal poses to visit.
   * @param planner_id Optional planner to use for computation.
   * @param use_start If true, use the start pose as the starting point of
   * the path; otherwise, use the robot's current pose.
   *
   * @return The computed path on success, std::nullopt on failure.
   */
  std::optional<Path> getPathThroughPoses(
    const PoseStamped & start, const std::vector<PoseStamped> & goals,
    const std::string & planner_id = "", bool use_start = false);

  /**
   * @brief Smooth an existing path.
   *
   * @param path Path to smooth
   * @param smoother_id Optional smoother to use
   * @param max_duration Maximum smoothing time [s]
   * @param check_for_collision Enable collision checking
   *
   * @return Smoothed path on success, std::nullopt on failure
   */
  std::optional<Path> smoothPath(
    const Path & path, const std::string & smoother_id = "", double max_duration = 2.0,
    bool check_for_collision = false);

  // ============================================================================
  // Task Management
  // ============================================================================

  /**
   * @brief Cancel the currently active navigation task (if any).
   *
   * @details If a goal is active this will attempt to cancel it via the
   * action client. Cancellation is best-effort and callers should check
   * `isTaskComplete()` and `getTaskResult()` to confirm the final state.
   */
  void cancelTask();

  /**
   * @brief Check whether the current task is complete.
   *
   * @return true if no task is running or the active task has finished
   * (succeeded, canceled or aborted).
   */
  bool isTaskComplete();

  /**
   * @brief Get a high-level TaskResult for the completed task.
   *
   * @return TaskResult indicating success, cancellation, failure, or unknown.
   */
  TaskResult getTaskResult();

  /**
   * @brief Get type-safe feedback for the active action.
   *
   * @tparam ActionT The action type (e.g., NavigateToPose).
   * @return Shared pointer to the latest feedback message, or nullptr if no
   * feedback is available or the current action type does not match ActionT.
   */
  template <typename ActionT>
  std::shared_ptr<const typename ActionT::Feedback> getFeedback()
  {
    auto handle = std::dynamic_pointer_cast<ActionHandle<ActionT>>(action_handle_);
    if (!handle) {
      RCLCPP_WARN(node_->get_logger(), "No active action handle or type mismatch");
      return nullptr;
    }
    return handle->getFeedback();
  }

  /**
   * @brief Get the typed result for a completed action.
   *
   * @tparam ActionT The action type for which to obtain the result.
   * @return Shared pointer to the action Result message, or nullptr on failure.
   *
   * @details This method will block until the action result is available.
   */
  template <typename ActionT>
  std::shared_ptr<const typename ActionT::Result> getResult()
  {
    auto handle = std::dynamic_pointer_cast<ActionHandle<ActionT>>(action_handle_);
    if (!handle) {
      RCLCPP_WARN(node_->get_logger(), "No active action handle or type mismatch");
      return nullptr;
    }
    return handle->getResult(node_);
  }

  // ============================================================================
  // Localization
  // ============================================================================

  /**
   * @brief Publish the initial pose estimate for the localization stack.
   *
   * @param initial_pose PoseWithCovarianceStamped containing pose and covariance.
   *
   * @details This method republishes the provided initial pose on the `initialpose`
   * topic so that AMCL or other localization nodes can initialize accordingly.
   */
  void setInitialPose(const PoseWithCovarianceStamped & initial_pose);

  // ============================================================================
  // Lifecycle Management
  // ============================================================================

  /**
   * @brief Block until Nav2 lifecycle nodes are active.
   *
   * @param navigator Name of the navigator lifecycle node (default: "bt_navigator").
   * @param localizer Name of the localization lifecycle node (default: "amcl").
   *
   * @details This helper waits for specified lifecycle nodes to reach the `active`
   * state (using the GetState lifecycle service). When `localizer == "amcl'`, the
   * Navigator will also wait for an initial pose to be acknowledged.
   */
  void waitUntilNav2Active(
    const std::string & navigator = "bt_navigator", const std::string & localizer = "amcl");

  /**
   * @brief Request startup (bring up) lifecycle-managed Nav2 nodes via ManageLifecycleNodes
   * services discovered on the ROS graph.
   *
   * @details The method scans available services for ManageLifecycleNodes and sends
   * a STARTUP command to them. This is useful when launching Nav2 components in a
   * lifecycle-managed configuration.
   */
  void lifecycleStartup();

  /**
   * @brief Request shutdown (bring down) lifecycle-managed Nav2 nodes via
   * ManageLifecycleNodes services discovered on the ROS graph.
   */
  void lifecycleShutdown();

  // ============================================================================
  // Costmap Management
  // ============================================================================

  /**
   * @brief Load a new map and switch the navigation stack to use it.
   *
   * @param map_filepath Filesystem path to the map YAML file (as used by nav2_map_server).
   * @return true when the request was accepted and map load succeeded, false otherwise.
   *
   * @details This method calls the `LoadMap` service provided by the map server
   * and returns 
   */
  bool changeMap(const std::string & map_filepath);

  /**
   * @brief Clear both global and local costmaps completely.
   *
   * @details Uses the `ClearEntireCostmap` service for both local and global costmaps.
   */
  void clearAllCostmaps();

  /**
   * @brief Clear only the local costmap.
   */
  void clearLocalCostmap();

  /**
   * @brief Clear only the global costmap.
   */
  void clearGlobalCostmap();

  /**
   * @brief Clear a circular region around a pose in the local costmap.
   *
   * @param pose Center pose in the map frame used for clearing.
   * @param distance Radius in meters to clear around the pose.
   */
  void clearLocalCostmapAroundPose(const PoseStamped & pose, double distance);

  /**
   * @brief Clear a circular region around a pose in the global costmap.
   *
   * @param pose Center pose in the map frame used for clearing.
   * @param distance Radius in meters to clear around the pose.
   */
  void clearGlobalCostmapAroundPose(const PoseStamped & pose, double distance);

  /**
   * @brief Clear the costmap except for a circular region centered on the robot.
   *
   * @param distance Radius in meters to preserve around the robot; the rest is cleared.
   */
  void clearCostmapExceptRegion(double distance);

  /**
   * @brief Retrieve the current global costmap from the nav2 costmap service.
   *
   * @return Optional Costmap message; std::nullopt on failure.
   */
  std::optional<Costmap> getGlobalCostmap();

  /**
   * @brief Retrieve the current local costmap from the nav2 costmap service.
   *
   * @return Optional Costmap message; std::nullopt on failure.
   */
  std::optional<Costmap> getLocalCostmap();

  /**
   * @brief Enable or disable the collision monitor.
   *
   * @param enable true to enable collision monitoring, false to disable it.
   *
   * @details This toggles the collision monitor component if available in the
   * running Nav2 system by calling the appropriate Toggle service.
   */
  void toggleCollisionMonitor(bool enable);

  // ============================================================================
  // Object Following
  // ============================================================================

  /**
   * @brief Follow a moving object by subscribing to a pose topic and issuing a
   * FollowObject action goal.
   *
   * @param topic ROS topic name publishing geometry_msgs::msg::PoseStamped for the target.
   * @param max_duration Maximum duration in seconds to follow the object (0 = unlimited).
   * @return true if the follow goal was accepted by the action server.
   */
  bool followObjectByTopic(const std::string & topic, double max_duration = 0.0);

  /**
   * @brief Follow a moving object by tracking a TF frame and issuing a
   * FollowObject action goal.
   *
   * @param frame TF frame of the object to follow (e.g., a fiducial or tracking frame).
   * @param max_duration Maximum duration in seconds to follow the object (0 = unlimited).
   * @return true if the follow goal was accepted by the action server.
   */
  bool followObjectByFrame(const std::string & frame, double max_duration = 0.0);

  // ============================================================================
  // Docking Management
  // ============================================================================

  /**
   * @brief Dock the robot by sending a DockRobot action goal using an explicit pose.
   *
   * @param dock_pose PoseStamped describing the docking location in the map frame.
   * @param dock_type Optional dock type or identifier to select a specific docking behavior.
   * @return true if the docking goal was accepted by the action server.
   */
  bool dockRobotByPose(const PoseStamped & dock_pose, const std::string & dock_type);

  /**
   * @brief Dock the robot by specifying a dock identifier (server-side lookup).
   *
   * @param dock_id Identifier used by the docking action/server to select the dock.
   * @return true if the docking goal was accepted by the action server.
   */
  bool dockRobotById(const std::string & dock_id);

  /**
   * @brief Request an undock action via the UndockRobot action server.
   *
   * @param dock_type Optional dock type used by some docking servers to control
   * undocking semantics.
   * @return true if the undock goal was accepted by the action server.
   */
  bool undockRobot(const std::string & dock_type = "");

private:
  // ============================================================================
  // Private Helper Methods
  // ============================================================================

  void publishInitialPose();
  void amclPoseCallback(const PoseWithCovarianceStamped::SharedPtr msg);
  void waitForInitialPose();
  void waitForNodeToActivate(const std::string & node_name);

  /**
   * @brief Send an action goal and manage the action client
   *
   * @param goal The goal message to send to the action server
   * @param action_name Name of the action server to send the goal to
   * @param client_ref Optional shared pointer to the action client. If not provided, a new client will be created.
   * @return true if the action goal was accepted by the action server.
   */
  template <typename ActionT>
  bool sendActionGoal(
    const typename ActionT::Goal & goal, const std::string & action_name,
    typename rclcpp_action::Client<ActionT>::SharedPtr & client_ref)
  {
    // Create a new action client if not provided
    if (!client_ref) {
      client_ref = rclcpp_action::create_client<ActionT>(node_, action_name);
    }

    // Create a new action handle using the client
    auto handle = std::make_shared<ActionHandle<ActionT>>(client_ref);

    // Send the goal to the action server
    auto status = handle->sendGoal(node_, goal);

    // Check if the goal was accepted by the action server
    if (status != GoalStatus::kAccepted) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to send %s goal", action_name.c_str());
      return false;
    }

    // Store the action handle for future use
    action_handle_ = handle;

    return true;
  }

  /**
   * @brief Call a ROS service and return the response.
   *
   * @param service_name Name of the ROS service to call.
   * @param request Request object to send to the service.
   * @param timeout Optional timeout for waiting for the service to become available.
   * @return Shared pointer to the service response, or nullptr on failure.
   */
  template <typename ServiceT>
  typename ServiceT::Response::SharedPtr callService(
    const std::string & service_name, typename ServiceT::Request::SharedPtr request,
    std::chrono::seconds timeout = std::chrono::seconds(5))
  {
    auto client = node_->create_client<ServiceT>(service_name);

    if (!client->wait_for_service(timeout)) {
      RCLCPP_ERROR(node_->get_logger(), "Service '%s' not available", service_name.c_str());
      return nullptr;
    }

    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS) {
      return future.get();
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Failed to call service '%s'", service_name.c_str());
      return nullptr;
    }
  }

  // ============================================================================
  // Member Variables
  // ============================================================================

  rclcpp::Node::SharedPtr node_;
  NavigatorConfig config_;

  // Initial pose management
  PoseWithCovarianceStamped initial_pose_;
  bool initial_pose_received_{false};

  // Publishers and subscribers
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;

  // Action clients (cached)
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr nav_through_poses_client_;
  rclcpp_action::Client<FollowWaypoints>::SharedPtr follow_waypoints_client_;
  // rclcpp_action::Client<FollowGPSWaypoints>::SharedPtr follow_gps_waypoints_client_;
  rclcpp_action::Client<FollowPath>::SharedPtr follow_path_client_;
  rclcpp_action::Client<ComputePathToPose>::SharedPtr compute_path_to_pose_client_;
  rclcpp_action::Client<ComputePathThroughPoses>::SharedPtr compute_path_through_poses_client_;
  rclcpp_action::Client<SmoothPath>::SharedPtr smooth_path_client_;
  rclcpp_action::Client<Spin>::SharedPtr spin_client_;
  rclcpp_action::Client<BackUp>::SharedPtr backup_client_;
  rclcpp_action::Client<DriveOnHeading>::SharedPtr drive_on_heading_client_;
  rclcpp_action::Client<AssistedTeleop>::SharedPtr assisted_teleop_client_;
  // rclcpp_action::Client<FollowObject>::SharedPtr follow_object_client_;
  // rclcpp_action::Client<DockRobot>::SharedPtr dock_robot_client_;
  // rclcpp_action::Client<UndockRobot>::SharedPtr undock_robot_client_;

  std::shared_ptr<ActionHandleBase> action_handle_;
};

}  // namespace nav2_simple_commander

#endif  // NAV2_SIMPLE_COMMANDER_CPP__NAVIGATOR_HPP_
