
#ifndef NAV2_SIMPLE_COMMANDER_CPP__NAVIGATOR_HPP_
#define NAV2_SIMPLE_COMMANDER_CPP__NAVIGATOR_HPP_

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "geographic_msgs/msg/geo_pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
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
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

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

  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using Path = nav_msgs::msg::Path;
  using GeoPose = geographic_msgs::msg::GeoPose;

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

}  // namespace Nav2SimpleCommander

#endif  // NAV2_SIMPLE_COMMANDER_CPP__NAVIGATOR_HPP_
