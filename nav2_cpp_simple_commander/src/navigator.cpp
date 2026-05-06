#include "nav2_cpp_simple_commander/navigator.hpp"

namespace nav2_simple_commander
{

Navigator::Navigator(rclcpp::Node::SharedPtr node, NavigatorConfig config)
: node_(std::move(node)), config_(config)
{
  // Create publisher to /initialpose
  initial_pose_pub_ =
    node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);

  // Create subscriber to /amcl_pose to monitor when localization acknowledges pose
  amcl_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "amcl_pose", 10, std::bind(&Navigator::amclPoseCallback, this, std::placeholders::_1));
}

Navigator::~Navigator()
{
  // Cancel any active goal if configured to do so
  if (config_.auto_cancel_on_destroy && action_handle_) {
    // Only cancel if the goal is still executing
    if (!action_handle_->isComplete(node_)) {
      action_handle_->cancel(node_);
    }
  }
}

bool Navigator::goToPose(
  const geometry_msgs::msg::PoseStamped & pose, const std::string & behavior_tree)
{
  NavigateToPose::Goal goal;
  goal.pose = pose;
  goal.behavior_tree = behavior_tree;
  return sendActionGoal<NavigateToPose>(goal, nsAction("navigate_to_pose"), nav_to_pose_client_);
}

bool Navigator::goThroughPoses(
  const std::vector<PoseStamped> & poses, const std::string & behavior_tree)
{
  NavigateThroughPoses::Goal goal;
  goal.poses = poses;
  goal.behavior_tree = behavior_tree;
  return sendActionGoal<NavigateThroughPoses>(
    goal, nsAction("navigate_through_poses"), nav_through_poses_client_);
}

bool Navigator::followWaypoints(const std::vector<geometry_msgs::msg::PoseStamped> & poses)
{
  FollowWaypoints::Goal goal;
  goal.poses = poses;

  return sendActionGoal<FollowWaypoints>(goal, nsAction("follow_waypoints"), follow_waypoints_client_);
}

// bool Navigator::followGpsWaypoints(const std::vector<GeoPose> & poses)
// {
//   FollowGPSWaypoints::Goal goal;
//   goal.gps_poses = poses;
//   return sendActionGoal<FollowGPSWaypoints>(goal, "follow_gps_waypoints", follow_gps_waypoints_client_);
// }

bool Navigator::spin(double spin_dist, double time_allowance)
{
  Spin::Goal goal;
  goal.target_yaw = spin_dist;
  goal.time_allowance = rclcpp::Duration::from_seconds(time_allowance);

  return sendActionGoal<Spin>(goal, nsAction("spin"), spin_client_);
}

bool Navigator::backup(double backup_dist, double backup_speed, double time_allowance)
{
  BackUp::Goal goal;

  geometry_msgs::msg::Point target;
  target.x = backup_dist;
  goal.target = target;
  goal.speed = backup_speed;
  goal.time_allowance = rclcpp::Duration::from_seconds(time_allowance);

  return sendActionGoal<BackUp>(goal, nsAction("back_up"), backup_client_);
}

bool Navigator::driveOnHeading(double dist, double speed, double time_allowance)
{
  DriveOnHeading::Goal goal;
  geometry_msgs::msg::Point target;
  target.x = dist;
  goal.target = target;
  goal.speed = speed;
  goal.time_allowance = rclcpp::Duration::from_seconds(time_allowance);

  return sendActionGoal<DriveOnHeading>(goal, nsAction("drive_on_heading"), drive_on_heading_client_);
}

bool Navigator::assistedTeleop(double time_allowance)
{
  AssistedTeleop::Goal goal;
  goal.time_allowance = rclcpp::Duration::from_seconds(time_allowance);

  return sendActionGoal<AssistedTeleop>(goal, nsAction("assisted_teleop"), assisted_teleop_client_);
}

bool Navigator::followPath(
  const Path & path, const std::string & controller_id, const std::string & goal_checker_id)
{
  FollowPath::Goal goal;
  goal.path = path;
  goal.controller_id = controller_id;
  goal.goal_checker_id = goal_checker_id;

  return sendActionGoal<FollowPath>(goal, nsAction("follow_path"), follow_path_client_);
}

std::optional<Navigator::Path> Navigator::getPath(
  const PoseStamped & start, const PoseStamped & end, const std::string & planner_id,
  bool use_start)
{
  ComputePathToPose::Goal goal;
  goal.start = start;
  goal.goal = end;
  goal.planner_id = planner_id;
  goal.use_start = use_start;

  if (!sendActionGoal<ComputePathToPose>(
        goal, nsAction("compute_path_to_pose"), compute_path_to_pose_client_)) {
    return std::nullopt;
  }

  // Wait for result via typed accessor
  auto result = getResult<ComputePathToPose>();
  if (result) {
    return result->path;
  }
  return std::nullopt;
}

std::optional<Navigator::Path> Navigator::getPathThroughPoses(
  const PoseStamped & start, const std::vector<PoseStamped> & goals, const std::string & planner_id,
  bool use_start)
{
  ComputePathThroughPoses::Goal goal;
  goal.start = start;
  goal.goals = goals;
  goal.planner_id = planner_id;
  goal.use_start = use_start;

  if (!sendActionGoal<ComputePathThroughPoses>(
        goal, nsAction("compute_path_through_poses"), compute_path_through_poses_client_)) {
    return std::nullopt;
  }

  // Wait for result via typed accessor
  auto result = getResult<ComputePathThroughPoses>();
  if (result) {
    return result->path;
  }
  return std::nullopt;
}

std::optional<Navigator::Path> Navigator::smoothPath(
  const Path & path, const std::string & smoother_id, double max_duration, bool check_for_collision)
{
  SmoothPath::Goal goal;
  goal.path = path;
  goal.smoother_id = smoother_id;
  goal.max_smoothing_duration = rclcpp::Duration::from_seconds(max_duration);
  goal.check_for_collisions = check_for_collision;

  if (!sendActionGoal<SmoothPath>(goal, nsAction("smooth_path"), smooth_path_client_)) {
    return std::nullopt;
  }

  // Wait for result via typed accessor
  auto result = getResult<SmoothPath>();
  if (result) {
    return result->path;
  }
  return std::nullopt;
}
void Navigator::publishInitialPose()
{
  // Convert PoseStamped → PoseWithCovarianceStamped
  geometry_msgs::msg::PoseWithCovarianceStamped msg;

  msg.header = initial_pose_.header;
  msg.pose.pose = initial_pose_.pose;

  // Optional: zero covariance
  for (auto & c : msg.pose.covariance) {
    c = 0.0;
  }

  RCLCPP_INFO(node_->get_logger(), "Publishing Initial Pose");
  initial_pose_pub_->publish(msg);
}

void Navigator::setInitialPose(const geometry_msgs::msg::PoseStamped & initial_pose)
{
  initial_pose_received_ = false;
  initial_pose_ = initial_pose;
  publishInitialPose();
}

void Navigator::amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  (void)msg;
  RCLCPP_DEBUG(node_->get_logger(), "Received AMCL pose");
  initial_pose_received_ = true;
}

void Navigator::waitForInitialPose()
{
  rclcpp::Rate rate(1.0);

  RCLCPP_INFO(node_->get_logger(), "Waiting for AMCL to report initial pose...");

  while (rclcpp::ok() && !initial_pose_received_) {
    publishInitialPose();  //in python code it uses set not publish //  // re-publish could be outside
    rclcpp::spin_some(node_);
    rate.sleep();
  }

  RCLCPP_INFO(node_->get_logger(), "Initial pose successfully received by AMCL.");
}

void Navigator::waitForNodeToActivate(const std::string & node_name)
{
  using GetState = lifecycle_msgs::srv::GetState;
  auto client = node_->create_client<GetState>(node_name + "/get_state");

  RCLCPP_INFO(node_->get_logger(), "Waiting for '%s' to become active...", node_name.c_str());

  while (!client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_INFO(
      node_->get_logger(), "Service '%s/get_state' not available, waiting...", node_name.c_str());
  }

  auto request = std::make_shared<GetState::Request>();
  std::string state = "unknown";

  rclcpp::Rate rate(0.5);  //delay 2 sec between each eteration

  while (rclcpp::ok() && state != "active") {
    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS) {
      state = future.get()->current_state.label;
      RCLCPP_DEBUG(
        node_->get_logger(), "Current state of '%s': %s", node_name.c_str(), state.c_str());
    } else {
      RCLCPP_WARN(node_->get_logger(), "Failed to get state for node '%s'", node_name.c_str());
    }

    rate.sleep();
  }

  RCLCPP_INFO(node_->get_logger(), "'%s' is now active", node_name.c_str());
}

void Navigator::waitUntilNav2Active(const std::string & navigator, const std::string & localizer)
{
  RCLCPP_INFO(node_->get_logger(), "Waiting for Nav2 to become active...");

  if (localizer != "robot_localization") {
    waitForNodeToActivate(localizer);
  }

  if (localizer == "amcl") {
    waitForInitialPose();
  }
  waitForNodeToActivate(navigator);

  RCLCPP_INFO(node_->get_logger(), "Nav2 is now active and ready.");
}

void Navigator::lifecycleStartup()
{
  using ManageLifecycleNodes = nav2_msgs::srv::ManageLifecycleNodes;
  RCLCPP_INFO(node_->get_logger(), "Starting up lifecycle nodes based on lifecycle_manager.");
  // Get all services
  std::map<std::string, std::vector<std::string>> services_and_types =
    node_->get_service_names_and_types();

  bool found_lifecycle_service = false;
  for (const auto & service : services_and_types) {
    const auto & service_name = service.first;
    const auto & service_type = service.second[0];

    //check for /lifecycle_manager_localization/manage_nodes and /lifecycle_manager_navigation/manage_nodes

    if (service_type == "nav2_msgs/srv/ManageLifecycleNodes") {
      found_lifecycle_service = true;
      // Create client
      auto client = node_->create_client<ManageLifecycleNodes>(service_name);

      // Wait for service
      // TODO timeout configure
      while (!client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(
          node_->get_logger(), "Service '%s' not available, waiting...", service_name.c_str());
      }

      //prepare request
      auto request = std::make_shared<ManageLifecycleNodes::Request>();
      request->command = ManageLifecycleNodes::Request::STARTUP;

      // Send async request
      auto future = client->async_send_request(request);

      // Wait for result, retry if needed
      while (rclcpp::ok()) {
        // TODO timeout configure
        if (
          rclcpp::spin_until_future_complete(node_, future, std::chrono::milliseconds(100)) ==
          rclcpp::FutureReturnCode::SUCCESS) {
          break;  // success
        } else {
          RCLCPP_WARN(
            node_->get_logger(), "Retrying lifecycle startup on service '%s'...",
            service_name.c_str());
          waitForInitialPose();  // fallback in case system not ready
        }
      }
    }
  }
  if (found_lifecycle_service) {
    RCLCPP_INFO(node_->get_logger(), "Nav2 is ready for use!");
  } else {
    RCLCPP_WARN(node_->get_logger(), "No Nav2 lifecycle services found. Is Nav2 launched?");
  }
}

void Navigator::lifecycleShutdown()
{
  using ManageLifecycleNodes = nav2_msgs::srv::ManageLifecycleNodes;

  RCLCPP_INFO(node_->get_logger(), "Shutting down lifecycle nodes based on lifecycle_manager.");

  // Get all available services and their types
  auto services_and_types = node_->get_service_names_and_types();

  for (const auto & service : services_and_types) {
    const auto service_name = service.first;
    const auto service_type = service.second[0];

    if (service_type == "nav2_msgs/srv/ManageLifecycleNodes") {
      RCLCPP_INFO(node_->get_logger(), "Shutting down service: %s", service_name.c_str());

      // Create client for ManageLifecycleNodes
      auto client = node_->create_client<ManageLifecycleNodes>(service_name);

      // Wait for the service to be available
      while (!client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(
          node_->get_logger(), "Service '%s' not available, waiting...", service_name.c_str());
      }

      // Create shutdown request
      auto request = std::make_shared<ManageLifecycleNodes::Request>();
      request->command = ManageLifecycleNodes::Request::SHUTDOWN;

      // Send async request and wait for it to complete
      auto future = client->async_send_request(request);
      if (
        rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        if (response->success) {
          RCLCPP_INFO(
            node_->get_logger(), "Successfully shut down node via '%s'", service_name.c_str());
        } else {
          RCLCPP_WARN(
            node_->get_logger(), "Shutdown request to '%s' returned failure", service_name.c_str());
        }
      } else {
        RCLCPP_ERROR(
          node_->get_logger(), "Failed to receive response from '%s'", service_name.c_str());
      }
    }
    //TODO check result and add even more logging
  }

  RCLCPP_INFO(node_->get_logger(), "Lifecycle nodes shutdown complete.");
}

bool Navigator::cancelTask()
{
  if (!action_handle_) {
    RCLCPP_WARN(node_->get_logger(), "No task to cancel");
    return false;  // No task exists
  }

  RCLCPP_INFO(node_->get_logger(), "Canceling current task");

  // Perform cancel and return the result
  bool result = action_handle_->cancel(node_);

  if (!result) {
    RCLCPP_WARN(node_->get_logger(), "Failed to cancel task");
  }

  return result;
}

bool Navigator::isTaskComplete()
{
  if (!action_handle_) {
    return true;
  }
  return action_handle_->isComplete(node_);
}

TaskResult Navigator::getTaskResult()
{
  if (!action_handle_) {
    RCLCPP_WARN(node_->get_logger(), "No active action handle.");
    return TaskResult::kUnknown;
  }

  auto result_code = action_handle_->getResultCode(node_);
  switch (result_code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      return TaskResult::kSucceeded;
    case rclcpp_action::ResultCode::CANCELED:
      return TaskResult::kCanceled;
    case rclcpp_action::ResultCode::ABORTED:
      return TaskResult::kFailed;
    default:
      return TaskResult::kUnknown;
  }
}

bool Navigator::changeMap(const std::string & map_url)
{
  // Create a request to load the map
  auto request = std::make_shared<LoadMap::Request>();
  request->map_url = map_url;

  // Call the service using the template
  auto response = callService<LoadMap>("map_server/load_map", request);

  if (response != nullptr) {
    RCLCPP_INFO(node_->get_logger(), "Map changed successfully: %s", map_url.c_str());
    return true;
  }
  RCLCPP_ERROR(node_->get_logger(), "Failed to load map: %s", map_url.c_str());
  return false;
}

void Navigator::clearAllCostmaps()
{
  clearLocalCostmap();
  clearGlobalCostmap();
}

void Navigator::clearLocalCostmap()
{
  // Create a request to clear the local costmap
  auto request = std::make_shared<ClearEntireCostmap::Request>();

  // Call the service using the template
  auto response =
    callService<ClearEntireCostmap>("local_costmap/clear_entirely_local_costmap", request);

  if (response != nullptr) {
    RCLCPP_INFO(node_->get_logger(), "Local costmap cleared successfully");
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Failed to clear local costmap");
  }
}

void Navigator::clearGlobalCostmap()
{
  // Create a request to clear the global costmap
  auto request = std::make_shared<ClearEntireCostmap::Request>();

  // Call the service using the template
  auto response =
    callService<ClearEntireCostmap>("global_costmap/clear_entirely_global_costmap", request);

  if (response != nullptr) {
    RCLCPP_INFO(node_->get_logger(), "Global costmap cleared successfully");
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Failed to clear global costmap");
  }
}

// void Navigator::clearLocalCostmapAroundPose(const PoseStamped & pose, double distance)
// {
//   // Create a request to clear the local costmap around pose
//   auto request = std::make_shared<ClearCostmapAroundPose::Request>();
//   request->pose = pose;
//   request->radius = distance;

//   // Call the service using the template
//   auto response =
//     callService<ClearCostmapAroundPose>("local_costmap/clear_costmap_around_pose", request);

//   if (response != nullptr) {
//     RCLCPP_INFO(
//       node_->get_logger(), "Local costmap cleared around pose (distance: %.2f m)", distance);
//   } else {
//     RCLCPP_ERROR(
//       node_->get_logger(), "Failed to clear local costmap around pose (distance: %.2f m)",
//       distance);
//   }
// }

// void Navigator::clearGlobalCostmapAroundPose(const PoseStamped & pose, double distance)
// {
//   // Create a request to clear the global costmap around pose
//   auto request = std::make_shared<ClearCostmapAroundPose::Request>();
//   request->pose = pose;
//   request->radius = distance;

//   // Call the service using the template
//   auto response =
//     callService<ClearCostmapAroundPose>("global_costmap/clear_costmap_around_pose", request);

//   if (response != nullptr) {
//     RCLCPP_INFO(
//       node_->get_logger(), "Global costmap cleared around pose (distance: %.2f m)", distance);
//   } else {
//     RCLCPP_ERROR(
//       node_->get_logger(), "Failed to clear global costmap around pose (distance: %.2f m)",
//       distance);
//   }
// }

// void Navigator::clearCostmapExceptRegion(double distance)
// {
//   // Create a request to clear the local costmap except region
//   auto request = std::make_shared<ClearCostmapExceptRegion::Request>();
//   request->radius = distance;

//   // Call the service using the template
//   auto response =
//     callService<ClearCostmapExceptRegion>("local_costmap/clear_costmap_except_region", request);

//   if (response != nullptr) {
//     RCLCPP_INFO(
//       node_->get_logger(), "Local costmap cleared except region (radius: %.2f m)", distance);
//   } else {
//     RCLCPP_ERROR(
//       node_->get_logger(), "Failed to clear local costmap except region (radius: %.2f m)",
//       distance);
//   }
// }

std::optional<Navigator::Costmap> Navigator::getLocalCostmap()
{
  // Create a request to get the costmap
  auto request = std::make_shared<GetCostmap::Request>();

  // Call the service using the template
  auto response = callService<GetCostmap>("local_costmap/get_costmap", request);

  if (response != nullptr) {
    RCLCPP_INFO(node_->get_logger(), "Retrieved local costmap successfully");
    return response->map;
  }
  RCLCPP_ERROR(node_->get_logger(), "Failed to retrieve local costmap");
  return std::nullopt;
}

std::optional<Navigator::Costmap> Navigator::getGlobalCostmap()
{
  // Create a request to get the costmap
  auto request = std::make_shared<GetCostmap::Request>();

  // Call the service using the template
  auto response = callService<GetCostmap>("global_costmap/get_costmap", request);

  if (response != nullptr) {
    RCLCPP_INFO(node_->get_logger(), "Retrieved global costmap successfully");
    return response->map;
  }
  RCLCPP_ERROR(node_->get_logger(), "Failed to retrieve global costmap");
  return std::nullopt;
}

// void Navigator::toggleCollisionMonitor(bool enable)
// {
//   // Create a request to toggle collision monitor
//   auto request = std::make_shared<Toggle::Request>();
//   request->enable = enable;

//   // Call the service using the template
//   auto response = callService<Toggle>("collision_monitor/toggle", request);

//   if (response != nullptr) {
//     if (response->success) {
//       RCLCPP_INFO(
//         node_->get_logger(), "Collision monitor toggled %s successfully", enable ? "ON" : "OFF");
//     } else {
//       RCLCPP_WARN(
//         node_->get_logger(), "Collision monitor toggle %s request failed: %s",
//         enable ? "ON" : "OFF", response->message.c_str());
//     }
//   } else {
//     RCLCPP_ERROR(
//       node_->get_logger(), "Failed to toggle collision monitor %s", enable ? "ON" : "OFF");
//   }
// }

// bool Navigator::followObjectByTopic(const std::string & topic, double max_duration)
// {
//   FollowObject::Goal goal;
//   goal.pose_topic = topic;
//   goal.tracked_frame = "";
//   goal.max_duration = rclcpp::Duration::from_seconds(max_duration);

//   return sendActionGoal<FollowObject>(goal, "follow_object", follow_object_client_);
// }

// bool Navigator::followObjectByFrame(const std::string & frame, double max_duration)
// {
//   FollowObject::Goal goal;
//   goal.pose_topic = "";
//   goal.tracked_frame = frame;
//   goal.max_duration = rclcpp::Duration::from_seconds(max_duration);

//   return sendActionGoal<FollowObject>(goal, "follow_object", follow_object_client_);
// }

// bool Navigator::dockRobotByPose(const PoseStamped & dock_pose, const std::string & dock_type)
// {
//   RCLCPP_INFO(node_->get_logger(), "Docking robot at pose with dock type: %s", dock_type.c_str());

//   DockRobot::Goal goal;
//   goal.use_dock_id = false;
//   goal.dock_pose = dock_pose;
//   goal.dock_type = dock_type;
//   goal.max_staging_time = 1000.0f;
//   goal.navigate_to_staging_pose = true;

//   return sendActionGoal<DockRobot>(goal, "dock_robot", dock_robot_client_);
// }

// bool Navigator::dockRobotById(const std::string & dock_id)
// {
//   RCLCPP_INFO(node_->get_logger(), "Docking robot at dock ID: %s", dock_id.c_str());

//   DockRobot::Goal goal;
//   goal.use_dock_id = true;
//   goal.dock_id = dock_id;
//   goal.max_staging_time = 1000.0f;
//   goal.navigate_to_staging_pose = true;

//   return sendActionGoal<DockRobot>(goal, "dock_robot", dock_robot_client_);
// }

// bool Navigator::undockRobot(const std::string & dock_type)
// {
//   RCLCPP_INFO(node_->get_logger(), "Undocking robot with dock type: %s", dock_type.c_str());

//   UndockRobot::Goal goal;
//   goal.dock_type = dock_type;
//   goal.max_undocking_time = 30.0f;

//   return sendActionGoal<UndockRobot>(goal, "undock_robot", undock_robot_client_);
// }

}  // namespace nav2_simple_commander
