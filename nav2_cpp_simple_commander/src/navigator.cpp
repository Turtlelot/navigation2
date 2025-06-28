#include "nav2_cpp_simple_commander/navigator.hpp"

#include <chrono>

#include "rclcpp_action/rclcpp_action.hpp"

namespace Nav2SimpleCommander
{

Navigator::Navigator(rclcpp::Node::SharedPtr node) : node_(std::move(node))
{
  // Create publisher to /initialpose
  initial_pose_pub_ =
    node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);

  // Create subscriber to /amcl_pose to monitor when localization acknowledges pose
  amcl_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "amcl_pose", 10, std::bind(&Navigator::amclPoseCallback, this, std::placeholders::_1));
}

bool Navigator::goToPose(
  const geometry_msgs::msg::PoseStamped & pose, const std::string & behavior_tree)
{
  NavigateToPose::Goal goal;
  goal.pose = pose;

  auto feedback_cb = [this](const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
    RCLCPP_INFO(node_->get_logger(), "Remaining distance: %.2f", feedback->distance_remaining);
  };
  return runAction<NavigateToPose>("navigate_to_pose", goal, feedback_cb);
}

bool Navigator::goThroughPoses(
  const std::vector<PoseStamped> & poses, const std::string & behavior_tree)
{
  NavigateThroughPoses::Goal goal;
  goal.poses = poses;
  goal.behavior_tree = behavior_tree;

  auto feedback_cb = [this](const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback) {
    RCLCPP_INFO(
      node_->get_logger(), "Remaining distance: %.2f, Remaining poses: %d",
      feedback->distance_remaining, feedback->number_of_poses_remaining);
  };

  return runAction<NavigateThroughPoses>("navigate_through_poses", goal, feedback_cb);
}

bool Navigator::followWaypoints(const std::vector<geometry_msgs::msg::PoseStamped> & poses)
{
  FollowWaypoints::Goal goal;
  goal.poses = poses;

  auto feedback_cb = [this](const std::shared_ptr<const FollowWaypoints::Feedback> feedback) {
    RCLCPP_INFO(
      node_->get_logger(), "Currently executing waypoint index: %d", feedback->current_waypoint);
  };

  return runAction<FollowWaypoints>("follow_waypoints", goal, feedback_cb);
}

bool Navigator::followGpsWaypoints(const std::vector<GeoPose> & poses)
{
  FollowGPSWaypoints::Goal goal;
  goal.gps_poses = poses;

  auto feedback_cb = [this](const std::shared_ptr<const FollowGPSWaypoints::Feedback> feedback) {
    RCLCPP_INFO(
      node_->get_logger(), "Currently executing waypoint index: %d", feedback->current_waypoint);
  };

  return runAction<FollowGPSWaypoints>("follow_gps_waypoints", goal, feedback_cb);
}
bool Navigator::spin(double spin_dist, double time_allowance)
{
  Spin::Goal goal;
  goal.target_yaw = spin_dist;
  goal.time_allowance = rclcpp::Duration::from_seconds(time_allowance);

  auto feedback_cb = [this](const std::shared_ptr<const Spin::Feedback> feedback) {
    RCLCPP_INFO(
      node_->get_logger(), "Angular distance traveled: %.2f", feedback->angular_distance_traveled);
  };

  return runAction<Spin>("spin", goal, feedback_cb);
}

bool Navigator::backup(double backup_dist, double backup_speed, double time_allowance)
{
  BackUp::Goal goal;

  geometry_msgs::msg::Point target;
  target.x = backup_dist;
  goal.target = target;
  goal.speed = backup_speed;
  goal.time_allowance = rclcpp::Duration::from_seconds(time_allowance);

  auto feedback_cb = [this](const std::shared_ptr<const BackUp::Feedback> feedback) {
    RCLCPP_INFO(node_->get_logger(), "Distance traveled: %.2f", feedback->distance_traveled);
  };

  return runAction<BackUp>("backup", goal, feedback_cb);
}

bool Navigator::driveOnHeading(double dist, double speed, double time_allowance)
{
  DriveOnHeading::Goal goal;
  geometry_msgs::msg::Point target;
  target.x = dist;
  goal.target = target;
  goal.speed = speed;
  goal.time_allowance = rclcpp::Duration::from_seconds(time_allowance);

  auto feedback_cb = [this](const std::shared_ptr<const DriveOnHeading::Feedback> feedback) {
    RCLCPP_INFO(node_->get_logger(), "Distance traveled: %.2f", feedback->distance_traveled);
  };

  return runAction<DriveOnHeading>("drive_on_heading", goal, feedback_cb);
}

bool Navigator::assistedTeleop(double time_allowance)
{
  AssistedTeleop::Goal goal;
  goal.time_allowance = rclcpp::Duration::from_seconds(time_allowance);

  auto feedback_cb = [this](const std::shared_ptr<const AssistedTeleop::Feedback> feedback) {
    RCLCPP_INFO(
      node_->get_logger(), "Current teleop duration: %d seconds",
      feedback->current_teleop_duration.sec);
  };
  return runAction<AssistedTeleop>("assisted_teleop", goal, feedback_cb);
}

bool Navigator::followPath(
  const Path & path, const std::string & controller_id, const std::string & goal_checker_id)
{
  FollowPath::Goal goal;
  goal.path = path;
  goal.controller_id = controller_id;
  goal.goal_checker_id = goal_checker_id;

  auto feedback_cb = [this](const std::shared_ptr<const FollowPath::Feedback> feedback) {
    RCLCPP_INFO(
      node_->get_logger(), "Distance to goal: %.2f, Speed: %.2f", feedback->distance_to_goal,
      feedback->speed);
  };

  return runAction<FollowPath>("follow_path", goal, feedback_cb);
}

bool Navigator::getPath(
  const PoseStamped & start, const PoseStamped & end, const std::string & planner_id,
  bool use_start)
{
  ComputePathToPose::Goal goal;
  goal.start = start;
  goal.goal = end;
  goal.planner_id = planner_id;
  goal.use_start = use_start;

  // No feedback callback for this action

  return runAction<ComputePathToPose>("compute_path_to_pose", goal);
}

bool Navigator::getPathThroughPoses(
  const PoseStamped & start, const std::vector<PoseStamped> goals, const std::string & planner_id,
  bool use_start)
{
  ComputePathThroughPoses::Goal goal;
  goal.start = start;
  goal.goals = goals;
  goal.planner_id = planner_id;
  goal.use_start = use_start;

  // No feedback callback for this action

  return runAction<ComputePathThroughPoses>("compute_path_through_poses", goal);
}

bool Navigator::smoothPath(
  const Path & path, const std::string & smoother_id, double max_duration, bool check_for_collision)
{
  SmoothPath::Goal goal;
  goal.path = path;
  goal.smoother_id = smoother_id;
  goal.max_smoothing_duration = rclcpp::Duration::from_seconds(max_duration);
  goal.check_for_collisions = check_for_collision;

  // No feedback callback for this action

  return runAction<SmoothPath>("smooth_path", goal);
}
void Navigator::publishInitialPose()
{
  RCLCPP_INFO(node_->get_logger(), "Publishing Initial Pose");
  initial_pose_pub_->publish(initial_pose_);
}

void Navigator::setInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose)
{
  initial_pose_received_ = false;
  initial_pose_ = initial_pose;
  publishInitialPose();
}

void Navigator::amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
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
template <typename ActionT>
bool Navigator::runAction(
  const std::string & action_name, const typename ActionT::Goal & goal,
  std::function<void(const std::shared_ptr<const typename ActionT::Feedback>)> feedback_cb)
{
  using ClientT = rclcpp_action::Client<ActionT>;
  using GoalHandleT = typename rclcpp_action::ClientGoalHandle<ActionT>;

  // create action client
  auto client = rclcpp_action::create_client<ActionT>(node_, action_name);

  // wait for server
  RCLCPP_INFO(node_->get_logger(), "Waiting for '%s' action server...", action_name.c_str());
  if (!client->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node_->get_logger(), "Action server '%s' not available", action_name.c_str());
    return false;
  }

  // prepare send_goal options
  typename ClientT::SendGoalOptions send_goal_options;
  rclcpp::Time last_feedback_time = node_->now();

  send_goal_options.feedback_callback =
    [this, &feedback_cb, last_feedback_time](
      typename GoalHandleT::SharedPtr,
      const std::shared_ptr<const typename ActionT::Feedback>
        feedback) mutable {  // modify captured-by-value variables (update the value only not the original var
      rclcpp::Time now = node_->now();
      // feedback every 2 seconds to reduce callback frequency
      if ((now - last_feedback_time).seconds() >= 2.0) {
        last_feedback_time = now;

        if (feedback_cb) {
          feedback_cb(feedback);
        }
      }
    };

  send_goal_options.goal_response_callback =
    [&](const typename GoalHandleT::SharedPtr & goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
      } else {
        RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
      }
    };

  send_goal_options.result_callback = [&](const typename GoalHandleT::WrappedResult & result) {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(node_->get_logger(), "Goal completed successfully");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(node_->get_logger(), "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(node_->get_logger(), "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
        break;
    }
  };

  // send goal
  auto goal_handle_future = client->async_send_goal(goal, send_goal_options);

  // spin until goal is sent
  if (
    rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
    rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "send_goal_async failed");
    return false;
  }

  // check acceptance
  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Goal to '%s' was rejected", action_name.c_str());
    return false;
  }

  // wait for result
  auto result_future = client->async_get_result(goal_handle);
  if (
    rclcpp::spin_until_future_complete(node_, result_future) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "get_result_async failed");
    return false;
  }

  // done! we ignore the actual result message here.
  return true;
}

}  // namespace Nav2SimpleCommander