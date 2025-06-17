#include "nav2_cpp_simple_commander/navigator.hpp"

#include <chrono>

#include "rclcpp_action/rclcpp_action.hpp"

namespace Nav2SimpleCommander
{

Navigator::Navigator(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {}

bool Navigator::goToPose(const geometry_msgs::msg::PoseStamped & pose)
{
  NavigateToPose::Goal goal;
  goal.pose = pose;
  return runAction<NavigateToPose>(
    "navigate_to_pose", goal,
    [this](const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
      RCLCPP_INFO(node_->get_logger(), "Remaining distance: %.2f", feedback->distance_remaining);
    });
}

bool Navigator::followWaypoints(const std::vector<geometry_msgs::msg::PoseStamped> & poses)
{
  FollowWaypoints::Goal goal;
  goal.poses = poses;
  return runAction<FollowWaypoints>(
    "follow_waypoints", goal,
    [this](const std::shared_ptr<const FollowWaypoints::Feedback> feedback) {
      RCLCPP_INFO(
        node_->get_logger(), "Currently executing waypoint index: %d", feedback->current_waypoint);
    });
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
        feedback) mutable {  //modify captured-by-value variables (update the value only not the original var
      rclcpp::Time now = node_->now();
      //feedback every 2 seconds to reduce callback frequency
      if ((now - last_feedback_time).seconds() >= 2.0) {
        last_feedback_time = now;

        if (feedback_cb) {
          feedback_cb(feedback);
        } else {
          RCLCPP_DEBUG(node_->get_logger(), "Feedback received (no callback set)");
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

// explicit instantiations for the two NAV2 actions
template bool Navigator::runAction<Navigator::NavigateToPose>(
  const std::string &, const Navigator::NavigateToPose::Goal &,
  std::function<
    void(std::shared_ptr<const Nav2SimpleCommander::Navigator::NavigateToPose::Feedback>)>);

template bool Navigator::runAction<Navigator::FollowWaypoints>(
  const std::string &, const Navigator::FollowWaypoints::Goal &,
  std::function<
    void(std::shared_ptr<const Nav2SimpleCommander::Navigator::FollowWaypoints::Feedback>)>);

}  // namespace Nav2SimpleCommander