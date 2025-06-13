#include "nav2_cpp_simple_commander/navigator.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <chrono>

namespace Nav2SimpleCommander
{

  Navigator::Navigator(rclcpp::Node::SharedPtr node)
      : node_(std::move(node))
  {
  }

  bool Navigator::goToPose(const geometry_msgs::msg::PoseStamped &pose)
  {
    NavigateToPose::Goal goal;
    goal.pose = pose;
    return runAction<NavigateToPose>(
        "navigate_to_pose",
        goal);
  }

  bool Navigator::followWaypoints(
      const std::vector<geometry_msgs::msg::PoseStamped> &poses)
  {
    FollowWaypoints::Goal goal;
    goal.poses = poses;
    return runAction<FollowWaypoints>(
        "follow_waypoints",
        goal);
  }

  template <typename ActionT>
  bool Navigator::runAction(
      const std::string &action_name,
      const typename ActionT::Goal &goal)
  {
    using ClientT = rclcpp_action::Client<ActionT>;
    using GoalHandleT = typename rclcpp_action::ClientGoalHandle<ActionT>;

    // create action client
    auto client = rclcpp_action::create_client<ActionT>(node_, action_name);

    // wait for server
    RCLCPP_INFO(node_->get_logger(),
                "Waiting for '%s' action server...", action_name.c_str());
    if (!client->wait_for_action_server(std::chrono::seconds(5)))
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "Action server '%s' not available", action_name.c_str());
      return false;
    }

    // prepare send_goal options
    typename ClientT::SendGoalOptions send_goal_options;
    send_goal_options.goal_response_callback = [&](const typename GoalHandleT::SharedPtr &goal_handle)
    {
      if (!goal_handle)
      {
        RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
      }
      else
      {
        RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
      }
    };

    send_goal_options.feedback_callback =
        [&](typename GoalHandleT::SharedPtr,
            const std::shared_ptr<const typename ActionT::Feedback> feedback)
    {
      RCLCPP_DEBUG(node_->get_logger(), "Feedback received");
    };

    send_goal_options.result_callback =
        [&](const typename GoalHandleT::WrappedResult &result)
    {
      switch (result.code)
      {
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
    if (rclcpp::spin_until_future_complete(node_, goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node_->get_logger(), "send_goal_async failed");
      return false;
    }

    // check acceptance
    auto goal_handle = goal_handle_future.get();
    if (!goal_handle)
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "Goal to '%s' was rejected", action_name.c_str());
      return false;
    }

    // wait for result
    auto result_future = client->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(node_, result_future) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node_->get_logger(), "get_result_async failed");
      return false;
    }

    // done! we ignore the actual result message here.
    return true;
  }

  // explicit instantiations for the two NAV2 actions
  template bool Navigator::runAction<Navigator::NavigateToPose>(
      const std::string &,
      const Navigator::NavigateToPose::Goal &);

  template bool Navigator::runAction<Navigator::FollowWaypoints>(
      const std::string &,
      const Navigator::FollowWaypoints::Goal &);

} // namespace Nav2SimpleCommander