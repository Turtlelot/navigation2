#include "navigator.hpp"
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

  /**
   * Core runner for any NAV2 Action:
   * 1) wait for server
   * 2) async_send_goal
   * 3) spin_until_future_complete
   * 4) check accepted
   * 5) async_get_result
   * 6) spin_until_future_complete
   */

  template <typename ActionT>
  bool Navigator::runAction(
      const std::string &action_name,
      const typename ActionT::Goal &goal)
  {
    using ClientT = rclcpp_action::Client<ActionT>;

    // 1) wait for server
    auto client = rclcpp_action::create_client<ActionT>(node_, action_name);
    RCLCPP_INFO(node_->get_logger(),
                "Waiting for '%s' action server...", action_name.c_str());
    if (!client->wait_for_action_server(std::chrono::seconds(5)))
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "Action server '%s' not available", action_name.c_str());
      return false;
    }

    // 2) send goal
    auto goal_handle_future = client->async_send_goal(goal);

    // 3) spin until goal is sent
    if (rclcpp::spin_until_future_complete(node_, goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node_->get_logger(), "send_goal_async failed");
      return false;
    }

    // 4) check acceptance
    auto goal_handle = goal_handle_future.get();
    if (!goal_handle)
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "Goal to '%s' was rejected", action_name.c_str());
      return false;
    }

    // 5) wait for result
    auto result_future = client->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(node_, result_future) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node_->get_logger(), "get_result_async failed");
      return false;
    }

    // 6) done! we ignore the actual result message here.
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