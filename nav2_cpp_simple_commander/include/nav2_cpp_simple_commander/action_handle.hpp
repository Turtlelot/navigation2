#ifndef NAV2_CPP_SIMPLE_COMMANDER__ACTION_HANDLE_HPP_
#define NAV2_CPP_SIMPLE_COMMANDER__ACTION_HANDLE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace Nav2SimpleCommander
{
// Interface for a generic action handle
class IActionHandle
{
public:
  virtual ~IActionHandle() = default;
  // Cancel the current goal
  virtual void cancel(const rclcpp::Node::SharedPtr & node) = 0;
  // Check if the action is done
  virtual bool isDone(const rclcpp::Node::SharedPtr & node) = 0;
};

// Template implementation of IActionHandle
template <typename ActionT>
class ActionHandleImpl : public IActionHandle
{
public:
  using ClientT = rclcpp_action::Client<ActionT>;
  using GoalHandleT = typename rclcpp_action::ClientGoalHandle<ActionT>;

  ActionHandleImpl(typename ClientT::SharedPtr client, std::shared_ptr<GoalHandleT> goal_handle)
  : client_(client), goal_handle_(goal_handle)
  {
    result_future_ = client_->async_get_result(goal_handle_);
  }

  // Cancel the goal and wait for the result to be returned
  void cancel(const rclcpp::Node::SharedPtr & node) override
  {
    if (!goal_handle_) {
      RCLCPP_WARN(node->get_logger(), "[cancel] Goal handle is null. Nothing to cancel.");
      return;
    }

    auto cancel_future = client_->async_cancel_goal(goal_handle_);
    auto cancel_ret =
      rclcpp::spin_until_future_complete(node, cancel_future, std::chrono::seconds(1));
    if (cancel_ret != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN(node->get_logger(), "[cancel] Cancel request did not complete successfully.");
      return;
    }

    if (
      rclcpp::spin_until_future_complete(node, result_future_, std::chrono::seconds(2)) ==
      rclcpp::FutureReturnCode::SUCCESS) {
      auto result = result_future_.get();
      // Check if the result indicates the action was successfully canceled
      if (result.code == rclcpp_action::ResultCode::CANCELED) {
        RCLCPP_INFO(node->get_logger(), "Cancelled");
      } else {
        RCLCPP_INFO(
          node->get_logger(), "Action ended with code: %d and not canceled",
          static_cast<int>(result.code));
      }
    }
  }

  // Return true if the action is completed
  bool isDone(const rclcpp::Node::SharedPtr & node) override
  {
    if (!result_future_.valid()) {
      RCLCPP_WARN(node->get_logger(), "Result future is invalid during isDone check.");
      return true;
    }

    auto status =
      rclcpp::spin_until_future_complete(node, result_future_, std::chrono::milliseconds(100));
    return status == rclcpp::FutureReturnCode::SUCCESS;
  }

private:
  typename ClientT::SharedPtr client_;
  std::shared_ptr<GoalHandleT> goal_handle_;
  std::shared_future<typename GoalHandleT::WrappedResult> result_future_;
};

}  // namespace Nav2SimpleCommander

#endif  // NAV2_CPP_SIMPLE_COMMANDER__ACTION_HANDLE_HPP_
