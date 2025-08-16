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
  // Returns the final result code (e.g., SUCCEEDED, CANCELED) after waiting for the action to finish
  virtual rclcpp_action::ResultCode getWrappedResultCode(const rclcpp::Node::SharedPtr & node) = 0;
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
  //TODO discuss 
  ActionHandleImpl() = default;

  // run the action 
  bool runAction(
    const rclcpp::Node::SharedPtr & node,
    const std::string & action_name,
    const typename ActionT::Goal & goal)
  {
    //TODO create action client (discussion: should we create it in the Navigator::NavigatToPose() method?)
    client_ = rclcpp_action::create_client<ActionT>(node, action_name);

    // wait for server
    RCLCPP_INFO(node->get_logger(), "Waiting for '%s' action server...", action_name.c_str());
    if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(node->get_logger(), "Action server '%s' not available", action_name.c_str());
      return false;
    }
    // prepare send_goal options
    typename ClientT::SendGoalOptions send_goal_options;
    send_goal_options.feedback_callback =
      [this](typename GoalHandleT::SharedPtr,
             const std::shared_ptr<const typename ActionT::Feedback> feedback)
      {
        this->setFeedback(feedback);
      };

    //send goal
    auto goal_handle_future = client_->async_send_goal(goal, send_goal_options);
    // spin until goal is sent
    if (rclcpp::spin_until_future_complete(node, goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "Failed to send goal to '%s'", action_name.c_str());
      return false;
    }
    // check acceptance
    goal_handle_ = goal_handle_future.get();
    if (!goal_handle_) {
      RCLCPP_ERROR(node->get_logger(), "Goal to '%s' was rejected", action_name.c_str());
      return false;
    }

    result_future_ = client_->async_get_result(goal_handle_);
    // done! we ignore the actual result message here.

    return true;
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
      if (!(result.code == rclcpp_action::ResultCode::CANCELED)) {
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
  
  std::shared_ptr<const typename ActionT::Feedback> getFeedback() const { return last_feedback_; }
  typename GoalHandleT::Result::SharedPtr getResult(const rclcpp::Node::SharedPtr & node)
  {
    if (!result_future_.valid()) {
      RCLCPP_WARN(node->get_logger(), "Result future is invalid.");
      return nullptr;
    }
    
    auto status = rclcpp::spin_until_future_complete(node, result_future_);
    if (status != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "Failed to get result.");
      return nullptr;
    }
    
    auto result = result_future_.get();
    return result.result;
  }
  
  rclcpp_action::ResultCode getWrappedResultCode(const rclcpp::Node::SharedPtr & node) override
  {
    if (!result_future_.valid()) {
      RCLCPP_WARN(node->get_logger(), "Result future is invalid.");
      return rclcpp_action::ResultCode::UNKNOWN;
    }
    
    auto status = rclcpp::spin_until_future_complete(node, result_future_);
    if (status != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "Failed to get result.");
      return rclcpp_action::ResultCode::UNKNOWN;
    }
    
    auto result = result_future_.get();
    return result.code;
  }
  
  private:
  void setFeedback(std::shared_ptr<const typename ActionT::Feedback> feedback)
  {
    last_feedback_ = feedback;
  }
  typename ClientT::SharedPtr client_;
  std::shared_ptr<GoalHandleT> goal_handle_;
  // Wrapped result is the code and result msg of the action
  std::shared_future<typename GoalHandleT::WrappedResult> result_future_;
  // Last received feedback
  std::shared_ptr<const typename ActionT::Feedback> last_feedback_;
};

}  // namespace Nav2SimpleCommander

#endif  // NAV2_CPP_SIMPLE_COMMANDER__ACTION_HANDLE_HPP_
