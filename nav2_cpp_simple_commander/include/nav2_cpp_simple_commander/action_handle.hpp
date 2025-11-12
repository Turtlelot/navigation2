#ifndef NAV2_CPP_SIMPLE_COMMANDER__ACTION_HANDLE_HPP_
#define NAV2_CPP_SIMPLE_COMMANDER__ACTION_HANDLE_HPP_

#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>

namespace nav2_simple_commander
{

namespace
{
// Timeout constants following Google Style (internal linkage)
constexpr auto kDefaultActionTimeout = std::chrono::seconds(5);
constexpr auto kCancelTimeout = std::chrono::seconds(1);
constexpr auto kResultTimeout = std::chrono::seconds(2);
constexpr auto kPollTimeout = std::chrono::milliseconds(100);
}  // namespace

/**
 * @brief Result of attempting to send a goal to an action server.
 *
 * This enum describes the immediate outcome of sendGoal() before the
 * action's final result is available. It indicates whether the goal was
 * accepted by the action server, rejected, the server was unavailable, or
 * sending failed.
 */
enum class GoalStatus { kAccepted, kRejected, kServerUnavailable, kSendFailed };

/**
 * @class ActionHandleBase
 * @brief Abstract base for type-erased action handles.
 *
 * ActionHandleBase defines a minimal interface used by Navigator to manage an
 * opaque active action (cancel, check completion, and query the result code).
 * Concrete action handles are created by the template `ActionHandle<ActionT>`
 * and stored as `std::shared_ptr<ActionHandleBase>` to allow handling different
 * action types uniformly.
 */
class ActionHandleBase
{
public:
  virtual ~ActionHandleBase() = default;

  /**
   * @brief Attempt to cancel the currently active goal.
   *
   * @param node The rclcpp node used for spinning while waiting on the cancel.
   * @return true when cancel successfully completed and was acknowledged, false otherwise.
   */
  virtual bool cancel(const rclcpp::Node::SharedPtr & node) = 0;

  /**
   * @brief Query whether the underlying action has completed.
   *
   * @param node The rclcpp node used for spinning while waiting for the result.
   * @return true when the action is complete (succeeded, canceled, or aborted).
   */
  virtual bool isComplete(const rclcpp::Node::SharedPtr & node) = 0;

  /**
   * @brief Return the action ResultCode for the finished action.
   *
   * @param node The rclcpp node used for spinning while retrieving the result.
   * @return rclcpp_action::ResultCode (SUCCESS, CANCELED, ABORTED, UNKNOWN).
   */
  virtual rclcpp_action::ResultCode getResultCode(const rclcpp::Node::SharedPtr & node) = 0;
};

/**
 * @class ActionHandle
 * @brief Template-based action handle for managing ROS2 action goals.
 *
 * @class ActionHandle<ActionT>
 * @brief Wraps an rclcpp_action::Client<ActionT> and manages the lifecycle of a single goal.
 * It provides helpers for sending goals, receiving feedback, canceling, and retrieving results.
 *
 * @tparam ActionT The ROS2 action type (e.g. nav2_msgs::action::NavigateToPose).
 */
template <typename ActionT>
class ActionHandle : public ActionHandleBase
{
public:
  using ClientT = rclcpp_action::Client<ActionT>;
  using GoalHandleT = typename rclcpp_action::ClientGoalHandle<ActionT>;
  using FeedbackT = typename ActionT::Feedback;
  using ResultT = typename ActionT::Result;
  using WrappedResult = typename GoalHandleT::WrappedResult;

  /**
   * @brief Construct an ActionHandle that uses an existing action client.
   *
   * @param client Shared pointer to the action client (must not be null).
   *
   * @throws std::invalid_argument if `client` is null.
   */
  explicit ActionHandle(typename ClientT::SharedPtr client) : client_(std::move(client))
  {
    if (!client_) {
      throw std::invalid_argument("ActionHandle: client cannot be null");
    }
  }

  // Delete copy operations
  ActionHandle(const ActionHandle &) = delete;
  ActionHandle & operator=(const ActionHandle &) = delete;

  // Default move operations
  ActionHandle(ActionHandle &&) = default;
  ActionHandle & operator=(ActionHandle &&) = default;

  /**
   * @brief Send a goal to the action server and register a feedback callback.
   *
   * @param node Shared pointer to the rclcpp::Node used for spinning while
   * waiting for the server and for the send to complete.
   * @param goal The action Goal message to send.
   * @return GoalStatus describing the immediate outcome (accepted/rejected/etc.).
   */
  GoalStatus sendGoal(const rclcpp::Node::SharedPtr & node, const typename ActionT::Goal & goal)
  {
    if (!client_->wait_for_action_server(kDefaultActionTimeout)) {
      RCLCPP_ERROR(node->get_logger(), "Action server not available after timeout");
      return GoalStatus::kServerUnavailable;
    }

    // Setup feedback callback
    typename ClientT::SendGoalOptions options;
    options.feedback_callback =
      [this](typename GoalHandleT::SharedPtr, const std::shared_ptr<const FeedbackT> feedback) {
        last_feedback_ = feedback;
      };

    // Send goal asynchronously
    auto goal_handle_future = client_->async_send_goal(goal, options);

    if (
      rclcpp::spin_until_future_complete(node, goal_handle_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "Failed to send goal");
      return GoalStatus::kSendFailed;
    }

    goal_handle_ = goal_handle_future.get();
    if (!goal_handle_) {
      RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
      return GoalStatus::kRejected;
    }

    // Get result future for later use
    result_future_ = client_->async_get_result(goal_handle_);

    return GoalStatus::kAccepted;
  }

  /**
   * @brief Request cancellation of the active goal.
   *
   * @param node Shared pointer to the rclcpp::Node used for spinning while
   * waiting for the cancel to be acknowledged.
   * @return true if cancel was successfully requested and the action reports
   * it was canceled; false otherwise.
   */
  bool cancel(const rclcpp::Node::SharedPtr & node) override
  {
    if (!goal_handle_) {
      RCLCPP_WARN(node->get_logger(), "No active goal handle. Nothing to cancel.");
      return false;
    }

    auto cancel_future = client_->async_cancel_goal(goal_handle_);
    auto cancel_result = rclcpp::spin_until_future_complete(node, cancel_future, kCancelTimeout);

    if (cancel_result != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN(node->get_logger(), "Cancel request did not complete successfully");
      return false;
    }

    // Wait for the result to confirm cancellation
    if (
      rclcpp::spin_until_future_complete(node, result_future_, kResultTimeout) ==
      rclcpp::FutureReturnCode::SUCCESS) {
      auto wrapped_result = result_future_.get();
      return wrapped_result.code == rclcpp_action::ResultCode::CANCELED;
    }

    return false;
  }

  /**
   * @brief Check whether the action has completed.
   *
   * @param node Shared pointer to the rclcpp::Node used for spinning while
   * waiting for the result.
   * @return true if the action result is available (action completed), false otherwise.
   */
  bool isComplete(const rclcpp::Node::SharedPtr & node) override
  {
    if (!result_future_.valid()) {
      RCLCPP_DEBUG(node->get_logger(), "Result future is invalid during isComplete check");
      return true;  // No active goal means "done"
    }

    auto status = rclcpp::spin_until_future_complete(node, result_future_, kPollTimeout);
    return status == rclcpp::FutureReturnCode::SUCCESS;
  }

  /**
   * @brief Retrieve the most recent feedback message received from the action server.
   *
   * @return Shared pointer to the latest feedback, or nullptr if none has been
   * received yet.
   */
  std::shared_ptr<const FeedbackT> getFeedback() const
  {
    return last_feedback_;
  }

  /**
   * @brief Block until the action result is available and return the result message.
   *
   * @param node Shared pointer to the rclcpp::Node used for spinning while waiting.
   * @return Shared pointer to the result message on success, or nullptr on failure.
   */
  std::shared_ptr<const ResultT> getResult(const rclcpp::Node::SharedPtr & node)
  {
    if (!result_future_.valid()) {
      RCLCPP_WARN(node->get_logger(), "Result future is invalid");
      return nullptr;
    }

    auto status = rclcpp::spin_until_future_complete(node, result_future_);
    if (status != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "Failed to get result");
      return nullptr;
    }

    auto wrapped_result = result_future_.get();
    return wrapped_result.result;
  }

  /**
   * @brief Retrieve the numeric result code for the finished action.
   *
   * @param node Shared pointer to the rclcpp::Node used for spinning while waiting.
   * @return rclcpp_action::ResultCode describing the final outcome.
   */
  rclcpp_action::ResultCode getResultCode(const rclcpp::Node::SharedPtr & node) override
  {
    if (!result_future_.valid()) {
      RCLCPP_WARN(node->get_logger(), "Result future is invalid");
      return rclcpp_action::ResultCode::UNKNOWN;
    }

    auto status = rclcpp::spin_until_future_complete(node, result_future_);
    if (status != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "Failed to get result code");
      return rclcpp_action::ResultCode::UNKNOWN;
    }

    return result_future_.get().code;
  }

private:
  typename ClientT::SharedPtr client_;
  std::shared_ptr<GoalHandleT> goal_handle_;
  std::shared_future<WrappedResult> result_future_;
  std::shared_ptr<const FeedbackT> last_feedback_;
};

}  // namespace nav2_simple_commander

#endif  // NAV2_CPP_SIMPLE_COMMANDER__ACTION_HANDLE_HPP_
