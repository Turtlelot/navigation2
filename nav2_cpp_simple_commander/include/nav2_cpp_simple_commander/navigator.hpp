
#ifndef NAV2_SIMPLE_COMMANDER_CPP__NAVIGATOR_HPP_
#define NAV2_SIMPLE_COMMANDER_CPP__NAVIGATOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"

namespace Nav2SimpleCommander
{
class Navigator
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;

  explicit Navigator(rclcpp::Node::SharedPtr node);

  /// Send a NavigateToPose goal. Returns true on success.
  bool goToPose(const geometry_msgs::msg::PoseStamped & pose);

  /// Send a FollowWaypoints goal. Returns true on success.
  bool followWaypoints(const std::vector<geometry_msgs::msg::PoseStamped> & poses);

private:
  rclcpp::Node::SharedPtr node_;

  /// The single templated runner: waits for server, sends goal, spins for
  /// result.
  template <typename ActionT>
  bool runAction(
    const std::string & action_name, const typename ActionT::Goal & goal,
    std::function<void(const std::shared_ptr<const typename ActionT::Feedback>)> feedback_cb);
};

}  // namespace Nav2SimpleCommander

// Explicit template instantiation declarations
extern template bool
Nav2SimpleCommander::Navigator::runAction<Nav2SimpleCommander::Navigator::NavigateToPose>(
  const std::string &, const Nav2SimpleCommander::Navigator::NavigateToPose::Goal &,
  std::function<
    void(std::shared_ptr<const Nav2SimpleCommander::Navigator::NavigateToPose::Feedback>)>);

extern template bool
Nav2SimpleCommander::Navigator::runAction<Nav2SimpleCommander::Navigator::FollowWaypoints>(
  const std::string &, const Nav2SimpleCommander::Navigator::FollowWaypoints::Goal &,
  std::function<
    void(std::shared_ptr<const Nav2SimpleCommander::Navigator::FollowWaypoints::Feedback>)>);

#endif  // NAV2_SIMPLE_COMMANDER_CPP__NAVIGATOR_HPP_
