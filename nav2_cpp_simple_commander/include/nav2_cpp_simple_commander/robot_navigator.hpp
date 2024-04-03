#ifndef NAV2_SIMPLE_COMMANDER_CPP__BASIC_NAVIGATOR_HPP_
#define NAV2_SIMPLE_COMMANDER_CPP__BASIC_NAVIGATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <action_msgs/msg/goal_status.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/compute_path_through_poses.hpp>
#include <nav2_msgs/action/back_up.hpp>
#include <nav2_msgs/action/spin.hpp>
#include <nav2_msgs/action/follow_gps_waypoints.hpp>
#include <nav2_msgs/action/assisted_teleop.hpp>
#include <nav2_msgs/action/smooth_path.hpp>
#include <nav2_msgs/srv/load_map.hpp>
#include <nav2_msgs/srv/get_costmap.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <nav2_msgs/srv/manage_lifecycle_nodes.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>

#include <memory>

class BasicNavigator : public rclcpp::Node
{
public:
    // here we will create Constructor

    // TaskResult enum class definition
    enum class TaskResult {
        UNKNOWN = 0,
        SUCCEEDED = 1,
        CANCELED = 2,
        FAILED = 3
    };

    //here we will creat public methods

private:
    // Private attributes and methods

    // Member variables
};


#endif // NAV2_SIMPLE_COMMANDER_CPP__BASIC_NAVIGATOR_HPP_