#ifndef NAV2_SIMPLE_COMMANDER_CPP__BASIC_NAVIGATOR_HPP_
#define NAV2_SIMPLE_COMMANDER_CPP__BASIC_NAVIGATOR_HPP_

#include <action_msgs/msg/goal_status.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <memory>
#include <nav2_msgs/action/assisted_teleop.hpp>
#include <nav2_msgs/action/back_up.hpp>
#include <nav2_msgs/action/compute_path_through_poses.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/follow_gps_waypoints.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/smooth_path.hpp>
#include <nav2_msgs/action/spin.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <nav2_msgs/srv/get_costmap.hpp>
#include <nav2_msgs/srv/load_map.hpp>
#include <nav2_msgs/srv/manage_lifecycle_nodes.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
class BasicNavigator : public rclcpp::Node
{
public:
    // here we will create Constructor

    // TaskResult enum class definition
    enum class TaskResult
    {
        UNKNOWN = 0,
        SUCCEEDED = 1,
        CANCELED = 2,
        FAILED = 3
    };

    // here we will creat public methods
    BasicNavigator()
    {
        nav_to_pose_client_ =
            rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        follow_gps_waypoints_client_ =
            rclcpp_action::create_client<FollowGPSWaypoints>(
                this, "follow_gps_waypoints");
        follow_path_client_ =
            rclcpp_action::create_client<FollowPath>(this, "follow_path");
        navigation_through_poses_client_ =
            rclcpp_action::create_client<NavigateThroughPoses>(
                this, "navigation_through_poses");
        smooth_path_client_ =
            rclcpp_action::create_client<SmoothPath>(this, "smooth_path");
        spin_client_ = rclcpp_action::create_client<Spin>(this, "spin");
        wait_client_ = rclcpp_action::create_client<Wait>(this, "wait");
    }

    // Commander API
    bool goToPose(const geometry_msgs::msg::PoseStamped &pose_stamped);
    TaskResult getResult();
    void getFeedback();
    void goThroughPoses();
    void setInitialPose();
    void followWaypoints();
    void followGPSWaypoints();
    void getPath();
    void getPathThroughPoses();
    void followPath();
    void driveOnHeading();
    void backup();
    void changeMap();

    // Costmap API
    void getSizeInMetersX();
    void getSizeInMetersY();
    void getOriginX();
    void getCostXY();

    // Footprint Collison Checker API
    void pointCost();

    // using for clients
private:
    using NavToPoseT = nav2_msgs::action::NavigateToPose;
    using FollowGPSWaypointsT = nav2_msgs::action::FollowGPSWaypoints;
    using FollowPathT = nav2_msgs::action::FollowPath;
    using NavigateThroughPosesT = nav2_msgs::action::NavigateThroughPoses;
    using SmoothPathT = nav2_msgs::action::SmoothPath;
    using SpinT = nav2_msgs::action::Spin;
    using WaitT = nav2_msgs::action::Wait;

    // using for goal handle
    using NavToPoseHandle = rclcpp_action::ClientGoalHandle<NavigateToPoseT>;
    using FollowGPSWaypointsHandle =
        rclcpp_action::ClientGoalHandle<FollowGPSWaypointsT>;
    using FollowPathHandle = rclcpp_action::ClientGoalHandle<FollowPathT>;
    using NavigateThroughPosesHandle =
        rclcpp_action::ClientGoalHandle<NavigateThroughPosesT>;
    using SmoothPathHandle = rclcpp_action::ClientGoalHandle<SmoothPathT>;
    using SpinHandle = rclcpp_action::ClientGoalHandle<SpinT>;
    using WaitHandle = rclcpp_action::ClientGoalHandle<WaitT>;

    // Private attributes and methods

    // Member variables
    // clients
    rclcpp_action::Client<NavToPoseT>::SharedPtr nav_to_pose_client_;
    rclcpp_action::Client<FollowGPSWaypointsT>::SharedPtr
        follow_gps_waypoints_client_;
    rclcpp_action::Client<FollowPathT>::SharedPtr follow_path_client_;
    rclcpp_action::Client<NavigateThroughPosesT>::SharedPtr
        navigation_through_poses_client_;
    rclcpp_action::Client<SmoothPathT>::SharedPtr smooth_path_client_;
    rclcpp_action::Client<SpinT>::SharedPtr spin_client_;
    rclcpp_action::Client<WaitT>::SharedPtr wait_client_;

    // future
    std::shared_future<NavToPoseHandle::SharedPtr> future_nav_to_pose_;
    std::shared_future<FollowGPSWaypointsHandle::SharedPtr>
        future_follow_gps_waypoints_;
    std::shared_future<FollowPathHandle::SharedPtr> future_follow_path_;
    std::shared_future<NavigateThroughPosesHandle::SharedPtr>
        future_navigation_through_poses_;
    std::shared_future<SmoothPathHandle::SharedPtr> future_smooth_path_;
    std::shared_future<SpinHandle::SharedPtr> future_spin_;
    std::shared_future<WaitHandle::SharedPtr> future_wait_;
};

#endif // NAV2_SIMPLE_COMMANDER_CPP__BASIC_NAVIGATOR_HPP_