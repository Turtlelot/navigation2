#include "nav2_cpp_simple_commander/navigator.hpp"
#include "rclcpp/rclcpp.hpp"

//Just to build
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("nav2_cpp_simple_commander_node");
  RCLCPP_INFO(node->get_logger(), "Nav2 Simple Commander Node Started");

  // Create an instance of the Navigator class
  Nav2SimpleCommander::Navigator navigator(node);

  // Example usage of the Navigator class
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = 1.0;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.w = 1.0;
  navigator.goToPose(pose);

  std::vector<geometry_msgs::msg::PoseStamped> waypoints;
  geometry_msgs::msg::PoseStamped waypoint;
  waypoint.header.frame_id = "map";
  waypoint.pose.position.y = 0.0;
  waypoint.pose.orientation.w = 1.0;

  for (size_t i = 0; i < 3; ++i) {
    waypoint.pose.position.x = 1.0 + i;
    waypoints.push_back(waypoint);
  }
  navigator.followWaypoints(waypoints);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
