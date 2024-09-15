#include "rclcpp/rclcpp.hpp"
#include "nav2_cpp_simple_commander/robot_navigator.hpp"

//Just to build
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BasicNavigator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
