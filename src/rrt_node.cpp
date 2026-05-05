#include <rclcpp/rclcpp.hpp>
#include <rrt/rrt.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("rrt");

    auto rrt = std::make_shared<RRT>(node);

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}