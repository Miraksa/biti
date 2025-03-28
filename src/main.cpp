#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "biti/crossdoor_node.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<biti::CrossDoorNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}