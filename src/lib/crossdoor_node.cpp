#include <biti/crossdoor_node.hpp>

namespace biti {

CrossDoorNode::CrossDoorNode() : Node("crossdoor_node")
{
    _crossdoor.registerNodes(_factory);

    auto tree = _factory.createTreeFromText(xml_text);

    // BT::printTreeRecursively(tree.rootNode());

    
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (rclcpp::ok())
    {
        _crossdoor.resetState();
        BT::NodeStatus status = BT::NodeStatus::RUNNING;
        while (rclcpp::ok() && status == BT::NodeStatus::RUNNING)
        {
            status = tree.tickRoot();
        }
        RCLCPP_INFO(this->get_logger(), "Tree execution finished with status: %s", BT::toStr(status).c_str());
    }
};

}