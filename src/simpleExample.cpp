#include "biti/bt_simple.hpp"

#define DEFAULT_BT_XML "/home/ambatron/Developer/ws_miraksa/src/biti/config/bt_test_tree.xml"

int main() {
    BehaviorTreeFactory factory;
    
    factory.registerNodeType<ApproachObject>("ApproachObject");

    factory.registerSimpleCondition("CheckBattery", std::bind(CheckBattery));

    GripperInterface gripper;

    factory.registerSimpleAction(
        "OpenGripper",
        std::bind(&GripperInterface::open, &gripper));

    factory.registerSimpleAction(
        "CloseGripper",
        std::bind(&GripperInterface::close, &gripper));

        auto tree = factory.createTreeFromFile(DEFAULT_BT_XML);


    tree.tickRoot();

    return 0;
}