#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "biti/crossdoor_behavior.hpp"
#include "std_srvs/srv/empty.hpp"

static const char* xml_text = R"(
<root main_tree_to_execute = "MainTree">
    <BehaviorTree ID="MainTree">
        <Sequence>
            <Fallback>
                <Inverter>
                    <IsDoorClosed/>
                </Inverter>
                <SubTree ID="DoorClosed"/>
            </Fallback>
            <PassThroughDoor/>
        </Sequence>
    </BehaviorTree>

    <BehaviorTree ID="DoorClosed">
        <Fallback>
            <OpenDoor/>
            <RetryUntilSuccessful num_attempts="5">
                <PickLock/>
            </RetryUntilSuccessful>
            <SmashDoor/>
        </Fallback>
    </BehaviorTree>
</root>
)";

class BehaviorTreeNode : public rclcpp::Node {
public:
    BehaviorTreeNode() : Node("behavior_tree") {

        _crossdoor.registerNodes(_factory);

        auto tree = _factory.createTreeFromText(xml_text);

        BT::printTreeRecursively(tree.rootNode());

        BT::NodeStatus status = BT::NodeStatus::RUNNING;
        while (rclcpp::ok()) {
            _crossdoor.resetState();
            BT::NodeStatus status = BT::NodeStatus::RUNNING;
            while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
                status = tree.tickRoot();
            }
            RCLCPP_INFO(this->get_logger(), "Tree execution finished with status: %s", BT::toStr(status).c_str());            
        }
    }
    

private:
    BT::BehaviorTreeFactory _factory;
    CrossDoor _crossdoor;
    bool _door_open = false;
    bool _door_locked = true;
    int _pick_attempts = 0;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr _reset_service;

    void handle_reset_request(const std::shared_ptr<rmw_request_id_t> request_header,
                                const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                std::shared_ptr<std_srvs::srv::Empty::Response> response) {
                                (void)request_header;
                                (void)request;
                                (void)response;

        RCLCPP_INFO(this->get_logger(), "Resetting state...");
        _crossdoor.resetState();
    }

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<BehaviorTreeNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}