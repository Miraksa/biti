#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
// #include "behaviortree_cpp_v3/bt_factory.h"
#include "biti/crossdoor_behavior.hpp"
#include "std_srvs/srv/empty.hpp"

namespace biti {
 
class CrossDoorNode: public rclcpp::Node
{
    public:
    CrossDoorNode() : Node("crossdoor_node") {}
    
    private:
    BT::BehaviorTreeFactory _factory;
    CrossDoor _crossdoor;
    bool _door_open = false;
    bool _door_locked = true;
    int _pick_attempts = 0;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr _reset_service;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _state_publisher;

    static inline const char* xml_text = R"(
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
};

} // namespace biti