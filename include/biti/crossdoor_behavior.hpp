#ifndef BITI__CROSSDOOR_NODE_HPP_
#define BITI__CROSSDOOR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "std_msgs/msg/string.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "random"

class CrossDoor
{
    public:
        void registerNodes(BT::BehaviorTreeFactory& factory);

        void resetState() {
            _door_open = false;
            _door_locked = true;
            _pick_attempts = 0;
        };

        BT::NodeStatus isDoorClosed();

        BT::NodeStatus passThroughDoor();

        BT::NodeStatus pickLock();

        BT::NodeStatus openDoor();

        BT::NodeStatus smashDoor();

    private:
        bool _door_open   = false;
        bool _door_locked = true;
        int _pick_attempts = 0;
};


#endif //BITI__CROSSDOOR_NODE_HPP_