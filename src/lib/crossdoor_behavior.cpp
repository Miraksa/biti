#include <biti/crossdoor_behavior.hpp>

inline void SleepMS(int ms)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

using BT::NodeStatus;

bool random_chance(double probability) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<> dis(0.0, 1.0);
    return dis(gen) < probability;
}

NodeStatus CrossDoor::isDoorClosed() {
    SleepMS(500);

    if (_door_open) {
        RCLCPP_INFO(rclcpp::get_logger("CrossDoor"), "isDoorClosed: SUCCESS");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("CrossDoor"), "isDoorClosed: FAILURE");
    }

  return !_door_open ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NodeStatus CrossDoor::passThroughDoor() {
    SleepMS(500);

    if (_door_open) {
        RCLCPP_INFO(rclcpp::get_logger("CrossDoor"), "passThroughDoor: SUCCESS");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("CrossDoor"), "passThroughDoor: FAILURE");
    }

  return _door_open ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NodeStatus CrossDoor::openDoor() {
    SleepMS(500);

    if (_door_locked) {
        RCLCPP_INFO(rclcpp::get_logger("CrossDoor"), "openDoor: FAILURE");
        return NodeStatus::FAILURE;
    } else {
        _door_open = true;
        RCLCPP_INFO(rclcpp::get_logger("CrossDoor"), "openDoor: SUCCESS");
        return NodeStatus::SUCCESS;
    }
}

NodeStatus CrossDoor::pickLock() {
    SleepMS(500);

    _pick_attempts++;

    if (_door_locked && random_chance(0.7)) {
        RCLCPP_INFO(rclcpp::get_logger("CrossDoor"), "attempt %d, pick a Lock: FAILURE", _pick_attempts);
        return NodeStatus::FAILURE;
    } else {
        _door_locked = false;
        _door_open = true;
        RCLCPP_INFO(rclcpp::get_logger("CrossDoor"), "attempt %d, pick a Lock: SUCCESS", _pick_attempts);
        return NodeStatus::SUCCESS;
    }
}

NodeStatus CrossDoor::smashDoor() {
    _door_locked = false;
    _door_open = true;
    // smash always works
    // std::cout << "smashDoor: SUCCESS" << std::endl;
    RCLCPP_INFO(rclcpp::get_logger("CrossDoor"), "smashDoor: SUCCESS");
    return NodeStatus::SUCCESS;
}

void CrossDoor::registerNodes(BT::BehaviorTreeFactory &factory) {
    factory.registerSimpleCondition(
        "IsDoorClosed", std::bind(&CrossDoor::isDoorClosed, this));
    
    factory.registerSimpleAction(
        "PassThroughDoor", std::bind(&CrossDoor::passThroughDoor, this));

    factory.registerSimpleAction(
        "OpenDoor", std::bind(&CrossDoor::openDoor, this));

    factory.registerSimpleAction(
        "PickLock", std::bind(&CrossDoor::pickLock, this));

    factory.registerSimpleCondition(
        "SmashDoor", std::bind(&CrossDoor::smashDoor, this));
}