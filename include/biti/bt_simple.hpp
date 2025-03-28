#ifndef BITI_BITI_BT_SIMPLE_HPP_
#define BITI_BITI_BT_SIMPLE_HPP_

#include <random>
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;

bool random_chance(double probability) {
  static std::random_device rd;
  static std::mt19937 gen(rd());
  static std::uniform_real_distribution<> dis(0.0, 1.0);
  return dis(gen) < probability;
}

// Action: ApproachObject
class ApproachObject : public BT::SyncActionNode {
public:
  ApproachObject(const std::string& name) :
      BT::SyncActionNode(name, {}) {}

  NodeStatus tick() override {
    if (random_chance(0.9)) {
      std::cout << "ApproachObject: " << this->name() << " failed" << std::endl;
      return NodeStatus::FAILURE;
    } else {
      std::cout << "ApproachObject: " << this->name() << " succeeded" << std::endl;
      return NodeStatus::SUCCESS;
    }
  }
};

// Condition: CheckBattery
NodeStatus CheckBattery() {
  if (random_chance(0.2)) {
    std::cout << "Battery LOW" << std::endl;
    return NodeStatus::FAILURE;
  } else {
    std::cout << "Battery OK" << std::endl;
    return NodeStatus::SUCCESS;
  }
}


// Action: GripperInterface
class GripperInterface {
public:
  GripperInterface() : _open(true) {}

  NodeStatus open() {
    _open = true;
    std::cout << "Gripper open" << std::endl;
    return NodeStatus::SUCCESS;
  }

  NodeStatus close() {
    _open = false;
    std::cout << "Gripper close" << std::endl;
    return NodeStatus::SUCCESS;
  }

private:
  bool _open;
};

#endif // BITI_BITI_BT_SIMPLE_HPP_