#include "rclcpp/rclcpp.hpp"
#include "mars_rover_bt_manager/condition_handlers.hpp"
#include "mars_rover_bt_manager/condition_manager.hpp"

using namespace BT;

// IsInLocation
IsInLocation::IsInLocation(const std::string &name, const NodeConfiguration &config, std::shared_ptr<ConditionManager> condition_manager) :
    ConditionNode(name, config), condition_manager_(condition_manager)
{}

NodeStatus IsInLocation::tick()
{
  std::cout << "Checking IsInLocation" << std::endl;

  bool value = condition_manager_->get_value("IsInLocation");

  std::cout << "Current value: " << value << std::endl;

  return value ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

// IsArmInPosition
IsArmInPosition::IsArmInPosition(const std::string &name, const NodeConfiguration &config, std::shared_ptr<ConditionManager> condition_manager) :
    ConditionNode(name, config), condition_manager_(condition_manager)
{}

PortsList IsArmInPosition::providedPorts ()
{
  return {InputPort<std::string>("function")};
}

NodeStatus IsArmInPosition::tick()
{
  std::cout << "Checking IsArmInPosition" << std::endl;

  Optional<std::string> function_msg = getInput<std::string>("function");
  if (!function_msg)
  {
    throw RuntimeError("missing required input [function]: ", function_msg.error());
  }
  std::string function_name = function_msg.value();

  bool value = condition_manager_->get_value("IsArmInPosition("+function_name+")");

  std::cout << "Current value: " << value << std::endl;

  return value ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

// IsPictureTaken
IsPictureTaken::IsPictureTaken(const std::string &name, const NodeConfiguration &config, std::shared_ptr<ConditionManager> condition_manager) :
    ConditionNode(name, config), condition_manager_(condition_manager)
{}

NodeStatus IsPictureTaken::tick()
{
  std::cout << "Checking IsPictureTaken" << std::endl;

  bool value = condition_manager_->get_value("IsPictureTaken");

  std::cout << "Current value: " << value << std::endl;

  return value ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

// IsDigged
IsDigged::IsDigged(const std::string &name, const NodeConfiguration &config, std::shared_ptr<ConditionManager> condition_manager) :
    ConditionNode(name, config), condition_manager_(condition_manager)
{}

NodeStatus IsDigged::tick()
{
  std::cout << "Checking IsDigged" << std::endl;

  bool value = condition_manager_->get_value("IsDigged");

  std::cout << "Current value: " << value << std::endl;

  return value ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

// IsContainerEmpty
IsContainerEmpty::IsContainerEmpty(const std::string &name, const NodeConfiguration &config, std::shared_ptr<ConditionManager> condition_manager) :
    ConditionNode(name, config), condition_manager_(condition_manager)
{}

NodeStatus IsContainerEmpty::tick()
{
  std::cout << "Checking IsContainerEmpty" << std::endl;

  bool value = condition_manager_->get_value("IsContainerEmpty");

  std::cout << "Current value: " << value << std::endl;

  return value ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

// IsTowardObject
IsTowardObject::IsTowardObject(const std::string &name, const NodeConfiguration &config, std::shared_ptr<ConditionManager> condition_manager) :
    ConditionNode(name, config), condition_manager_(condition_manager)
{}

NodeStatus IsTowardObject::tick()
{
  std::cout << "Checking IsTowardObject" << std::endl;

  bool value = condition_manager_->get_value("IsTowardObject");

  std::cout << "Current value: " << value << std::endl;

  return value ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

// IsRockInLeft
IsRockInLeft::IsRockInLeft(const std::string &name, const NodeConfiguration &config, std::shared_ptr<ConditionManager> condition_manager) :
    ConditionNode(name, config), condition_manager_(condition_manager)
{}

NodeStatus IsRockInLeft::tick()
{
  std::cout << "Checking IsRockInLeft" << std::endl;

  bool value = condition_manager_->get_value("IsRockInLeft");

  std::cout << "Current value: " << value << std::endl;

  return value ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

// IsRockInRight
IsRockInRight::IsRockInRight(const std::string &name, const NodeConfiguration &config, std::shared_ptr<ConditionManager> condition_manager) :
    ConditionNode(name, config), condition_manager_(condition_manager)
{}

NodeStatus IsRockInRight::tick()
{
  std::cout << "Checking IsRockInRight" << std::endl;

  bool value = condition_manager_->get_value("IsRockInRight");

  std::cout << "Current value: " << value << std::endl;

  return value ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

// OnDust
OnDust::OnDust(const std::string &name, const NodeConfiguration &config, std::shared_ptr<ConditionManager> condition_manager) :
    ConditionNode(name, config), condition_manager_(condition_manager)
{}

NodeStatus OnDust::tick()
{
  std::cout << "Checking OnDust" << std::endl;

  bool value = condition_manager_->get_value("OnDust");

  std::cout << "Current value: " << value << std::endl;

  return value ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

// OnRocks
OnRocks::OnRocks(const std::string &name, const NodeConfiguration &config, std::shared_ptr<ConditionManager> condition_manager) :
    ConditionNode(name, config), condition_manager_(condition_manager)
{}

NodeStatus OnRocks::tick()
{
  std::cout << "Checking OnRocks" << std::endl;

  bool value = condition_manager_->get_value("OnRocks");

  std::cout << "Current value: " << value << std::endl;

  return value ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}
