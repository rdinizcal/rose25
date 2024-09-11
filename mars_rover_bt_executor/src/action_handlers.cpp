#include "rclcpp/rclcpp.hpp"
#include "mars_rover_bt_executor/action_handlers.hpp"
#include "mars_rover_bt_executor/condition_manager.hpp"

using namespace BT;

// Move
Move::Move(const std::string &name, const NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher)
   :  SyncActionNode(name, config), publisher_(publisher)
{}

PortsList Move::providedPorts ()
{
  return {InputPort<std::string>("time"), InputPort<std::string>("speed")};
}

NodeStatus Move::tick()
{
  std::cout << "Executing Move action" << std::endl;

  Optional<std::string> time_msg = getInput<std::string>("time");
  if (!time_msg)
  {
    throw RuntimeError("missing required input [time]: ", time_msg.error() );
  }

  Optional<std::string> speed_msg = getInput<std::string>("speed");
  if (!speed_msg)
  {
    throw RuntimeError("missing required input [speed]: ", speed_msg.error() );
  }

  double time = std::stod(time_msg.value());
  double speed = std::stod(speed_msg.value());

  publisher_->move(speed, time);

  return NodeStatus::SUCCESS;
}

// Stop
Stop::Stop(const std::string &name, const NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher) :
    SyncActionNode(name, config), publisher_(publisher)
{}

NodeStatus Stop::tick()
{
  std::cout << "Executing Stop action" << std::endl;

  return NodeStatus::SUCCESS;
}

// Move Forward
MoveForward::MoveForward(const std::string &name, const NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher)
   :  SyncActionNode(name, config), publisher_(publisher)
{}

PortsList MoveForward::providedPorts ()
{
  return {InputPort<std::string>("time")};
}

NodeStatus MoveForward::tick()
{
  std::cout << "Executing MoveForward action" << std::endl;

  Optional<std::string> time_msg = getInput<std::string>("time");
  if (!time_msg)
  {
    throw RuntimeError("missing required input [time]: ", time_msg.error() );
  }

  double time = std::stod(time_msg.value());

  publisher_->move_forward(time);

  return NodeStatus::SUCCESS;
}

// TurnLeft
TurnLeft::TurnLeft(const std::string &name, const NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher) :
    SyncActionNode(name, config), publisher_(publisher)
{}

PortsList TurnLeft::providedPorts ()
{
  return {InputPort<std::string>("angle")};
}

NodeStatus TurnLeft::tick()
{
  std::cout << "Executing TurnLeft action" << std::endl;

  Optional<std::string> angle_msg = getInput<std::string>("angle");
  if (!angle_msg)
  {
    throw RuntimeError("missing required input [angle]: ", angle_msg.error() );
  }

  double angle = std::stod(angle_msg.value());
  std::cout << "Turning left of " << angle << " degrees" << std::endl;

  publisher_->turn_left(angle);

  return NodeStatus::SUCCESS;
}

// TurnRight
TurnRight::TurnRight(const std::string &name, const NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher) :
    SyncActionNode(name, config), publisher_(publisher)
{}

PortsList TurnRight::providedPorts ()
{
  return {InputPort<std::string>("angle")};
}

NodeStatus TurnRight::tick()
{
  std::cout << "Executing TurnRight action" << std::endl;

  Optional<std::string> angle_msg = getInput<std::string>("angle");
  if (!angle_msg)
  {
    throw RuntimeError("missing required input [angle]: ", angle_msg.error() );
  }

  double angle = std::stod(angle_msg.value());
  std::cout << "Turning right of " << angle << " degrees" << std::endl;

  publisher_->turn_right(angle);

  return NodeStatus::SUCCESS;
}

// OpenToolArm
OpenToolArm::OpenToolArm(const std::string &name, const NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher) :
    SyncActionNode(name, config), publisher_(publisher)
{}

NodeStatus OpenToolArm::tick()
{
  std::cout << "Executing OpenToolArm action" << std::endl;

  publisher_->open_tool_arm();

  return NodeStatus::SUCCESS;
}

// CloseToolArm
CloseToolArm::CloseToolArm(const std::string &name, const NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher) :
    SyncActionNode(name, config), publisher_(publisher)
{}

NodeStatus CloseToolArm::tick()
{
  std::cout << "Executing CloseToolArm action" << std::endl;

  publisher_->close_tool_arm();

  return NodeStatus::SUCCESS;
}

// OpenMast
OpenMast::OpenMast(const std::string &name, const NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher) :
    SyncActionNode(name, config), publisher_(publisher)
{}

NodeStatus OpenMast::tick()
{
  std::cout << "Executing OpenMast action" << std::endl;

  publisher_->open_mast();

  return NodeStatus::SUCCESS;
}

// CloseMast
CloseMast::CloseMast(const std::string &name, const NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher) :
    SyncActionNode(name, config), publisher_(publisher)
{}

NodeStatus CloseMast::tick()
{
  std::cout << "Executing CloseMast action" << std::endl;

  publisher_->close_mast();

  return NodeStatus::SUCCESS;
}

// RotateMast
RotateMast::RotateMast(const std::string &name, const NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher) :
    SyncActionNode(name, config), publisher_(publisher)
{}

NodeStatus RotateMast::tick()
{
  std::cout << "Executing RotateMast action" << std::endl;

  publisher_->rotate_mast();

  return NodeStatus::SUCCESS;
}

// CheckCondition
CheckCondition::CheckCondition(const std::string &name, const NodeConfiguration &config, std::shared_ptr<ConditionManager> condition_manager) :
    SyncActionNode(name, config), condition_manager_(condition_manager)
{}

PortsList CheckCondition::providedPorts ()
{
  return {
    InputPort<std::string>("condition"),
    InputPort<std::string>("value")
  };
}

NodeStatus CheckCondition::tick()
{
  std::cout << "Executing CheckCondition action" << std::endl;

  Optional<std::string> condition_msg = getInput<std::string>("condition");
  if (!condition_msg)
  {
    // Here we assume that the condition is already in the blackboard.
    // If not, we should try to get in some way. If we can not, then raise an error.
    throw RuntimeError("missing required input [condition]: ", condition_msg.error());
  }
  std::string condition_name = condition_msg.value();
  std::cout << "Condition name " << condition_name << std::endl;

  Optional<std::string> ref_value_opt = getInput<std::string>("value");
  if (!ref_value_opt)
  {
    throw RuntimeError("missing required input [value]: ", ref_value_opt.error());
  }
  bool ref_value = (ref_value_opt.value() == "true");
  std::cout << "Required value " << ref_value << std::endl;

  auto value = condition_manager_->get_value(condition_name);

  std::cout << "Current value: " << value << std::endl;

  return ref_value == value ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}