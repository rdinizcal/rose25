#include "rclcpp/rclcpp.hpp"
#include "mars_rover_bt_manager/action_handlers.hpp"
#include <chrono>
#include <future>
#include <cmath>

using namespace BT;

// Move
Move::Move(const std::string &name, const NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher, std::shared_ptr<ConditionManager> condition_manager)
   :  StatefulActionNode(name, config), publisher_(publisher), condition_manager_(condition_manager)
{}

PortsList Move::providedPorts ()
{
  return {InputPort<std::string>("time"), InputPort<std::string>("speed")};
}

NodeStatus Move::onStart()
{
  std::cout << "Starting Move action" << std::endl;

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

  future_ = std::async(std::launch::async, [this, time, speed]() {
    auto converted_time = std::chrono::nanoseconds(static_cast<int64_t>(time * 1e9));
    publisher_->move(speed);
    std::cout << "This thread is now waiting for the action to be completed before stopping" << std::endl;
    std::this_thread::sleep_for(std::chrono::nanoseconds(converted_time));
    publisher_->stop();
    std::cout << "Move action ended" << std::endl;
  });

  return NodeStatus::RUNNING;
}

NodeStatus Move::onRunning()
{
  if (future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
  {
    return NodeStatus::RUNNING;
  }
  else
  {
    std::cout << "Move action returning success" << std::endl;
    condition_manager_->set_value("IsInLocation", true);
    return NodeStatus::SUCCESS;
  }
}

void Move::onHalted()
{
  publisher_->stop();
  std::cout << "Move action stopped" << std::endl;
}

// Stop
Stop::Stop(const std::string &name, const NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher, std::shared_ptr<ConditionManager> condition_manager) :
    SyncActionNode(name, config), publisher_(publisher), condition_manager_(condition_manager)
{}

NodeStatus Stop::tick()
{
  std::cout << "Executing Stop action" << std::endl;

  return NodeStatus::SUCCESS;
}

// Move Forward
MoveForward::MoveForward(const std::string &name, const NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher, std::shared_ptr<ConditionManager> condition_manager)
   :  StatefulActionNode(name, config), publisher_(publisher), condition_manager_(condition_manager)
{}

PortsList MoveForward::providedPorts ()
{
  return {InputPort<std::string>("time"), InputPort<std::string>("speed")};
}

NodeStatus MoveForward::onStart()
{
  std::cout << "Starting MoveForward action" << std::endl;

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

  future_ = std::async(std::launch::async, [this, time, speed]() {
    auto converted_time = std::chrono::nanoseconds(static_cast<int64_t>(time * 1e9));
    publisher_->move_forward(speed);
    std::cout << "This thread is now waiting for the action to be completed before stopping" << std::endl;
    std::this_thread::sleep_for(std::chrono::nanoseconds(converted_time));
    publisher_->stop();
  });

  return NodeStatus::RUNNING;
}

NodeStatus MoveForward::onRunning()
{
  if (future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
  {
    return NodeStatus::RUNNING;
  }
  else
  {
    std::cout << "MoveForward action returning success" << std::endl;
    condition_manager_->set_value("IsInLocation", true);
    return NodeStatus::SUCCESS;
  }
}

void MoveForward::onHalted()
{
  publisher_->stop();
  std::cout << "MoveForward action stopped" << std::endl;
}

// Move Backward
MoveBackward::MoveBackward(const std::string &name, const NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher, std::shared_ptr<ConditionManager> condition_manager)
   :  StatefulActionNode(name, config), publisher_(publisher), condition_manager_(condition_manager)
{}

PortsList MoveBackward::providedPorts ()
{
  return {InputPort<std::string>("time"), InputPort<std::string>("speed")};
}

NodeStatus MoveBackward::onStart()
{
  std::cout << "Starting MoveBackward action" << std::endl;

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

  future_ = std::async(std::launch::async, [this, time, speed]() {
    auto converted_time = std::chrono::nanoseconds(static_cast<int64_t>(time * 1e9));
    publisher_->move_backward(speed);
    std::cout << "This thread is now waiting for the action to be completed before stopping" << std::endl;
    std::this_thread::sleep_for(std::chrono::nanoseconds(converted_time));
    publisher_->stop();
  });

  return NodeStatus::RUNNING;
}

NodeStatus MoveBackward::onRunning()
{
  if (future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
  {
    return NodeStatus::RUNNING;
  }
  else
  {
    std::cout << "MoveBackward action returning success" << std::endl;
    condition_manager_->set_value("IsInLocation", true);
    return NodeStatus::SUCCESS;
  }
}

void MoveBackward::onHalted()
{
  publisher_->stop();
  std::cout << "MoveBackward action stopped" << std::endl;
}

// TurnLeft
TurnLeft::TurnLeft(const std::string &name, const NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher, std::shared_ptr<ConditionManager> condition_manager) :
    StatefulActionNode(name, config), publisher_(publisher), condition_manager_(condition_manager)
{}

PortsList TurnLeft::providedPorts ()
{
  return {InputPort<std::string>("angle")};
}

NodeStatus TurnLeft::onStart()
{
  std::cout << "Executing TurnLeft action" << std::endl;

  Optional<std::string> angle_msg = getInput<std::string>("angle");
  if (!angle_msg)
  {
    throw RuntimeError("missing required input [angle]: ", angle_msg.error() );
  }

  double angle = std::stod(angle_msg.value());
  std::cout << "Turning left of " << angle << " degrees" << std::endl;

  // Compute rotation time
  double radiant_angle = angle * M_PI / 180.0;
  double duration = radiant_angle / 0.4;
  std::cout << "Movement duration: " << duration << std::endl;

  future_ = std::async(std::launch::async, [this, duration]() {
    auto converted_time = std::chrono::nanoseconds(static_cast<int64_t>(duration * 1e9));
    publisher_->turn_left();
    std::cout << "This thread is now waiting for the action to be completed before stopping" << std::endl;
    std::this_thread::sleep_for(std::chrono::nanoseconds(converted_time));
    publisher_->stop();
  });

  return NodeStatus::RUNNING;
}

NodeStatus TurnLeft::onRunning()
{
  if (future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
  {
    return NodeStatus::RUNNING;
  }
  else
  {
    std::cout << "TurnLeft action returning success" << std::endl;
    condition_manager_->negate_value("IsTowardObject");
    return NodeStatus::SUCCESS;
  }
}

void TurnLeft::onHalted()
{
  publisher_->stop();
  std::cout << "TurnLeft action stopped" << std::endl;
}

// TurnRight
TurnRight::TurnRight(const std::string &name, const NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher, std::shared_ptr<ConditionManager> condition_manager) :
    StatefulActionNode(name, config), publisher_(publisher), condition_manager_(condition_manager)
{}

PortsList TurnRight::providedPorts ()
{
  return {InputPort<std::string>("angle")};
}

NodeStatus TurnRight::onStart()
{
  std::cout << "Executing TurnRight action" << std::endl;

  Optional<std::string> angle_msg = getInput<std::string>("angle");
  if (!angle_msg)
  {
    throw RuntimeError("missing required input [angle]: ", angle_msg.error() );
  }

  double angle = std::stod(angle_msg.value());
  std::cout << "Turning right of " << angle << " degrees" << std::endl;

    // Compute rotation time
  double radiant_angle = angle * M_PI / 180.0;
  double duration = radiant_angle / 0.4;
  std::cout << "Movement duration: " << duration << std::endl;

  future_ = std::async(std::launch::async, [this, duration]() {
    auto converted_time = std::chrono::nanoseconds(static_cast<int64_t>(duration * 1e9));
    publisher_->turn_right();
    std::cout << "This thread is now waiting for the action to be completed before stopping" << std::endl;
    std::this_thread::sleep_for(std::chrono::nanoseconds(converted_time));
    publisher_->stop();
  });

  return NodeStatus::RUNNING;
}

NodeStatus TurnRight::onRunning()
{
  if (future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
  {
    return NodeStatus::RUNNING;
  }
  else
  {
    std::cout << "TurnRight action returning success" << std::endl;
    condition_manager_->negate_value("IsTowardObject");
    return NodeStatus::SUCCESS;
  }
}

void TurnRight::onHalted()
{
  publisher_->stop();
  std::cout << "TurnRight action stopped" << std::endl;
}

// OpenToolArm
OpenToolArm::OpenToolArm(const std::string &name, const NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher, std::shared_ptr<ConditionManager> condition_manager) :
    StatefulActionNode(name, config), publisher_(publisher), condition_manager_(condition_manager)
{}

NodeStatus OpenToolArm::onStart()
{
  std::cout << "Starting OpenToolArm action" << std::endl;

  future_ = std::async(std::launch::async, [this]() {
    publisher_->open_tool_arm();
    std::cout << "Waiting for the action to be completed before stopping" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(4));
    publisher_->stop();
  });

  return NodeStatus::RUNNING;
}

NodeStatus OpenToolArm::onRunning()
{
  if (future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
  {
    return NodeStatus::RUNNING;
  }
  else
  {
    std::cout << "OpenToolArm action returning success" << std::endl;
    condition_manager_->set_value("IsInLocation(taking_picture)", false);
    condition_manager_->set_value("IsInLocation(drilling)", false);
    return NodeStatus::SUCCESS;
  }
}

void OpenToolArm::onHalted()
{
  publisher_->stop();
  std::cout << "OpenToolArm action stopped" << std::endl;
}

// CloseToolArm
CloseToolArm::CloseToolArm(const std::string &name, const NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher, std::shared_ptr<ConditionManager> condition_manager) :
    StatefulActionNode(name, config), publisher_(publisher), condition_manager_(condition_manager)
{}

NodeStatus CloseToolArm::onStart()
{
  std::cout << "Executing CloseToolArm action" << std::endl;

  future_ = std::async(std::launch::async, [this]() {
    publisher_->close_tool_arm();
    std::cout << "Waiting for the action to be completed before stopping" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(4));
    publisher_->stop();
  });

  return NodeStatus::RUNNING;
}

NodeStatus CloseToolArm::onRunning()
{
  if (future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
  {
    return NodeStatus::RUNNING;
  }
  else
  {
    std::cout << "CloseToolArm action returning success" << std::endl;
    condition_manager_->set_value("IsInLocation(taking_picture)", false);
    condition_manager_->set_value("IsInLocation(drilling)", false);
    return NodeStatus::SUCCESS;
  }
}

void CloseToolArm::onHalted()
{
  publisher_->stop();
  std::cout << "CloseToolArm action stopped" << std::endl;
}

// RotateToolArm
RotateToolArm::RotateToolArm(const std::string &name, const NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher, std::shared_ptr<ConditionManager> condition_manager) :
    StatefulActionNode(name, config), publisher_(publisher), condition_manager_(condition_manager)
{}

NodeStatus RotateToolArm::onStart()
{
  std::cout << "Executing RotateToolArm action" << std::endl;

  future_ = std::async(std::launch::async, [this]() {
    publisher_->close_tool_arm();
    std::cout << "Waiting for the action to be completed before stopping" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(4));
    publisher_->stop();
  });

  return NodeStatus::RUNNING;
}

NodeStatus RotateToolArm::onRunning()
{
  if (future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
  {
    return NodeStatus::RUNNING;
  }
  else
  {
    std::cout << "RotateToolArm action returning success" << std::endl;
    condition_manager_->negate_value("IsInLocation(taking_picture)");
    condition_manager_->negate_value("IsInLocation(drilling)");
    return NodeStatus::SUCCESS;
  }
}

void RotateToolArm::onHalted()
{
  publisher_->stop();
  std::cout << "RotateToolArm action stopped" << std::endl;
}

// OpenMast
OpenMast::OpenMast(const std::string &name, const NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher, std::shared_ptr<ConditionManager> condition_manager) :
    StatefulActionNode(name, config), publisher_(publisher), condition_manager_(condition_manager)
{}

NodeStatus OpenMast::onStart()
{
  std::cout << "Executing OpenMast action" << std::endl;

  future_ = std::async(std::launch::async, [this]() {
    publisher_->open_mast();
    std::cout << "Waiting for the action to be completed before stopping" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    publisher_->stop();
  });

  return NodeStatus::RUNNING;
}

NodeStatus OpenMast::onRunning()
{
  if (future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
  {    
    return NodeStatus::RUNNING;
  }
  else
  {
    std::cout << "OpenMast action returning success" << std::endl;
    condition_manager_->set_value("IsMastOpen", true);
    return NodeStatus::SUCCESS;
  }
}

void OpenMast::onHalted()
{
  publisher_->stop();
  std::cout << "OpenMast action stopped" << std::endl;
}

// CloseMast
CloseMast::CloseMast(const std::string &name, const NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher, std::shared_ptr<ConditionManager> condition_manager) :
    StatefulActionNode(name, config), publisher_(publisher), condition_manager_(condition_manager)
{}

NodeStatus CloseMast::onStart()
{
  std::cout << "Executing CloseMast action" << std::endl;

  future_ = std::async(std::launch::async, [this]() {
    publisher_->close_mast();
    std::cout << "Waiting for the action to be completed before stopping" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    publisher_->stop();
  });

  return NodeStatus::RUNNING;
}

NodeStatus CloseMast::onRunning()
{
  if (future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
  {
    return NodeStatus::RUNNING;
  }
  else
  {
    std::cout << "CloseMast action returning success" << std::endl;
    condition_manager_->set_value("IsMastOpen", false);
    return NodeStatus::SUCCESS;
  }
}

void CloseMast::onHalted()
{
  publisher_->stop();
  std::cout << "CloseMast action stopped" << std::endl;
}

// RotateMast
RotateMast::RotateMast(const std::string &name, const NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher, std::shared_ptr<ConditionManager> condition_manager) :
    StatefulActionNode(name, config), publisher_(publisher), condition_manager_(condition_manager)
{}

NodeStatus RotateMast::onStart()
{
  std::cout << "Executing RotateMast action" << std::endl;

  future_ = std::async(std::launch::async, [this]() {
    publisher_->rotate_mast();
    std::cout << "Waiting for the action to be completed before stopping" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(12));
    publisher_->stop();
  });

  return NodeStatus::RUNNING;
}

NodeStatus RotateMast::onRunning()
{
  if (future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
  {
    return NodeStatus::RUNNING;
  }
  else
  {
    std::cout << "RotateMast action returning success" << std::endl;
    return NodeStatus::SUCCESS;
  }
}

void RotateMast::onHalted()
{
  publisher_->stop();
  std::cout << "RotateMast action stopped" << std::endl;
}

// Drill
Drill::Drill(const std::string &name, const NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher, std::shared_ptr<ConditionManager> condition_manager) :
    SyncActionNode(name, config), publisher_(publisher), condition_manager_(condition_manager)
{}

NodeStatus Drill::tick()
{
  std::cout << "Executing Drill action" << std::endl;
  publisher_->drill();
  condition_manager_->set_value("IsDigged", true);
  return NodeStatus::SUCCESS;
}

// Trickle
Trickle::Trickle(const std::string &name, const NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher, std::shared_ptr<ConditionManager> condition_manager) :
    SyncActionNode(name, config), publisher_(publisher), condition_manager_(condition_manager)
{}

NodeStatus Trickle::tick()
{
  std::cout << "Executing Trickle action" << std::endl;
  publisher_->trickle();
  condition_manager_->set_value("IsContainerEmpty", true);
  return NodeStatus::SUCCESS;
}

// TakePicture
TakePicture::TakePicture(const std::string &name, const NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher, std::shared_ptr<ConditionManager> condition_manager) :
    SyncActionNode(name, config), publisher_(publisher), condition_manager_(condition_manager)
{}

NodeStatus TakePicture::tick()
{
  std::cout << "Executing TakePicture action" << std::endl;
  publisher_->take_picture();
  condition_manager_->set_value("IsPictureTaken", true);
  return NodeStatus::SUCCESS;
}
