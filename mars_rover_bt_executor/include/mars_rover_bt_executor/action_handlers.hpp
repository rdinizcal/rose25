#ifndef ACTION_NODES
#define ACTION_NODES

#include "behaviortree_cpp_v3/action_node.h"
#include "mars_rover_bt_executor/action_publisher.hpp"
#include "mars_rover_bt_executor/condition_manager.hpp"
#include "geometry_msgs/msg/twist.hpp"

class Move : public BT::SyncActionNode
{
  public:
    Move(const std::string &name, const BT::NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher);

    static BT::PortsList providedPorts ();

    BT::NodeStatus tick() override;

  private:
    std::shared_ptr<ActionPublisher> publisher_;

};

class Stop : public BT::SyncActionNode
{
  public:
    Stop(const std::string &name, const BT::NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher);

    BT::NodeStatus tick() override;

  private:
    std::shared_ptr<ActionPublisher> publisher_;
};

class MoveForward : public BT::SyncActionNode
{
  public:
    MoveForward(const std::string &name, const BT::NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher);

    static BT::PortsList providedPorts ();

    BT::NodeStatus tick() override;

  private:
    std::shared_ptr<ActionPublisher> publisher_;

};

class TurnLeft : public BT::SyncActionNode
{
  public:
    TurnLeft(const std::string &name, const BT::NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher);

    static BT::PortsList providedPorts ();

    BT::NodeStatus tick() override;

  private:
    std::shared_ptr<ActionPublisher> publisher_;
};

class TurnRight : public BT::SyncActionNode
{
  public:
    TurnRight(const std::string &name, const BT::NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher);

    static BT::PortsList providedPorts ();

    BT::NodeStatus tick() override;

  private:
    std::shared_ptr<ActionPublisher> publisher_;
};

class OpenToolArm : public BT::SyncActionNode
{
  public:
    OpenToolArm(const std::string &name, const BT::NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher);

    BT::NodeStatus tick() override;

  private:
    std::shared_ptr<ActionPublisher> publisher_;
};

class CloseToolArm : public BT::SyncActionNode
{
  public:
    CloseToolArm(const std::string &name, const BT::NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher);

    BT::NodeStatus tick() override;

  private:
    std::shared_ptr<ActionPublisher> publisher_;
};

class OpenMast : public BT::SyncActionNode
{
  public:
    OpenMast(const std::string &name, const BT::NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher);

    BT::NodeStatus tick() override;

  private:
    std::shared_ptr<ActionPublisher> publisher_;
};

class CloseMast : public BT::SyncActionNode
{
  public:
    CloseMast(const std::string &name, const BT::NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher);

    BT::NodeStatus tick() override;

  private:
    std::shared_ptr<ActionPublisher> publisher_;
};

class RotateMast : public BT::SyncActionNode
{
  public:
    RotateMast(const std::string &name, const BT::NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher);

    BT::NodeStatus tick() override;

  private:
    std::shared_ptr<ActionPublisher> publisher_;
};

class CheckCondition : public BT::SyncActionNode
{
  public:
    CheckCondition(const std::string &name, const BT::NodeConfiguration &config, std::shared_ptr<ConditionManager> condition_manager);

    static BT::PortsList providedPorts ();

    BT::NodeStatus tick() override;

  private:
    std::shared_ptr<ConditionManager> condition_manager_;

};

#endif