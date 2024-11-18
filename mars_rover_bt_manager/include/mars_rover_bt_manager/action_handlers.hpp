#ifndef ACTION_NODES
#define ACTION_NODES

#include "behaviortree_cpp_v3/action_node.h"
#include "mars_rover_bt_manager/action_publisher.hpp"
#include "mars_rover_bt_manager/condition_manager.hpp"

class Move : public BT::StatefulActionNode
{
  public:
    Move(const std::string &name, const BT::NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher, std::shared_ptr<ConditionManager> condition_manager);

    static BT::PortsList providedPorts ();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    std::shared_ptr<ActionPublisher> publisher_;
    std::shared_ptr<ConditionManager> condition_manager_;
    std::future<void> future_;
};

class Stop : public BT::SyncActionNode
{
  public:
    Stop(const std::string &name, const BT::NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher, std::shared_ptr<ConditionManager> condition_manager);

    BT::NodeStatus tick() override;

  private:
    std::shared_ptr<ActionPublisher> publisher_;
    std::shared_ptr<ConditionManager> condition_manager_;
};

class MoveForward : public BT::StatefulActionNode
{
  public:
    MoveForward(const std::string &name, const BT::NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher, std::shared_ptr<ConditionManager> condition_manager);

    static BT::PortsList providedPorts ();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    std::shared_ptr<ActionPublisher> publisher_;
    std::shared_ptr<ConditionManager> condition_manager_;
    std::future<void> future_;

};

class MoveBackward : public BT::StatefulActionNode
{
  public:
    MoveBackward(const std::string &name, const BT::NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher, std::shared_ptr<ConditionManager> condition_manager);

    static BT::PortsList providedPorts ();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    std::shared_ptr<ActionPublisher> publisher_;
    std::shared_ptr<ConditionManager> condition_manager_;
    std::future<void> future_;

};

class TurnLeft : public BT::StatefulActionNode
{
  public:
    TurnLeft(const std::string &name, const BT::NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher, std::shared_ptr<ConditionManager> condition_manager);

    static BT::PortsList providedPorts ();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    std::shared_ptr<ActionPublisher> publisher_;
    std::shared_ptr<ConditionManager> condition_manager_;
    std::future<void> future_;
};

class TurnRight : public BT::StatefulActionNode
{
  public:
    TurnRight(const std::string &name, const BT::NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher, std::shared_ptr<ConditionManager> condition_manager);

    static BT::PortsList providedPorts ();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    std::shared_ptr<ActionPublisher> publisher_;
    std::shared_ptr<ConditionManager> condition_manager_;
    std::future<void> future_;
};

class OpenToolArm : public BT::StatefulActionNode
{
  public:
    OpenToolArm(const std::string &name, const BT::NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher, std::shared_ptr<ConditionManager> condition_manager);

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    std::shared_ptr<ActionPublisher> publisher_;
    std::shared_ptr<ConditionManager> condition_manager_;
    std::future<void> future_;
};

class CloseToolArm : public BT::StatefulActionNode
{
  public:
    CloseToolArm(const std::string &name, const BT::NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher, std::shared_ptr<ConditionManager> condition_manager);

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    std::shared_ptr<ActionPublisher> publisher_;
    std::shared_ptr<ConditionManager> condition_manager_;
    std::future<void> future_;
};

class RotateToolArm : public BT::StatefulActionNode
{
  public:
    RotateToolArm(const std::string &name, const BT::NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher, std::shared_ptr<ConditionManager> condition_manager);

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    std::shared_ptr<ActionPublisher> publisher_;
    std::shared_ptr<ConditionManager> condition_manager_;
    std::future<void> future_;
};

class OpenMast : public BT::StatefulActionNode
{
  public:
    OpenMast(const std::string &name, const BT::NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher, std::shared_ptr<ConditionManager> condition_manager);

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    std::shared_ptr<ActionPublisher> publisher_;
    std::shared_ptr<ConditionManager> condition_manager_;
    std::future<void> future_;
};

class CloseMast : public BT::StatefulActionNode
{
  public:
    CloseMast(const std::string &name, const BT::NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher, std::shared_ptr<ConditionManager> condition_manager);

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    std::shared_ptr<ActionPublisher> publisher_;
    std::shared_ptr<ConditionManager> condition_manager_;
    std::future<void> future_;
};

class RotateMast : public BT::StatefulActionNode
{
  public:
    RotateMast(const std::string &name, const BT::NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher, std::shared_ptr<ConditionManager> condition_manager);

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    std::shared_ptr<ActionPublisher> publisher_;
    std::shared_ptr<ConditionManager> condition_manager_;
    std::future<void> future_;
};

class Drill : public BT::SyncActionNode
{
  public:
    Drill(const std::string &name, const BT::NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher, std::shared_ptr<ConditionManager> condition_manager);

    BT::NodeStatus tick() override;

  private:
    std::shared_ptr<ActionPublisher> publisher_;
    std::shared_ptr<ConditionManager> condition_manager_;
};

class Trickle : public BT::SyncActionNode
{
  public:
    Trickle(const std::string &name, const BT::NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher, std::shared_ptr<ConditionManager> condition_manager);

    BT::NodeStatus tick() override;

  private:
    std::shared_ptr<ActionPublisher> publisher_;
    std::shared_ptr<ConditionManager> condition_manager_;
};

class TakePicture : public BT::SyncActionNode
{
  public:
    TakePicture(const std::string &name, const BT::NodeConfiguration &config, std::shared_ptr<ActionPublisher> publisher, std::shared_ptr<ConditionManager> condition_manager);

    BT::NodeStatus tick() override;

  private:
    std::shared_ptr<ActionPublisher> publisher_;
    std::shared_ptr<ConditionManager> condition_manager_;
};

#endif