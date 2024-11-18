#ifndef CONDITION_NODES
#define CONDITION_NODES

#include "behaviortree_cpp_v3/condition_node.h"
#include "mars_rover_bt_manager/condition_manager.hpp"

class IsInLocation : public BT::ConditionNode
{
  public:
    IsInLocation(const std::string &name, const BT::NodeConfiguration &config, std::shared_ptr<ConditionManager> condition_manager);

    BT::NodeStatus tick() override;

  private:
    std::shared_ptr<ConditionManager> condition_manager_;
};

class IsArmInPosition : public BT::ConditionNode
{
  public:
    IsArmInPosition(const std::string &name, const BT::NodeConfiguration &config, std::shared_ptr<ConditionManager> condition_manager);

    static BT::PortsList providedPorts ();

    BT::NodeStatus tick() override;

  private:
    std::shared_ptr<ConditionManager> condition_manager_;
};

class IsPictureTaken : public BT::ConditionNode
{
  public:
    IsPictureTaken(const std::string &name, const BT::NodeConfiguration &config, std::shared_ptr<ConditionManager> condition_manager);

    BT::NodeStatus tick() override;

  private:
    std::shared_ptr<ConditionManager> condition_manager_;
};

class IsDigged : public BT::ConditionNode
{
  public:
    IsDigged(const std::string &name, const BT::NodeConfiguration &config, std::shared_ptr<ConditionManager> condition_manager);

    BT::NodeStatus tick() override;

  private:
    std::shared_ptr<ConditionManager> condition_manager_;
};

class IsContainerEmpty : public BT::ConditionNode
{
  public:
    IsContainerEmpty(const std::string &name, const BT::NodeConfiguration &config, std::shared_ptr<ConditionManager> condition_manager);

    BT::NodeStatus tick() override;

  private:
    std::shared_ptr<ConditionManager> condition_manager_;
};

class IsTowardObject : public BT::ConditionNode
{
  public:
    IsTowardObject(const std::string &name, const BT::NodeConfiguration &config, std::shared_ptr<ConditionManager> condition_manager);

    BT::NodeStatus tick() override;

  private:
    std::shared_ptr<ConditionManager> condition_manager_;
};

class IsRockInLeft : public BT::ConditionNode
{
  public:
    IsRockInLeft(const std::string &name, const BT::NodeConfiguration &config, std::shared_ptr<ConditionManager> condition_manager);

    BT::NodeStatus tick() override;

  private:
    std::shared_ptr<ConditionManager> condition_manager_;
};

class IsRockInRight : public BT::ConditionNode
{
  public:
    IsRockInRight(const std::string &name, const BT::NodeConfiguration &config, std::shared_ptr<ConditionManager> condition_manager);

    BT::NodeStatus tick() override;

  private:
    std::shared_ptr<ConditionManager> condition_manager_;
};

class OnDust : public BT::ConditionNode
{
  public:
    OnDust(const std::string &name, const BT::NodeConfiguration &config, std::shared_ptr<ConditionManager> condition_manager);

    BT::NodeStatus tick() override;

  private:
    std::shared_ptr<ConditionManager> condition_manager_;
};

class OnRocks : public BT::ConditionNode
{
  public:
    OnRocks(const std::string &name, const BT::NodeConfiguration &config, std::shared_ptr<ConditionManager> condition_manager);

    BT::NodeStatus tick() override;

  private:
    std::shared_ptr<ConditionManager> condition_manager_;
};

#endif
