#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "mars_rover_bt_executor/action_handlers.hpp"
#include "mars_rover_bt_executor/action_publisher.hpp"
#include "mars_rover_bt_executor/condition_manager.hpp"


int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("mars_rover_bt_executor_node");

    // Getting behavior tree path
    node->declare_parameter<std::string>("bt_path", "");

    std::string xml_file;
    node->get_parameter("bt_path", xml_file);

    if (xml_file.empty())
    {
        RCLCPP_ERROR(node->get_logger(), "No bt_path parameter has been specified.");
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Behavior tree path: %s", xml_file.c_str());

    // Getting robot namespace (if any)
    node->declare_parameter<std::string>("namespace", "");

    std::string robot_namespace;
    node->get_parameter("namespace", robot_namespace);

    RCLCPP_INFO(node->get_logger(), "Robot namespace: %s", robot_namespace.c_str());

    // Instantiating publisher class
    auto publisher = std::make_shared<ActionPublisher>(node);

    // Instantiating condition manager class
    auto condition_manager = std::make_shared<ConditionManager>(node);

    // Configuring behavior tree parser
    BT::BehaviorTreeFactory factory;

    factory.registerBuilder<Move>(
        "Move",
        [publisher](const std::string &name, const BT::NodeConfiguration &config) {
            return std::make_unique<Move>(name, config, publisher);
    });
    factory.registerBuilder<Stop>(
        "Stop",
        [publisher](const std::string &name, const BT::NodeConfiguration &config) {
            return std::make_unique<Stop>(name, config, publisher);
    });
    factory.registerBuilder<MoveForward>(
        "MoveForward",
        [publisher](const std::string &name, const BT::NodeConfiguration &config) {
            return std::make_unique<MoveForward>(name, config, publisher);
    });
    factory.registerBuilder<TurnLeft>(
        "TurnLeft",
        [publisher](const std::string &name, const BT::NodeConfiguration &config) {
            return std::make_unique<TurnLeft>(name, config, publisher);
    });
    factory.registerBuilder<TurnRight>(
        "TurnRight",
        [publisher](const std::string &name, const BT::NodeConfiguration &config) {
            return std::make_unique<TurnRight>(name, config, publisher);
    });
    factory.registerBuilder<OpenToolArm>(
        "OpenToolArm",
        [publisher](const std::string &name, const BT::NodeConfiguration &config) {
            return std::make_unique<OpenToolArm>(name, config, publisher);
    });
    factory.registerBuilder<CloseToolArm>(
        "CloseToolArm",
        [publisher](const std::string &name, const BT::NodeConfiguration &config) {
            return std::make_unique<CloseToolArm>(name, config, publisher);
    });
    factory.registerBuilder<OpenMast>(
        "OpenMast",
        [publisher](const std::string &name, const BT::NodeConfiguration &config) {
            return std::make_unique<OpenMast>(name, config, publisher);
    });
    factory.registerBuilder<CloseMast>(
        "CloseMast",
        [publisher](const std::string &name, const BT::NodeConfiguration &config) {
            return std::make_unique<CloseMast>(name, config, publisher);
    });
    factory.registerBuilder<RotateMast>(
        "RotateMast",
        [publisher](const std::string &name, const BT::NodeConfiguration &config) {
            return std::make_unique<RotateMast>(name, config, publisher);
    });

    factory.registerBuilder<CheckCondition>(
        "CheckCondition",
        [condition_manager](const std::string &name, const BT::NodeConfiguration &config) {
            return std::make_unique<CheckCondition>(name, config, condition_manager);
    });

    auto tree = factory.createTreeFromFile(xml_file);

    tree.tickRootWhileRunning();
  
    // Shutdown the node when finished
    rclcpp::shutdown();

    return 0;
}