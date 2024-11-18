#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "mars_rover_bt_manager/action_handlers.hpp"
#include "mars_rover_bt_manager/action_publisher.hpp"
#include "mars_rover_bt_manager/condition_handlers.hpp"
#include "mars_rover_bt_manager/condition_manager.hpp"


class BtManager: public rclcpp::Node {
    public:
        BtManager() : Node("mars_rover_bt_manager_node") {
        
            RCLCPP_INFO(this->get_logger(), "Starting BT Manager...");
            subscriber_ = this->create_subscription<std_msgs::msg::String>("behavior_tree", 10, std::bind(&BtManager::update_tree_callback, this, std::placeholders::_1));

            // Getting robot namespace (if any)
            this->declare_parameter<std::string>("namespace", "");
            std::string robot_namespace;
            this->get_parameter("namespace", robot_namespace);
            RCLCPP_INFO(this->get_logger(), "Robot namespace: %s", robot_namespace.c_str());
        }

        ~BtManager()
        {
            // Stop tree on destroy
            stop_tree_ = true;
            if (executor_ && executor_->joinable())
            {
                executor_->join();
            }
        }

        void init_bt_manager()
        {            
            // Instantiating action publisher class
            auto action_publisher = std::make_shared<ActionPublisher>(shared_from_this());

            // Instantiating condition manager class
            auto condition_manager = std::make_shared<ConditionManager>(shared_from_this());

            // Registering action nodes
            factory_ = std::make_shared<BT::BehaviorTreeFactory>();

            factory_->registerBuilder<Move>(
                "Move",
                [action_publisher, condition_manager](const std::string &name, const BT::NodeConfiguration &config) {
                    return std::make_unique<Move>(name, config, action_publisher, condition_manager);
            });
            factory_->registerBuilder<Stop>(
                "Stop",
                [action_publisher, condition_manager](const std::string &name, const BT::NodeConfiguration &config) {
                    return std::make_unique<Stop>(name, config, action_publisher, condition_manager);
            });
            factory_->registerBuilder<MoveForward>(
                "MoveForward",
                [action_publisher, condition_manager](const std::string &name, const BT::NodeConfiguration &config) {
                    return std::make_unique<MoveForward>(name, config, action_publisher, condition_manager);
            });
            factory_->registerBuilder<MoveBackward>(
                "MoveBackward",
                [action_publisher, condition_manager](const std::string &name, const BT::NodeConfiguration &config) {
                    return std::make_unique<MoveForward>(name, config, action_publisher, condition_manager);
            });
            factory_->registerBuilder<TurnLeft>(
                "TurnLeft",
                [action_publisher, condition_manager](const std::string &name, const BT::NodeConfiguration &config) {
                    return std::make_unique<TurnLeft>(name, config, action_publisher, condition_manager);
            });
            factory_->registerBuilder<TurnRight>(
                "TurnRight",
                [action_publisher, condition_manager](const std::string &name, const BT::NodeConfiguration &config) {
                    return std::make_unique<TurnRight>(name, config, action_publisher, condition_manager);
            });
            factory_->registerBuilder<OpenToolArm>(
                "OpenToolArm",
                [action_publisher, condition_manager](const std::string &name, const BT::NodeConfiguration &config) {
                    return std::make_unique<OpenToolArm>(name, config, action_publisher, condition_manager);
            });
            factory_->registerBuilder<CloseToolArm>(
                "CloseToolArm",
                [action_publisher, condition_manager](const std::string &name, const BT::NodeConfiguration &config) {
                    return std::make_unique<CloseToolArm>(name, config, action_publisher, condition_manager);
            });
            factory_->registerBuilder<RotateToolArm>(
                "RotateToolArm",
                [action_publisher, condition_manager](const std::string &name, const BT::NodeConfiguration &config) {
                    return std::make_unique<RotateToolArm>(name, config, action_publisher, condition_manager);
            });
            factory_->registerBuilder<OpenMast>(
                "OpenMast",
                [action_publisher, condition_manager](const std::string &name, const BT::NodeConfiguration &config) {
                    return std::make_unique<OpenMast>(name, config, action_publisher, condition_manager);
            });
            factory_->registerBuilder<CloseMast>(
                "CloseMast",
                [action_publisher, condition_manager](const std::string &name, const BT::NodeConfiguration &config) {
                    return std::make_unique<CloseMast>(name, config, action_publisher, condition_manager);
            });
            factory_->registerBuilder<RotateMast>(
                "RotateMast",
                [action_publisher, condition_manager](const std::string &name, const BT::NodeConfiguration &config) {
                    return std::make_unique<RotateMast>(name, config, action_publisher, condition_manager);
            });
            factory_->registerBuilder<Drill>(
                "Drill",
                [action_publisher, condition_manager](const std::string &name, const BT::NodeConfiguration &config) {
                    return std::make_unique<Drill>(name, config, action_publisher, condition_manager);
            });
            factory_->registerBuilder<Trickle>(
                "Trickle",
                [action_publisher, condition_manager](const std::string &name, const BT::NodeConfiguration &config) {
                    return std::make_unique<Trickle>(name, config, action_publisher, condition_manager);
            });
            factory_->registerBuilder<TakePicture>(
                "TakePicture",
                [action_publisher, condition_manager](const std::string &name, const BT::NodeConfiguration &config) {
                    return std::make_unique<TakePicture>(name, config, action_publisher, condition_manager);
            });

            // Registering condition nodes
            factory_->registerBuilder<IsInLocation>(
                "IsInLocation",
                [condition_manager](const std::string &name, const BT::NodeConfiguration &config) {
                    return std::make_unique<IsInLocation>(name, config, condition_manager);
            });
            factory_->registerBuilder<IsArmInPosition>(
                "IsArmInPosition",
                [condition_manager](const std::string &name, const BT::NodeConfiguration &config) {
                    return std::make_unique<IsArmInPosition>(name, config, condition_manager);
            });
            factory_->registerBuilder<IsPictureTaken>(
                "IsPictureTaken",
                [condition_manager](const std::string &name, const BT::NodeConfiguration &config) {
                    return std::make_unique<IsPictureTaken>(name, config, condition_manager);
            });
            factory_->registerBuilder<IsDigged>(
                "IsDigged",
                [condition_manager](const std::string &name, const BT::NodeConfiguration &config) {
                    return std::make_unique<IsDigged>(name, config, condition_manager);
            });
            factory_->registerBuilder<IsContainerEmpty>(
                "IsContainerEmpty",
                [condition_manager](const std::string &name, const BT::NodeConfiguration &config) {
                    return std::make_unique<IsContainerEmpty>(name, config, condition_manager);
            });
            factory_->registerBuilder<IsTowardObject>(
                "IsTowardObject",
                [condition_manager](const std::string &name, const BT::NodeConfiguration &config) {
                    return std::make_unique<IsTowardObject>(name, config, condition_manager);
            });
            factory_->registerBuilder<IsRockInLeft>(
                "IsRockInLeft",
                [condition_manager](const std::string &name, const BT::NodeConfiguration &config) {
                    return std::make_unique<IsRockInLeft>(name, config, condition_manager);
            });
            factory_->registerBuilder<IsRockInRight>(
                "IsRockInRight",
                [condition_manager](const std::string &name, const BT::NodeConfiguration &config) {
                    return std::make_unique<IsRockInRight>(name, config, condition_manager);
            });
            factory_->registerBuilder<OnDust>(
                "OnDust",
                [condition_manager](const std::string &name, const BT::NodeConfiguration &config) {
                    return std::make_unique<OnDust>(name, config, condition_manager);
            });
            factory_->registerBuilder<OnRocks>(
                "OnRocks",
                [condition_manager](const std::string &name, const BT::NodeConfiguration &config) {
                    return std::make_unique<OnRocks>(name, config, condition_manager);
            });

            // Getting XMl and running it if provided
            this->declare_parameter<std::string>("bt_path", "");
            std::string xml_file;
            this->get_parameter("bt_path", xml_file);

            if (xml_file.empty())
            {
                RCLCPP_INFO(this->get_logger(), "No bt_path parameter has been specified: not running any tree");
            } else {
                RCLCPP_INFO(this->get_logger(), "Behavior tree path: %s", xml_file.c_str());

                tree_ = factory_->createTreeFromFile(xml_file);

                // Run tree executor
                executor_ = std::make_unique<std::thread>(&BtManager::executeTree, this);
            }
        }

    private:
        void executeTree()
        {
            while (rclcpp::ok() && !stop_tree_)
            {
                auto result = tree_.tickRoot();
                if (result == BT::NodeStatus::SUCCESS || result == BT::NodeStatus::FAILURE)
                {
                    RCLCPP_INFO(this->get_logger(), "Behavior Tree execution completed: %s", toStr(result).c_str());
                    break;
                }
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }

        void update_tree_callback(std_msgs::msg::String::SharedPtr msg)
        {
            std::cout << "Received a new version of the tree: stopping the current one and running the new one" << std::endl;

            // Stop current tree
            stop_tree_ = true;
            if (executor_ && executor_->joinable())
            {
                executor_->join();
            }
            tree_ = factory_->createTreeFromText(msg->data);

            // Start again current tree
            stop_tree_ = false;
            executor_ = std::make_unique<std::thread>(&BtManager::executeTree, this);
        }

        BT::Tree tree_;
        std::shared_ptr<BT::BehaviorTreeFactory> factory_;
        std::unique_ptr<std::thread> executor_;
        std::atomic<bool> stop_tree_ = false;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto bt_manager = std::make_shared<BtManager>();
    bt_manager->init_bt_manager();
    rclcpp::spin(bt_manager);
    rclcpp::shutdown();
    return 0;
}