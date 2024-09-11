#ifndef CONDITION_MANAGER
#define CONDITION_MANAGER

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <map>

class ConditionManager
{

public:
    ConditionManager(std::shared_ptr<rclcpp::Node> node);
    bool get_value(std::string condition_name);

private:
    void battery_condition_update_callback(const std_msgs::msg::Bool &msg);

    std::map<std::string, bool> condition_dictionary_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr battery_condition_update_subscription_;

    std::shared_ptr<rclcpp::Node> node_;
};

#endif