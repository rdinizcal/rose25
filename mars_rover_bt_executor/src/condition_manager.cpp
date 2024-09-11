#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include "mars_rover_bt_executor/condition_manager.hpp"
#include <map>

ConditionManager::ConditionManager(std::shared_ptr<rclcpp::Node> node)
    : node_(node)
{
    battery_condition_update_subscription_ = node_->create_subscription<std_msgs::msg::Bool>("/copilot/handlerCopilot", 10, std::bind(&ConditionManager::battery_condition_update_callback, this, std::placeholders::_1));
}

bool ConditionManager::get_value(std::string condition_name)
{
    auto search = condition_dictionary_.find(condition_name);
    return search != condition_dictionary_.end() ? search->second : false;
}

void ConditionManager::battery_condition_update_callback(const std_msgs::msg::Bool &msg)
{
    auto value = msg.data;
    RCLCPP_INFO(node_->get_logger(), "low_battery condition updated. New value: %d", value);
    condition_dictionary_["low_battery"];
}