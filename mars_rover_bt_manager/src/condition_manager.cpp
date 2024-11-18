#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "mars_rover_bt_manager/condition_manager.hpp"
#include <map>
#include <nlohmann/json.hpp>

ConditionManager::ConditionManager(std::shared_ptr<rclcpp::Node> node)
    : node_(node)
{
    condition_update_subscription_ = node_->create_subscription<std_msgs::msg::String>("condition_update", 10, std::bind(&ConditionManager::condition_update_callback, this, std::placeholders::_1));
}

bool ConditionManager::get_value(std::string condition_name)
{
    auto search = condition_dictionary_.find(condition_name);
    return search != condition_dictionary_.end() ? search->second : false;
}

void ConditionManager::set_value(std::string condition_name, bool value)
{
    condition_dictionary_[condition_name] = value;
}

void ConditionManager::negate_value(std::string condition_name)
{
    auto entry = condition_dictionary_.find(condition_name);
    if (entry != condition_dictionary_.end()) {
        entry->second = !entry->second;
    }
}

void ConditionManager::condition_update_callback(const std_msgs::msg::String &msg)
{
    try {
        auto json_data = nlohmann::json::parse(msg.data);
        std::string condition_name = json_data.at("condition");
        bool value = json_data.at("value");

        set_value(condition_name, value);

        RCLCPP_INFO(node_->get_logger(), "Updated condition: %s to %s", condition_name.c_str(), value ? "true" : "false");

    } catch (const nlohmann::json::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to parse JSON: %s", e.what());
    }
}