#ifndef CONDITION_MANAGER
#define CONDITION_MANAGER

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <map>

class ConditionManager
{

public:
    ConditionManager(std::shared_ptr<rclcpp::Node> node);
    bool get_value(std::string condition_name);
    void set_value(std::string condition_name, bool value);
    void negate_value(std::string condition_name);

private:
    void condition_update_callback(const std_msgs::msg::String &msg);

    std::map<std::string, bool> condition_dictionary_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr condition_update_subscription_;

    std::shared_ptr<rclcpp::Node> node_;
};

#endif