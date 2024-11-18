#ifndef ACTION_PUBLISHER
#define ACTION_PUBLISHER

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/empty.hpp>
#include "mars_rover_srvs/srv/move_service.hpp"

class ActionPublisher
{

public:
    ActionPublisher(std::shared_ptr<rclcpp::Node> node);
    
    void stop();
    void move(double speed);
    void move_forward(double speed);
    void move_backward(double speed);
    void turn_left();
    void turn_right();
    void open_tool_arm();
    void close_tool_arm();
    void rotate_tool_arm();
    void open_mast();
    void close_mast();
    void rotate_mast();
    void drill();
    void trickle();
    void take_picture();

private:
    std::shared_ptr<rclcpp::Node> node_;

    rclcpp::Client<mars_rover_srvs::srv::MoveService>::SharedPtr move_client_;

    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr turn_left_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr turn_right_client_;

    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr move_stop_client_;

    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr open_arm_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr close_arm_client_;

    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr open_mast_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr close_mast_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr rotate_mast_client_;
};

#endif
