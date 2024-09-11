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
    void move(double speed, double duration);
    void move_forward(double duration);
    void turn_left(double duration);
    void turn_right(double duration);
    void open_tool_arm();
    void close_tool_arm();
    void open_mast();
    void close_mast();
    void rotate_mast();

private:
    void wait_then_stop(double duration);

    std::shared_ptr<rclcpp::Node> node_;

    rclcpp::Client<mars_rover_srvs::srv::MoveService>::SharedPtr move_client_;

    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr move_forward_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr turn_left_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr turn_right_client_;

    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr move_stop_client_;

    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr open_arm_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr close_arm_client_;

    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr open_mast_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr close_mast_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr rotate_mast_client_;

    static const int arm_movement_time_ = 4;
    static const int mast_closing_opening_time_ = 1;
    static const int mast_rotation_time_ = 12;

    const double z_angular_speed_ = 0.4;
};

#endif
