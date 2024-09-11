#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/empty.hpp>
#include "mars_rover_bt_executor/action_publisher.hpp"
#include "mars_rover_srvs/srv/move_service.hpp"
#include <cmath>

ActionPublisher::ActionPublisher(std::shared_ptr<rclcpp::Node> node) :
    node_(node)
{
    move_client_ = node_->create_client<mars_rover_srvs::srv::MoveService>("/move");

    move_forward_client_ = node_->create_client<std_srvs::srv::Empty>("/move_forward");
    turn_left_client_ = node_->create_client<std_srvs::srv::Empty>("/turn_left");
    turn_right_client_ = node_->create_client<std_srvs::srv::Empty>("/turn_right");

    move_stop_client_ = node_->create_client<std_srvs::srv::Empty>("/move_stop");

    open_arm_client_ = node_->create_client<std_srvs::srv::Empty>("/open_arm");
    close_arm_client_ = node_->create_client<std_srvs::srv::Empty>("/close_arm");

    open_mast_client_ = node_->create_client<std_srvs::srv::Empty>("/mast_open");
    close_mast_client_ = node_->create_client<std_srvs::srv::Empty>("/mast_close");
    rotate_mast_client_ = node_->create_client<std_srvs::srv::Empty>("/mast_rotate");
}

void ActionPublisher::stop()
{
    RCLCPP_INFO(node_->get_logger(), "Calling /move_stop service");
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();

    auto future = move_stop_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node_->get_logger(), "Service call compete");
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Service call error");
    }
}

void ActionPublisher::move(double speed, double duration)
{
    RCLCPP_INFO(node_->get_logger(), "Calling /move service");
    auto request = std::make_shared<mars_rover_srvs::srv::MoveService::Request>();
    request->twist.linear.x = speed;

    auto future = move_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node_->get_logger(), "Service call compete");
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Service call error");
    }

    this->wait_then_stop(duration);
}

void ActionPublisher::move_forward(double duration)
{
    RCLCPP_INFO(node_->get_logger(), "Calling /move_forward service to move forward");
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();

    auto future = move_forward_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node_->get_logger(), "Service call compete");
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Service call error");
    }

    this->wait_then_stop(duration);
}

void ActionPublisher::turn_left(double angle)
{
    RCLCPP_INFO(node_->get_logger(), "Calling /turn_left service");
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();

    auto future = turn_left_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node_->get_logger(), "Service call compete");
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Service call error");
    }

    // Compute rotation time
    double radiant_angle = angle * M_PI / 180.0;
    double duration = radiant_angle / z_angular_speed_;
    RCLCPP_INFO(node_->get_logger(), "Movement duration: %f", duration);

    this->wait_then_stop(duration);
}

void ActionPublisher::turn_right(double angle)
{
    RCLCPP_INFO(node_->get_logger(), "Calling /turn_right service");
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();

    auto future = turn_right_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node_->get_logger(), "Service call compete");
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Service call error");
    }

    // Compute rotation time
    double radiant_angle = angle * M_PI / 180.0;
    double duration = radiant_angle / z_angular_speed_;
    RCLCPP_INFO(node_->get_logger(), "Movement duration: %f", duration);

    this->wait_then_stop(duration);
}

void ActionPublisher::open_tool_arm()
{
    RCLCPP_INFO(node_->get_logger(), "Calling /open_arm service");
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();

    auto future = move_stop_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node_->get_logger(), "Service call compete");
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Service call error");
    }

    this->wait_then_stop(arm_movement_time_);
}

void ActionPublisher::close_tool_arm()
{
    RCLCPP_INFO(node_->get_logger(), "Calling /close_arm service");
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();

    auto future = move_stop_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node_->get_logger(), "Service call compete");
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Service call error");
    }

    this->wait_then_stop(arm_movement_time_);
}

void ActionPublisher::open_mast()
{
    RCLCPP_INFO(node_->get_logger(), "Calling /mast_open service");
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();

    auto future = move_stop_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node_->get_logger(), "Service call compete");
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Service call error");
    }

    this->wait_then_stop(mast_closing_opening_time_);
}

void ActionPublisher::close_mast()
{
    RCLCPP_INFO(node_->get_logger(), "Calling /mast_close service");
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();

    auto future = move_stop_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node_->get_logger(), "Service call compete");
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Service call error");
    }

    this->wait_then_stop(mast_closing_opening_time_);
}

void ActionPublisher::rotate_mast()
{
    RCLCPP_INFO(node_->get_logger(), "Calling /mast_rotate service");
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();

    auto future = move_stop_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node_->get_logger(), "Service call compete");
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Service call error");
    }

    this->wait_then_stop(mast_rotation_time_);
}

void ActionPublisher::wait_then_stop(double wait_time)
{
    RCLCPP_INFO(node_->get_logger(), "Waiting for the action to be completed before stopping");
    auto converted_time = std::chrono::nanoseconds(static_cast<int64_t>(wait_time * 1e9));
    rclcpp::sleep_for(std::chrono::nanoseconds(converted_time));
    this->stop();
}