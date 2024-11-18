#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/empty.hpp>
#include "mars_rover_bt_manager/action_publisher.hpp"
#include "mars_rover_srvs/srv/move_service.hpp"

ActionPublisher::ActionPublisher(std::shared_ptr<rclcpp::Node> node) :
    node_(node)
{
    move_client_ = node_->create_client<mars_rover_srvs::srv::MoveService>("/move");

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

void ActionPublisher::move(double speed)
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
}

void ActionPublisher::move_forward(double speed)
{
    RCLCPP_INFO(node_->get_logger(), "Calling /move service to move forward");
    auto request = std::make_shared<mars_rover_srvs::srv::MoveService::Request>();
    request->twist.linear.x = speed;

    auto future = move_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node_->get_logger(), "Service call compete");
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Service call error");
    }
}

void ActionPublisher::move_backward(double speed)
{
    RCLCPP_INFO(node_->get_logger(), "Calling /move service to move backward");
    auto request = std::make_shared<mars_rover_srvs::srv::MoveService::Request>();
    request->twist.linear.x = -1 * speed;

    auto future = move_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node_->get_logger(), "Service call compete");
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Service call error");
    }
}

void ActionPublisher::turn_left()
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

}

void ActionPublisher::turn_right()
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
}

void ActionPublisher::rotate_tool_arm()
{
    RCLCPP_INFO(node_->get_logger(), "Calling rotate arm service (mocking call)");
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
}

void ActionPublisher::drill()
{
    RCLCPP_INFO(node_->get_logger(), "Calling drill service (mocking call)");
}

void ActionPublisher::trickle()
{
    RCLCPP_INFO(node_->get_logger(), "Calling trickle service (mocking call)");
}

void ActionPublisher::take_picture()
{
    RCLCPP_INFO(node_->get_logger(), "Calling camera service (mocking call)");
}
