#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist

class BatteryPublisher(Node):

    def __init__(self):
        super().__init__('battery_publisher')
        self.publisher_ = self.create_publisher(Float32, '/demo/topicf', 10)
        self.subscription = self.create_subscription(Twist,'/cmd_vel',self.listener_callback,10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.battery_level = 1 # Starting battery level
        self.robot_moving=False
    
    def listener_callback(msg):
        # Extract linear and angular velocities from the Twist message
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        linear_z = msg.linear.z
        angular_x = msg.angular.x
        angular_y = msg.angular.y
        angular_z = msg.angular.z
        
        # Check if all velocities are zero
        if (linear_x == 0.0 and linear_y == 0.0 and linear_z == 0.0 and
            angular_x == 0.0 and angular_y == 0.0 and angular_z == 0.0):
            self.robot_moving=False
        else:
            self.robot_moving=True

    def timer_callback(self):
        
        # Generate a random increment value between 0.01 and 0.05
        increment = random.uniform(0.01, 0.05)
        self.battery_level += increment

        if(self.robot_moving):
          # Generate a random decrement value between 0.01 and 0.05
          decrement = random.uniform(0.01, 0.05)
          self.battery_level -= decrement
        
        # Ensure battery level does not go below 0
        if self.battery_level < 0:
            self.battery_level = 0
          
        # Ensure battery level does not go above 1
        if self.battery_level > 1:
            self.battery_level = 1
          
        msg = Float32()
        msg.data = self.battery_level
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data:.2f}% battery.')

def main(args=None):
    rclpy.init(args=args)
    battery_publisher = BatteryPublisher()
    
    try:
        rclpy.spin(battery_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        battery_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
