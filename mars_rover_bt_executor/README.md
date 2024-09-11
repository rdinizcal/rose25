# Adaptable Behavior Tree

## Installation and run

1. Create a ROS2 colcon workspace by running `colcon build` inside your workspace folder
2. Clone this repository into your workspace's `src` folder
3. Run `colcon build` in the workspace's root
4. Run `. install/setup.bash`
5. Launch the ROS node: `ros2 launch mars_rover_bt_executor mars_rover_bt_executor_launch.py bt_path:=<path-to-bt-xml-file>`
