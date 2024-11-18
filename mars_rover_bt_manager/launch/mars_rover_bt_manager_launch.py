from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    bt_path_arg = DeclareLaunchArgument(
        'bt_path',
        default_value='',
        description='Path to the Behavior Tree XML file'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
    )

    mars_rover_bt_manager_node = Node(
        package='mars_rover_bt_manager',
        executable='mars_rover_bt_manager_node',
        name='mars_rover_bt_manager_node',
        output='screen',
        parameters=[
            {'bt_path': LaunchConfiguration('bt_path')},
            {'namespace': LaunchConfiguration('namespace')}
        ]
    )

    return LaunchDescription([
        bt_path_arg,
        namespace_arg,
        mars_rover_bt_manager_node
    ])
