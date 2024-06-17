from launch import LaunchDescription
from launch_ros.actions import Node

#Structure for a Launch File: To execute: Colcon Build; ros2 launch cgsr1_pkg xlaunch.py

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package_name',
            executable='motor_turn_counter',
            name='motor_turn_counter',
            output='screen',
        ),
    ])