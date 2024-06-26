from launch import LaunchDescription
from launch_ros.actions import Node

#Structure for a Launch File: To execute: Colcon Build; ros2 launch cgsr1_pkg xlaunch.py

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cgsr1_pkg',
            executable='linus_automation',
            name='linus_automation',
            output='screen',
        ),
    ])