from launch import LaunchDescription
from launch_ros.actions import Node

#Structure for a Launch File: To execute: Colcon Build; ros2 launch cgsr1_pkg xlaunch.py

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cgsr1_pkg',
            executable='web_interface',
            name='web_interface',
            output='screen',
        ),
        
        Node(
            package='cgsr1_pkg',
            executable='calibration',
            name='calibration',
            output='screen',
        ),
        
        Node(
            package='cgsr1_pkg',
            executable='nodecontroller',
            name='nodecontroller',
            output='screen',
        ),
        
        Node(
            package='cgsr1_pkg',
            executable='nodemanager',
            name='nodemanager',
            output='screen',
        ),
    ])