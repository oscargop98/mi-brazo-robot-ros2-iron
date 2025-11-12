from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="xolobot_arm_server",
            executable="xolobot_arm_server", 
            name="arm_server", 
            output="screen"  
        )
    ])
