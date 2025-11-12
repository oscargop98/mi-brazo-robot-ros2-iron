from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('xolobot_control'),
        'config',
        'xolobot_control.yaml'
    )

    return LaunchDescription([
        # robot_state_publisher con remapeo
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            remappings=[('/joint_states', '/xolobot_arm/joint_states')],
        ),

        # controller_manager - Cargar configuración
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[config_file],
            output='screen',
        ),

        # Spawner para cada controlador
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_controller',
                'joint1_position_controller',
                'joint2_position_controller',
                'joint3_position_controller',
                'joint4_position_controller',
                'joint5_position_controller',
                'joint6_position_controller',
            ],
            output='screen',
        ),
    ])
