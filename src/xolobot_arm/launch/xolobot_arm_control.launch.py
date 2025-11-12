import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_xolobot_arm = get_package_share_directory('xolobot_arm')
    world_path = os.path.join(package_xolobot_arm, "worlds", "coca_levitando.world")
    urdf_path = os.path.join(package_xolobot_arm, "models", "xolobot.urdf")
    sdf_path = os.path.join(package_xolobot_arm, "models", "xolobot_arm.sdf")
    objeto_path = os.path.join(package_xolobot_arm, "models/utileria", "objeto.sdf")
    soporte_path = os.path.join(package_xolobot_arm, "models/utileria", "soporte.sdf")
    #objeto_path2 = os.path.join(package_xolobot_arm, "models/utileria", "objeto2.sdf")
    #soporte_path2 = os.path.join(package_xolobot_arm, "models/utileria", "soporte2.sdf")

    # Alinear tiempo de ros con el de la simulacion
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
        output='screen'
    )

    spawn_model = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_xolobot',
        arguments=['-file', sdf_path, '-entity', 'xolobot_arm', '-x', '0', '-y', '0', '-z', '0.55'],
        output='screen'
    )
    
    objeto = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_lata',
        arguments=['-file', objeto_path, '-entity', 'objeto'],
        output='screen'
    )
    
    soporte = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_soporte',
        arguments=['-file', soporte_path, '-entity', 'soporte'],
        output='screen'
    )
    
    #objeto2 = Node(
    #    package='gazebo_ros',
    #    executable='spawn_entity.py',
    #    name='spawn_lata2',
    #    arguments=['-file', objeto_path2, '-entity', 'objeto2'],
    #    output='screen'
    #)
    
    #soporte2 = Node(
    #    package='gazebo_ros',
    #    executable='spawn_entity.py',
    #   name='spawn_soporte2',
    #    arguments=['-file', soporte_path2, '-entity', 'soporte2'],
    #    output='screen'
    #)

    load_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_trajectory_controller'],
        output='screen'
    )
    #load_effort_controller = ExecuteProcess(
    #    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'effort_controller'],
    #    output='screen'
    #)


    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_model,
        objeto,
        #objeto2,
        soporte,
        #soporte2,
        load_trajectory_controller,
        #load_effort_controller
    ])
