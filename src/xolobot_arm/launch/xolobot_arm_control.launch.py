import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_xolobot_arm = get_package_share_directory('xolobot_arm')
    world_path = os.path.join(package_xolobot_arm, "worlds", "coca_levitando.world")
    urdf_path = os.path.join(package_xolobot_arm, "models", "xolobot.urdf")
    sdf_path = os.path.join(package_xolobot_arm, "models", "xolobot_arm.sdf")
    objeto_path = os.path.join(package_xolobot_arm, "models/utileria", "objeto.sdf")
    soporte_path = os.path.join(package_xolobot_arm, "models/utileria", "soporte.sdf")
    yaml_config_path = os.path.join(get_package_share_directory('xolobot_control'), "config", "xolobot_control.yaml")
    #objeto_path2 = os.path.join(package_xolobot_arm, "models/utileria", "objeto2.sdf")
    #soporte_path2 = os.path.join(package_xolobot_arm, "models/utileria", "soporte2.sdf")

    # Alinear tiempo de ros con el de la simulacion
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -v4 ', world_path]}.items()
    )

    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    with open(sdf_path, 'r') as sdf_file:
        sdf_content = sdf_file.read()
    
    sdf_content = sdf_content.replace('$(find xolobot_control)/config/xolobot_control.yaml', yaml_config_path)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
        output='screen'
    )

    spawn_model = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_xolobot',
        arguments=['-string', sdf_content, '-name', 'xolobot_arm', '-x', '0', '-y', '0', '-z', '0.55'],
        output='screen'
    )
    
    objeto = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_lata',
        arguments=['-file', objeto_path, '-name', 'objeto', '-x', '0.270909', '-y', '0.256721', '-z', '0.846173'],
        output='screen'
    )
    
    soporte = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_soporte',
        arguments=['-file', soporte_path, '-name', 'soporte', '-x', '0.270909', '-y', '0.256721', '-z', '0.816173'],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/bumper_states_palma@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/bumper_states_antebrazo@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/bumper_states_pulgar_3@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/bumper_states_indice_3@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/bumper_states_cordial_3@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/bumper_states_anular_3@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/bumper_states_menique_3@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/xolobot_arm/attach@std_msgs/msg/Empty]gz.msgs.Empty',
        ],
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

    load_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
        output="screen",
    )
    #load_effort_controller = ExecuteProcess(
    #    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'effort_controller'],
    #    output='screen'
    #)

    set_qt_env = SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb')
    append_gz_env = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(package_xolobot_arm, 'models'))
    append_ign_env = AppendEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', os.path.join(package_xolobot_arm, 'models'))

    return LaunchDescription([
        set_qt_env,
        append_gz_env,
        append_ign_env,
        gazebo,
        bridge,
        robot_state_publisher,
        spawn_model,
        objeto,
        #objeto2,
        soporte,
        #soporte2,
        load_trajectory_controller,
        #load_effort_controller
    ])
