import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    ignition_ros2_control_demos_path = os.path.join(
        get_package_share_directory('ign_robot'))

    xacro_file = os.path.join(ignition_ros2_control_demos_path,
                              'urdf',
                              'test_cart_velocity_1m_z.xacro.urdf')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    ignition_spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=['-string', doc.toxml(),
                   '-name', 'cartpole',
                   '-allow_renaming', 'true'],
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
            'velocity_controller'],
        output='screen'
    )

    load_imu_sensor_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'imu_sensor_broadcaster'],
        output='screen'
    )

    return LaunchDescription([
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_ign_gazebo'),
                              'launch', 'ign_gazebo.launch.py')]),
            launch_arguments=[('ign_args', [' -r -v 4 empty.sdf'])]),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ignition_spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_trajectory_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_trajectory_controller,
                on_exit=[load_imu_sensor_broadcaster],
            )
        ),
        node_robot_state_publisher,
        ignition_spawn_entity,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])
