import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import xacro


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    ign_robot_path = os.path.join(
        get_package_share_directory('ign_robot'))

    xacro_file = os.path.join(ign_robot_path,
                              'urdf',
                              'ign_robot.urdf')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    print(params)

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )

    ignition_spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=['-string', doc.toxml(),
                   '-name', 'cartpole',
                   '-allow_renaming', 'true'],
    )
    # ros2 run ros_ign_bridge parameter_bridge /image@sensor_msgs/msg/Image@ignition.msgs.Image

    # teleop twist joy
    joy_launch_arg = DeclareLaunchArgument(
                        'hw_type',
                        default_value=TextSubstitution(text='DualSense')
    )
    joy_node = ComposableNodeContainer(
                name='joy_container',
                namespace='p9n',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='joy',
                        plugin='joy::Joy',
                        name='joy',
                    ),
                    ComposableNode(
                        package='ign_robot',
                        plugin='joy2twist::Joy2Twist',
                        name='joy2twist',
                        parameters=[{
                            'hw_type': LaunchConfiguration('hw_type')
                        }],
                        remappings=[
                            ('/cmd_vel', '/diff_drive_base_controller/cmd_vel_unstamped')
                        ]
                    ),
                    ComposableNode(
                        package='ign_robot',
                        plugin='joy2direct::Joy2Direct',
                        name='joy2direct',
                        remappings=[
                            ('commands', '/slidar_velocity_controller/commands')
                        ]
                    ),
                    ComposableNode(
                        package='ign_robot',
                        plugin='joy2angle::Joy2Angle',
                        name='joy2angle',
                        remappings=[
                            ('commands', '/cart_angle_controller/commands')
                        ]
                    )
                ]
    )

# broadcaster
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )

    # sleep
    sleep = ExecuteProcess(
        cmd=['sleep', '1'],
        output='screen'
    )

# diff drive controller -------------------------------------------------------
    diff_drive = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'diff_drive_base_controller'],
        output='screen'
    )
# direct ---------------------------------------------------------------
    direct_acting = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
            'slidar_velocity_controller'],
        output='screen'
    )
    cart_angle_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
            'cart_angle_controller'],
        output='screen'
    )
# imu ---------------------------------------------------------------
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
        
        node_robot_state_publisher,
        ignition_spawn_entity,

        joy_launch_arg,
        joy_node,
        
        # sleep ---------------------------------------------------------------
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ignition_spawn_entity,
                on_exit=[sleep],
            )
        ),
        # ign-spawn
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=sleep,
                on_exit=[load_joint_state_controller],
            )
        ),
        # diff-drive -------------------------------------------------------
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[diff_drive],
            )
        ),
        # direct ---------------------------------------------
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[direct_acting],
            )
        ),
        # cart2cart ---------------------------------------------------------------
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[cart_angle_controller],
            )
        ),
        # imu (<- direct) ---------------------------------------------
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=direct_acting,
                on_exit=[load_imu_sensor_broadcaster],
            )
        ),


        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])
