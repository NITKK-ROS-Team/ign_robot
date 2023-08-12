import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    # Launch Arguments

    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=use_sim_time,
        description='If true, use simulated clock')

    ign_robot_path = os.path.join(
        get_package_share_directory('ign_robot'))

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare('ign_robot'), "config", "omni_config.yaml"])
    xacro_file = os.path.join(ign_robot_path, 'xacro', 'omni.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', doc.toxml(),
                   '-name', 'cartpole',
                   '-allow_renaming', 'true'],
    )
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
                plugin='joy2omni::Joy2Omni',
                name='joy2omni',
                parameters=[{
                    'hw_type': LaunchConfiguration('hw_type')
                }],
                remappings=[
                    ('/command0', '/omni_0/commands'),
                    ('/command1', '/omni_1/commands'),
                    ('/command2', '/omni_2/commands'),
                    ('/command3', '/omni_3/commands'),
                ]
            )
        ]
    )

# broadcaster
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    omni_0_controller = ExecuteProcess(cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'omni_0'], output='screen')
    omni_1_controller = ExecuteProcess(cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'omni_1'], output='screen')
    omni_2_controller = ExecuteProcess(cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'omni_2'], output='screen')
    omni_3_controller = ExecuteProcess(cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'omni_3'], output='screen')

    ign_ros2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_ign_gazebo'),
                          'launch', 'ign_gazebo.launch.py')]),
            launch_arguments=[('ign_args', [' -r -v 4 empty.sdf'])])


# ====================
    sleep = ExecuteProcess(
        cmd=['sleep', '5'],
        output='screen'
    )
    return LaunchDescription([
        sim_time_arg,
        joy_launch_arg,

        ign_ros2,
        node_robot_state_publisher,
        ignition_spawn_entity,
        joy_node,

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ignition_spawn_entity,
                on_exit=[sleep]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=sleep,
                on_exit=[load_joint_state_controller],
            )
        ),
        # direct ---------------------------------------------
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[
                    omni_0_controller,
                    omni_1_controller,
                    omni_2_controller,
                    omni_3_controller
                ]
            )
        ),
    ])
