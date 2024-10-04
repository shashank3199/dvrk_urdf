from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    # Get robot description via xacro
    robot_description = {
        "robot_description": ParameterValue(
            Command([
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare("ecm_description"), "urdf", "ecm.urdf.xacro"]
                )
            ]),
            value_type=str
        )
    }

    # Path to the controller configuration file
    controller_file = "ecm.controllers.yaml"

    # Path to the controller configuration file
    controller_config = PathJoinSubstitution(
        [FindPackageShare("ecm_description"), "config", controller_file]
    )

    # For ros2_control_node
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[controller_config, {'use_sim_time': True}],
        remappings=[('~/robot_description', '/robot_description')]
    )

    # For robot_state_publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {'publish_robot_description': True, 'use_sim_time': True}],
    )

    # Load joint state broadcaster
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Load the joint group position controller
    load_joint_group_position_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_position_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Joint Position Controller Node
    joint_position_controller = Node(
        package="ecm_description",
        executable="ecm_joint_controller",
        output="screen",
        parameters=[{'use_sim_time': True}]
    )

    ecm_rviz_file = PathJoinSubstitution(
        [FindPackageShare("ecm_description"), "rviz", "ecm_description.rviz"]
    )

    # For RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', ecm_rviz_file],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[rviz_node],
        )
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        load_joint_state_broadcaster,
        load_joint_group_position_controller,
        joint_position_controller,
        delay_rviz_after_joint_state_broadcaster_spawner
    ])
