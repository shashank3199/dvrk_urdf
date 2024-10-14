from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    urdf_arg = DeclareLaunchArgument(
        "urdf",
        default_value="psm.classic.urdf.xacro",
        description="Name of the URDF file",
    )

    tool_arg = DeclareLaunchArgument(
        "tool",
        default_value="sca",
        description="Name of the tool",
        choices=["blade", "caudier_blade", "caudier", "sca_blade", "sca", "snake", "P420006", "SF826001"],
    )

    # Get robot description via xacro
    robot_description = {
        "robot_description": ParameterValue(
            Command([
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare("psm_description"), "urdf", LaunchConfiguration("urdf")]
                ),
                " ",
                "tool:=", LaunchConfiguration("tool")
            ]),
            value_type=str
        )
    }

    # Controller Configuration File Generator
    controller_yaml_generator = Node(
        package="psm_description",
        executable="generate_controller.py",
        output="screen",
        arguments=[LaunchConfiguration("urdf"), LaunchConfiguration("tool")]
    )

    # Path to the controller configuration file
    controller_file = "psm.controllers.yaml"

    # Path to the controller configuration file
    controller_config = PathJoinSubstitution(
        [FindPackageShare("psm_description"), "config", controller_file]
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
        package="psm_description",
        executable="psm_joint_controller",
        output="screen",
        parameters=[{'use_sim_time': True}]
    )

    psm_rviz_file = PathJoinSubstitution(
        [FindPackageShare("psm_description"), "rviz", "psm_description.rviz"]
    )

    # For RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', psm_rviz_file],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[rviz_node],
        )
    )


    delay_till_controller_yaml_generator = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller_yaml_generator,
            on_exit=[
                robot_state_publisher,
                controller_manager,
                load_joint_state_broadcaster,
                load_joint_group_position_controller,
                joint_position_controller,
                delay_rviz_after_joint_state_broadcaster_spawner
            ],
        )
    )

    return LaunchDescription([
        urdf_arg,
        tool_arg,
        controller_yaml_generator,
        delay_till_controller_yaml_generator,
    ])
