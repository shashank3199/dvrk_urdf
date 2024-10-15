from launch import LaunchDescription

from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue


def generate_launch_description():

    # Get the URDF Xacro file
    urdf_arg = DeclareLaunchArgument(
        "urdf",
        description="Name of the URDF Xacro file"
    )

    # Get robot description via xacro
    ecm_description = {
        "robot_description": ParameterValue(
            Command([
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare("ecm_description"), "urdf", LaunchConfiguration("urdf")]
                )
            ]),
            value_type=str
        )
    }

    # For robot_state_publisher Node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[ecm_description]
    )

    # For joint_state_publisher_gui Node
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        parameters=[ecm_description]
    )

    # Path to the RViz file
    ecm_rviz_file = PathJoinSubstitution(
        [FindPackageShare("ecm_description"), "rviz", "ecm_description.rviz"]
    )

    # RViz2 Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", ecm_rviz_file],
    )

    return LaunchDescription([
        urdf_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
