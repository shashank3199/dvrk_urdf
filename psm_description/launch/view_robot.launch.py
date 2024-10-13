from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    urdf_arg = DeclareLaunchArgument(
        "urdf",
        description="Name of the URDF Xacro file",
    )

    tool_arg = DeclareLaunchArgument(
        "tool",
        default_value="sca",
        description="Name of the tool",
        choices=["blade", "caudier_blade", "caudier", "sca_blade", "sca", "snake", "P420006", "SF826001"],
    )

    psm_description = {
        "robot_description": ParameterValue(
            Command([
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare("psm_description"), "urdf", LaunchConfiguration("urdf")]),
                " ",
                "tool:=", LaunchConfiguration("tool")
            ]),
            value_type=str
        )
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[psm_description]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        parameters=[psm_description]
    )

    psm_rviz_file = PathJoinSubstitution(
        [FindPackageShare("psm_description"), "rviz", "psm_description.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", psm_rviz_file],
    )

    return LaunchDescription([
        tool_arg,
        urdf_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
