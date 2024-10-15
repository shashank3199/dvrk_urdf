from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue


def generate_launch_description():

    # Get robot description via xacro
    daVinciCore_description = {
        "robot_description": ParameterValue(
            Command([
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare("davinci_core_description"), "urdf", "daVinci.urdf.xacro"]),
            ]),
            value_type=str
        )
    }

    # For robot_state_publisher Node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[daVinciCore_description]
    )

    # For joint_state_publisher_gui Node
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        parameters=[daVinciCore_description]
    )

    # For rviz2 Node
    davinci_core_rviz_file = PathJoinSubstitution(
        [FindPackageShare("davinci_core_description"), "rviz", "davinci_core_description.rviz"]
    )

    # For rviz2 Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", davinci_core_rviz_file],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
