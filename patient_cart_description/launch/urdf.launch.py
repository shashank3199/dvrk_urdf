from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    file_arg = DeclareLaunchArgument(
        "file",
        description="Name of the file"
    )

    patient_cart_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("patient_cart_description"), "urdf", LaunchConfiguration("file")]),
        ]
    )

    patient_cart_description = {
        "robot_description": ParameterValue(
            patient_cart_description_content, value_type=str)
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[patient_cart_description]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        parameters=[patient_cart_description]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
    )

    return LaunchDescription([
        file_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
