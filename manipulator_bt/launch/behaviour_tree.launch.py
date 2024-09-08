import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import UnlessCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    bt_node = Node(
        package="manipulator_bt",
        executable="pickup_box_bt",
        output="screen",
    )
    detect_aruco_node = Node(
        package="manipulator_bt",
        executable="detect_aruco",
        output="log",
    )

    return LaunchDescription(
        [
            bt_node,
            detect_aruco_node
        ]
    )