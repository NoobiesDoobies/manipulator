from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    manipulator_task_server = Node(
        package="manipulator_server",
        executable="manipulator_task_server_node"
    )

    mobile_bot_task_server = Node(
        package="manipulator_server",
        executable="mobile_bot_task_server_node"
    )

    return LaunchDescription([
        manipulator_task_server,
        mobile_bot_task_server
    ])