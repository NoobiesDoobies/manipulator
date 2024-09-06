from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    task_server_node = Node(
        package="manipulator_server",
        executable="task_server_node",
    )

    return LaunchDescription([
        task_server_node,
    ])