import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

def generate_launch_description():
    gazebo = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("manipulator_description"),
                "launch",
                "gazebo.launch.py"
            )
        )
    
    controller = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("manipulator_bringup"),
                "launch",
                "controller.launch.py"
            ),
            launch_arguments={"is_sim": "True"}.items()
        )
    
    moveit = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("manipulator_moveit"),
                "launch",
                "moveit.launch.py"
            ),
            launch_arguments={"is_sim": "True"}.items(),
        )
    
    remote_interface = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("manipulator_server"),
                "launch",
                "remote_interface.launch.py"
            ),
        )
    rviz_config = os.path.join(
        get_package_share_directory("manipulator_bringup"),
            "rviz",
            "simulation_config.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
  
    )

    
    teleop_keyboard = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("manipulator_bringup"),
            "launch",
            "teleop_keyboard.launch.py"
        )
    )

    return LaunchDescription([
        gazebo,
        controller,
        moveit,
        rviz_node,
        remote_interface,
        teleop_keyboard,
    ])