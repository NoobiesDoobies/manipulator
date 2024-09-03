import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['xterm', '-e', 'ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true --remap cmd_vel:=/omni_wheel_controller/cmd_vel'],
            output='screen',
            shell=True
        )
    ])