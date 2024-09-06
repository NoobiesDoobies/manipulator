import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    manipulator_description_description_dir = get_package_share_directory('manipulator_description')
    manipulator_description_description_share = os.path.join(get_package_prefix('manipulator_description'), 'share')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    model_arg = DeclareLaunchArgument(name='model', default_value=os.path.join(
                                        manipulator_description_description_dir, 'description', 'robot.urdf.xacro'
                                        ),
                                      description='Absolute path to robot urdf file'
    )

    env_var = SetEnvironmentVariable('GAZEBO_MODEL_PATH', manipulator_description_description_share)

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    world_file = os.path.join(
        get_package_share_directory("manipulator_description"),
        # "worlds", "simple_wall_following.world"
        # "worlds", "maze_3_6x6.world"
        # "worlds", "self_made_maze.world"
        # "worlds", "cafe.world"
        # "worlds", "empty.world"
        "worlds", "aruco_on_table2.world"
    )

    start_gazebo_server = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'world': world_file}.items()
    )

    # start_gazebo_server = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')
    #     )
    # )

    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py')
        )
    )

    spawn_robot = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'manipulator_description',
                                   '-topic', 'robot_description',
                                      '-x', '0', '-y', '0', '-z', '0.3',
                                  ],
                        output='screen'
    )
    

    return LaunchDescription([
        env_var,
        model_arg,
        start_gazebo_server,
        # start_gazebo_client,
        robot_state_publisher_node,
        spawn_robot
    ])