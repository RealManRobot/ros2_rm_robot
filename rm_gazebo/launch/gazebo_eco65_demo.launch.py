import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    unified_launch = os.path.join(
        get_package_share_directory("rm_gazebo"),
        "launch",
        "rm_gazebo.launch.py",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("model", default_value="auto", choices=["auto", "stl", "dae"]),
            DeclareLaunchArgument("start_gazebo", default_value="true"),
            DeclareLaunchArgument("use_gazebo_gui", default_value="true"),
            DeclareLaunchArgument(
                "clock_topic", default_value="/clock"
            ),
            DeclareLaunchArgument(
                "joint_states_topic",
                default_value="auto",
                choices=["auto"],
            ),
            DeclareLaunchArgument(
                "spawn_entity_timeout", default_value="120"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(unified_launch),
                launch_arguments={
                    "arm_type": "eco65",
                    "model": LaunchConfiguration("model"),
                    "start_gazebo": LaunchConfiguration("start_gazebo"),
                    "use_gazebo_gui": LaunchConfiguration("use_gazebo_gui"),
                    "clock_topic": LaunchConfiguration("clock_topic"),
                    "joint_states_topic": LaunchConfiguration(
                        "joint_states_topic"
                    ),
                    "spawn_entity_timeout": LaunchConfiguration(
                        "spawn_entity_timeout"
                    ),
                    "use_sim_time": "true",
                }.items(),
            ),
        ]
    )
