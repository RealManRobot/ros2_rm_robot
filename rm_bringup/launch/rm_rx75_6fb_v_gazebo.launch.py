import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    start_gazebo = LaunchConfiguration("start_gazebo")
    joint_states_topic = LaunchConfiguration("joint_states_topic")

    rm_rx75_gazebo_up = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rm_gazebo"),
                "launch",
                "gazebo_rx75_6fb_v_demo.launch.py",
            )
        ),
        launch_arguments={
            "start_gazebo": start_gazebo,
            "joint_states_topic": joint_states_topic,
        }.items(),
    )

    rm_rx75_gazebo_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rm_rx75_config"),
                "launch",
                "gazebo_moveit_demo_6fb_v.launch.py",
            )
        ),
        launch_arguments={
            "joint_states_topic": joint_states_topic,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_gazebo", default_value="true"),
            DeclareLaunchArgument(
                "joint_states_topic",
                default_value="/joint_state_broadcaster/joint_states",
            ),
            rm_rx75_gazebo_up,
            rm_rx75_gazebo_moveit,
        ]
    )
