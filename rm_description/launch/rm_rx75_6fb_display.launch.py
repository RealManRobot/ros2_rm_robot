import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    actions = [
        DeclareLaunchArgument("model", default_value="auto", choices=["auto", "stl", "dae"]),
        DeclareLaunchArgument("left_xyz", default_value="0 0.075 1.5"),
        DeclareLaunchArgument("left_rpy", default_value="3.14 -1.57 1.5707963267949"),
        DeclareLaunchArgument("right_xyz", default_value="0 -0.075 1.5"),
        DeclareLaunchArgument("right_rpy", default_value="3.14 -1.57 -1.5707963267949"),
        DeclareLaunchArgument("use_joint_state_bridge", default_value="false"),
        DeclareLaunchArgument("use_joint_state_publisher_gui", default_value="true"),
    ]
    launch_arguments = {
        "model": LaunchConfiguration("model"),
        "arm_type": "rx75-6fb",
        "use_sim_time": "false",
        "joint_states_topic": "/joint_states",
        "left_joint_states_topic": "/left_arm/joint_states",
        "right_joint_states_topic": "/right_arm/joint_states",
        "left_xyz": LaunchConfiguration("left_xyz"),
        "left_rpy": LaunchConfiguration("left_rpy"),
        "right_xyz": LaunchConfiguration("right_xyz"),
        "right_rpy": LaunchConfiguration("right_rpy"),
        "use_joint_state_bridge": LaunchConfiguration("use_joint_state_bridge"),
        "use_joint_state_publisher_gui": LaunchConfiguration("use_joint_state_publisher_gui"),
        "use_rviz": "false",
    }
    unified_launch = os.path.join(
        get_package_share_directory("rm_description"),
        "launch",
        "rm_description.launch.py",
    )
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(unified_launch),
            launch_arguments=launch_arguments.items(),
        )
    )
    return LaunchDescription(actions)
