import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    unified_launch = os.path.join(
        get_package_share_directory("rm_bringup"),
        "launch",
        "rm_bringup.launch.py",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_moveit_rviz",
                default_value="true",
                description="Start RViz from the RX75 MoveIt launch",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(unified_launch),
                launch_arguments={
                    "arm_type": "rx75-6fb",
                    "mode": "real",
                    "use_rviz": LaunchConfiguration("use_moveit_rviz"),
                }.items(),
            ),
        ]
    )
