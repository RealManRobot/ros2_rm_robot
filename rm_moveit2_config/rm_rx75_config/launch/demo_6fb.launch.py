import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    description_share = get_package_share_directory("rm_description")
    moveit_share = get_package_share_directory("rm_rx75_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument("db", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(moveit_share, "launch", "static_virtual_joint_tfs.launch.py")
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(description_share, "launch", "rm_rx75_6fb_display.launch.py")
                ),
                launch_arguments={
                    "use_joint_state_bridge": "false",
                    "use_joint_state_publisher_gui": "true",
                    "use_rviz": "false",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(moveit_share, "launch", "move_group.launch.py")
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(moveit_share, "launch", "moveit_rviz.launch.py")
                ),
                condition=IfCondition(LaunchConfiguration("use_rviz")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(moveit_share, "launch", "warehouse_db.launch.py")
                ),
                condition=IfCondition(LaunchConfiguration("db")),
            ),
        ]
    )
