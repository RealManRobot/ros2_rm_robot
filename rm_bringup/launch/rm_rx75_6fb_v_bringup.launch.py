import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_moveit_rviz = LaunchConfiguration("use_moveit_rviz")

    rm_rx75_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rm_driver"),
                "launch",
                "rm_rx75_driver.launch.py",
            )
        )
    )

    rm_rx75_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rm_description"),
                "launch",
                "rm_rx75_6fb_v_display.launch.py",
            )
        ),
        launch_arguments={
            "use_joint_state_bridge": "true",
            "use_joint_state_publisher_gui": "false",
            "use_rviz": "false",
        }.items(),
    )

    rm_rx75_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rm_control"),
                "launch",
                "rm_rx75_control.launch.py",
            )
        )
    )

    rm_rx75_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rm_rx75_config"),
                "launch",
                "real_moveit_demo_6fb_v.launch.py",
            )
        ),
        launch_arguments={
            "use_rviz": use_moveit_rviz,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_moveit_rviz", default_value="true"),
            rm_rx75_driver,
            rm_rx75_description,
            rm_rx75_control,
            rm_rx75_moveit,
        ]
    )
