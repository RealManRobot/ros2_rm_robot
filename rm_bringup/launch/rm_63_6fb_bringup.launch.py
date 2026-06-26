import os
from  ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    rm_63_driver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('rm_driver')),'launch', 'rm_63_driver.launch.py'))
    )

    rm_63_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('rm_description'), 'launch', 'rm_63_6fb_display.launch.py')),
    )

    rm_63_control = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('rm_control')),'launch', 'rm_63_control.launch.py'))
    )

    rm_63_moveit_config = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('rm_63_config')),'launch', 'real_moveit_demo_6fb.launch.py'))
    )

    return LaunchDescription([
    rm_63_driver,
    rm_63_description,
    rm_63_control,
    rm_63_moveit_config
    ])