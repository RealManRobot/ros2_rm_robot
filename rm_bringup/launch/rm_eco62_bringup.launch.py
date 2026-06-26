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

    rm_eco62_driver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('rm_driver')),'launch', 'rm_eco62_driver.launch.py'))
    )

    rm_eco62_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('rm_description'), 'launch', 'rm_eco62_display.launch.py')),
    )

    rm_eco62_control = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('rm_control')),'launch', 'rm_eco62_control.launch.py'))
    )

    rm_eco62_moveit_config = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('rm_eco62_config')),'launch', 'real_moveit_demo.launch.py'))
    )

    return LaunchDescription([
    rm_eco62_driver,
    rm_eco62_description,
    rm_eco62_control,
    rm_eco62_moveit_config
    ])