import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter


def generate_launch_description():

    rm_65_gazebo_up = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('rm_gazebo')),'launch', 'gazebo_65_demo.launch.py'))
    )
    rm_65_gazebo_moveit = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('rm_65_config')),'launch', 'gazebo_moveit_demo.launch.py'))
    )

    return LaunchDescription([
        SetParameter(name="use_sim_time", value=True),
        rm_65_gazebo_up,
        TimerAction(period=8.0, actions=[rm_65_gazebo_moveit]),
    ])
