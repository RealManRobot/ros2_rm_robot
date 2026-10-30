"""Factories for the historical model-specific bringup entry points."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_legacy_bringup(arm_type, mode):
    """Route one legacy filename into the component-level unified launch."""
    arguments = {
        'arm_type': arm_type,
        'mode': mode,
    }
    actions = []

    # Preserve the historical RX75 real-launch argument name.
    if arm_type.startswith('rx75-') and mode == 'real':
        actions.append(
            DeclareLaunchArgument(
                'use_moveit_rviz',
                default_value='true',
                description='Start RViz from the RX75 MoveIt launch',
            )
        )
        arguments['use_rviz'] = LaunchConfiguration('use_moveit_rviz')

    unified_launch = os.path.join(
        get_package_share_directory('rm_bringup'),
        'launch',
        'rm_bringup.launch.py',
    )
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(unified_launch),
            launch_arguments=arguments.items(),
        )
    )
    return LaunchDescription(actions)
