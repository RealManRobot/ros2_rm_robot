import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, LaunchConfiguration

import xacro

def generate_launch_description():

    realman_xacro_file = os.path.join(get_package_share_directory('rm_description'), 'urdf',
                                        'rm_eco63.urdf')
    robot_description = Command(
        [FindExecutable(name='xacro'), ' ', realman_xacro_file])

    return LaunchDescription([
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                respawn=True,
                parameters=[{'robot_description': robot_description}],
                output='screen'
            ),
        ])
