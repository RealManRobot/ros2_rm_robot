import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():

    realman_xacro_file = os.path.join(get_package_share_directory('rm_75_moveit_config'), 'config',
                                        'rm_75.urdf.xacro')
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
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_transform_publisher",
                output="log",
                arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
            ),
            # Node(
            #     package='joint_state_publisher_gui',
            #     executable='joint_state_publisher_gui',
            #     name='joint_state_publisher_gui'
            # )
        ])
