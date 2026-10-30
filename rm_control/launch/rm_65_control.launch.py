from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    unified_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('rm_control'), 'launch', 'rm_control.launch.py']
            )
        ),
        launch_arguments={'arm_type': '65'}.items(),
    )
    return LaunchDescription([unified_launch])
