from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    unified_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('rm_driver'), 'launch', 'rm_driver.launch.py']
            )
        ),
        launch_arguments={'arm_type': 'rx75'}.items(),
    )
    return LaunchDescription([unified_launch])
