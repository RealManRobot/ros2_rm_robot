import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

sys.path.insert(0, os.path.join(get_package_share_directory('rm_gazebo'), 'launch'))
from gz_demo_common import generate_gz_demo_launch


def generate_launch_description():

    rm_gazebo_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rm_63_config'),
                'launch',
                'gazebo_moveit_demo.launch.py',
            )
        )
    )

    return generate_gz_demo_launch(
        urdf_filename='gazebo_63_description.urdf.xacro',
        robot_name_in_model='rml_63_description',
        controller_names=['joint_state_broadcaster', 'rm_group_controller'],
        post_spawn_actions=[rm_gazebo_moveit],
    )
