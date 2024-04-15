import launch
import os
import yaml
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    arm_config = os.path.join(get_package_share_directory('rm_driver'),'config','rm_75_config.yaml')

    with open(arm_config,'r') as f:
        params = yaml.safe_load(f)["rm_driver"]["ros__parameters"]

    return LaunchDescription([

        Node(
            package= "rm_driver",                 #功能包。
            executable= "rm_driver",         #节点。
            parameters= [arm_config
                ],             #接入参数文件
            output= 'screen'
            )

    ])