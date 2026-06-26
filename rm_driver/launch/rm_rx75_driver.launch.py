import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def load_driver_params(config_path):
    with open(config_path, "r") as config_file:
        return yaml.safe_load(config_file)["rm_driver"]["ros__parameters"]


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory("rm_driver"), "config")

    left_config = os.path.join(config_dir, "rm_rx75_left_config.yaml")
    right_config = os.path.join(config_dir, "rm_rx75_right_config.yaml")

    left_driver = Node(
        package="rm_driver",
        executable="rm_driver",
        namespace="left_arm",
        parameters=[load_driver_params(left_config)],
        output="screen",
    )

    right_driver = Node(
        package="rm_driver",
        executable="rm_driver",
        namespace="right_arm",
        parameters=[load_driver_params(right_config)],
        output="screen",
    )

    return LaunchDescription([
        left_driver,
        right_driver,
    ])
