from moveit_configs_utils import MoveItConfigsBuilder

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "rm_eco62_description", package_name="rm_eco62_config"
    ).to_moveit_configs()

    return LaunchDescription(
        [
            DeclareLaunchArgument("cartesian_y_offset", default_value="0.03"),
            DeclareLaunchArgument("cartesian_z_offset", default_value="-0.03"),
            DeclareLaunchArgument("eef_step", default_value="0.01"),
            DeclareLaunchArgument("jump_threshold", default_value="0.0"),
            DeclareLaunchArgument("required_fraction", default_value="0.95"),
            DeclareLaunchArgument("current_state_wait_sec", default_value="10.0"),
            Node(
                package="moveit_node",
                executable="simple_cartesian_line_node",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    {
                        "cartesian_y_offset": ParameterValue(
                            LaunchConfiguration("cartesian_y_offset"), value_type=float
                        ),
                        "cartesian_z_offset": ParameterValue(
                            LaunchConfiguration("cartesian_z_offset"), value_type=float
                        ),
                        "eef_step": ParameterValue(
                            LaunchConfiguration("eef_step"), value_type=float
                        ),
                        "jump_threshold": ParameterValue(
                            LaunchConfiguration("jump_threshold"), value_type=float
                        ),
                        "required_fraction": ParameterValue(
                            LaunchConfiguration("required_fraction"), value_type=float
                        ),
                        "current_state_wait_sec": ParameterValue(
                            LaunchConfiguration("current_state_wait_sec"), value_type=float
                        ),
                    },
                ],
            ),
        ]
    )
