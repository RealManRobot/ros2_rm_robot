from moveit_configs_utils import MoveItConfigsBuilder

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "rm_eco63_description", package_name="rm_eco63_config"
    ).to_moveit_configs()

    return LaunchDescription(
        [
            DeclareLaunchArgument("planning_group", default_value="rm_group"),
            DeclareLaunchArgument("current_state_wait_sec", default_value="10.0"),
            DeclareLaunchArgument("velocity_scaling", default_value="0.3"),
            DeclareLaunchArgument("acceleration_scaling", default_value="0.3"),
            DeclareLaunchArgument("planning_time", default_value="5.0"),
            DeclareLaunchArgument("home_named_target", default_value="forward"),
            DeclareLaunchArgument("enable_pose_target", default_value="false"),
            DeclareLaunchArgument("pose_target_csv", default_value=""),
            DeclareLaunchArgument("pose_target_position_in_mm", default_value="true"),
            DeclareLaunchArgument("pose_target_rpy_in_degrees", default_value="false"),
            DeclareLaunchArgument("pose_reference_frame", default_value=""),
            Node(
                package="rm_moveit2",
                executable="simple_moveit_node",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    {
                        "planning_group": LaunchConfiguration("planning_group"),
                        "current_state_wait_sec": ParameterValue(
                            LaunchConfiguration("current_state_wait_sec"), value_type=float
                        ),
                        "velocity_scaling": ParameterValue(
                            LaunchConfiguration("velocity_scaling"), value_type=float
                        ),
                        "acceleration_scaling": ParameterValue(
                            LaunchConfiguration("acceleration_scaling"), value_type=float
                        ),
                        "planning_time": ParameterValue(
                            LaunchConfiguration("planning_time"), value_type=float
                        ),
                        "home_named_target": LaunchConfiguration("home_named_target"),
                        "enable_pose_target": ParameterValue(
                            LaunchConfiguration("enable_pose_target"), value_type=bool
                        ),
                        "pose_target_csv": LaunchConfiguration("pose_target_csv"),
                        "pose_target_position_in_mm": ParameterValue(
                            LaunchConfiguration("pose_target_position_in_mm"), value_type=bool
                        ),
                        "pose_target_rpy_in_degrees": ParameterValue(
                            LaunchConfiguration("pose_target_rpy_in_degrees"), value_type=bool
                        ),
                        "pose_reference_frame": LaunchConfiguration("pose_reference_frame"),
                    },
                ],
            ),
        ]
    )
