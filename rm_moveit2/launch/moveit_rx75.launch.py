import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def build_moveit_config():
    description_path = os.path.join(
        get_package_share_directory("rm_description"),
        "urdf",
        "rm_rx75-6fb_v.urdf.xacro",
    )

    return (
        MoveItConfigsBuilder(
            "rm_rx75_6fb_description",
            package_name="rm_rx75_config",
        )
        .robot_description(file_path=description_path)
        .robot_description_semantic(
            file_path="config/rm_rx75_6fb_description.srdf",
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl", "pilz_industrial_motion_planner"],
            load_all=False,
        )
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )


def generate_launch_description():
    moveit_config = build_moveit_config()

    return LaunchDescription(
        [
            DeclareLaunchArgument("planning_group", default_value="right_arm"),
            DeclareLaunchArgument("current_state_wait_sec", default_value="10.0"),
            DeclareLaunchArgument("velocity_scaling", default_value="0.3"),
            DeclareLaunchArgument("acceleration_scaling", default_value="0.3"),
            DeclareLaunchArgument("planning_time", default_value="5.0"),
            DeclareLaunchArgument("home_named_target", default_value="forward"),
            DeclareLaunchArgument("prefer_named_start", default_value="true"),
            DeclareLaunchArgument("enable_cartesian_demo", default_value="false"),
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
                        "prefer_named_start": ParameterValue(
                            LaunchConfiguration("prefer_named_start"), value_type=bool
                        ),
                        "enable_cartesian_demo": ParameterValue(
                            LaunchConfiguration("enable_cartesian_demo"), value_type=bool
                        ),
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
