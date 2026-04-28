import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
            "rm_rx75_dual",
            package_name="rm_rx75_config",
        )
        .robot_description(file_path=description_path)
        .robot_description_semantic(
            file_path="config/rm_rx75_6fb_v_description.srdf",
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(
            file_path="config/moveit_controllers_gazebo.yaml",
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
    joint_states_topic = LaunchConfiguration("joint_states_topic")

    allow_trajectory_execution = ParameterValue(
        LaunchConfiguration("allow_trajectory_execution"),
        value_type=bool,
    )
    should_publish = ParameterValue(
        LaunchConfiguration("publish_monitored_planning_scene"),
        value_type=bool,
    )

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": allow_trajectory_execution,
        "capabilities": ParameterValue(
            LaunchConfiguration("capabilities"),
            value_type=str,
        ),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"),
            value_type=str,
        ),
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
    }

    trajectory_execution_overrides = {
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.15,
    }

    rviz_parameters = [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
        {"use_sim_time": True},
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("allow_trajectory_execution", default_value="true"),
            DeclareLaunchArgument(
                "publish_monitored_planning_scene",
                default_value="true",
            ),
            DeclareLaunchArgument("capabilities", default_value=""),
            DeclareLaunchArgument("disable_capabilities", default_value=""),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "joint_states_topic",
                default_value="/joint_state_broadcaster/joint_states",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(
                    get_package_share_directory("rm_rx75_config"),
                    "config",
                    "moveit.rviz",
                ),
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    move_group_configuration,
                    trajectory_execution_overrides,
                    {"use_sim_time": True},
                ],
                remappings=[
                    ("/joint_states", joint_states_topic),
                    ("joint_states", joint_states_topic),
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                parameters=rviz_parameters,
                condition=IfCondition(LaunchConfiguration("use_rviz")),
                remappings=[
                    ("/joint_states", joint_states_topic),
                    ("joint_states", joint_states_topic),
                ],
                output="screen",
            ),
        ]
    )
