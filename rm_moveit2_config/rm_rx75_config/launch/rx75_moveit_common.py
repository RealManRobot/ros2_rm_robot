import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_rx75_moveit_launch(
    description_xacro,
    srdf_file,
    controllers_file,
    joint_states_default="/joint_state_broadcaster/joint_states",
):
    package_name = "rm_rx75_config"
    joint_states_topic = LaunchConfiguration("joint_states_topic")

    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("rm_description"),
            "urdf",
            description_xacro,
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    robot_description_semantic = {
        "robot_description_semantic": load_file(package_name, f"config/{srdf_file}")
    }

    # Pass kinematics as raw dict (top-level keys, NOT wrapped under robot_description_kinematics).
    # MoveIt2 Foxy reads left_arm.kinematics_solver / right_arm.kinematics_solver directly.
    kinematics_yaml = load_yaml(package_name, "config/kinematics.yaml")

    joint_limits_yaml = load_yaml(package_name, "config/joint_limits.yaml")

    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        package_name,
        "config/ompl_planning.yaml",
    )
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    moveit_simple_controllers_yaml = load_yaml(
        package_name,
        f"config/{controllers_file}",
    )
    if "moveit_controller_manager" in moveit_simple_controllers_yaml:
        moveit_controllers = moveit_simple_controllers_yaml
        moveit_controllers["moveit_manage_controllers"] = False
    else:
        moveit_controllers = {
            "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
            "moveit_controller_manager": (
                "moveit_simple_controller_manager/MoveItSimpleControllerManager"
            ),
        }

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

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.15,
    }

    common_parameters = [
        robot_description,
        robot_description_semantic,
        kinematics_yaml,
        joint_limits_yaml,
        ompl_planning_pipeline_config,
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
                default_value=joint_states_default,
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(
                    get_package_share_directory(package_name),
                    "config",
                    "moveit.rviz",
                ),
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[
                    *common_parameters,
                    trajectory_execution,
                    moveit_controllers,
                    move_group_configuration,
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
                parameters=common_parameters,
                condition=IfCondition(LaunchConfiguration("use_rviz")),
                remappings=[
                    ("/joint_states", joint_states_topic),
                    ("joint_states", joint_states_topic),
                ],
                output="screen",
            ),
        ]
    )


def generate_rx75_gazebo_moveit_launch(description_xacro, srdf_file):
    return generate_rx75_moveit_launch(
        description_xacro,
        srdf_file,
        "moveit_controllers_gazebo.yaml",
        "/joint_state_broadcaster/joint_states",
    )


def generate_rx75_real_moveit_launch(description_xacro, srdf_file):
    return generate_rx75_moveit_launch(
        description_xacro,
        srdf_file,
        "moveit_controllers.yaml",
        "/joint_states",
    )
