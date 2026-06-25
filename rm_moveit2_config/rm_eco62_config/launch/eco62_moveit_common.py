import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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


def generate_moveit_gazebo_launch(robot_xacro, xacro_mappings=None):
    package_name = "rm_eco62_config"

    tutorial_arg = DeclareLaunchArgument(
        "rviz_tutorial",
        default_value="False",
        description="Tutorial flag",
    )

    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory(package_name),
            "config",
            robot_xacro,
        ),
        mappings=xacro_mappings or {},
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    robot_description_semantic = {
        "robot_description_semantic": load_file(
            package_name,
            "config/rm_eco62_description.srdf",
        )
    }

    kinematics_yaml = load_yaml(package_name, "config/kinematics.yaml")

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
        "config/moveit_controllers.yaml",
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": (
            "moveit_simple_controller_manager/MoveItSimpleControllerManager"
        ),
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "use_fake_hardware": False,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    rviz_config = LaunchConfiguration("rviz_config")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
    )

    return LaunchDescription(
        [
            tutorial_arg,
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(
                    get_package_share_directory(package_name),
                    "config",
                    "moveit.rviz",
                ),
            ),
            rviz_node,
            move_group_node,
        ]
    )
