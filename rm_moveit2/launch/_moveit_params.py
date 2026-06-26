"""Foxy-native replacement for MoveItConfigsBuilder.

Loads MoveIt2 config parameters manually (URDF, SRDF, kinematics,
joint limits, OMPL planning, trajectory execution) so that
rm_moveit2 launch files no longer depend on moveit_configs_utils.
"""

import os
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory


def _load_file(package_name, file_path):
    abs_path = os.path.join(get_package_share_directory(package_name), file_path)
    with open(abs_path, "r") as f:
        return f.read()


def _load_yaml(package_name, file_path):
    abs_path = os.path.join(get_package_share_directory(package_name), file_path)
    with open(abs_path, "r") as f:
        return yaml.safe_load(f)


def load_moveit_params(
    urdf_package,
    urdf_file,
    config_package,
    srdf_file,
    kinematics_file="config/kinematics.yaml",
    joint_limits_file="config/joint_limits.yaml",
    ompl_file="config/ompl_planning.yaml",
    controllers_file="config/moveit_controllers.yaml",
    xacro_mappings=None,
):
    """Return a dict equivalent to MoveItConfigsBuilder(...).to_dict().

    Parameters
    ----------
    urdf_package : str
        Package containing the URDF xacro (e.g. "rm_description" or config pkg).
    urdf_file : str
        Relative path to URDF xacro inside urdf_package.
    config_package : str
        MoveIt config package name (e.g. "rm_rx75_config").
    srdf_file : str
        Relative path to SRDF inside config_package (e.g. "config/xxx.srdf").
    """

    # --- robot_description (URDF) ---
    xacro_kwargs = {}
    if xacro_mappings:
        xacro_kwargs["mappings"] = xacro_mappings
    urdf_xml = xacro.process_file(
        os.path.join(get_package_share_directory(urdf_package), urdf_file),
        **xacro_kwargs,
    )
    robot_description = {"robot_description": urdf_xml.toxml()}

    # --- robot_description_semantic (SRDF) ---
    robot_description_semantic = {
        "robot_description_semantic": _load_file(config_package, srdf_file)
    }

    # --- robot_description_kinematics ---
    kinematics_yaml = _load_yaml(config_package, kinematics_file)
    robot_description_kinematics = {
        "robot_description_kinematics": kinematics_yaml
    }

    # --- joint_limits ---
    joint_limits_yaml = _load_yaml(config_package, joint_limits_file)
    joint_limits = {}
    if joint_limits_yaml:
        joint_limits = {"robot_description_planning": joint_limits_yaml}

    # --- planning_pipelines (OMPL) ---
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
    ompl_yaml = _load_yaml(config_package, ompl_file)
    if ompl_yaml:
        ompl_planning_pipeline_config["move_group"].update(ompl_yaml)

    # --- trajectory_execution & controller manager ---
    controllers_yaml = _load_yaml(config_package, controllers_file)
    if "moveit_controller_manager" in controllers_yaml:
        moveit_controllers = controllers_yaml
        moveit_controllers["moveit_manage_controllers"] = False
    else:
        moveit_controllers = {
            "moveit_simple_controller_manager": controllers_yaml,
            "moveit_controller_manager": (
                "moveit_simple_controller_manager/MoveItSimpleControllerManager"
            ),
        }
    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.15,
    }

    # --- merge all ---
    params = {}
    params.update(robot_description)
    params.update(robot_description_semantic)
    params.update(robot_description_kinematics)
    params.update(joint_limits)
    params.update(ompl_planning_pipeline_config)
    params.update(trajectory_execution)
    params.update(moveit_controllers)
    return params
