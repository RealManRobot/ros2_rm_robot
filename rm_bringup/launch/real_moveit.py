"""Internal MoveIt actions for Foxy real-robot bringup."""

from pathlib import Path
import runpy
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import EmitEvent
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def build_moveit_config(arm_type, model="auto", mount_mappings=None):
    bringup = runpy.run_path(str(Path(__file__).with_name("rm_bringup.launch.py")))
    arm_type, _, plan = bringup["_resolve_selection"](arm_type, "real")
    description_share = Path(get_package_share_directory("rm_description"))
    description = runpy.run_path(str(description_share / "launch/rm_description.launch.py"))
    _, (model_file, _, mappings, dual_arm) = description["_resolve_model"](arm_type, model)
    mappings = dict(mappings)
    if dual_arm:
        mappings.update(mount_mappings or {})
    model_path = description_share / "urdf" / model_file
    config_path = Path(get_package_share_directory(plan[0])) / "config"
    stem = "rml_63" if plan[0] == "rm_63_config" else plan[0][:-len("_config")]
    semantic = stem + "_description.srdf"
    if dual_arm:
        semantic = "rm_" + arm_type.replace("-", "_") + "_description.srdf"

    def load_yaml(filename):
        with (config_path / filename).open() as stream:
            return yaml.safe_load(stream)

    controllers = load_yaml("moveit_controllers.yaml")
    if "moveit_controller_manager" not in controllers:
        controllers = {
            "moveit_simple_controller_manager": controllers,
            "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        }
    controllers["moveit_manage_controllers"] = False
    pipeline = {
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
    pipeline.update(load_yaml("ompl_planning.yaml"))
    # Foxy reads group kinematics directly, as in rx75_moveit_common.py.
    return {
        "description": {"robot_description": xacro.process_file(str(model_path), mappings=mappings).toxml()},
        "semantic": {"robot_description_semantic": (config_path / semantic).read_text()},
        "kinematics": load_yaml("kinematics.yaml"),
        "limits": {"robot_description_planning": load_yaml("joint_limits.yaml")},
        "pipeline": {"move_group": pipeline},
        "controllers": controllers,
        "rviz": str(config_path / "moveit.rviz"),
    }


def build_real_moveit_actions(
    context, arm_type, model, joint_states_topic, *, allow_execution, use_rviz,
):
    mount_defaults = {'left_xyz': '0 -0.075 0.5', 'left_rpy': '3.14 1.57 1.5707963267949', 'right_xyz': '0 0.075 0.5', 'right_rpy': '3.14 1.57 -1.5707963267949'}
    topic = joint_states_topic.strip()
    if not topic:
        raise RuntimeError("joint_states_topic must not be empty")
    config = build_moveit_config(arm_type, model, {
        name: LaunchConfiguration(name, default=default).perform(context)
        for name, default in mount_defaults.items()
    })
    remappings = [("joint_states", topic)]
    common = [config[key] for key in ("description", "semantic", "kinematics", "limits", "pipeline")]
    actions = [Node(
        package="moveit_ros_move_group", executable="move_group", output="screen",
        on_exit=[EmitEvent(event=Shutdown(reason="move_group exited"))],
        remappings=remappings,
        parameters=[*common, config["controllers"], {
            "allow_trajectory_execution": allow_execution,
            "publish_robot_description": True,
            "publish_robot_description_semantic": True,
            "publish_planning_scene": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
            "monitor_dynamics": False,
            "trajectory_execution.allowed_execution_duration_scaling": 1.2,
            "trajectory_execution.allowed_goal_duration_margin": 0.5,
            "trajectory_execution.allowed_start_tolerance": 0.15,
        }],
    )]
    if use_rviz:
        actions.append(Node(package="rviz2", executable="rviz2", name="rviz2",
                            arguments=["-d", config["rviz"]], output="screen",
                            parameters=common, remappings=remappings))
    return actions
