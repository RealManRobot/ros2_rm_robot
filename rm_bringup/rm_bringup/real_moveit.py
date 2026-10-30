"""MoveIt nodes used by the unified real-robot bringup."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

from rm_bringup.variant_catalog import resolve_model
from rm_description.variant_catalog import resolve_model as resolve_description


def build_moveit_config(arm_type, model="auto", mount_mappings=None):
    plan = resolve_model(arm_type, "real")
    spec = resolve_description(plan.arm_type, model)
    description = Path(get_package_share_directory("rm_description"))
    model_file = description / "urdf" / spec.model_file
    if not model_file.is_file():
        raise ValueError(f"Robot description is missing: {model_file}")
    stem = (
        "rml_63_description" if plan.moveit.package == "rm_63_config"
        else f"{plan.moveit.package.removesuffix('_config')}_description"
    )
    semantic = f"config/{stem}.srdf"
    if plan.arm_profile.topology == "dual":
        semantic = f"config/rm_{plan.arm_type.replace('-', '_')}_description.srdf"
    mappings = dict(spec.xacro_mappings)
    if spec.dual_arm:
        mappings.update(mount_mappings or {})
    return (
        MoveItConfigsBuilder(stem, package_name=plan.moveit.package)
        .robot_description(file_path=str(model_file), mappings=mappings)
        .robot_description_semantic(file_path=semantic)
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


def build_real_moveit_actions(
    context, arm_type, model, joint_states_topic, *, allow_execution, use_rviz,
):
    mount_defaults = {
        "left_xyz": "0 -0.075 1.5",
        "left_rpy": "1.5707963267949 -1.5707963267949 0",
        "right_xyz": "0 0.075 1.5",
        "right_rpy": "-1.5707963267949 -1.5707963267949 0",
    }
    try:
        config = build_moveit_config(
            arm_type, model,
            {name: LaunchConfiguration(name, default=default).perform(context)
             for name, default in mount_defaults.items()},
        )
    except ValueError as exc:
        raise RuntimeError(str(exc)) from exc
    topic = joint_states_topic.strip()
    if not topic:
        raise RuntimeError("joint_states_topic must not be empty")
    remappings = [("joint_states", topic)]
    actions = [Node(
        package="moveit_ros_move_group", executable="move_group",
        output="screen", remappings=remappings,
        parameters=[config.to_dict(), {
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
        rviz_file = Path(config.package_path) / "config" / "moveit.rviz"
        actions.append(Node(
            package="rviz2", executable="rviz2", name="rviz2",
            arguments=["-d", str(rviz_file)], output="screen",
            parameters=[config.robot_description, config.robot_description_semantic,
                        config.robot_description_kinematics,
                        config.planning_pipelines, config.joint_limits],
            remappings=remappings,
        ))
    return actions
