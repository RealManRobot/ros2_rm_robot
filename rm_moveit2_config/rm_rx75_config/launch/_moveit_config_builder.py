import os

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def _base_builder():
    return (
        MoveItConfigsBuilder(
            "rm_rx75_dual",
            package_name="rm_rx75_config",
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl", "pilz_industrial_motion_planner"],
            load_all=False,
        )
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
    )


def build_real_moveit_config():
    return (
        _base_builder()
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory("rm_description"),
                "urdf",
                "rm_rx75-6fb.urdf.xacro",
            )
        )
        .robot_description_semantic(
            file_path="config/rm_rx75_6fb_description.srdf",
        )
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
        .to_moveit_configs()
    )


def build_sim_moveit_config():
    return (
        _base_builder()
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory("rm_description"),
                "urdf",
                "rm_rx75-6fb_v.urdf.xacro",
            )
        )
        .robot_description_semantic(
            file_path="config/rm_rx75_6fb_v_description.srdf",
        )
        .trajectory_execution(
            file_path="config/moveit_controllers_gazebo.yaml",
            moveit_manage_controllers=False,
        )
        .to_moveit_configs()
    )
