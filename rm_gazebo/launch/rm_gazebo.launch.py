import re
import runpy

from ament_index_python.packages import get_package_share_directory
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, str(Path(__file__).resolve().parent))

from gz_demo_common import (  # noqa: E402
    generate_gazebo_classic_demo_actions,
    resolve_auto_joint_states_topic,
)


NORMAL_CONTROLLERS = (
    "joint_state_broadcaster",
    "rm_group_controller",
)
RX75_CONTROLLERS = (
    "joint_state_broadcaster",
    "left_arm_controller",
    "right_arm_controller",
)
DEFAULT_JOINT_STATES_TOPIC = "/joint_states"
RX75_JOINT_STATES_TOPIC = "/joint_state_broadcaster/joint_states"
ECO63_STATIC_TRANSFORMS = (
    {
        "x": 0,
        "y": 0,
        "z": 0,
        "roll": 0,
        "pitch": 0,
        "yaw": 0,
        "parent": "world",
        "child": "base_root",
    },
)
_TRUE_VALUES = {"1", "true", "yes", "on"}
_FALSE_VALUES = {"0", "false", "no", "off"}


# Each entry is a compatibility contract with one of the 21 historical
# gazebo_*_demo.launch.py entry points.
MODEL_CATALOG = {
    "63": {
        "urdf_filename": "gazebo_63_description.urdf.xacro",
        "robot_name_in_model": "rml_63_description",
        "controller_names": NORMAL_CONTROLLERS,
    },
    "63-6f": {
        "urdf_filename": "gazebo_63_6fb_description.urdf.xacro",
        "robot_name_in_model": "rml_63_description",
        "controller_names": NORMAL_CONTROLLERS,
        "xacro_mappings": {"link6_type": "Link6_6f"},
    },
    "63-6fb": {
        "urdf_filename": "gazebo_63_6fb_description.urdf.xacro",
        "robot_name_in_model": "rml_63_description",
        "controller_names": NORMAL_CONTROLLERS,
        "xacro_mappings": {"link6_type": "Link6_6fb"},
    },
    "63_iii": {
        "urdf_filename": "gazebo_63_III_description.urdf.xacro",
        "robot_name_in_model": "rml_63_description",
        "controller_names": NORMAL_CONTROLLERS,
        "xacro_mappings": {
            "link6_type": "Link6",
            "base_type": "base_link_III",
        },
    },
    "63_iii-6fb": {
        "urdf_filename": "gazebo_63_III_description.urdf.xacro",
        "robot_name_in_model": "rml_63_description",
        "controller_names": NORMAL_CONTROLLERS,
        "xacro_mappings": {
            "link6_type": "Link6_6fb",
            "base_type": "base_link_III",
        },
    },
    "65": {
        "urdf_filename": "gazebo_65_description.urdf.xacro",
        "robot_name_in_model": "rm_65_description",
        "controller_names": NORMAL_CONTROLLERS,
    },
    "65-6f": {
        "urdf_filename": "gazebo_65_6fb_description.urdf.xacro",
        "robot_name_in_model": "rm_65_description",
        "controller_names": NORMAL_CONTROLLERS,
        "xacro_mappings": {"link6_type": "Link6_6f"},
    },
    "65-6fb": {
        "urdf_filename": "gazebo_65_6fb_description.urdf.xacro",
        "robot_name_in_model": "rm_65_description",
        "controller_names": NORMAL_CONTROLLERS,
        "xacro_mappings": {"link6_type": "Link6_6fb"},
    },
    "75": {
        "urdf_filename": "gazebo_75_description.urdf.xacro",
        "robot_name_in_model": "rm_75_description",
        "controller_names": NORMAL_CONTROLLERS,
    },
    "75-6f": {
        "urdf_filename": "gazebo_75_6fb_description.urdf.xacro",
        "robot_name_in_model": "rm_75_description",
        "controller_names": NORMAL_CONTROLLERS,
        "xacro_mappings": {"link7_type": "Link7_6f"},
    },
    "75-6fb": {
        "urdf_filename": "gazebo_75_6fb_description.urdf.xacro",
        "robot_name_in_model": "rm_75_description",
        "controller_names": NORMAL_CONTROLLERS,
        "xacro_mappings": {"link7_type": "Link7_6fb"},
    },
    "eco62": {
        "urdf_filename": "gazebo_eco62_description.urdf.xacro",
        "robot_name_in_model": "rm_eco62_description",
        "controller_names": NORMAL_CONTROLLERS,
    },
    "eco63": {
        "urdf_filename": "gazebo_eco63_description.urdf.xacro",
        "robot_name_in_model": "rm_eco63_description",
        "controller_names": NORMAL_CONTROLLERS,
        "xacro_mappings": {"link6_type": "Link6"},
        "static_transforms": ECO63_STATIC_TRANSFORMS,
    },
    "eco63-6fb": {
        "urdf_filename": "gazebo_eco63_6fb_description.urdf.xacro",
        "robot_name_in_model": "rm_eco63_description",
        "controller_names": NORMAL_CONTROLLERS,
        "xacro_mappings": {"link6_type": "Link6_6fb"},
        "static_transforms": ECO63_STATIC_TRANSFORMS,
    },
    "eco65": {
        "urdf_filename": "gazebo_eco65_description.urdf.xacro",
        "robot_name_in_model": "rm_eco65_description",
        "controller_names": NORMAL_CONTROLLERS,
    },
    "eco65-6f": {
        "urdf_filename": "gazebo_eco65_6fb_description.urdf.xacro",
        "robot_name_in_model": "rm_eco65_description",
        "controller_names": NORMAL_CONTROLLERS,
        "xacro_mappings": {"link6_type": "Link6_6f"},
    },
    "eco65-6fb": {
        "urdf_filename": "gazebo_eco65_6fb_description.urdf.xacro",
        "robot_name_in_model": "rm_eco65_description",
        "controller_names": NORMAL_CONTROLLERS,
        "xacro_mappings": {"link6_type": "Link6_6fb"},
    },
    "gen72": {
        "urdf_filename": "gazebo_gen72_description.urdf.xacro",
        "robot_name_in_model": "rm_gen72_description",
        "controller_names": NORMAL_CONTROLLERS,
    },
    "gen72_ii": {
        "urdf_filename": "gazebo_gen72_II_description.urdf.xacro",
        "robot_name_in_model": "rm_gen72_description",
        "controller_names": NORMAL_CONTROLLERS,
    },
    "rx75-6fb": {
        "urdf_filename": "gazebo_rx75_6fb_description.urdf.xacro",
        "robot_name_in_model": "rm_rx75_dual",
        "controller_names": RX75_CONTROLLERS,
        "joint_states_topic_default": RX75_JOINT_STATES_TOPIC,
    },
    "rx75-6fb-v": {
        "urdf_filename": "gazebo_rx75_6fb_v_description.urdf.xacro",
        "robot_name_in_model": "rm_rx75_dual",
        "controller_names": RX75_CONTROLLERS,
        "joint_states_topic_default": RX75_JOINT_STATES_TOPIC,
    },
}

if len(MODEL_CATALOG) != 21:
    raise RuntimeError("Gazebo model catalog must contain 21 arm types.")

ARM_TYPES = tuple(MODEL_CATALOG)


ARM_TYPE_ALIASES = {
    "63": "63",
    "rm63": "63",
    "rml63": "63",
    "63iii": "63_iii",
    "rm63iii": "63_iii",
    "rml63iii": "63_iii",
    "65": "65",
    "rm65": "65",
    "75": "75",
    "rm75": "75",
    "eco62": "eco62",
    "rmeco62": "eco62",
    "eco63": "eco63",
    "rmeco63": "eco63",
    "eco65": "eco65",
    "rmeco65": "eco65",
    "gen72": "gen72",
    "rmgen72": "gen72",
    "gen72ii": "gen72_ii",
    "rmgen72ii": "gen72_ii",
    "rx75": "rx75",
    "rmrx75": "rx75",
}

ARM_TYPE_ALIASES.update({
    alias + arm_type[len(family):].replace("-", ""): arm_type
    for alias, family in ARM_TYPE_ALIASES.items()
    for arm_type in MODEL_CATALOG
    if arm_type == family or arm_type.startswith(family + "-")
})


def _compact_token(value):
    normalized = re.sub(r"[\s-]+", "_", value.lower())
    normalized = re.sub(r"_+", "_", normalized).strip("_")
    return normalized.replace("_", "")


def normalize_arm_type(value):
    raw_value = value.strip() if isinstance(value, str) else ""
    normalized = ARM_TYPE_ALIASES.get(_compact_token(raw_value))
    if normalized not in MODEL_CATALOG:
        raise ValueError(
            f"Unsupported arm_type: {raw_value or value}. "
            f"Valid arm types: {', '.join(ARM_TYPES)}."
        )
    return normalized


def resolve_model(arm_type, model="auto"):
    arm_type = normalize_arm_type(arm_type)
    description = runpy.run_path(str(Path(get_package_share_directory("rm_description"))
                                    / "launch/rm_description.launch.py"))
    description["_resolve_model"](arm_type, model)
    return {**MODEL_CATALOG[arm_type], "arm_type": arm_type, "model": model}


def _boolean_value(name, value):
    normalized = value.strip().casefold()
    if normalized in _TRUE_VALUES:
        return True
    if normalized in _FALSE_VALUES:
        return False
    raise RuntimeError(
        f"Launch argument '{name}' must be a boolean value; received '{value}'."
    )


def _launch_setup(context):
    specification = resolve_model(
        LaunchConfiguration("arm_type").perform(context),
        LaunchConfiguration("model").perform(context),
    )
    requested_joint_states_topic = LaunchConfiguration(
        "joint_states_topic"
    ).perform(
        context
    )
    joint_states_topic = resolve_auto_joint_states_topic(
        requested_joint_states_topic,
        specification.get(
            "joint_states_topic_default",
            DEFAULT_JOINT_STATES_TOPIC,
        ),
    )
    requested_clock_topic = LaunchConfiguration(
        "clock_topic"
    ).perform(context).strip()
    if not requested_clock_topic:
        raise RuntimeError("Launch argument 'clock_topic' must not be empty.")
    clock_topic = (
        "/clock" if requested_clock_topic == "auto" else requested_clock_topic
    )

    start_gazebo = _boolean_value(
        "start_gazebo", LaunchConfiguration("start_gazebo").perform(context)
    )
    use_gazebo_gui = _boolean_value(
        "use_gazebo_gui",
        LaunchConfiguration("use_gazebo_gui").perform(context),
    )
    use_sim_time = _boolean_value(
        "use_sim_time", LaunchConfiguration("use_sim_time").perform(context)
    )
    spawn_entity_timeout = LaunchConfiguration(
        "spawn_entity_timeout"
    ).perform(context).strip()
    try:
        if float(spawn_entity_timeout) <= 0:
            raise ValueError
    except ValueError as exc:
        raise RuntimeError(
            "Launch argument 'spawn_entity_timeout' must be a positive number."
        ) from exc

    return generate_gazebo_classic_demo_actions(
        urdf_filename=specification["urdf_filename"],
        robot_name_in_model=specification["robot_name_in_model"],
        controller_names=specification["controller_names"],
        xacro_mappings=specification.get("xacro_mappings"),
        arm_type=specification["arm_type"],
        model=specification["model"],
        static_transforms=specification.get("static_transforms"),
        start_gazebo="true" if start_gazebo else "false",
        use_gazebo_gui="true" if use_gazebo_gui else "false",
        clock_topic=clock_topic,
        joint_states_topic=joint_states_topic,
        spawn_entity_timeout=spawn_entity_timeout,
        use_sim_time=use_sim_time,
    )


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "arm_type",
                description=(
                    "Robot model including end-link version, e.g. 65-6f or eco63-6fb"
                ),
            ),
            DeclareLaunchArgument(
                "model", default_value="auto", choices=["auto", "stl", "dae"],
                description="auto prefers available DAE models; stl selects the original mesh",
            ),
            DeclareLaunchArgument(
                "start_gazebo",
                default_value="true",
                description=(
                    "Start Gazebo Classic; false connects to an existing world"
                ),
            ),
            DeclareLaunchArgument("use_gazebo_gui", default_value="true"),
            DeclareLaunchArgument(
                "clock_topic",
                default_value="auto",
                description="ROS clock topic, or auto for /clock",
            ),
            DeclareLaunchArgument(
                "joint_states_topic",
                default_value="auto",
                choices=["auto"],
                description=(
                    "Only auto is currently supported; it selects the legacy "
                    "default for the chosen model"
                ),
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use the Gazebo clock in robot_state_publisher",
            ),
            DeclareLaunchArgument(
                "spawn_entity_timeout",
                default_value="120",
                description="Seconds to wait for Gazebo spawn_entity service",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
