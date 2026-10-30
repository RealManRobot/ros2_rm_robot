import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from rm_description.variant_catalog import (
    MODEL_FORMATS, normalize_arm_type, resolve_gazebo_model,
)

sys.path.insert(0, str(Path(__file__).resolve().parent))

from gz_demo_common import (  # noqa: E402
    generate_gz_demo_actions,
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
    },
    "eco63-6fb": {
        "urdf_filename": "gazebo_eco63_6fb_description.urdf.xacro",
        "robot_name_in_model": "rm_eco63_description",
        "controller_names": NORMAL_CONTROLLERS,
        "xacro_mappings": {"link6_type": "Link6_6fb"},
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


def resolve_model(arm_type, model="auto"):
    arm_type = normalize_arm_type(arm_type)
    specification = MODEL_CATALOG[arm_type]
    selected_model = resolve_gazebo_model(arm_type, model)
    return {**specification, "arm_type": arm_type, "model": selected_model}


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

    return generate_gz_demo_actions(
        urdf_filename=specification["urdf_filename"],
        robot_name_in_model=specification["robot_name_in_model"],
        controller_names=specification["controller_names"],
        xacro_mappings=specification.get("xacro_mappings"),
        arm_type=specification["arm_type"],
        model=specification["model"],
        start_gazebo=LaunchConfiguration("start_gazebo"),
        joint_states_topic=joint_states_topic,
        use_sim_time=LaunchConfiguration("use_sim_time"),
    )


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "arm_type",
                description=(
                    "Robot model including end-link version, e.g. 65, "
                    "65-6f, eco63-6fb, or rx75-6fb-v"
                ),
            ),
            DeclareLaunchArgument(
                "model", default_value="auto", choices=MODEL_FORMATS,
                description="auto prefers available GLB visuals; stl selects the original mesh",
            ),
            DeclareLaunchArgument(
                "start_gazebo",
                default_value="true",
                description="Start Gazebo Sim; false connects to an existing world",
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
            OpaqueFunction(function=_launch_setup),
        ]
    )
