"""Unified robot-description launch entry point for all supported RealMan arms."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction
from launch.events import Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_TRUE_VALUES = {"1", "true", "yes", "on"}
_FALSE_VALUES = {"0", "false", "no", "off"}

# Keep the supported matrix in this public entry point.  This deliberately
# avoids another helper/catalog file while preserving the legacy model files.
_MODELS = {
    "63": ("rml_63.urdf", "rm_63.rviz", {}, False),
    "63-6f": (
        "rml_63.urdf.xacro", "rm_63.rviz", {"link6_type": "Link6_6f"}, False
    ),
    "63-6fb": (
        "rml_63.urdf.xacro", "rm_63.rviz", {"link6_type": "Link6_6fb"}, False
    ),
    "63_iii": (
        "rml_63.urdf.xacro", "rm_63.rviz",
        {"link6_type": "Link6", "base_type": "base_link_III"}, False
    ),
    "63_iii-6fb": (
        "rml_63.urdf.xacro", "rm_63.rviz",
        {"link6_type": "Link6_6fb", "base_type": "base_link_III"}, False
    ),
    "65": ("rm_65.urdf", "rm_65.rviz", {}, False),
    "65-6f": (
        "rm_65.urdf.xacro", "rm_65.rviz", {"link6_type": "Link6_6f"}, False
    ),
    "65-6fb": (
        "rm_65.urdf.xacro", "rm_65.rviz", {"link6_type": "Link6_6fb"}, False
    ),
    "75": ("rm_75.urdf", "rm_75.rviz", {}, False),
    "75-6f": (
        "rm_75.urdf.xacro", "rm_75.rviz", {"link7_type": "Link7_6f"}, False
    ),
    "75-6fb": (
        "rm_75.urdf.xacro", "rm_75.rviz", {"link7_type": "Link7_6fb"}, False
    ),
    "eco62": (
        "rm_eco62.urdf.xacro", "rm_eco62.rviz", {}, False
    ),
    "eco63": ("rm_eco63.urdf", "rm_eco63.rviz", {}, False),
    "eco63-6fb": (
        "rm_eco63.urdf.xacro", "rm_eco63.rviz",
        {"link6_type": "Link6_6fb"}, False
    ),
    "eco65": ("rm_eco65.urdf", "rm_eco65.rviz", {}, False),
    "eco65-6f": (
        "rm_eco65.urdf.xacro", "rm_eco65.rviz",
        {"link6_type": "Link6_6f"}, False
    ),
    "eco65-6fb": (
        "rm_eco65.urdf.xacro", "rm_eco65.rviz",
        {"link6_type": "Link6_6fb"}, False
    ),
    "gen72": ("rm_gen72.urdf", "rm_gen72.rviz", {}, False),
    "gen72_ii": (
        "rm_gen72_II.urdf", "rm_gen72.rviz", {}, False
    ),
    "rx75-6fb": (
        "rm_rx75-6fb.urdf.xacro", "rm_rx75.rviz", {}, True
    ),
    "rx75-6fb-v": (
        "rm_rx75-6fb_v.urdf.xacro", "rm_rx75.rviz", {}, True
    ),
}


MODEL_FORMATS = ("auto", "stl", "dae")
_DAE_MODELS = {
    arm_type: ("imported/" + filename + ".urdf.xacro", _MODELS[arm_type][1], {}, _MODELS[arm_type][3])
    for arm_type, filename in {
        "65": "65_standard",
        "65-6f": "65_6f",
        "65-6fb": "65_6fb",
        "75": "75_standard",
        "75-6f": "75_6f",
        "75-6fb": "75_6fb",
        "eco62": "eco62_standard",
        "eco63": "eco63_standard",
        "eco63-6fb": "eco63_6fb",
        "eco65": "eco65_standard",
        "eco65-6f": "eco65_6f",
        "eco65-6fb": "eco65_6fb",
        "rx75-6fb": "rx75_6fb",
        "rx75-6fb-v": "rx75_6fb_v",
    }.items()
}


def _resolve_model(arm_type, model="auto"):
    normalized = arm_type.strip().casefold().replace("_6", "-6")
    normalized = normalized.replace("-6fb_v", "-6fb-v")
    if normalized not in _MODELS:
        raise RuntimeError(
            f"Unsupported arm_type='{arm_type}'. Valid values: "
            + ", ".join(_MODELS)
        )
    model = model.strip().casefold()
    if model not in MODEL_FORMATS:
        raise RuntimeError(f"Unsupported model='{model}'; use auto, stl or dae.")
    if model == "dae" and normalized not in _DAE_MODELS:
        raise RuntimeError(f"No DAE model for arm_type='{arm_type}'.")
    catalog = _DAE_MODELS if model != "stl" and normalized in _DAE_MODELS else _MODELS
    return normalized, catalog[normalized]



def _apply_visual_model(robot_description, arm_type, model="auto"):
    """Apply DAE visuals without changing the caller's physical model."""
    _, spec = _resolve_model(arm_type, model)
    if not spec[0].startswith("imported/"):
        return robot_description

    from copy import deepcopy
    from pathlib import Path
    import xml.etree.ElementTree as ET
    import xacro

    source = Path(get_package_share_directory("rm_description")) / "urdf" / spec[0]
    visuals = ET.fromstring(xacro.process_file(str(source)).toxml())
    robot = ET.fromstring(robot_description)
    links = {link.get("name"): link for link in robot.findall("link")}
    replaced = set()
    for source_link in visuals.findall("link"):
        if not source_link.findall("visual"):
            continue
        name = source_link.get("name")
        if name not in links:
            raise ValueError("DAE visual link '{}' is missing from arm_type='{}'.".format(name, arm_type))
        target = links[name]
        for visual in target.findall("visual"):
            target.remove(visual)
        for visual in source_link.findall("visual"):
            visual = deepcopy(visual)
            for mesh in visual.findall("geometry/mesh"):
                filename = mesh.get("filename", "")
                if filename.startswith("package://"):
                    package, relative = filename[len("package://"):].split("/", 1)
                    mesh.set("filename", (Path(get_package_share_directory(package)) / relative).as_uri())
            target.append(visual)
        replaced.add(name)
    # Gazebo solid colors override the colors stored in DAE materials.
    for gazebo in robot.findall("gazebo"):
        if gazebo.get("reference") in replaced:
            for material in gazebo.findall("material"):
                gazebo.remove(material)
    return ET.tostring(robot, encoding="unicode")

def _launch_value(context, name):
    return LaunchConfiguration(name).perform(context)


def _boolean_value(name, value):
    normalized = value.strip().casefold()
    if normalized in _TRUE_VALUES:
        return True
    if normalized in _FALSE_VALUES:
        return False
    raise RuntimeError(
        f"Launch argument '{name}' must be a boolean value; received '{value}'."
    )


def _xacro_command(model_path, mappings):
    command = [FindExecutable(name="xacro"), " ", model_path]
    for name, value in mappings.items():
        if any(character.isspace() for character in value):
            command.extend([" ", f"{name}:='", value, "'"])
        else:
            command.extend([" ", f"{name}:=", value])
    return Command(command)


def _launch_setup(context):
    requested_arm_type = _launch_value(context, "arm_type")
    requested_model = _launch_value(context, "model")
    arm_type, legacy = _resolve_model(requested_arm_type, "stl")
    mappings = dict(legacy[2])
    for mapping_name in ("link6_type", "link7_type", "base_type"):
        override = _launch_value(context, f"{mapping_name}_override").strip()
        if override:
            if mapping_name not in mappings:
                raise RuntimeError(f"'{mapping_name}_override' is not valid for '{requested_arm_type}'.")
            mappings[mapping_name] = override
    if mappings != legacy[2]:
        if requested_model.strip().casefold() == "dae":
            raise RuntimeError("Custom STL mesh overrides cannot be used with model='dae'.")
        if requested_model.strip().casefold() == "auto":
            requested_model = "stl"
    _, spec = _resolve_model(
        requested_arm_type, requested_model
    )
    model_file, rviz_file, fixed_mappings, dual_arm = spec
    description_share = get_package_share_directory("rm_description")
    model_path = os.path.join(description_share, "urdf", model_file)
    rviz_path = os.path.join(description_share, "rviz", rviz_file)
    if not os.path.isfile(model_path):
        raise RuntimeError(
            f"Model file for {arm_type} does not exist: {model_path}"
        )

    if model_file.startswith("imported/"):
        mappings = dict(fixed_mappings)

    if dual_arm:
        mappings.update(
            {
                "left_xyz": _launch_value(context, "left_xyz"),
                "left_rpy": _launch_value(context, "left_rpy"),
                "right_xyz": _launch_value(context, "right_xyz"),
                "right_rpy": _launch_value(context, "right_rpy"),
            }
        )

    use_sim_time = _boolean_value(
        "use_sim_time", _launch_value(context, "use_sim_time")
    )
    use_gui = _boolean_value(
        "use_joint_state_publisher_gui",
        _launch_value(context, "use_joint_state_publisher_gui"),
    )
    use_rviz = _boolean_value("use_rviz", _launch_value(context, "use_rviz"))

    bridge_value = _launch_value(context, "use_joint_state_bridge")
    if bridge_value.strip().casefold() == "auto":
        use_bridge = dual_arm
    else:
        use_bridge = _boolean_value("use_joint_state_bridge", bridge_value)

    joint_states_topic = _launch_value(context, "joint_states_topic").strip()
    left_joint_states_topic = _launch_value(
        context, "left_joint_states_topic"
    ).strip()
    right_joint_states_topic = _launch_value(
        context, "right_joint_states_topic"
    ).strip()
    if not joint_states_topic:
        raise RuntimeError("Launch argument 'joint_states_topic' must not be empty.")
    if use_bridge and (not left_joint_states_topic or not right_joint_states_topic):
        raise RuntimeError(
            "The left and right joint-state topics must not be empty when the bridge is enabled."
        )

    robot_state_parameters = {
        "robot_description": ParameterValue(_xacro_command(model_path, mappings), value_type=str)
    }
    if use_sim_time:
        robot_state_parameters["use_sim_time"] = True

    robot_state_kwargs = {}
    if joint_states_topic != "/joint_states":
        robot_state_kwargs["remappings"] = [
            ("joint_states", joint_states_topic)
        ]

    actions = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            respawn=True,
            parameters=[robot_state_parameters],
            output="screen",
            **robot_state_kwargs,
        )
    ]

    if use_gui:
        gui_kwargs = {}
        if joint_states_topic != "/joint_states":
            gui_kwargs["remappings"] = [("joint_states", joint_states_topic)]
        if use_sim_time:
            gui_kwargs["parameters"] = [{"use_sim_time": True}]
        actions.append(
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
                **gui_kwargs,
            )
        )

    if use_bridge:
        bridge_parameters = {}
        if left_joint_states_topic != "/left_arm/joint_states":
            bridge_parameters["left_topic"] = left_joint_states_topic
        if right_joint_states_topic != "/right_arm/joint_states":
            bridge_parameters["right_topic"] = right_joint_states_topic
        if joint_states_topic != "/joint_states":
            bridge_parameters["output_topic"] = joint_states_topic
        if use_sim_time:
            bridge_parameters["use_sim_time"] = True
        bridge_kwargs = {}
        if bridge_parameters:
            bridge_kwargs["parameters"] = [bridge_parameters]
        actions.append(
            Node(
                on_exit=[
                    EmitEvent(
                        event=Shutdown(
                            reason="RX75 joint-state bridge exited"
                        )
                    )
                ],
                package="rm_description",
                executable="dual_arm_joint_state_bridge.py",
                name="dual_arm_joint_state_bridge",
                output="screen",
                **bridge_kwargs,
            )
        )

    if use_rviz:
        rviz_kwargs = {"parameters": [robot_state_parameters]}
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_path],
                output="screen",
                **rviz_kwargs,
            )
        )

    return actions


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
                "model", default_value="auto", choices=MODEL_FORMATS,
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "joint_states_topic", default_value="/joint_states"
            ),
            DeclareLaunchArgument(
                "use_joint_state_bridge",
                default_value="auto",
                description="auto enables the bridge for dual-arm RX75 models",
            ),
            DeclareLaunchArgument(
                "left_joint_states_topic",
                default_value="/left_arm/joint_states",
            ),
            DeclareLaunchArgument(
                "right_joint_states_topic",
                default_value="/right_arm/joint_states",
            ),
            DeclareLaunchArgument(
                "use_joint_state_publisher_gui", default_value="false"
            ),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("left_xyz", default_value="0 0.075 1.5"),
            DeclareLaunchArgument(
                "left_rpy", default_value="3.14 -1.57 1.5707963267949"
            ),
            DeclareLaunchArgument("right_xyz", default_value="0 -0.075 1.5"),
            DeclareLaunchArgument(
                "right_rpy", default_value="3.14 -1.57 -1.5707963267949"
            ),
            # Internal compatibility hooks used by legacy display wrappers.
            DeclareLaunchArgument("link6_type_override", default_value=""),
            DeclareLaunchArgument("link7_type_override", default_value=""),
            DeclareLaunchArgument("base_type_override", default_value=""),
            OpaqueFunction(function=_launch_setup),
        ]
    )
