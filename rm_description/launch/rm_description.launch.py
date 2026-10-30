# Copyright 2026 realman-robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unified robot-description launch entry point for all supported RealMan arms."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from rm_description.variant_catalog import (
    DESCRIPTION_MODELS, MODEL_FORMATS, normalize_arm_type, resolve_model,
)


_TRUE_VALUES = {"1", "true", "yes", "on"}
_FALSE_VALUES = {"0", "false", "no", "off"}


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
    try:
        spec = resolve_model(
            requested_arm_type, _launch_value(context, "model"),
        )
    except ValueError as exc:
        raise RuntimeError(str(exc)) from exc

    arm_type = normalize_arm_type(requested_arm_type)
    description_share = get_package_share_directory("rm_description")
    model_path = os.path.join(description_share, "urdf", spec.model_file)
    rviz_path = os.path.join(description_share, "rviz", spec.rviz_file)
    if not os.path.isfile(model_path):
        raise RuntimeError(
            f"Model file for {arm_type} does not exist: {model_path}"
        )

    mappings = dict(spec.xacro_mappings)
    original_mappings = dict(DESCRIPTION_MODELS[arm_type].xacro_mappings)
    for mapping_name in ("link6_type", "link7_type", "base_type"):
        override = _launch_value(context, f"{mapping_name}_override").strip()
        if override:
            if mapping_name not in original_mappings:
                raise RuntimeError(
                    f"'{mapping_name}_override' is not valid for "
                    f"arm_type='{requested_arm_type}'."
                )
            if mapping_name in mappings:
                mappings[mapping_name] = override
            elif override != original_mappings[mapping_name]:
                raise RuntimeError(
                    f"'{mapping_name}_override' cannot change a DAE variant. "
                    "Select arm_type or use model:=stl for custom STL mappings."
                )

    if spec.dual_arm:
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
        use_bridge = spec.dual_arm
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
        "robot_description": ParameterValue(
            _xacro_command(model_path, mappings), value_type=str
        )
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
                package="rm_description",
                executable="dual_arm_joint_state_bridge.py",
                name="dual_arm_joint_state_bridge",
                output="screen",
                **bridge_kwargs,
            )
        )

    if use_rviz:
        rviz_kwargs = {}
        if use_sim_time:
            rviz_kwargs["parameters"] = [{"use_sim_time": True}]
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
                    "Robot model including end-link version, e.g. 65, "
                    "65-6f, eco63-6fb, or rx75-6fb-v"
                ),
            ),
            DeclareLaunchArgument(
                "model", default_value="auto", choices=MODEL_FORMATS,
                description="auto prefers available DAE visuals; stl selects the original model",
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
            DeclareLaunchArgument("left_xyz", default_value="0 -0.075 1.5"),
            DeclareLaunchArgument(
                "left_rpy",
                default_value="1.5707963267949 -1.5707963267949 0",
            ),
            DeclareLaunchArgument("right_xyz", default_value="0 0.075 1.5"),
            DeclareLaunchArgument(
                "right_rpy",
                default_value="-1.5707963267949 -1.5707963267949 0",
            ),
            # Internal compatibility hooks used by legacy display wrappers.
            DeclareLaunchArgument("link6_type_override", default_value=""),
            DeclareLaunchArgument("link7_type_override", default_value=""),
            DeclareLaunchArgument("base_type_override", default_value=""),
            OpaqueFunction(function=_launch_setup),
        ]
    )
