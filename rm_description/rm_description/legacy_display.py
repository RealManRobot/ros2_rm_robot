"""Factory for the historical ``*_display.launch.py`` entry points."""

import os
from typing import Sequence, Tuple

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from rm_description.variant_catalog import MODEL_FORMATS


LegacyArgument = Tuple[str, str]


def generate_legacy_display_launch(
    arm_type: str,
    *,
    xacro_arguments: Sequence[LegacyArgument] = (),
    rx_display: bool = False,
) -> LaunchDescription:
    """Include the generic launch file while retaining a legacy entry's defaults."""

    actions = [DeclareLaunchArgument("model", default_value="auto", choices=MODEL_FORMATS)]
    launch_arguments = {
        "arm_type": arm_type,
        "model": LaunchConfiguration("model"),
        # Pin values that the old launch files did not expose.  This prevents a
        # same-named argument in an including launch file from changing legacy
        # behavior through the shared launch context.
        "use_sim_time": "false",
        "joint_states_topic": "/joint_states",
        "left_joint_states_topic": "/left_arm/joint_states",
        "right_joint_states_topic": "/right_arm/joint_states",
        "left_xyz": "0 0.075 1.5",
        "left_rpy": "3.14 -1.57 1.5707963267949",
        "right_xyz": "0 -0.075 1.5",
        "right_rpy": "3.14 -1.57 -1.5707963267949",
    }

    for argument_name, default_value in xacro_arguments:
        actions.append(
            DeclareLaunchArgument(argument_name, default_value=default_value)
        )
        launch_arguments[f"{argument_name}_override"] = LaunchConfiguration(
            argument_name
        )

    if rx_display:
        rx_defaults = (
            ("left_xyz", "0 0.075 1.5"),
            ("left_rpy", "3.14 -1.57 1.5707963267949"),
            ("right_xyz", "0 -0.075 1.5"),
            ("right_rpy", "3.14 -1.57 -1.5707963267949"),
            ("use_joint_state_bridge", "false"),
            ("use_joint_state_publisher_gui", "true"),
            ("use_rviz", "true"),
        )
        for argument_name, default_value in rx_defaults:
            actions.append(
                DeclareLaunchArgument(argument_name, default_value=default_value)
            )
            launch_arguments[argument_name] = LaunchConfiguration(argument_name)
    else:
        launch_arguments.update(
            {
                "use_joint_state_bridge": "false",
                "use_joint_state_publisher_gui": "false",
                "use_rviz": "false",
            }
        )

    generic_launch = os.path.join(
        get_package_share_directory("rm_description"),
        "launch",
        "rm_description.launch.py",
    )
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(generic_launch),
            launch_arguments=launch_arguments.items(),
        )
    )
    return LaunchDescription(actions)
