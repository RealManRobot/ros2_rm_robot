import os
import re
import tempfile
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import xacro

from rm_description.variant_catalog import MODEL_FORMATS, apply_visual_model


_CREATE_SUCCESS_MARKER = "OK creation of entity."
_CREATE_FAILURE_MARKERS = (
    "timed out",
    "failed to create entity",
    "failed request to create entity",
    "error discovering service",
    "error parsing response",
)


def gazebo_entity_creation_succeeded(returncode, output):
    """Reject ros_gz_sim/create's known timeout-with-zero-exit behavior."""
    normalized_output = output.casefold()
    return (
        returncode == 0
        and _CREATE_SUCCESS_MARKER.casefold() in normalized_output
        and not any(marker in normalized_output for marker in _CREATE_FAILURE_MARKERS)
    )


def resolve_auto_joint_states_topic(value, default_topic):
    """Resolve the only supported public selector to an internal topic."""
    requested = value.strip()
    if requested != 'auto':
        raise RuntimeError(
            "Launch argument 'joint_states_topic' currently only supports "
            f"'auto'; received '{requested or '<empty>'}'."
        )
    return default_topic


def _generate_resolved_gz_demo_actions(
    context,
    *,
    joint_states_topic_default,
    **action_arguments,
):
    requested = LaunchConfiguration('joint_states_topic').perform(context)
    return generate_gz_demo_actions(
        **action_arguments,
        joint_states_topic=resolve_auto_joint_states_topic(
            requested,
            joint_states_topic_default,
        ),
    )


def get_ros2_control_backend():
    # The Humble model files in this repository explicitly name the modern
    # gz_ros2_control plugin.  Keep backend discovery consistent with the URDF
    # instead of selecting the legacy Ignition shim independently.
    try:
        get_package_share_directory("gz_ros2_control")
    except PackageNotFoundError as exc:
        raise RuntimeError(
            "Missing ROS package 'gz_ros2_control'. Install "
            "'ros-humble-gz-ros2-control' (or its Humble compatibility shim) "
            "before launching Gazebo demos."
        ) from exc

    return {
        "package": "gz_ros2_control",
        "hardware_plugin": "gz_ros2_control/GazeboSimSystem",
        "plugin_filename": "gz_ros2_control-system",
        "plugin_name": "gz_ros2_control::GazeboSimROS2ControlPlugin",
    }


def generate_gz_demo_launch(
    *,
    urdf_filename,
    robot_name_in_model,
    controller_names,
    xacro_mappings=None,
    joint_states_topic_default="/joint_states",
):
    """
    Build the original two-argument Gazebo demo launch description.

    This API is kept for source compatibility with downstream launch files. New
    package launch files should select a model through ``rm_gazebo.launch.py``.
    """
    return LaunchDescription(
        [
            DeclareLaunchArgument("start_gazebo", default_value="true"),
            DeclareLaunchArgument(
                "joint_states_topic",
                default_value="auto",
                choices=["auto"],
                description=(
                    "Only auto is supported; it resolves to "
                    f"{joint_states_topic_default} for this legacy entry"
                ),
            ),
            OpaqueFunction(
                function=_generate_resolved_gz_demo_actions,
                kwargs={
                    "urdf_filename": urdf_filename,
                    "robot_name_in_model": robot_name_in_model,
                    "controller_names": controller_names,
                    "xacro_mappings": xacro_mappings,
                    "start_gazebo": LaunchConfiguration("start_gazebo"),
                    "joint_states_topic_default": joint_states_topic_default,
                    "use_sim_time": True,
                },
            ),
        ]
    )


def generate_gz_demo_actions(
    *,
    urdf_filename,
    robot_name_in_model,
    controller_names,
    xacro_mappings=None,
    start_gazebo=True,
    joint_states_topic="/joint_states",
    use_sim_time=True,
    arm_type=None,
    model="auto",
):
    """Build the shared Gazebo actions for one already-resolved arm variant."""
    package_name = "rm_gazebo"
    world_name = "empty"

    ros2_control_backend = get_ros2_control_backend()

    pkg_share = get_package_share_directory(package_name)
    pkg_prefix = get_package_prefix(package_name)
    description_share = get_package_share_directory("rm_description")
    urdf_model_path = os.path.join(pkg_share, "config", urdf_filename)
    gz_resource_parent = os.path.dirname(description_share)

    robot_description = xacro.process_file(
        urdf_model_path,
        mappings={
            **(xacro_mappings or {}),
            "ros2_control_hardware_plugin": ros2_control_backend["hardware_plugin"],
            "ros2_control_plugin_filename": ros2_control_backend["plugin_filename"],
            "ros2_control_plugin_name": ros2_control_backend["plugin_name"],
        },
    ).toxml()
    if arm_type is not None:
        robot_description = apply_visual_model(robot_description, arm_type, model)
    params = {"robot_description": robot_description}

    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            gz_resource_parent,
            ":",
            EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
        ],
    )

    ign_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=[
            gz_resource_parent,
            ":",
            EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value=""),
        ],
    )

    gz_plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=[
            EnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", default_value=""),
            ":",
            EnvironmentVariable("LD_LIBRARY_PATH", default_value=""),
        ],
    )

    ign_plugin_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
        value=[
            EnvironmentVariable("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", default_value=""),
            ":",
            EnvironmentVariable("LD_LIBRARY_PATH", default_value=""),
        ],
    )

    gazebo_wrapper = os.path.join(
        pkg_prefix,
        "lib",
        package_name,
        "gz_sim_clean_exit.py",
    )
    gazebo_arguments = [
        gazebo_wrapper,
        "/usr/bin/ign",
        "gazebo",
        "-v",
        "4",
        "-r",
    ]
    system_gui_config = "/usr/share/ignition/ignition-gazebo6/gui/gui.config"
    camera_cleanup = []
    if robot_name_in_model == "rm_rx75_dual":
        # Keep the installed GUI layout; override only the RX75 camera in a temporary copy.
        with open(system_gui_config, encoding="utf-8") as source:
            gui_config, count = re.subn(
                r"<camera_pose>[^<]*</camera_pose>",
                "<camera_pose>2.5 0 2 0 0.45 3.141592653589793</camera_pose>",
                source.read(), count=1,
            )
        if count != 1:
            raise RuntimeError("Gazebo GUI configuration is missing camera_pose")
        camera_directory = tempfile.TemporaryDirectory(prefix="rm_rx75_")
        gui_config_path = os.path.join(camera_directory.name, "gui.config")
        with open(gui_config_path, "w", encoding="utf-8") as target:
            target.write(gui_config)
        camera_cleanup.append(RegisterEventHandler(OnShutdown(
            on_shutdown=lambda event, context: camera_directory.cleanup(),
        )))
        system_gui_config = gui_config_path
    if os.path.isfile(system_gui_config):
        gazebo_arguments.extend(["--gui-config", system_gui_config])
    gazebo_arguments.extend([f"{world_name}.sdf", "--force-version", "6"])

    gazebo = ExecuteProcess(
        cmd=gazebo_arguments,
        output="screen",
        additional_env={"QT_LOGGING_RULES": "*.warning=false"},
        condition=IfCondition(start_gazebo),
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "use_sim_time": ParameterValue(
                    use_sim_time,
                    value_type=bool,
                )
            },
            params,
            {"publish_frequency": 15.0},
        ],
        remappings=[
            ("/joint_states", joint_states_topic),
            ("joint_states", joint_states_topic),
        ],
        output="screen",
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[f"/world/{world_name}/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        remappings=[(f"/world/{world_name}/clock", "/clock")],
        output="screen",
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world",
            world_name,
            "-topic",
            "robot_description",
            "-name",
            robot_name_in_model,
        ],
        output="screen",
        cached_output=True,
    )

    spawn_controllers = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            *controller_names,
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
            "--switch-timeout",
            "120",
            "--service-call-timeout",
            "30",
            "--activate-as-group",
        ],
        output="screen",
    )

    def _handle_spawn_exit(event, context):
        if context.is_shutdown:
            return None
        spawn_output = "\n".join(
            (spawn_entity.get_stdout(), spawn_entity.get_stderr())
        )
        if not gazebo_entity_creation_succeeded(event.returncode, spawn_output):
            raise RuntimeError(
                "Gazebo entity creation failed with status "
                f"{event.returncode}; controllers were not started. "
                "Check that /world/empty/create exists and that Gazebo is "
                "running."
            )
        return [TimerAction(period=2.0, actions=[spawn_controllers])]

    close_evt1 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=_handle_spawn_exit,
        )
    )

    return [
        *camera_cleanup,
        gz_resource_path,
        ign_resource_path,
        gz_plugin_path,
        ign_plugin_path,
        close_evt1,
        gazebo,
        node_robot_state_publisher,
        clock_bridge,
        spawn_entity,
    ]


def generate_legacy_gz_demo_launch(
    *,
    arm_type,
    joint_states_topic_default="/joint_states",
):
    """Include the unified entry while preserving a legacy launch interface."""
    generic_launch_path = os.path.join(
        get_package_share_directory("rm_gazebo"),
        "launch",
        "rm_gazebo.launch.py",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("model", default_value="auto", choices=MODEL_FORMATS),
            DeclareLaunchArgument("start_gazebo", default_value="true"),
            DeclareLaunchArgument(
                "joint_states_topic",
                default_value="auto",
                choices=["auto"],
                description=(
                    "Only auto is supported; it resolves to "
                    f"{joint_states_topic_default} for this legacy entry"
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(generic_launch_path),
                launch_arguments={
                    "arm_type": arm_type,
                    "model": LaunchConfiguration("model"),
                    "start_gazebo": LaunchConfiguration("start_gazebo"),
                    "joint_states_topic": LaunchConfiguration(
                        "joint_states_topic"
                    ),
                    "use_sim_time": "true",
                }.items(),
            ),
        ]
    )
