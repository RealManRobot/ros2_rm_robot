"""Unified component-level bringup for all supported RealMan arms."""

import asyncio
import os
import runpy
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueCoroutine,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.events import IncludeLaunchDescription as IncludeLaunchDescriptionEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


_TRUE_VALUES = {"1", "true", "yes", "on"}
_FALSE_VALUES = {"0", "false", "no", "off"}
_CONTROLLER_INFO_TIMEOUT = 10.0

_ARM_TYPE_ALIASES = {
    "63": "63",
    "rm63": "63",
    "63iii": "63_iii",
    "rm63iii": "63_iii",
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

# package, real MoveIt launch, Gazebo MoveIt launch
_MODELS = {
    "63": (
        "rm_63_config", "real_moveit_demo.launch.py",
        "gazebo_moveit_demo.launch.py"
    ),
    "63-6f": (
        "rm_63_config", "real_moveit_demo_6f.launch.py",
        "gazebo_moveit_demo_6f.launch.py"
    ),
    "63-6fb": (
        "rm_63_config", "real_moveit_demo_6fb.launch.py",
        "gazebo_moveit_demo_6fb.launch.py"
    ),
    "63_iii": (
        "rm_63_config", "real_moveit_demo_III.launch.py",
        "gazebo_moveit_demo_III.launch.py"
    ),
    "63_iii-6fb": (
        "rm_63_config", "real_moveit_demo_III_6fb.launch.py",
        "gazebo_moveit_demo_III_6fb.launch.py"
    ),
    "65": (
        "rm_65_config", "real_moveit_demo.launch.py",
        "gazebo_moveit_demo.launch.py"
    ),
    "65-6f": (
        "rm_65_config", "real_moveit_demo_6f.launch.py",
        "gazebo_moveit_demo_6f.launch.py"
    ),
    "65-6fb": (
        "rm_65_config", "real_moveit_demo_6fb.launch.py",
        "gazebo_moveit_demo_6fb.launch.py"
    ),
    "75": (
        "rm_75_config", "real_moveit_demo.launch.py",
        "gazebo_moveit_demo.launch.py"
    ),
    "75-6f": (
        "rm_75_config", "real_moveit_demo_6f.launch.py",
        "gazebo_moveit_demo_6f.launch.py"
    ),
    "75-6fb": (
        "rm_75_config", "real_moveit_demo_6fb.launch.py",
        "gazebo_moveit_demo_6fb.launch.py"
    ),
    "eco62": (
        "rm_eco62_config", "real_moveit_demo.launch.py",
        "gazebo_moveit_demo.launch.py"
    ),
    "eco63": (
        "rm_eco63_config", "real_moveit_demo.launch.py",
        "gazebo_moveit_demo.launch.py"
    ),
    "eco63-6fb": (
        "rm_eco63_config", "real_moveit_demo_6fb.launch.py",
        "gazebo_moveit_demo_6fb.launch.py"
    ),
    "eco65": (
        "rm_eco65_config", "real_moveit_demo.launch.py",
        "gazebo_moveit_demo.launch.py"
    ),
    "eco65-6f": (
        "rm_eco65_config", "real_moveit_demo_6f.launch.py",
        "gazebo_moveit_demo_6f.launch.py"
    ),
    "eco65-6fb": (
        "rm_eco65_config", "real_moveit_demo_6fb.launch.py",
        "gazebo_moveit_demo_6fb.launch.py"
    ),
    "gen72": (
        "rm_gen72_config", "real_moveit_demo.launch.py",
        "gazebo_moveit_demo.launch.py"
    ),
    "gen72_ii": (
        "rm_gen72_config", "real_moveit_demo_II.launch.py",
        "gazebo_moveit_demo_II.launch.py"
    ),
    "rx75-6fb": (
        "rm_rx75_config", "real_moveit_demo_6fb.launch.py",
        "gazebo_moveit_demo_6fb.launch.py"
    ),
    "rx75-6fb-v": (
        "rm_rx75_config", "real_moveit_demo_6fb_v.launch.py",
        "gazebo_moveit_demo_6fb_v.launch.py"
    ),
}

if len(_MODELS) != 21:
    raise RuntimeError("Bringup model catalog must contain 21 arm types.")

_ARM_TYPE_ALIASES.update({
    alias + arm_type[len(family):].replace("-", ""): arm_type
    for alias, family in _ARM_TYPE_ALIASES.items()
    for arm_type in _MODELS
    if arm_type == family or arm_type.startswith(family + "-")
})


def _value(context, name):
    return LaunchConfiguration(name).perform(context)


def _compact_token(value):
    normalized = re.sub(r"[\s-]+", "_", value.lower())
    normalized = re.sub(r"_+", "_", normalized).strip("_")
    return normalized.replace("_", "")


def _normalize_arm_type(value, controller_version=3):
    if controller_version not in (3, 4):
        raise RuntimeError("controller_version must be 3 or 4.")
    raw_value = value.strip() if isinstance(value, str) else ""
    token = _compact_token(raw_value)
    if controller_version == 4:
        token = re.sub(r"6f(v?)$", r"6fb\1", token)
    arm_type = _ARM_TYPE_ALIASES.get(token)
    if arm_type not in _MODELS:
        valid = ", ".join(_MODELS)
        raise RuntimeError(
            f"Unsupported arm_type: {raw_value or value}. Valid arm types: {valid}."
        )
    return arm_type


def _resolve_selection(arm_type, mode, controller_version=3):
    canonical_type = _normalize_arm_type(arm_type, controller_version)
    canonical_mode = mode.strip().casefold()
    if canonical_mode not in ("real", "gazebo"):
        raise RuntimeError(
            f"Unsupported mode: {mode}. Valid modes: real, gazebo."
        )

    return canonical_type, canonical_mode, _MODELS[canonical_type]


def _boolean_value(name, value):
    normalized = value.strip().casefold()
    if normalized in _TRUE_VALUES:
        return True
    if normalized in _FALSE_VALUES:
        return False
    raise RuntimeError(
        f"Launch argument '{name}' must be a boolean value; received '{value}'."
    )


def _include(package, launch_file, arguments=None):
    launch_path = os.path.join(
        get_package_share_directory(package), "launch", launch_file
    )
    if not os.path.isfile(launch_path):
        raise RuntimeError(
            "Bringup component does not exist: "
            f"package={package}, launch_file={launch_file}"
        )
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments=(arguments or {}).items(),
    )


def _moveit_action(
    context,
    arm_type,
    mode,
    moveit_spec,
    joint_states_topic,
):
    package, real_launch, gazebo_launch = moveit_spec
    arguments = {
        "allow_trajectory_execution": str(_boolean_value(
            "allow_trajectory_execution", _value(context, "allow_trajectory_execution")
        )).lower(),
        "use_rviz": str(_boolean_value("use_rviz", _value(context, "use_rviz"))).lower(),
    }
    if mode == "real" and _value(context, "model").strip().casefold() != "stl" and package in (
        "rm_65_config", "rm_75_config", "rm_eco62_config", "rm_eco63_config", "rm_eco65_config", "rm_rx75_config"
    ):
        helper = runpy.run_path(os.path.join(
            get_package_share_directory("rm_bringup"), "launch", "real_moveit.py",
        ))
        return GroupAction(actions=helper["build_real_moveit_actions"](
            context, arm_type, _value(context, "model"), joint_states_topic,
            allow_execution=_boolean_value("allow_trajectory_execution", arguments["allow_trajectory_execution"]),
            use_rviz=_boolean_value("use_rviz", arguments["use_rviz"]),
        ))
    if arm_type.startswith("rx75-"):
        arguments["joint_states_topic"] = joint_states_topic
    if mode == "gazebo":
        arguments["model"] = _value(context, "model")
    return _include(
        package,
        real_launch if mode == "real" else gazebo_launch,
        arguments,
    )


def _driver_action(context, arm_type):
    return _include("rm_driver", "rm_driver.launch.py", {
        "arm_type": arm_type,
        "driver_config": _value(context, "driver_config"),
        "left_driver_config": _value(context, "left_driver_config"),
        "right_driver_config": _value(context, "right_driver_config"),
    })


async def _read_controller_version(context, arm_type):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rm_ros_interfaces.msg import RobotInfo
    from std_msgs.msg import Empty

    ros_context = Context()
    rclpy.init(context=ros_context)
    node = None
    executor = None
    try:
        node = rclpy.create_node(
            'rm_bringup_controller_info', context=ros_context,
            use_global_arguments=False,
        )
        executor = SingleThreadedExecutor(context=ros_context)
        executor.add_node(node)
        parent_namespace = context.launch_configurations.get(
            'ros_namespace', ''
        ).strip('/')
        parent_namespace = '/' + parent_namespace if parent_namespace else ''
        namespaces = (
            ('/left_arm', '/right_arm')
            if arm_type.startswith('rx75-') else ('',)
        )
        replies = {}
        publishers = []
        for namespace in namespaces:
            topic = parent_namespace + namespace + '/rm_driver/get_robot_info'
            node.create_subscription(
                RobotInfo, topic + '_result',
                lambda message, key=namespace: replies.update({key: message}), 1,
            )
            publishers.append(node.create_publisher(Empty, topic + '_cmd', 1))

        deadline = asyncio.get_running_loop().time() + _CONTROLLER_INFO_TIMEOUT
        while asyncio.get_running_loop().time() < deadline:
            for publisher in publishers:
                if publisher.get_subscription_count():
                    publisher.publish(Empty())
            executor.spin_once(timeout_sec=0)
            if len(replies) == len(namespaces):
                versions = {
                    message.robot_controller_version for message in replies.values()
                }
                if not all(message.state for message in replies.values()) or len(versions) != 1:
                    raise RuntimeError(
                        'Cannot determine a consistent controller generation from rm_driver.'
                    )
                version, = versions
                if version not in (3, 4):
                    raise RuntimeError(
                        'Unsupported controller generation reported by rm_driver: '
                        f'{version}.'
                    )
                return version
            await asyncio.sleep(0.1)
        raise RuntimeError(
            'Timed out reading controller information from rm_driver; '
            'model selection was not guessed.'
        )
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown(context=ros_context)


async def _finish_real_bringup(context, arm_type):
    version = await _read_controller_version(context, arm_type)
    actions = _compose_bringup(context, controller_version=version, driver_started=True)
    context.emit_event_sync(IncludeLaunchDescriptionEvent(LaunchDescription(actions)))


def _build_real_actions(
    context,
    arm_type,
    moveit_spec,
    use_moveit,
):
    actions = [
        _include(
            "rm_description",
            "rm_description.launch.py",
            {
                "arm_type": arm_type,
                "model": _value(context, "model"),
                "use_sim_time": "false",
                "joint_states_topic": "/joint_states",
                "use_joint_state_bridge": (
                    "true" if arm_type.startswith("rx75-") else "false"
                ),
                "use_joint_state_publisher_gui": "false",
                "use_rviz": "false",
            },
        ),
        _include(
            "rm_control",
            "rm_control.launch.py",
            {
                "arm_type": arm_type,
                "follow": _value(context, "follow"),
            },
        ),
    ]
    if use_moveit:
        actions.append(
            _moveit_action(
                context, arm_type, "real", moveit_spec, "/joint_states"
            )
        )
    return actions


def _build_gazebo_actions(
    context,
    arm_type,
    moveit_spec,
    use_moveit,
):
    for name in (
        "driver_config",
        "left_driver_config",
        "right_driver_config",
        "follow",
    ):
        if _value(context, name).strip().casefold() != "auto":
            raise RuntimeError(
                f"Launch argument '{name}' only applies in mode='real'."
            )

    joint_states_topic = (
        "/joint_state_broadcaster/joint_states"
        if arm_type.startswith("rx75-")
        else "/joint_states"
    )
    actions = [
        _include(
            "rm_gazebo",
            "rm_gazebo.launch.py",
            {
                "arm_type": arm_type,
                "start_gazebo": _value(context, "start_gazebo"),
                "model": _value(context, "model"),
                "use_gazebo_gui": _value(context, "use_gazebo_gui"),
                "joint_states_topic": "auto",
                "use_sim_time": "true",
            },
        )
    ]
    if use_moveit:
        actions.append(
            TimerAction(
                period=8.0,
                actions=[
                    _moveit_action(
                        context,
                        arm_type,
                        "gazebo",
                        moveit_spec,
                        joint_states_topic,
                    )
                ],
            )
        )
    return actions


def _compose_bringup(context, controller_version=None, driver_started=False):
    requested = _value(context, "arm_type")
    requested_mode = _value(context, "mode")
    needs_controller_info = False
    if requested_mode.strip().casefold() == "real" and controller_version is None:
        integrated = _resolve_selection(requested, requested_mode, 4)
        try:
            original = _resolve_selection(requested, requested_mode)
        except RuntimeError:
            original = None
        needs_controller_info = original is None or original[0] != integrated[0]
        arm_type, mode, moveit_spec = integrated if needs_controller_info else original
    else:
        arm_type, mode, moveit_spec = _resolve_selection(
            requested, requested_mode,
            controller_version if controller_version is not None else 3,
        )
    model = _value(context, "model").strip().casefold()
    description = runpy.run_path(os.path.join(
        get_package_share_directory("rm_description"), "launch", "rm_description.launch.py",
    ))
    description["_resolve_model"](arm_type, model)

    use_moveit = _boolean_value("use_moveit", _value(context, "use_moveit"))
    use_rviz = _boolean_value("use_rviz", _value(context, "use_rviz"))
    _boolean_value(
        "allow_trajectory_execution",
        _value(context, "allow_trajectory_execution"),
    )
    _boolean_value("start_gazebo", _value(context, "start_gazebo"))
    _boolean_value("use_gazebo_gui", _value(context, "use_gazebo_gui"))

    requested_joint_states_topic = _value(
        context, "joint_states_topic"
    ).strip()
    if requested_joint_states_topic != "auto":
        raise RuntimeError(
            "Launch argument 'joint_states_topic' currently only supports "
            f"'auto'; received '{requested_joint_states_topic or '<empty>'}'."
        )
    if use_rviz and not use_moveit:
        raise RuntimeError(
            "Launch argument 'use_rviz=true' requires 'use_moveit=true'."
        )

    if mode == "real":
        if needs_controller_info:
            return [
                GroupAction(actions=[_driver_action(context, arm_type)]),
                OpaqueCoroutine(coroutine=_finish_real_bringup, kwargs={"arm_type": arm_type}),
            ]
        actions = _build_real_actions(context, arm_type, moveit_spec, use_moveit)
        return actions if driver_started else [_driver_action(context, arm_type), *actions]
    return _build_gazebo_actions(
        context,
        arm_type,
        moveit_spec,
        use_moveit,
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
            ),
            DeclareLaunchArgument("mode", default_value="real"),
            DeclareLaunchArgument(
                "allow_trajectory_execution",
                default_value="true",
                description=(
                    "Allow MoveIt trajectory execution; use false for the "
                    "first real-robot validation"
                ),
            ),
            DeclareLaunchArgument("use_moveit", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("driver_config", default_value="auto"),
            DeclareLaunchArgument("left_driver_config", default_value="auto"),
            DeclareLaunchArgument("right_driver_config", default_value="auto"),
            DeclareLaunchArgument("follow", default_value="auto"),
            DeclareLaunchArgument(
                "joint_states_topic",
                default_value="auto",
                choices=["auto"],
            ),
            DeclareLaunchArgument("start_gazebo", default_value="true"),
            DeclareLaunchArgument("use_gazebo_gui", default_value="true"),
            OpaqueFunction(function=_compose_bringup),
        ]
    )
