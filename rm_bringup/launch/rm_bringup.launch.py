"""Unified, component-level bringup for every supported RealMan arm."""

import asyncio
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueCoroutine,
    OpaqueFunction,
    TimerAction,
)
from launch.events import IncludeLaunchDescription as IncludeLaunchDescriptionEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from rm_bringup.variant_catalog import LaunchReference, resolve_model
from rm_description.variant_catalog import (
    GLB_MODELS, MODEL_FORMATS,
    resolve_gazebo_model,
    resolve_model as resolve_description,
)


_TRUE_VALUES = {'1', 'true', 'yes', 'on'}
_FALSE_VALUES = {'0', 'false', 'no', 'off'}
_CONTROLLER_INFO_TIMEOUT = 10.0


def _value(context, name):
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


def _include(reference: LaunchReference, arguments=None):
    launch_path = os.path.join(
        get_package_share_directory(reference.package),
        'launch',
        reference.launch_file,
    )
    if not os.path.isfile(launch_path):
        raise RuntimeError(
            'Bringup component does not exist: '
            f'package={reference.package}, '
            f'launch_file={reference.launch_file}'
        )
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments=(arguments or {}).items(),
    )


def _resolved_joint_states_topic(context, plan):
    requested = _value(context, 'joint_states_topic').strip()
    if requested != 'auto':
        raise RuntimeError(
            "Launch argument 'joint_states_topic' currently only supports "
            f"'auto'; received '{requested or '<empty>'}'."
        )
    return plan.arm_profile.joint_states_topic(plan.mode)


def _moveit_action(context, plan, joint_states_topic, model):
    moveit_arguments = {
        'allow_trajectory_execution': str(_boolean_value(
            'allow_trajectory_execution',
            _value(context, 'allow_trajectory_execution'),
        )).lower(),
        'use_rviz': str(_boolean_value(
            'use_rviz', _value(context, 'use_rviz')
        )).lower(),
    }
    if model == 'glb' and plan.mode == 'real':
        moveit_arguments.update({
            'arm_type': plan.arm_type,
            'model': model,
            'joint_states_topic': joint_states_topic,
            'use_robot_state_publisher': 'false',
        })
        return _include(
            LaunchReference('rm_bringup', 'rm_moveit.launch.py'),
            moveit_arguments,
        )
    if plan.arm_profile.topology == 'dual' and plan.mode == 'gazebo':
        moveit_arguments['joint_states_topic'] = joint_states_topic
    if plan.mode == 'gazebo' and plan.arm_type in GLB_MODELS:
        moveit_arguments['model'] = model

    return _include(plan.moveit, moveit_arguments)


def _driver_action(context, plan):
    return _include(plan.driver, {
        'arm_type': plan.arm_type,
        'driver_config': _value(context, 'driver_config'),
        'left_driver_config': _value(context, 'left_driver_config'),
        'right_driver_config': _value(context, 'right_driver_config'),
    })


async def _read_controller_version(context, plan):
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
            if plan.arm_profile.topology == 'dual' else ('',)
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


async def _finish_real_bringup(context, plan):
    version = await _read_controller_version(context, plan)
    actions = _compose_bringup(context, controller_version=version, driver_started=True)
    context.emit_event_sync(IncludeLaunchDescriptionEvent(LaunchDescription(actions)))


def _build_real_actions(context, plan, joint_states_topic, use_moveit, model):
    actions = [
        _include(
            plan.description,
            {
                'arm_type': plan.arm_type,
                'model': model,
                'use_sim_time': 'false',
                'joint_states_topic': joint_states_topic,
                'use_joint_state_bridge': (
                    'true'
                    if plan.arm_profile.use_joint_state_bridge
                    else 'false'
                ),
                'use_joint_state_publisher_gui': 'false',
                'use_rviz': 'false',
            },
        ),
        _include(
            plan.control,
            {
                'arm_type': plan.arm_type,
                'follow': _value(context, 'follow'),
            },
        ),
    ]
    if use_moveit:
        actions.append(_moveit_action(context, plan, joint_states_topic, model))
    return actions


def _build_gazebo_actions(context, plan, joint_states_topic, use_moveit, model):
    for name in (
        'driver_config',
        'left_driver_config',
        'right_driver_config',
        'follow',
    ):
        if _value(context, name).strip().casefold() != 'auto':
            raise RuntimeError(
                f"Launch argument '{name}' only applies in mode='real'."
            )

    actions = [
        _include(
            plan.gazebo,
            {
                'arm_type': plan.arm_type,
                'start_gazebo': _value(context, 'start_gazebo'),
                'model': model,
                'joint_states_topic': 'auto',
            },
        )
    ]
    if use_moveit:
        actions.append(
            TimerAction(
                period=8.0,
                actions=[
                    _moveit_action(context, plan, joint_states_topic, model)
                ],
            )
        )
    return actions


def _compose_bringup(context, controller_version=None, driver_started=False):
    needs_controller_info = False
    try:
        arm_type, mode = _value(context, 'arm_type'), _value(context, 'mode')
        if mode.strip().casefold() == 'real' and controller_version is None:
            integrated_plan = resolve_model(arm_type, mode, 4)
            try:
                plan = resolve_model(arm_type, mode)
            except ValueError:
                plan = None
            needs_controller_info = (
                plan is None or plan.arm_type != integrated_plan.arm_type
            )
            if needs_controller_info:
                plan = integrated_plan
        else:
            plan = resolve_model(
                arm_type, mode,
                controller_version if controller_version is not None else 3,
            )
    except ValueError as exc:
        raise RuntimeError(str(exc)) from exc

    requested_model = _value(context, 'model').strip().casefold()
    try:
        resolve_description(
            plan.arm_type, requested_model
        )
    except ValueError as exc:
        raise RuntimeError(str(exc)) from exc
    model = (
        'glb' if plan.mode == 'real' and requested_model != 'stl'
        and plan.arm_type in GLB_MODELS else 'stl'
    )
    if plan.mode == 'gazebo':
        try:
            model = resolve_gazebo_model(
                plan.arm_type, requested_model,
            )
        except ValueError as exc:
            raise RuntimeError(str(exc)) from exc

    use_moveit = _boolean_value(
        'use_moveit', _value(context, 'use_moveit')
    )
    use_rviz = _boolean_value('use_rviz', _value(context, 'use_rviz'))
    _boolean_value(
        'allow_trajectory_execution',
        _value(context, 'allow_trajectory_execution'),
    )
    _boolean_value(
        'start_gazebo',
        _value(context, 'start_gazebo'),
    )
    if use_rviz and not use_moveit:
        raise RuntimeError(
            "Launch argument 'use_rviz=true' requires 'use_moveit=true'."
        )

    joint_states_topic = _resolved_joint_states_topic(context, plan)
    if plan.mode == 'real':
        if needs_controller_info:
            return [
                # Keep the driver's canonical selector from replacing the user's input.
                GroupAction(actions=[_driver_action(context, plan)]),
                OpaqueCoroutine(coroutine=_finish_real_bringup, kwargs={'plan': plan}),
            ]
        actions = _build_real_actions(
            context, plan, joint_states_topic, use_moveit, model
        )
        return actions if driver_started else [_driver_action(context, plan), *actions]
    return _build_gazebo_actions(
        context, plan, joint_states_topic, use_moveit, model
    )


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'arm_type',
                description=(
                    'Robot model including end-link version, e.g. 65, '
                    '65-6f, eco63-6fb, or rx75-6fb-v'
                ),
            ),
            DeclareLaunchArgument(
                'mode',
                default_value='real',
                description='Bringup mode: real or gazebo',
            ),
            DeclareLaunchArgument(
                'model', default_value='auto', choices=list(MODEL_FORMATS),
                description='auto selects GLB when available; stl selects the original model',
            ),
            DeclareLaunchArgument(
                'allow_trajectory_execution',
                default_value='true',
                description='Allow MoveIt to execute trajectories',
            ),
            DeclareLaunchArgument(
                'use_moveit',
                default_value='true',
                description='Start move_group and its supporting nodes',
            ),
            DeclareLaunchArgument(
                'use_rviz',
                default_value='true',
                description='Start the MoveIt RViz process',
            ),
            DeclareLaunchArgument(
                'driver_config',
                default_value='auto',
                description='Single-arm driver YAML or auto',
            ),
            DeclareLaunchArgument(
                'left_driver_config',
                default_value='auto',
                description='RX75 left-arm driver YAML or auto',
            ),
            DeclareLaunchArgument(
                'right_driver_config',
                default_value='auto',
                description='RX75 right-arm driver YAML or auto',
            ),
            DeclareLaunchArgument(
                'follow',
                default_value='auto',
                description=(
                    'Driver-follow mode: auto selects false for all models; '
                    'true or false overrides it'
                ),
            ),
            DeclareLaunchArgument(
                'joint_states_topic',
                default_value='auto',
                choices=['auto'],
                description=(
                    'Only auto is currently supported; it selects the '
                    'model/mode default joint-state topic'
                ),
            ),
            DeclareLaunchArgument(
                'start_gazebo',
                default_value='true',
                description='Start the Gazebo server and client',
            ),
            OpaqueFunction(function=_compose_bringup),
        ]
    )
