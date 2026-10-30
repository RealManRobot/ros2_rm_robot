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

import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


_ARM_TYPE_ALIASES = {
    '63': '63',
    'rm63': '63',
    '63iii': '63_iii',
    'rm63iii': '63_iii',
    '65': '65',
    'rm65': '65',
    '75': '75',
    'rm75': '75',
    'eco62': 'eco62',
    'rmeco62': 'eco62',
    'eco63': 'eco63',
    'rmeco63': 'eco63',
    'eco65': 'eco65',
    'rmeco65': 'eco65',
    'gen72': 'gen72',
    'rmgen72': 'gen72',
    'gen72ii': 'gen72_ii',
    'rmgen72ii': 'gen72_ii',
    'rx75': 'rx75',
    'rmrx75': 'rx75',
}

_DRIVER_PROFILES = {
    '63': {
        'suffixes': ('6f', '6fb'),
        'topology': 'single',
        'config': 'rm_63_config.yaml',
    },
    '63_iii': {
        'suffixes': ('6f', '6fb'),
        'topology': 'single',
        'config': 'rm_63_config.yaml',
    },
    '65': {
        'suffixes': ('6f', '6fb'),
        'topology': 'single',
        'config': 'rm_65_config.yaml',
    },
    '75': {
        'suffixes': ('6f', '6fb'),
        'topology': 'single',
        'config': 'rm_75_config.yaml',
    },
    'eco62': {
        'topology': 'single',
        'config': 'rm_eco62_config.yaml',
    },
    'eco63': {
        'suffixes': ('6f', '6fb'),
        'topology': 'single',
        'config': 'rm_eco63_config.yaml',
    },
    'eco65': {
        'suffixes': ('6f', '6fb'),
        'topology': 'single',
        'config': 'rm_eco65_config.yaml',
    },
    'gen72': {
        'topology': 'single',
        'config': 'rm_gen72_config.yaml',
    },
    'gen72_ii': {
        'topology': 'single',
        'config': 'rm_gen72_config.yaml',
    },
    'rx75': {
        'suffixes': ('6f', '6f-v', '6fb', '6fb-v'),
        'topology': 'dual',
        'left_config': 'rm_rx75_left_config.yaml',
        'right_config': 'rm_rx75_right_config.yaml',
    },
}


_ARM_TYPE_ALIASES.update({
    alias + suffix.replace('-', ''): family
    for alias, family in _ARM_TYPE_ALIASES.items()
    for suffix in _DRIVER_PROFILES[family].get('suffixes', ())
})


def _compact_token(value):
    normalized = re.sub(r'[\s-]+', '_', value.lower())
    normalized = re.sub(r'_+', '_', normalized).strip('_')
    return normalized.replace('_', '')


def normalize_arm_type(value):
    if not isinstance(value, str) or not value.strip():
        raise ValueError('arm_type must be a non-empty string.')

    normalized = _ARM_TYPE_ALIASES.get(_compact_token(value.strip()))
    if normalized is None:
        valid_types = ', '.join(
            family + ('-' + suffix if suffix else '')
            for family, profile in _DRIVER_PROFILES.items()
            for suffix in ('',) + profile.get('suffixes', ())
        )
        raise ValueError(
            f'Unsupported arm_type: {value.strip()}. '
            f'Valid arm types: {valid_types}.'
        )
    return normalized


def _is_auto(value, argument_name):
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f'{argument_name} must be "auto" or a file path.')
    return value.strip().lower() == 'auto'


def _resolve_config_path(requested, default_path, argument_name):
    if _is_auto(requested, argument_name):
        resolved = default_path
    else:
        resolved = os.path.abspath(os.path.expanduser(requested.strip()))

    if not os.path.isfile(resolved):
        raise FileNotFoundError(f'{argument_name} does not exist: {resolved}')
    return resolved


def _load_driver_params(config_path):
    try:
        with open(config_path, 'r', encoding='utf-8') as config_file:
            config = yaml.safe_load(config_file)
    except yaml.YAMLError as exception:
        raise ValueError(
            f'Invalid driver YAML in {config_path}: {exception}'
        ) from exception

    if not isinstance(config, dict):
        raise ValueError(
            f'Driver config must contain a mapping: {config_path}'
        )
    node_config = config.get('rm_driver')
    if not isinstance(node_config, dict):
        raise ValueError(
            f'Driver config is missing rm_driver mapping: {config_path}'
        )
    parameters = node_config.get('ros__parameters')
    if not isinstance(parameters, dict):
        raise ValueError(
            'Driver config is missing rm_driver.ros__parameters mapping: '
            f'{config_path}'
        )
    return parameters


def build_driver_specs(
    arm_type,
    driver_config='auto',
    left_driver_config='auto',
    right_driver_config='auto',
    package_share=None,
):
    canonical_type = normalize_arm_type(arm_type)
    profile = _DRIVER_PROFILES[canonical_type]
    if package_share is None:
        package_share = get_package_share_directory('rm_driver')
    config_dir = os.path.join(package_share, 'config')

    if profile['topology'] == 'single':
        if not _is_auto(left_driver_config, 'left_driver_config'):
            raise ValueError(
                'left_driver_config is only valid when arm_type=rx75.'
            )
        if not _is_auto(right_driver_config, 'right_driver_config'):
            raise ValueError(
                'right_driver_config is only valid when arm_type=rx75.'
            )

        default_config = os.path.join(config_dir, profile['config'])
        config_path = _resolve_config_path(
            driver_config,
            default_config,
            'driver_config',
        )
        return [
            {
                'namespace': None,
                'parameters': [config_path],
            }
        ]

    if not _is_auto(driver_config, 'driver_config'):
        raise ValueError(
            'driver_config is only valid for single-arm models. '
            'Use left_driver_config and right_driver_config for RX75.'
        )

    left_path = _resolve_config_path(
        left_driver_config,
        os.path.join(config_dir, profile['left_config']),
        'left_driver_config',
    )
    right_path = _resolve_config_path(
        right_driver_config,
        os.path.join(config_dir, profile['right_config']),
        'right_driver_config',
    )
    return [
        {
            'namespace': 'left_arm',
            'parameters': [_load_driver_params(left_path)],
        },
        {
            'namespace': 'right_arm',
            'parameters': [_load_driver_params(right_path)],
        },
    ]


def _shutdown_on_driver_failure(label):
    def _handle_exit(event, context):
        if event.returncode == 0 or context.is_shutdown:
            return None
        reason = (
            f'{label} rm_driver exited with status {event.returncode}; '
            'shutting down the complete launch.'
        )
        # Raising from the event callback makes LaunchService stop every
        # sibling action and, unlike a normal Shutdown event, preserves a
        # non-zero ros2 launch exit status for callers and supervisors.
        raise RuntimeError(reason)

    return _handle_exit


def _launch_setup(context):
    specs = build_driver_specs(
        arm_type=LaunchConfiguration('arm_type').perform(context),
        driver_config=LaunchConfiguration('driver_config').perform(context),
        left_driver_config=LaunchConfiguration(
            'left_driver_config'
        ).perform(context),
        right_driver_config=LaunchConfiguration(
            'right_driver_config'
        ).perform(context),
    )

    actions = []
    for spec in specs:
        node_arguments = {
            'package': 'rm_driver',
            'executable': 'rm_driver',
            'parameters': spec['parameters'],
            'output': 'screen',
        }
        if spec['namespace'] is not None:
            node_arguments['namespace'] = spec['namespace']
        driver_node = Node(**node_arguments)
        label = spec['namespace'] or 'single-arm'
        actions.extend(
            [
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=driver_node,
                        on_exit=_shutdown_on_driver_failure(label),
                    )
                ),
                # Register before starting the process so an immediate
                # configuration/ownership failure cannot race the handler.
                driver_node,
            ]
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'arm_type',
                description=(
                    'Robot model including supported end-link suffixes, '
                    'e.g. 65, 65-6fb and eco63-6fb'
                ),
            ),
            DeclareLaunchArgument(
                'driver_config',
                default_value='auto',
                description=(
                    'Single-arm driver YAML path, or auto for the model '
                    'default'
                ),
            ),
            DeclareLaunchArgument(
                'left_driver_config',
                default_value='auto',
                description='RX75 left-arm driver YAML path, or auto',
            ),
            DeclareLaunchArgument(
                'right_driver_config',
                default_value='auto',
                description='RX75 right-arm driver YAML path, or auto',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
