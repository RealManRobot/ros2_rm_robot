import re

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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

_CONTROL_PROFILES = {
    '63': {
        'suffixes': ('6f', '6fb'),
        'arm_type_code': 632,
        'default_follow': False,
        'namespaces': (None,),
    },
    '63_iii': {
        'suffixes': ('6f', '6fb'),
        'arm_type_code': 632,
        'default_follow': False,
        'namespaces': (None,),
    },
    '65': {
        'suffixes': ('6f', '6fb'),
        'arm_type_code': 65,
        'default_follow': False,
        'namespaces': (None,),
    },
    '75': {
        'suffixes': ('6f', '6fb'),
        'arm_type_code': 75,
        'default_follow': False,
        'namespaces': (None,),
    },
    'eco62': {
        'arm_type_code': 621,
        'default_follow': False,
        'namespaces': (None,),
    },
    'eco63': {
        'suffixes': ('6f', '6fb'),
        'arm_type_code': 634,
        'default_follow': False,
        'namespaces': (None,),
    },
    'eco65': {
        'suffixes': ('6f', '6fb'),
        'arm_type_code': 651,
        'default_follow': False,
        'namespaces': (None,),
    },
    'gen72': {
        'arm_type_code': 72,
        'default_follow': False,
        'namespaces': (None,),
    },
    'gen72_ii': {
        'arm_type_code': 72,
        'default_follow': False,
        'namespaces': (None,),
    },
    'rx75': {
        'suffixes': ('6f', '6f-v', '6fb', '6fb-v'),
        'arm_type_code': 75,
        'default_follow': False,
        'namespaces': ('left_arm', 'right_arm'),
    },
}


_ARM_TYPE_ALIASES.update({
    alias + suffix.replace('-', ''): family
    for alias, family in _ARM_TYPE_ALIASES.items()
    for suffix in _CONTROL_PROFILES[family].get('suffixes', ())
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
            for family, profile in _CONTROL_PROFILES.items()
            for suffix in ('',) + profile.get('suffixes', ())
        )
        raise ValueError(
            f'Unsupported arm_type: {value.strip()}. '
            f'Valid arm types: {valid_types}.'
        )
    return normalized


def resolve_follow(value, default_follow):
    if not isinstance(value, str) or not value.strip():
        raise ValueError('follow must be one of: auto, true, false.')

    normalized = value.strip().lower()
    if normalized == 'auto':
        return default_follow
    if normalized == 'true':
        return True
    if normalized == 'false':
        return False
    raise ValueError(
        f'Invalid follow value: {value}. '
        'Expected exactly one of: auto, true, false.'
    )


def build_control_specs(arm_type, follow='auto'):
    canonical_type = normalize_arm_type(arm_type)
    profile = _CONTROL_PROFILES[canonical_type]
    resolved_follow = resolve_follow(follow, profile['default_follow'])

    return [
        {
            'namespace': namespace,
            'follow': resolved_follow,
            'arm_type_code': profile['arm_type_code'],
        }
        for namespace in profile['namespaces']
    ]


def _launch_setup(context):
    specs = build_control_specs(
        arm_type=LaunchConfiguration('arm_type').perform(context),
        follow=LaunchConfiguration('follow').perform(context),
    )

    actions = []
    for spec in specs:
        node_arguments = {
            'package': 'rm_control',
            'executable': 'rm_control',
            'parameters': [
                {'follow': spec['follow']},
                {'arm_type': spec['arm_type_code']},
            ],
            'output': 'screen',
            'on_exit': [
                EmitEvent(event=Shutdown(reason='rm_control exited'))
            ],
        }
        if spec['namespace'] is not None:
            node_arguments['namespace'] = spec['namespace']
        actions.append(Node(**node_arguments))
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'arm_type',
                description=(
                    'Robot model including supported end-link suffixes, '
                    'e.g. 65, 65-6fb or eco63-6fb'
                ),
            ),
            DeclareLaunchArgument(
                'follow',
                default_value='auto',
                description=(
                    'Trajectory following mode: auto, true, or false. '
                    'auto selects low-follow mode for every model.'
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
