"""
Canonical component plans for every supported robot model.

The catalog is deliberately explicit. Launch filenames, hardware profile
reuse, dual-arm topology, and joint-state defaults are product capabilities;
they must not be inferred from string concatenation.
"""

from __future__ import annotations

from dataclasses import dataclass
from rm_description.variant_catalog import normalize_arm_type


PACKAGE_NAME = 'rm_bringup'

MODES = ('real', 'gazebo')


@dataclass(frozen=True)
class LaunchReference:
    """One installed launch file."""

    package: str
    launch_file: str


@dataclass(frozen=True)
class ArmProfile:
    """Hardware/control topology shared by one or more product models."""

    arm_type: str
    driver_profile: str
    control_profile: str
    topology: str
    real_joint_states_topic: str
    gazebo_joint_states_topic: str
    use_joint_state_bridge: bool = False

    def joint_states_topic(self, mode: str) -> str:
        if mode == 'real':
            return self.real_joint_states_topic
        if mode == 'gazebo':
            return self.gazebo_joint_states_topic
        raise ValueError(
            f"Unsupported mode: {mode}. Valid modes: {', '.join(MODES)}."
        )


ARM_PROFILES = {
    '63': ArmProfile(
        '63', '63', '63', 'single', '/joint_states', '/joint_states'
    ),
    '63_iii': ArmProfile(
        '63_iii', '63', '63', 'single', '/joint_states', '/joint_states'
    ),
    '65': ArmProfile(
        '65', '65', '65', 'single', '/joint_states', '/joint_states'
    ),
    '75': ArmProfile(
        '75', '75', '75', 'single', '/joint_states', '/joint_states'
    ),
    'eco62': ArmProfile(
        'eco62', 'eco62', 'eco62', 'single', '/joint_states', '/joint_states'
    ),
    'eco63': ArmProfile(
        'eco63', 'eco63', 'eco63', 'single', '/joint_states', '/joint_states'
    ),
    'eco65': ArmProfile(
        'eco65', 'eco65', 'eco65', 'single', '/joint_states', '/joint_states'
    ),
    'gen72': ArmProfile(
        'gen72', 'gen72', 'gen72', 'single', '/joint_states', '/joint_states'
    ),
    'gen72_ii': ArmProfile(
        'gen72_ii', 'gen72', 'gen72', 'single', '/joint_states', '/joint_states'
    ),
    'rx75': ArmProfile(
        'rx75',
        'rx75',
        'rx75',
        'dual',
        '/joint_states',
        '/joint_state_broadcaster/joint_states',
        use_joint_state_bridge=True,
    ),
}


@dataclass(frozen=True)
class ModelEntry:
    """The two legacy launch files for one complete arm_type."""

    arm_type: str
    family: str
    real_launch: str
    gazebo_launch: str

    def launch_file(self, mode: str) -> str:
        """Return the legacy launch filename for a canonical mode."""
        if mode == 'real':
            return self.real_launch
        if mode == 'gazebo':
            return self.gazebo_launch
        raise ValueError(
            f"Unsupported mode: {mode}. Valid modes: {', '.join(MODES)}."
        )


@dataclass(frozen=True)
class ResolvedBringupPlan:
    """A normalized, component-level plan for the unified launch."""

    arm_type: str
    mode: str
    arm_profile: ArmProfile
    driver: LaunchReference
    description: LaunchReference
    control: LaunchReference
    gazebo: LaunchReference
    moveit: LaunchReference
    legacy: LaunchReference

    @property
    def components(self) -> tuple[LaunchReference, ...]:
        if self.mode == 'real':
            return (self.driver, self.description, self.control, self.moveit)
        return (self.gazebo, self.moveit)


_MODEL_ENTRIES = (
    ModelEntry(
        '63',
        '63',
        'rm_63_bringup.launch.py',
        'rm_63_gazebo.launch.py',
    ),
    ModelEntry(
        '63-6f',
        '63',
        'rm_63_6f_bringup.launch.py',
        'rm_63_6f_gazebo.launch.py',
    ),
    ModelEntry(
        '63-6fb',
        '63',
        'rm_63_6fb_bringup.launch.py',
        'rm_63_6fb_gazebo.launch.py',
    ),
    ModelEntry(
        '63_iii',
        '63_iii',
        'rm_63_III_bringup.launch.py',
        'rm_63_III_gazebo.launch.py',
    ),
    ModelEntry(
        '63_iii-6fb',
        '63_iii',
        'rm_63_III_6fb_bringup.launch.py',
        'rm_63_III_6fb_gazebo.launch.py',
    ),
    ModelEntry(
        '65',
        '65',
        'rm_65_bringup.launch.py',
        'rm_65_gazebo.launch.py',
    ),
    ModelEntry(
        '65-6f',
        '65',
        'rm_65_6f_bringup.launch.py',
        'rm_65_6f_gazebo.launch.py',
    ),
    ModelEntry(
        '65-6fb',
        '65',
        'rm_65_6fb_bringup.launch.py',
        'rm_65_6fb_gazebo.launch.py',
    ),
    ModelEntry(
        '75',
        '75',
        'rm_75_bringup.launch.py',
        'rm_75_gazebo.launch.py',
    ),
    ModelEntry(
        '75-6f',
        '75',
        'rm_75_6f_bringup.launch.py',
        'rm_75_6f_gazebo.launch.py',
    ),
    ModelEntry(
        '75-6fb',
        '75',
        'rm_75_6fb_bringup.launch.py',
        'rm_75_6fb_gazebo.launch.py',
    ),
    ModelEntry(
        'eco62',
        'eco62',
        'rm_eco62_bringup.launch.py',
        'rm_eco62_gazebo.launch.py',
    ),
    ModelEntry(
        'eco63',
        'eco63',
        'rm_eco63_bringup.launch.py',
        'rm_eco63_gazebo.launch.py',
    ),
    ModelEntry(
        'eco63-6fb',
        'eco63',
        'rm_eco63_6fb_bringup.launch.py',
        'rm_eco63_6fb_gazebo.launch.py',
    ),
    ModelEntry(
        'eco65',
        'eco65',
        'rm_eco65_bringup.launch.py',
        'rm_eco65_gazebo.launch.py',
    ),
    ModelEntry(
        'eco65-6f',
        'eco65',
        'rm_eco65_6f_bringup.launch.py',
        'rm_eco65_6f_gazebo.launch.py',
    ),
    ModelEntry(
        'eco65-6fb',
        'eco65',
        'rm_eco65_6fb_bringup.launch.py',
        'rm_eco65_6fb_gazebo.launch.py',
    ),
    ModelEntry(
        'gen72',
        'gen72',
        'rm_gen72_bringup.launch.py',
        'rm_gen72_gazebo.launch.py',
    ),
    ModelEntry(
        'gen72_ii',
        'gen72_ii',
        'rm_gen72_II_bringup.launch.py',
        'rm_gen72_II_gazebo.launch.py',
    ),
    ModelEntry(
        'rx75-6fb',
        'rx75',
        'rm_rx75_6fb_bringup.launch.py',
        'rm_rx75_6fb_gazebo.launch.py',
    ),
    ModelEntry(
        'rx75-6fb-v',
        'rx75',
        'rm_rx75_6fb_v_bringup.launch.py',
        'rm_rx75_6fb_v_gazebo.launch.py',
    ),
)

MODEL_CATALOG = {entry.arm_type: entry for entry in _MODEL_ENTRIES}
if len(MODEL_CATALOG) != len(_MODEL_ENTRIES):
    raise RuntimeError('Duplicate arm_type entries in model catalog.')
ARM_TYPES = tuple(MODEL_CATALOG)

_GENERIC_DRIVER = LaunchReference('rm_driver', 'rm_driver.launch.py')
_GENERIC_DESCRIPTION = LaunchReference(
    'rm_description', 'rm_description.launch.py'
)
_GENERIC_CONTROL = LaunchReference('rm_control', 'rm_control.launch.py')
_GENERIC_GAZEBO = LaunchReference('rm_gazebo', 'rm_gazebo.launch.py')

_MOVEIT_LAUNCHES = {
    '63': (
        'rm_63_config',
        'real_moveit_demo.launch.py',
        'gazebo_moveit_demo.launch.py',
    ),
    '63-6f': (
        'rm_63_config',
        'real_moveit_demo_6f.launch.py',
        'gazebo_moveit_demo_6f.launch.py',
    ),
    '63-6fb': (
        'rm_63_config',
        'real_moveit_demo_6fb.launch.py',
        'gazebo_moveit_demo_6fb.launch.py',
    ),
    '63_iii': (
        'rm_63_config',
        'real_moveit_demo_III.launch.py',
        'gazebo_moveit_demo_III.launch.py',
    ),
    '63_iii-6fb': (
        'rm_63_config',
        'real_moveit_demo_III_6fb.launch.py',
        'gazebo_moveit_demo_III_6fb.launch.py',
    ),
    '65': (
        'rm_65_config',
        'real_moveit_demo.launch.py',
        'gazebo_moveit_demo.launch.py',
    ),
    '65-6f': (
        'rm_65_config',
        'real_moveit_demo_6f.launch.py',
        'gazebo_moveit_demo_6f.launch.py',
    ),
    '65-6fb': (
        'rm_65_config',
        'real_moveit_demo_6fb.launch.py',
        'gazebo_moveit_demo_6fb.launch.py',
    ),
    '75': (
        'rm_75_config',
        'real_moveit_demo.launch.py',
        'gazebo_moveit_demo.launch.py',
    ),
    '75-6f': (
        'rm_75_config',
        'real_moveit_demo_6f.launch.py',
        'gazebo_moveit_demo_6f.launch.py',
    ),
    '75-6fb': (
        'rm_75_config',
        'real_moveit_demo_6fb.launch.py',
        'gazebo_moveit_demo_6fb.launch.py',
    ),
    'eco62': (
        'rm_eco62_config',
        'real_moveit_demo.launch.py',
        'gazebo_moveit_demo.launch.py',
    ),
    'eco63': (
        'rm_eco63_config',
        'real_moveit_demo.launch.py',
        'gazebo_moveit_demo.launch.py',
    ),
    'eco63-6fb': (
        'rm_eco63_config',
        'real_moveit_demo_6fb.launch.py',
        'gazebo_moveit_demo_6fb.launch.py',
    ),
    'eco65': (
        'rm_eco65_config',
        'real_moveit_demo.launch.py',
        'gazebo_moveit_demo.launch.py',
    ),
    'eco65-6f': (
        'rm_eco65_config',
        'real_moveit_demo_6f.launch.py',
        'gazebo_moveit_demo_6f.launch.py',
    ),
    'eco65-6fb': (
        'rm_eco65_config',
        'real_moveit_demo_6fb.launch.py',
        'gazebo_moveit_demo_6fb.launch.py',
    ),
    'gen72': (
        'rm_gen72_config',
        'real_moveit_demo.launch.py',
        'gazebo_moveit_demo.launch.py',
    ),
    'gen72_ii': (
        'rm_gen72_config',
        'real_moveit_demo_II.launch.py',
        'gazebo_moveit_demo_II.launch.py',
    ),
    'rx75-6fb': (
        'rm_rx75_config',
        'real_moveit_demo_6fb.launch.py',
        'gazebo_moveit_demo_6fb.launch.py',
    ),
    'rx75-6fb-v': (
        'rm_rx75_config',
        'real_moveit_demo_6fb_v.launch.py',
        'gazebo_moveit_demo_6fb_v.launch.py',
    ),
}

if set(_MOVEIT_LAUNCHES) != set(MODEL_CATALOG):
    raise RuntimeError('MoveIt launch map and model catalog are out of sync.')

def _require_token(value: str | None, parameter: str) -> str:
    if value is None or not isinstance(value, str) or not value.strip():
        if parameter == 'arm_type':
            raise ValueError(
                f"arm_type is required. Valid arm types: {', '.join(ARM_TYPES)}."
            )
        raise ValueError(f'{parameter} must be a non-empty string.')
    return value.strip()


def normalize_mode(value: str | None) -> str:
    """Normalize a launch mode while rejecting unsupported simulation aliases."""
    raw_value = _require_token(value, 'mode')
    normalized = raw_value.lower()
    if normalized not in MODES:
        raise ValueError(
            f"Unsupported mode: {raw_value}. Valid modes: {', '.join(MODES)}."
        )
    return normalized


def resolve_model(
    arm_type: str | None,
    mode: str | None = 'real',
    controller_version: int = 3,
) -> ResolvedBringupPlan:
    """Resolve public selector values to a component-level bringup plan."""
    canonical_arm_type = normalize_arm_type(arm_type, controller_version)
    canonical_mode = normalize_mode(mode)
    entry = MODEL_CATALOG[canonical_arm_type]
    moveit_package, real_moveit_launch, gazebo_moveit_launch = (
        _MOVEIT_LAUNCHES[canonical_arm_type]
    )
    moveit_launch = (
        real_moveit_launch if canonical_mode == 'real' else gazebo_moveit_launch
    )
    return ResolvedBringupPlan(
        arm_type=canonical_arm_type,
        mode=canonical_mode,
        arm_profile=ARM_PROFILES[entry.family],
        driver=_GENERIC_DRIVER,
        description=_GENERIC_DESCRIPTION,
        control=_GENERIC_CONTROL,
        gazebo=_GENERIC_GAZEBO,
        moveit=LaunchReference(moveit_package, moveit_launch),
        legacy=LaunchReference(PACKAGE_NAME, entry.launch_file(canonical_mode)),
    )
