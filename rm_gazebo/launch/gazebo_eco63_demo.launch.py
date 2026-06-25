import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from gz_demo_common import generate_gz_demo_launch


def generate_launch_description():
    return generate_gz_demo_launch(
        urdf_filename='gazebo_eco63_description.urdf.xacro',
        robot_name_in_model='rm_eco63_description',
        controller_names=['joint_state_broadcaster', 'rm_group_controller'],
        xacro_mappings={'link6_type': 'Link6'},
        clock_topic_default='/rm_eco63/clock',
        joint_states_topic_default='/joint_state_broadcaster/joint_states',
        static_transforms=[
            {
                'x': 0,
                'y': 0,
                'z': 0,
                'roll': 0,
                'pitch': 0,
                'yaw': 0,
                'parent': 'world',
                'child': 'base_root',
            },
        ],
    )
