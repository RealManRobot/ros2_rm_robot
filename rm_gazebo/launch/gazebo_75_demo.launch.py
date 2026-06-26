import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from gz_demo_common import generate_gz_demo_launch


def generate_launch_description():
    return generate_gz_demo_launch(
        urdf_filename='gazebo_75_description.urdf.xacro',
        robot_name_in_model='rm_75_description',
        controller_names=['joint_state_broadcaster', 'rm_group_controller'],
    )
