import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from rx75_moveit_common import generate_rx75_gazebo_moveit_launch


def generate_launch_description():
    return generate_rx75_gazebo_moveit_launch(
        "rm_rx75-6fb_v.urdf.xacro",
        "rm_rx75_6fb_v_description.srdf",
    )
