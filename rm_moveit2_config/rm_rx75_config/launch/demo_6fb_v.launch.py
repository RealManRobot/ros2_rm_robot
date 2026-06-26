import os
import sys

sys.path.append(os.path.dirname(__file__))
from rx75_moveit_common import generate_rx75_real_moveit_launch


def generate_launch_description():
    return generate_rx75_real_moveit_launch(
        "rm_rx75-6fb_v.urdf.xacro",
        "rm_rx75_6fb_v_description.srdf",
    )
