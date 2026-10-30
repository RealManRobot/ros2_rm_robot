import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from eco62_moveit_common import generate_moveit_real_launch


def generate_launch_description():
    return generate_moveit_real_launch(
        "rm_eco62_6fb_description.urdf.xacro",
        {"link6_type": "Link6_6fb"},
    )
