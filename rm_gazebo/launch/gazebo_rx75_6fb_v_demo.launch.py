import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from gz_demo_common import generate_gz_demo_launch


def generate_launch_description():
    return generate_gz_demo_launch(
        urdf_filename="gazebo_rx75_6fb_v_description.urdf.xacro",
        robot_name_in_model="rm_rx75_dual",
        controller_names=[
            "joint_state_broadcaster",
            "left_arm_controller",
            "right_arm_controller",
        ],
        joint_states_topic_default="/joint_state_broadcaster/joint_states",
    )
