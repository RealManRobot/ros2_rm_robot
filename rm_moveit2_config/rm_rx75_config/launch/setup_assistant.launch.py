import os
import sys

from moveit_configs_utils.launches import generate_setup_assistant_launch

sys.path.append(os.path.dirname(__file__))
from _moveit_config_builder import build_real_moveit_config


def generate_launch_description():
    return generate_setup_assistant_launch(build_real_moveit_config())
