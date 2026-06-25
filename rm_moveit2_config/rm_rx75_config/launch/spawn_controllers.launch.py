import os
import sys

from moveit_configs_utils.launches import generate_spawn_controllers_launch

sys.path.append(os.path.dirname(__file__))
from _moveit_config_builder import build_sim_moveit_config


def generate_launch_description():
    return generate_spawn_controllers_launch(build_sim_moveit_config())
