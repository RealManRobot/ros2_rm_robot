import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from gz_demo_common import generate_legacy_gz_demo_launch  # noqa: E402


def generate_launch_description():
    return generate_legacy_gz_demo_launch(
        arm_type="eco65-6f",
    )
