from rm_bringup.legacy_bringup import generate_legacy_bringup


def generate_launch_description():
    return generate_legacy_bringup(
        '63_iii', 'gazebo'
    )
