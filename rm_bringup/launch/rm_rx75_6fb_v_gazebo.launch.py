from rm_bringup.legacy_bringup import generate_legacy_bringup


def generate_launch_description():
    return generate_legacy_bringup(
        'rx75-6fb-v', 'gazebo'
    )
