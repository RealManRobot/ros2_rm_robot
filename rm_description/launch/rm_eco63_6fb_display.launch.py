from rm_description.legacy_display import generate_legacy_display_launch


def generate_launch_description():
    return generate_legacy_display_launch(
        "eco63-6fb", xacro_arguments=(("link6_type", "Link6_6fb"),)
    )
