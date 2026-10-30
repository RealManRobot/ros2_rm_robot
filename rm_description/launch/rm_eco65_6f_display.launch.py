from rm_description.legacy_display import generate_legacy_display_launch


def generate_launch_description():
    return generate_legacy_display_launch(
        "eco65-6f", xacro_arguments=(("link6_type", "Link6_6f"),)
    )
