from rm_description.legacy_display import generate_legacy_display_launch


def generate_launch_description():
    return generate_legacy_display_launch(
        "75-6fb", xacro_arguments=(("link7_type", "Link7_6fb"),)
    )
