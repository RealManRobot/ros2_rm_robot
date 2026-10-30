from rm_description.legacy_display import generate_legacy_display_launch


def generate_launch_description():
    return generate_legacy_display_launch(
        "63_iii",
        xacro_arguments=(
            ("link6_type", "Link6"),
            ("base_type", "base_link_III"),
        ),
    )
