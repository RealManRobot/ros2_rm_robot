from launch import LaunchDescription
from launch_ros.actions import Node


def _rm_control_node(namespace, arm_type):
    return Node(
        package="rm_control",
        executable="rm_control",
        namespace=namespace,
        name="rm_control",
        output="screen",
        parameters=[
            {
                "follow": True,
                "arm_type": arm_type,
            }
        ],
        remappings=[
            (
                "/rm_driver/movej_canfd_cmd",
                f"/{namespace}/rm_driver/movej_canfd_cmd",
            ),
            (
                "rm_driver/move_stop_cmd",
                f"/{namespace}/rm_driver/move_stop_cmd",
            ),
        ],
    )


def generate_launch_description():
    return LaunchDescription(
        [
            _rm_control_node("left_arm", 75),
            _rm_control_node("right_arm", 75),
        ]
    )
