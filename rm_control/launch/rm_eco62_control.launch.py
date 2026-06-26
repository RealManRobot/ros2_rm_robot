from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    control_node = Node(
        package='rm_control',
        executable='rm_control',
        parameters=[
            {'follow': False},
            {'arm_type': 621}
        ],
        output='screen',
    )

    ld.add_action(control_node)
    return ld
