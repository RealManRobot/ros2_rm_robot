from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'left_execute_motion', default_value='false',
            description='Allow a 0.5 degree move of the left wrist joint'),
        DeclareLaunchArgument(
            'right_execute_motion', default_value='false',
            description='Allow a 0.5 degree move of the right wrist joint'),
        Node(
            package='control_arm_move',
            executable='slight_move_demo',
            name='slight_move_demo',
            namespace='left_arm',
            parameters=[{
                'joint_index': 7,
                'delta_rad': 0.008726646259971648,
                'speed': 5,
                'return_to_start': True,
                'execute_motion': ParameterValue(
                    LaunchConfiguration('left_execute_motion'), value_type=bool),
            }],
            output='screen',
        ),
        Node(
            package='control_arm_move',
            executable='slight_move_demo',
            name='slight_move_demo',
            namespace='right_arm',
            parameters=[{
                'joint_index': 7,
                'delta_rad': 0.008726646259971648,
                'speed': 5,
                'return_to_start': True,
                'execute_motion': ParameterValue(
                    LaunchConfiguration('right_execute_motion'), value_type=bool),
            }],
            output='screen',
        ),
    ])
