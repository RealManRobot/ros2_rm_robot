from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'arm_namespace', default_value='',
            description='Namespace containing the target rm_driver node'),
        DeclareLaunchArgument(
            'execute_motion', default_value='false',
            description='Explicitly arm the bounded physical movement'),
        DeclareLaunchArgument(
            'joint_index', default_value='1',
            description='One-based joint index to move'),
        DeclareLaunchArgument(
            'delta_rad', default_value='0.008726646259971648',
            description='Relative joint movement in radians; absolute maximum is 1 degree'),
        DeclareLaunchArgument(
            'speed', default_value='5',
            description='Conservative MoveJ speed in the range 1..20'),
        DeclareLaunchArgument(
            'return_to_start', default_value='true',
            description='Return to the captured start joint position after the slight move'),
        Node(
            package='control_arm_move',
            executable='slight_move_demo',
            name='slight_move_demo',
            namespace=LaunchConfiguration('arm_namespace'),
            output='screen',
            parameters=[{
                'execute_motion': ParameterValue(
                    LaunchConfiguration('execute_motion'), value_type=bool),
                'joint_index': ParameterValue(
                    LaunchConfiguration('joint_index'), value_type=int),
                'delta_rad': ParameterValue(
                    LaunchConfiguration('delta_rad'), value_type=float),
                'speed': ParameterValue(
                    LaunchConfiguration('speed'), value_type=int),
                'return_to_start': ParameterValue(
                    LaunchConfiguration('return_to_start'), value_type=bool),
            }],
        ),
    ])
