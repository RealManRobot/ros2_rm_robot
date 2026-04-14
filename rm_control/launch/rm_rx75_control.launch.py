from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    left_control_node = Node(
    package='rm_control', #节点所在的功能包
    executable='rm_control', #表示要运行的可执行文件名或脚本名字.py
    namespace="left_arm",
    parameters= [
                    {'follow': True},
                    {'arm_type': 75}
                ],             #接入参数文件
    output='screen', #用于将话题信息打印到屏幕
    )
    right_control_node = Node(
        package='rm_control',
        executable='rm_control',
        namespace="right_arm",
        parameters= [
                {'follow': True},
                {'arm_type': 75}
        ],
        output='screen',
    )
    return LaunchDescription([
        left_control_node,
        right_control_node,
    ])
