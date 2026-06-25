from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    control_node = Node(
    package='rm_control', #节点所在的功能包
    executable='rm_control', #表示要运行的可执行文件名或脚本名字.py
    parameters= [
                    {'follow': False},
                    {'arm_type': 631}
                ],             #接入参数文件
    output='screen', #用于将话题信息打印到屏幕
    )

    ld.add_action(control_node)
    return ld

