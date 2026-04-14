from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    # TODO需要在CPP中加入分别控制两个机械臂的两种topic分别控制 而不是同时控制两个机械臂
    left_move_node = Node(
    package='control_arm_move', #节点所在的功能包
    executable='move_demo', #表示要运行的可执行文件名或脚本名字.py
    namespace= "left_arm",
    parameters= [
                    {'arm_dof': 7}
                ],             #接入自由度参数
    output='screen', #用于将话题信息打印到屏幕
    )

    right_move_node = Node(
    package='control_arm_move', #节点所在的功能包
    executable='move_demo', #表示要运行的可执行文件名或脚本名字.py
    namespace= "right_arm",
    parameters= [
                    {'arm_dof': 7}
                ],             #接入自由度参数
    output='screen', #用于将话题信息打印到屏幕
    )

    return LaunchDescription([
        left_move_node,
        right_move_node,
    ])


