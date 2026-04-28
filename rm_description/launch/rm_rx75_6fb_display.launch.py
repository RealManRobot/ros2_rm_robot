import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory("rm_description")
    model_path = os.path.join(description_share, "urdf", "rm_rx75-6fb.urdf.xacro")
    rviz_config = os.path.join(description_share, "rviz", "rm_rx75.rviz")


    left_xyz = LaunchConfiguration("left_xyz")
    left_rpy = LaunchConfiguration("left_rpy")
    right_xyz = LaunchConfiguration("right_xyz")
    right_rpy = LaunchConfiguration("right_rpy")
    use_joint_state_bridge = LaunchConfiguration("use_joint_state_bridge")
    use_joint_state_publisher_gui = LaunchConfiguration("use_joint_state_publisher_gui")
    use_rviz = LaunchConfiguration("use_rviz")

    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            model_path,
            " ",
            "left_xyz:='",
            left_xyz,
            "' ",
            "left_rpy:='",
            left_rpy,
            "' ",
            "right_xyz:='",
            right_xyz,
            "' ",
            "right_rpy:='",
            right_rpy,
            "'",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("left_xyz", default_value="0 -0.075 0.5"),
            DeclareLaunchArgument("left_rpy", default_value="3.14 1.57 1.5707963267949"),
            DeclareLaunchArgument("right_xyz", default_value="0 0.075 0.5"),
            DeclareLaunchArgument("right_rpy", default_value="3.14 1.57 -1.5707963267949"),
            DeclareLaunchArgument("use_joint_state_bridge", default_value="false"),
            DeclareLaunchArgument("use_joint_state_publisher_gui", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                respawn=True,
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                condition=IfCondition(use_joint_state_publisher_gui),
                output="screen",
            ),
            Node(
                package="rm_description",
                executable="dual_arm_joint_state_bridge.py",
                name="dual_arm_joint_state_bridge",
                condition=IfCondition(use_joint_state_bridge),
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
