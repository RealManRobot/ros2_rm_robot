import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

import xacro

sys.path.insert(0, str(Path(__file__).resolve().parent))

from gz_demo_common import get_ros2_control_backend


def generate_launch_description():
    package_name = "rm_gazebo"
    robot_name_in_model = "rm_rx75_dual"
    world_name = "empty"
    start_gazebo = LaunchConfiguration("start_gazebo")
    joint_states_topic = LaunchConfiguration("joint_states_topic")

    ros2_control_backend = get_ros2_control_backend()

    pkg_share = get_package_share_directory(package_name)
    description_share = get_package_share_directory("rm_description")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    urdf_model_path = os.path.join(pkg_share, "config", "gazebo_rx75_6fb_description.urdf.xacro")
    gz_resource_parent = os.path.dirname(description_share)

    robot_description = xacro.process_file(
        urdf_model_path,
        mappings={
            "ros2_control_hardware_plugin": ros2_control_backend["hardware_plugin"],
            "ros2_control_plugin_filename": ros2_control_backend["plugin_filename"],
            "ros2_control_plugin_name": ros2_control_backend["plugin_name"],
        },
    ).toxml()
    params = {"robot_description": robot_description}

    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            gz_resource_parent,
            ":",
            EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
        ],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-v 4 -r {world_name}.sdf"}.items(),
        condition=IfCondition(start_gazebo),
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"use_sim_time": True}, params, {"publish_frequency": 15.0}],
        remappings=[
            ("/joint_states", joint_states_topic),
            ("joint_states", joint_states_topic),
        ],
        output="screen",
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[f"/world/{world_name}/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        remappings=[(f"/world/{world_name}/clock", "/clock")],
        output="screen",
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world",
            world_name,
            "-topic",
            "robot_description",
            "-name",
            robot_name_in_model,
        ],
        output="screen",
    )

    spawn_controllers = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "left_arm_controller",
            "right_arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
            "--switch-timeout",
            "120",
            "--service-call-timeout",
            "30",
            "--activate-as-group",
        ],
        output="screen",
    )

    close_evt1 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[TimerAction(period=2.0, actions=[spawn_controllers])],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_gazebo",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "joint_states_topic",
                default_value="/joint_state_broadcaster/joint_states",
            ),
            gz_resource_path,
            close_evt1,
            gazebo,
            node_robot_state_publisher,
            clock_bridge,
            spawn_entity,
        ]
    )
