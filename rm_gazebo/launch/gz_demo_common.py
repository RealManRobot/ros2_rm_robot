import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
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


def generate_gz_demo_launch(
    *,
    urdf_filename,
    robot_name_in_model,
    controller_names,
    xacro_mappings=None,
    joint_states_topic_default="/joint_states",
):
    package_name = "rm_gazebo"
    world_name = "empty"
    start_gazebo = LaunchConfiguration("start_gazebo")
    joint_states_topic = LaunchConfiguration("joint_states_topic")

    try:
        get_package_share_directory("gz_ros2_control")
    except PackageNotFoundError as exc:
        raise RuntimeError(
            "Missing ROS package 'gz_ros2_control'. Install 'ros-humble-gz-ros2-control' "
            "or the shim package 'ros-humble-ign-ros2-control' before launching Gazebo demos."
        ) from exc

    pkg_share = get_package_share_directory(package_name)
    description_share = get_package_share_directory("rm_description")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    urdf_model_path = os.path.join(pkg_share, "config", urdf_filename)
    gz_resource_parent = os.path.dirname(description_share)

    robot_description = xacro.process_file(
        urdf_model_path,
        mappings=xacro_mappings or {},
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
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
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

    unpause_world = ExecuteProcess(
        cmd=[
            "ign",
            "service",
            "-s",
            f"/world/{world_name}/control",
            "--reqtype",
            "ignition.msgs.WorldControl",
            "--reptype",
            "ignition.msgs.Boolean",
            "--timeout",
            "3000",
            "--req",
            "pause: false",
        ],
        output="screen",
    )

    spawn_controllers = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            *controller_names,
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
            on_exit=[TimerAction(period=2.0, actions=[unpause_world])],
        )
    )
    close_evt2 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=unpause_world,
            on_exit=[TimerAction(period=2.0, actions=[spawn_controllers])],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_gazebo", default_value="true"),
            DeclareLaunchArgument(
                "joint_states_topic",
                default_value=joint_states_topic_default,
            ),
            gz_resource_path,
            close_evt1,
            close_evt2,
            gazebo,
            node_robot_state_publisher,
            clock_bridge,
            spawn_entity,
        ]
    )
