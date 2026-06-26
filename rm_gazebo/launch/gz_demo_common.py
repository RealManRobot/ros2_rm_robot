import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro


def generate_gazebo_classic_demo_launch(
    *,
    urdf_filename,
    robot_name_in_model,
    controller_names,
    xacro_mappings=None,
    static_transforms=None,
    clock_topic_default="/clock",
    joint_states_topic_default="/joint_states",
    post_spawn_actions=None,
):
    package_name = "rm_gazebo"
    start_gazebo = LaunchConfiguration("start_gazebo")
    use_gazebo_gui = LaunchConfiguration("use_gazebo_gui")
    clock_topic = LaunchConfiguration("clock_topic")
    joint_states_topic = LaunchConfiguration("joint_states_topic")
    spawn_entity_timeout = LaunchConfiguration("spawn_entity_timeout")

    pkg_share = get_package_share_directory(package_name)
    gazebo_ros_share = get_package_share_directory("gazebo_ros")
    urdf_model_path = os.path.join(pkg_share, "config", urdf_filename)

    robot_description = xacro.process_file(
        urdf_model_path,
        mappings=xacro_mappings or {},
    ).toxml()
    params = {"robot_description": robot_description}

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, "launch", "gzserver.launch.py")
        ),
        launch_arguments={
            "init": "true",
            "factory": "true",
            "verbose": "true",
        }.items(),
        condition=IfCondition(start_gazebo),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, "launch", "gzclient.launch.py")
        ),
        launch_arguments={
            "verbose": "true",
        }.items(),
        condition=IfCondition(use_gazebo_gui),
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"use_sim_time": True}, params, {"publish_frequency": 15.0}],
        remappings=[
            ("/clock", clock_topic),
            ("/joint_states", joint_states_topic),
            ("joint_states", joint_states_topic),
        ],
        output="screen",
    )

    static_transform_nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                str(transform["x"]),
                str(transform["y"]),
                str(transform["z"]),
                str(transform["roll"]),
                str(transform["pitch"]),
                str(transform["yaw"]),
                transform["parent"],
                transform["child"],
            ],
            output="screen",
        )
        for transform in (static_transforms or [])
    ]

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            robot_name_in_model,
            "-timeout",
            spawn_entity_timeout,
            "-spawn_service_timeout",
            spawn_entity_timeout,
        ],
        output="screen",
    )

    spawn_controllers = [
        ExecuteProcess(
            cmd=[
                "ros2",
                "run",
                "controller_manager",
                "spawner.py",
                controller,
                "--controller-manager",
                "/controller_manager",
            ],
            output="screen",
        )
        for controller in controller_names
    ]

    def spawn_controllers_after_success(event, _):
        if event.returncode == 0:
            actions = [TimerAction(period=2.0, actions=spawn_controllers)]
            if post_spawn_actions:
                if isinstance(post_spawn_actions, list):
                    actions.extend(post_spawn_actions)
                else:
                    actions.append(post_spawn_actions)
            return actions
        return []

    spawn_controllers_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=spawn_controllers_after_success,
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_gazebo", default_value="true"),
            DeclareLaunchArgument("use_gazebo_gui", default_value="true"),
            DeclareLaunchArgument("clock_topic", default_value=clock_topic_default),
            DeclareLaunchArgument(
                "joint_states_topic",
                default_value=joint_states_topic_default,
            ),
            DeclareLaunchArgument(
                "spawn_entity_timeout",
                default_value="120",
                description="Seconds to wait for Gazebo spawn_entity service.",
            ),
            spawn_controllers_event,
            gzserver,
            gzclient,
            node_robot_state_publisher,
            *static_transform_nodes,
            spawn_entity,
        ]
    )


# Keep the old helper name so the per-arm launch files do not need to change.
generate_gz_demo_launch = generate_gazebo_classic_demo_launch
