import os
import tempfile
import runpy

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import xacro


def resolve_auto_joint_states_topic(value, default_topic):
    requested = value.strip()
    if requested != "auto":
        raise RuntimeError(
            "Launch argument 'joint_states_topic' currently only supports "
            f"'auto'; received '{requested or '<empty>'}'."
        )
    return default_topic


def _shutdown(reason):
    return EmitEvent(event=Shutdown(reason=reason))


def generate_gazebo_classic_demo_actions(
    *,
    urdf_filename,
    robot_name_in_model,
    controller_names,
    xacro_mappings=None,
    static_transforms=None,
    start_gazebo=True,
    use_gazebo_gui=True,
    clock_topic="/clock",
    joint_states_topic="/joint_states",
    spawn_entity_timeout="120",
    use_sim_time=True,
    post_spawn_actions=None,
    arm_type=None,
    model="auto",
):
    """Build shared Gazebo Classic actions for one selected robot model."""
    package_name = "rm_gazebo"
    pkg_share = get_package_share_directory(package_name)
    gazebo_ros_share = get_package_share_directory("gazebo_ros")
    urdf_model_path = os.path.join(pkg_share, "config", urdf_filename)

    if not os.path.isfile(urdf_model_path):
        raise RuntimeError(f"Gazebo model file does not exist: {urdf_model_path}")

    robot_description = xacro.process_file(
        urdf_model_path,
        mappings=xacro_mappings or {},
    ).toxml()
    if arm_type is not None:
        description = runpy.run_path(os.path.join(
            get_package_share_directory("rm_description"), "launch", "rm_description.launch.py",
        ))
        robot_description = description["_apply_visual_model"](robot_description, arm_type, model)
    params = {"robot_description": robot_description}

    server_arguments = {
        "init": "true",
        "factory": "true",
        "server_required": "true",
        "verbose": "true",
    }
    camera_cleanup = []
    if robot_name_in_model == "rm_rx75_dual":
        camera_directory = tempfile.TemporaryDirectory(prefix="rm_rx75_")
        world_path = os.path.join(camera_directory.name, "rx75.world")
        # Gazebo Classic reads its initial camera from the world GUI element.
        with open(world_path, "w", encoding="utf-8") as target:
            target.write("""<sdf version="1.5">
  <world name="default">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>2.5 0 2 0 0.45 3.141592653589793</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>
  </world>
</sdf>
""")
        server_arguments["world"] = world_path
        camera_cleanup.append(RegisterEventHandler(OnShutdown(
            on_shutdown=lambda event, context: camera_directory.cleanup(),
        )))

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, "launch", "gzserver.launch.py")
        ),
        launch_arguments=server_arguments.items(),
        condition=IfCondition(start_gazebo),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, "launch", "gzclient.launch.py")
        ),
        launch_arguments={"verbose": "true"}.items(),
        condition=IfCondition(use_gazebo_gui),
    )

    node_robot_state_publisher = Node(
        on_exit=[
            _shutdown("Gazebo robot_state_publisher exited")
        ],
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "use_sim_time": ParameterValue(
                    use_sim_time,
                    value_type=bool,
                )
            },
            params,
            {"publish_frequency": 15.0},
        ],
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

    def spawn_controllers_after_success(event, _context):
        if event.returncode != 0:
            return [
                _shutdown(
                    f"Gazebo entity '{robot_name_in_model}' failed to spawn "
                    f"(exit code {event.returncode})."
                )
            ]

        actions = [TimerAction(period=2.0, actions=spawn_controllers)]
        if post_spawn_actions:
            if isinstance(post_spawn_actions, list):
                actions.extend(post_spawn_actions)
            else:
                actions.append(post_spawn_actions)
        return actions

    spawn_controllers_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=spawn_controllers_after_success,
        )
    )

    controller_exit_events = []
    for controller_name, controller_process in zip(
        controller_names, spawn_controllers
    ):
        def stop_on_controller_failure(
            event,
            _context,
            name=controller_name,
        ):
            if event.returncode == 0:
                return []
            return [
                _shutdown(
                    f"Controller '{name}' failed to start "
                    f"(exit code {event.returncode})."
                )
            ]

        controller_exit_events.append(
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=controller_process,
                    on_exit=stop_on_controller_failure,
                )
            )
        )

    return [
        *camera_cleanup,
        spawn_controllers_event,
        *controller_exit_events,
        gzserver,
        gzclient,
        node_robot_state_publisher,
        *static_transform_nodes,
        spawn_entity,
    ]


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
    """Build a legacy per-model Gazebo Classic launch description."""
    actions = generate_gazebo_classic_demo_actions(
        urdf_filename=urdf_filename,
        robot_name_in_model=robot_name_in_model,
        controller_names=controller_names,
        xacro_mappings=xacro_mappings,
        static_transforms=static_transforms,
        start_gazebo=LaunchConfiguration("start_gazebo"),
        use_gazebo_gui=LaunchConfiguration("use_gazebo_gui"),
        clock_topic=LaunchConfiguration("clock_topic"),
        joint_states_topic=LaunchConfiguration("joint_states_topic"),
        spawn_entity_timeout=LaunchConfiguration("spawn_entity_timeout"),
        use_sim_time=True,
        post_spawn_actions=post_spawn_actions,
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
            *actions,
        ]
    )


# Keep the old helper name so the per-arm launch files do not need to change.
generate_gz_demo_launch = generate_gazebo_classic_demo_launch
