import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.event_handlers import OnProcessExit

from ament_index_python.packages import get_package_share_directory

import xacro

def generate_launch_description():
    package_name = 'rm_gazebo'

    robot_name_in_model = 'rm_gen72_description'

    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, f'config/gazebo_gen72_description.urdf.xacro')

    
    print("---", urdf_model_path)

    doc = xacro.parse(open(urdf_model_path))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    print("urdf", doc.toxml())

    # 启动gazebo
    gazebo =  ExecuteProcess(
        cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen')

    # 启动了robot_state_publisher节点后，该节点会发布 robot_description 话题，话题内容是模型文件urdf的内容
    # 并且会订阅 /joint_states 话题，获取关节的数据，然后发布tf和tf_static话题.
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True}, params, {"publish_frequency":15.0}],
        output='screen'
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', f'{robot_name_in_model}'], 
                        output='screen')

    # gazebo在加载urdf时，根据urdf的设定，会启动一个joint_states节点
    # 关节状态发布器
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    # 路径执行控制器，也就是那个action
    # 这个rm_group_controller需要根据urdf文件里面引用的ros2_controllers.yaml里面的名字确定
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'rm_group_controller'],
        output='screen'
    )

    # 用下面这两个估计是想控制好各个节点的启动顺序
    # 监听 spawn_entity_cmd，当其退出（完全启动）时，启动load_joint_state_controller
    close_evt1 =  RegisterEventHandler( 
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
    )
    # 监听 load_joint_state_controller，当其退出（完全启动）时，启动load_joint_trajectory_controller
    close_evt2 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller],
            )
    )
    
    ld = LaunchDescription([
        close_evt1,
        close_evt2,
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
    ])

    return ld
