<div align="right">
  
[中文简体](https://github.com/RealManRobot/ros2_rm_robot/blob/humble1.0.1/rm_driver/doc/%E7%9D%BF%E5%B0%94%E6%9B%BC%E6%9C%BA%E6%A2%B0%E8%87%82ROS2rm_driver%E8%AF%9D%E9%A2%98%E8%AF%A6%E7%BB%86%E8%AF%B4%E6%98%8E.md)|
[English](https://github.com/RealManRobot/ros2_rm_robot/blob/humble1.0.1/rm_driver/doc/RealMan%20Robotic%20Arm%20rm_driver%20Topic%20Detailed%20Description%20(ROS2).md)

</div>

<div align="center">

# RealMan Robotic Arm rm_driver Topic Detailed Description (ROS2) V1.0


 


RealMan Intelligent Technology (Beijing) Co., Ltd.
 
Revision History:

|No. | Date | Comment |
| :---: | :---- | :---: |
|V1.0 | 2024-2-18 | Draft |
		
</div>

## Content
* 1.[Introduction](#Introduction)
* 2.[Error Description](#Error_Description)
* 2.1[Controller error type](#Controller_error_type)
* 2.2[Joint error type](#Joint_error_type)
* 2.3[API error type](#API_error_type)
* 3.[ROS Function Package Robotic Arm Instructions](#ROS_Function_Package_Robotic_Arm_Instructions)
* 3.1[Joint Configuration](#Joint_Configuration)
* 3.1.1[Clear the joint's error code](#Clear_the_joint's_error_code)
* 3.2[Functions related to the work coordinate system settings](#Functions_related_to_the_work_coordinate_system_settings)
* 3.2.1[Change the current work coordinate system](#Change_the_current_work_coordinate_system)
* 3.3[Coordinate system query](#Coordinate_system_query)
* 3.3.1[Get the current tool coordinate system](#Get_the_current_tool_coordinate_system)
* 3.3.2[Get all tool coordinate system names](#Get_all_tool_coordinate_system_names)
* 3.3.3[Get the current work coordinate system](#Get_the_current_work_coordinate_system)
* 3.3.4[Get all work coordinate system names](#Get_all_work_coordinate_system_names)
* 3.4[Functions related to the arm state query](#Functions_related_to_the_arm_state_query)
* 3.4.1[Get the current state of the robot arm - return each joint angle + Euler angle](#Get_the_current_state_of_the_robot_arm-return_each_joint_angle_and_Euler_angle)
* 3.4.2[Get the current state of the robotic arm - return each joint radians + quaternion](#Get_the_current_state_of_the_robotic_arm-return_each_joint_radians_and_quaternion)
* 3.5[Functions related to motion planning of the robotic arm](#Functions_related_to_motion_planning_of_the_robotic_arm)
* 3.5.1[Joint Space Motion](#Joint_Space_Motion)
* 3.5.2[Linear motion in Cartesian space](#Linear_motion_in_Cartesian_space)
* 3.5.3[Circular motion in Cartesian space](#Circular_motion_in_Cartesian_space)
* 3.5.4[Joint angle CANFD transmission](#Joint_angle_CANFD_transmission)
* 3.5.5[Pose CANFD transmission](#Pose_CANFD_transmission)
* 3.5.6[Joint space planning to target pose](#Joint_space_planning_to_target_pose)
* 3.5.7[Trajectory emergency stop](#Trajectory_emergency_stop)
* 3.6[Functions related to controller configuration](#Functions_related_to_controller_configuration)
* 3.6.1[Get the controller's version](#Get_the_controller's_version)
* 3.7[Functions related to the IO configuration of the end tool](#Functions_related_to_the_IO_configuration_of_the_end_tool)
* 3.7.1[Setting the tool voltage output](#Setting_the_tool_voltage_output)
* 3.8[Functions related to the control of the end gripper - optional](#Functions_related_to_the_control_of_the_end_gripper)
* 3.8.1[Setting the Gripper Pick](#Setting_the_Gripper_Pick)
* 3.8.2[Setting the gripper pick-on](#Setting_the_gripper_pick-on)
* 3.8.3[Setting the gripper to the given position](#Setting_the_gripper_to_the_given_position)
* 3.9[Functions related to the drag teach and trajectory reproduction](#Functions_related_to_the_drag_teach_and_trajectory_reproduction)
* 3.9.1[Set the force-position mixing control](#Set_the_force-position_mixing_control)
* 3.9.2[Stop the force-position mixing control](#Stop_the_force-position_mixing_control)
* 3.10[Functions related to the use of six-axis force sensors at the end - optional](#Functions_related_to_the_use_of_six-axis_force_sensors_at_the_end)
* 3.10.1[Clearing the six-axis force Data](#Clearing_the_six-axis_force_Data)
* 3.11[Functions related to the control of the five-finger dexterous hand - optional](#Functions_related_to_the_control_of_the_five-finger_dexterous_hand)
* 3.11.1[Setting the serial number of the dexterous hand posture](#Setting_the_serial_number_of_the_dexterous_hand_posture)
* 3.11.2[Set the dexterous hand action sequence number](#Set_the_dexterous_hand_action_sequence_number)
* 3.11.3[Setting the angles of various degrees of freedom for the dexterous hand](#Setting_the_angles_of_various_degrees_of_freedom_for_the_dexterous_hand)
* 3.11.4[Setting the dexterous hand speed](#Setting_the_dexterous_hand_speed)
* 3.11.5[Setting the force threshold of the dexterous hand](#Setting_the_force_threshold_of_the_dexterous_hand)
* 3.12[Lifting mechanism](#Lifting_mechanism)
* 3.12.1[Speed open-loop control of the lifting mechanism](#Speed_open-loop_control_of_the_lifting_mechanism)
* 3.12.2[Position closed-loop control of the lifting mechanism](#Position_closed-loop_control_of_the_lifting_mechanism)
* 3.12.3[Get the lifting mechanism state](#Get_the_lifting_mechanism_state)
* 3.13[Functions related to the transmissive force-position compensation Mode](#Functions_related_to_the_transmissive_force-position_compensation_Mode)
* 3.13.1[Starting the transmissive force-position mixing control compensation mode](#Starting_the_transmissive_force-position_mixing_control_compensation_mode)
* 3.13.2[Stopping the transmissive force-position mixing control compensation mode](#Stopping_the_transmissive_force-position_mixing_control_compensation_mode)
* 3.13.3[Transmissive force-position mixing control compensation - joint](#Transmissive_force-position_mixing_control_compensation-joint)
* 3.13.4[Transmissive force-position mixing control compensation - pose](#Transmissive_force-position_mixing_control_compensation-pose)
* 3.14[Robotic arm state active reporting](#Robotic_arm_state_active_reporting)
* 3.14.1[Setting UDP robotic arm state active reporting configuration](#Setting_UDP_robotic_arm_state_active_reporting_configuration)
* 3.14.2[Getting UDP robotic arm state active reporting configuration](#Getting_UDP_robotic_arm_state_active_reporting_configuration)
* 3.14.3[UDP robotic arm state active reporting](#UDP_robotic_arm_state_active_reporting)

 
## Introduction
RealMan provides ROS2 function packages based on API to help users control the robotic arm using ROS2. If you want to learn more about controlling the robotic arm, you can refer to the API documentation and instructions. In practical use, the user can establish communication with the robotic arm through the Ethernet port and control the robotic arm.
## Error_Description
### Controller_error_type
| Serial No. | Error code (hexadecimal) | Error content |
| :---: | :---- | :---: |
| 1 | 0x0000 | System is normal |
| 2 | 0x1001 | Joint communication is abnormal |
| 3 | 0x1002 | The target angle exceeds the limit |
| 4 | 0x1003 | This position is inaccessible and is a singular point |
| 5 | 0x1004 | Real-time kernel communication error |
| 6 | 0x1005 | Joint communication bus error |
| 7 | 0x1006 | Planning layer kernel error |
| 8 | 0x1007 | Joint Overspeed |
| 9 | 0x1008 | The end interface board cannot be connected |
| 10 | 0x1009 | Overspeed limit |
| 11 | 0x100A | Overacceleration limit |
| 12 | 0x100B | Joint brake is not opened |
| 13 | 0x100C | Overspeed during drag teach |
| 14 | 0x100D | Robotic arm collision |
| 15 | 0x100E | No work coordinate system is available |
| 16 | 0x100F | No tool coordinate system is available |
| 17 | 0x1010 | Joint failure enabling error |
### Joint_error_type

| Serial No. | Error code (hexadecimal) | Error content |
| :---: | :---- | :---: |
| 1 | 0x0000 | Joint is normal |
| 2 | 0x0001 | FOC error |
| 3 | 0x0002 | Overvoltage |
| 4 | 0x0004 | Undervoltage |
| 5 | 0x0008 | Overtemperature |
| 6 | 0x0010 | Start failed |
| 7 | 0x0020 | Encoder error |
| 8 | 0x0040 | Overcurrent |
| 9 | 0x0080 | Software error |
| 10 | 0x0100 | Temperature sensor error |
| 11 | 0x0200 | Position limit-out error |
| 12 | 0x0400 | Illegal joint ID |
| 13 | 0x0800 | Position tracking error |
| 14 | 0x1000 | Current detection error |
| 15 | 0x2000 | Brake opening failed |
| 16 | 0x4000 | Position command step warning |
| 17 | 0x8000 | Multi-coil joint's coil lost the number |
| 18 | 0xF000 | Communication frame loss |
### API_error_type

| Serial No. | Error code (hexadecimal) | Error content |
| :---: | :---- | :---: |
| 1 | 0x0000 | The system is running normally |
| 2 | 0x0001 | Message request returns FALSE |
| 3 | 0x0002 | The robotic arm is not initialized or the input model is illegal |
| 4 | 0x0003 | Illegal timeout |
| 5 | 0x0004 | Socket initialization failed |
| 6 | 0x0005 | Socket connection failed |
| 7 | 0x0006 | Socket sending failed |
| 8 | 0x0007 | Socket communication timeout |
| 9 | 0x0008 | Unknown error |
| 10 | 0x0009 | Incomplete data |
| 11 | 0x000A | Array length error |
| 12 | 0x000B | Data type error |
| 13 | 0x000C | Model error |
| 14 | 0x000D | Missing callback function |
| 15 | 0x000E | The robotic arm stopped abnormally |
| 16 | 0x000F | The trajectory file name is too long |
| 17 | 0x0010 | Trajectory file verification failed |
| 18 | 0x0011 | The trajectory file read failed |
| 19 | 0x0012 | The controller busy, please try again later |
| 20 | 0x0013 | Illegal input |
| 21 | 0x0014 | The data queue is full |
| 22 | 0x0015 | Calculation failed |
| 23 | 0x0016 | File opening failed |
| 24 | 0x0017 | Force control calibration manual stop |
| 25 | 0x0018 | No tracks to save |
## ROS_Function_Package_Robotic_Arm_Instructions
This section describes how to query and control the robotic arm through the topic of ROS.
### Joint_Configuration
#### Clear_the_joint's_error_code

| Function description | Clear_the_joint's_error_code |
| :---: | :---- |
| Parameter description | Jointerrclear.msg<br>uint8 joint_num：the corresponding joint number, from the base to the robotic arm gripper, the number is 1-6.<br>bool block：whether it is a blocking mode，bool type，true:blocking，false:non-blocking |
| Command example | ros2 topic pub /rm_driver/set_joint_err_clear_cmd rm_ros_interfaces/msg/Jointerrclear "joint_num: 1 <br>block: true" |
| Return value | true-set successfully，false-set failed |
| Return example | ros2 topic echo /rm_driver/set_joint_err_clear_result |

### Functions_related_to_the_work_coordinate_system_settings
#### Change_the_current_work_coordinate_system
| Function description | Change_the_current_work_coordinate_system |
| :---: | :---- |
| Parameter description | ROS msg std_msgs::msg::String |
| Command example | ros2 topic pub /rm_driver/change_work_frame_cmd std_msgs/msg/String "data: 'Base'" |
| Return value | true-set successfully，false-set failed |
| Return example | ros2 topic echo /rm_driver/change_work_frame_result |
### Coordinate_system_query
#### Get_the_current_tool_coordinate_system
| Function description | Get_the_current_tool_coordinate_system |
| :---: | :---- |
| Parameter description | ROS msg std_msgs::msg::Empty |
| Command example | ros2 topic pub --once /rm_driver/get_current_tool_frame_cmd std_msgs/msg/Empty "{}" |
| Return value | Current tool coordinate system name |
| Return example | ros2 topic echo /rm_driver/get_current_tool_frame_result |
#### Get_all_tool_coordinate_system_names
| Function description | Get_all_tool_coordinate_system_names |
| :---: | :---- |
| Parameter description | ROS msg std_msgs::msg::Empty |
| Command example | ros2 topic pub /rm_driver/get_all_tool_frame_cmd std_msgs/msg/Empty "{}" |
| Return value | All names of the current tool coordinate system |
| Return example | ros2 topic echo /rm_driver/get_all_tool_frame_result |
#### Get_the_current_work_coordinate_system
| Function description | Get_the_current_work_coordinate_system |
| :---: | :---- |
| Parameter description | ROS msg std_msgs::msg::Empty |
| Command example | ros2 topic pub --once /rm_driver/get_curr_workFrame_cmd std_msgs/msg/Empty "{}" |
| Return value | true-set successfully，false-set failed |
| Return example | ros2 topic echo /rm_driver/get_curr_workFrame_result |
#### Get_all_work_coordinate_system_names
| Function description | Get_all_work_coordinate_system_names |
| :---: | :---- |
| Parameter description | ROS msg std_msgs::msg::Empty |
| Command example | ros2 topic pub --once /rm_driver/get_all_work_frame_cmd std_msgs/msg/Empty "{}" |
| Return value | All work coordinate system names |
| Return example | ros2 topic echo /rm_driver/change_work_frame_result |
### Functions_related_to_the_arm_state_query
#### Get_the_current_state_of_the_robot_arm-return_each_joint_angle_and_Euler_angle
| Function description | Retrieve the current state of the robotic arm |
| :---: | :---- |
| Parameter description | ROS msg std_msgs::msg::Empty |
| Command example | ros2 topic pub --once /rm_driver/get_current_arm_state_cmd std_msgs/msg/Empty "{}" |
| Return value | The current robotic arm joint state (angle) + pose information (Euler angle) + error information |
| Return example | ros2 topic echo /rm_driver/get_current_arm_original_state_result |
#### Get_the_current_state_of_the_robotic_arm-return_each_joint_radians_and_quaternion
| Function description | Retrieve the current state of the robotic arm |
| :---: | :---- |
| Parameter description | ROS msg std_msgs::msg::Empty |
| Command example | ros2 topic pub --once /rm_driver/get_current_arm_state_cmd std_msgs/msg/Empty "{}" |
| Return value | The current robotic arm joint state (radians) + pose information (quaternion) + error information |
| Return example | ros2 topic echo /rm_driver/get_current_arm_state_result |
### Functions_related_to_motion_planning_of_the_robotic_arm
#### Joint_Space_Motion
| Function description | Joint space move MOVEJ |
| :---: | :---- |
| Parameter description | Movej.msg<br>float32[6] joint：joint angle, unit: radians.<br>uint8 speed：speed percentage ratio coefficient, 0~100.<br>bool block：whether it is a blocking mode，bool type，true:blocking，false:non-blocking. |
| Command example | 6-degree of freedom<br>ros2 topic pub --once /rm_driver/movej_cmd rm_ros_interfaces/msg/Movej "joint: [0, 0, 0, 0, 0, 0]<br>speed: 20<br>block: true <br>dof: 6"<br>7-degree of freedom<br>ros2 topic pub --once /rm_driver/movej_cmd rm_ros_interfaces/msg/Movej "joint: [0, 0, 0, 0, 0, 0, 0]<br>speed: 20<br>block: true<br>trajectory_connect: 0<br>dof: 7" |
| Return value | Successful return: true; failure returns: false, the driver terminal returns an error code. |
| Return example | ros2 topic echo /rm_driver/movej_result |
#### Linear_motion_in_Cartesian_space
| Function description | Linear motion in Cartesian space MOVEL |
| :---: | :---- |
| Parameter description | Movel.msg<br>geometry_msgs/Pose pose：robotic arm pose，geometry_msgs/Pose type，x, y, z coordinates (float type, unit: m) + quaternion.<br>uint8 speed：speed percentage ratio coefficient, 0~100。<br>bool : whether it is a blocking mode, bool type, true: blocking, false: non-blocking. |
| Command example | First, use MoveJP<br>ros2 topic pub --once /rm_driver/movej_p_cmd rm_ros_interfaces/msg/Movejp "pose:<br>  position:<br>    x: -0.317239<br>    y: 0.120903<br>    z: 0.255765<br>  orientation:<br>    x: -0.983404<br>    y: -0.178432<br>    z: 0.032271<br>    w: 0.006129<br>speed: 20<br>block: true"<br>Then use MoveL<br>ros2 topic pub --once /rm_driver/movel_cmd rm_ros_interfaces/msg/Movel "pose:<br>  position:<br>    x: -0.317239<br>    y: 0.120903<br>    z: 0.295765<br>  orientation:<br>    x: -0.983404<br>    y: -0.178432<br>    z: 0.032271<br>    w: 0.006129<br>speed: 20<br>trajectory_connect: 0<br>block: true" |
| Return value | Successful return: true; failure returns: false, the driver terminal returns an error code. |
| Return example | ros2 topic echo /rm_driver/movel_result |
#### Circular_motion_in_Cartesian_space
| Function description | Circular motion in Cartesian space MOVEC |
| :---: | :---- |
| Parameter description | Movec.msg<br>geometry_msgs/Pose pose_mid：middle pose，geometry_msgs/Pose type，x, y, z coordinates (float type, unit: m) + quaternion.<br>geometry_msgs/Pose pose_end：end pose，geometry_msgs/Posetype, x, y, z coordinates (float type, unit: m) + quaternion.<br>uint8 speed：speed percentage ratio coefficient, 0-100.<br>bool : whether it is a blocking mode, bool type, true: blocking, false: non-blocking. |
| Command example | First, use movej_p to reach the specified position<br>ros2 topic pub --once /rm_driver/movej_p_cmd rm_ros_interfaces/msg/Movejp "pose:<br>  position:<br>    x: 0.274946<br>    y: -0.058786<br>    z: 0.299028<br>  orientation:<br>    x: 0.7071<br>    y: -0.7071<br>    z: 0.0<br>w: 0.0<br>speed: 0<br>block: true"<br>Use movec to reach the specified position<br>ros2 topic pub --once /rm_driver/movec_cmd rm_ros_interfaces/msg/Movec "pose_mid:<br>  position:<br>    x: 0.324946<br>    y: -0.008786<br>    z: 0.299028<br>  orientation:<br>    x: 0.7071<br>    y: -0.7071<br>    z: 0.0<br>    w: 0.0<br>pose_end:<br>  position:<br>    x: 0.274946<br>    y: 0.041214<br>    z: 0.299028<br>  orientation:<br>    x: 0.7071<br>    y: -0.7071<br>    z: 0.0<br>    w: 0.0<br>speed: 20<br>trajectory_connect: 0<br>block: false<br>loop: 0" |
| Return value | Successful return: true; failure returns: false, the driver terminal returns an error code. |
| Return example | ros2 topic echo /rm_driver/movec_result |
#### Joint_angle_CANFD_transmission
| Function description | Joint angle CANFD transmission |
| :---: | :---- |
| Parameter description | Jointpos.msg<br>float32[6] joint：joint angle, unit: radians.<br>bool follow：follow state, true: high following, false: low following, default high following if not set.<br>float32 expand：expand joint, unit: radians. |
| Command example | Transmission needs to send multiple continuous points to achieve, simply by the following command and can not achieve the function, the current moveit2 control using angle transmission control mode.<br>ros2 topic pub /rm_driver/movej_canfd_cmd rm_ros_interfaces/msg/Jointpos "joint: [0, 0, 0, 0, 0, 0]<br>follow: false<br>expand: 0.0<br>dof: 6" |
| Return value | Success: no return value; Failure return: the driver terminal returns an error code. |
	
#### Pose_CANFD_transmission
| Function description | Pose CANFD transmission |
| :---: | :---- |
| Parameter description | Jointpos.msg<br>geometry_msgs/Pose pose: transmission pose, geometry_msgs/Pose type, x, y, z coordinates (float type, unit: m) + quaternion.<br>bool  follows: follow state, true: high following, false: low following, default high following if not set. |
| Command example | It needs to be a large number (10 or more) of continuous position points, simply by the following command and can not achieve the function, with more than a 2ms period continuous release.<br>ros2 topic pub /rm_driver/movep_canfd_cmd rm_ros_interfaces/msg/Cartepos "pose:<br>  position:<br>    x: 0.0<br>    y: 0.0<br>    z: 0.0<br>  orientation:<br>    x: 0.0<br>    y: 0.0<br>    z: 0.0<br>    w: 1.0<br>follow: false" |
| Return value | Success: no return value; Failure return: the driver terminal returns an error code. |
#### Joint_space_planning_to_target_pose
| Function description | Joint space planning to target pose MOVEJP |
| :---: | :---- |
| Parameter description | Movejp.msg<br>geometry_msgs/Pose target pose, x, y, z coordinates (float type, unit: m) + quaternion.<br>uint8 speed：speed percentage ratio coefficient, 0-100.<br>bool block：whether it is a blocking mode, bool type, true: blocking, false: non-blocking. |
| Command example | ros2 topic pub --once /rm_driver/movej_p_cmd rm_ros_interfaces/msg/Movejp "pose:<br>  position:<br>    x: -0.317239<br>    y: 0.120903<br>    z: 0.255765<br>  orientation:<br>    x: -0.983404<br>    y: -0.178432<br>    z: 0.032271<br>    w: 0.006129<br>speed: 20<br>block: true" |
| Return value | Successful return: true; failure returns: false, the driver terminal returns an error code. |
| Return example | ros2 topic echo /rm_driver/movej_p_result |
#### Trajectory_emergency_stop
| Function description | Motion planning trajectory emergency stop |
| :---: | :---- |
| Parameter description | ROS msg std_msgs::msg::Bool<br>bool data：whether the trajectory is emergency stop, true: emergency stop, false: not emergency stop. |
| Command example | ros2 topic pub /rm_driver/move_stop_cmd std_msgs/msg/Bool "data: true" |
| Return value | Successful return: true; failure returns: false, the driver terminal returns an error code. |
| Return example | ros2 topic echo /rm_driver/move_stop_result |
### Functions_related_to_controller_configuration
#### Get_the_controller's_version
| Function description | Get the controller's version |
| :---: | :---- |
| Parameter description | Armsoftversion.msg<br>string plan version: the read user interface kernel version number.<br>string ctrlversion: real-time kernel version number.<br>string kernal1: the version number of sub-core 1 of the real-time kernel.<br>string kernal2: the version number of sub-core 2 of the real-time kernel.<br>string product version: robotic arm model. |
| Command example | ros2 topic pub /rm_driver/get_arm_software_version_cmd std_msgs/msg/Empty "{}" |
| Return value | Successful return: version information; Failure return: the driver terminal returns an error code. |
| Return example | ros2 topic echo /rm_driver/get_arm_software_version_result |
### Functions_related_to_the_IO_configuration_of_the_end_tool
#### Setting_the_tool_voltage_output
| Function description | Setting the tool voltage output |
| :---: | :---- |
| Parameter description | ROS msg：std_msgs::msg::UInt16<br>uint16 data：power output type, range:0~3   0-0V，1-5V，2-12V，3-24V |
| Command example | ros2 topic pub --once /rm_driver/set_tool_voltage_cmd std_msgs/msg/UInt16 "data: 0" |
| Return value | Successful return: true; failure returns: false, the driver terminal returns an error code. |
| Return example | ros2 topic echo /rm_driver/set_tool_voltage_result |
### Functions_related_to_the_control_of_the_end_gripper
The RealMan robotic arm is equipped with an Inspire Robots EG2-4C2 gripper. The robotic arm controller has made the gripper's ROS control mode available to the user to facilitate user operation.
#### Setting_the_Gripper_Pick
| Function description | Setting the gripper pick |
| :---: | :---- |
| Parameter description | Gripperpick.msg<br>uint16 speed：1～1000,representing the opening and closing speed of the gripper, dimensionless.<br>uint16 force:representing the gripping force of the gripper, maximum 1.5 kg.<br>bool block: whether it is a blocking mode, bool type, true: blocking, false: non-blocking. |
| Command example | ros2 topic pub --once /rm_driver/set_gripper_pick_cmd rm_ros_interfaces/msg/Gripperpick "speed: 200<br>force: 200<br>block: true" |
| Return value | Successful return: true; failure returns: false, the driver terminal returns an error code. |
| Return example | ros2 topic echo /rm_driver/set_gripper_pick_result |
#### Setting_the_gripper_pick-on
| Function description | Setting the gripper pick-on |
| :---: | :---- |
| Parameter description | Gripperpick.msg<br>uint16 speed：1～1000, representing the opening and closing speed of the gripper, dimensionless.<br>uint16 force：1～1000,representing the gripping force of the gripper, maximum 1.5 kg.<br>bool block: whether it is a blocking mode, bool type, true: blocking, false: non-blocking. |
| Command example | ros2 topic pub --once /rm_driver/set_gripper_pick_on_cmd rm_ros_interfaces/msg/Gripperpick "speed: 200<br>force: 200<br>block: true" |
| Return value | Successful return: true; failure returns: false, the driver terminal returns an error code. |
| Return example | ros2 topic echo /rm_driver/set_gripper_pick_on_result |
#### Setting_the_gripper_to_the_given_position
| Function description | Setting the gripper to the given position |
| :---: | :---- |
| Parameter description | Gripperset.msg<br>uint16 position：target position of the gripper, range: 1-1000, representing the opening degree of the gripper: 0-70 mm.<br>bool block: whether it is a blocking mode, bool type, true: blocking, false: non-blocking. |
| Command example | ros2 topic pub --once /rm_driver/set_gripper_position_cmd rm_ros_interfaces/msg/Gripperset "position: 500<br>block: true" |
| Return value | Successful return: true; failure returns: false, the driver terminal returns an error code. |
| Return example | ros2 topic echo /rm_driver/set_gripper_position_result |
### Functions_related_to_the_drag_teach_and_trajectory_reproduction
#### Set_the_force-position_mixing_control
| Function description | Set the force-position mixing control |
| :---: | :---- |
| Parameter description | Setforceposition.msg<br>uint8 sensor: 0 - One-axis force; 1 - Six-axis force<br>uint8 mode: 0 - Base coordinate system force control; 1 - Tool coordinate system force control<br>uint8 direction:Force control direction; 0 - Along the X-axis; 1 - Along the Y-axis; 2 - Along the Z-axis; 3 - Along the RX posture direction; 4 - Along the RY posture direction; 5 - Along the RZ posture direction<br>int16 n: The value of force, unit: N, accuracy: 0.1N<br>bool block: whether it is a blocking mode, bool type, true: blocking, false: non-blocking. |
| Command example | ros2 topic pub --once /rm_driver/set_force_postion_cmd rm_ros_interfaces/msg/Setforceposition "sensor: 1<br>mode: 0<br>direction: 2<br>n: 3<br>block: false" |
| Return value | Successful return: true; failure returns: false, the driver terminal returns an error code. |
| Return example | ros2 topic echo /rm_driver/set_force_postion_result |
#### Stop_the_force-position_mixing_control
| Function description | Stop the force-position mixing control |
| :---: | :---- |
| Parameter description | std_msgs::msg::Bool<br>bool data: whether it is a blocking mode, bool type, true: blocking, false: non-blocking. |
| Command example | ros2 topic pub /rm_driver/stop_force_postion_cmd std_msgs/msg/Bool "data: true" |
| Return value | Successful return: true; failure returns: false, the driver terminal returns an error code. |
| Return example | ros2 topic echo /rm_driver/clear_force_data_result |
### Functions_related_to_the_use_of_six-axis_force_sensors_at_the_end
The RealMan RM-65F robotic arm has an integrated six-axis force sensor at the end without external wiring. Users can operate the six-axis force through ROS topics.
#### Clearing_the_six-axis_force_Data
| Function description | Clearing the six-axis force data |
| :---: | :---- |
| Parameter description | std_msgs::msg::Bool<br>bool data: whether it is a blocking mode, bool type, true: blocking, false: non-blocking. |
| Command example | ros2 topic pub /rm_driver/clear_force_data_cmd std_msgs/msg/Bool "data: true" |
| Return value | Successful return: true; failure returns: false, the driver terminal returns an error code. |
| Return example | ros2 topic echo /rm_driver/clear_force_data_result |
### Functions_related_to_the_control_of_the_five-finger_dexterous_hand
The RealMan RM-65 robotic arm has been equipped with a five-finger dexterous hand at the end. Users can set the hand through the ROS.
#### Setting_the_serial_number_of_the_dexterous_hand_posture
| Function description | Setting the serial number of the dexterous hand posture |
| :---: | :---- |
| Parameter description | Handposture.msg<br>uint16 posture_num：the serial number of the posture pre-saved in the dexterous hand, ranging from 1 to 40.<br>bool data: whether it is a blocking mode, bool type, true: blocking, false: non-blocking. |
| Command example | ros2 topic pub --once /rm_driver/set_hand_posture_cmd rm_ros_interfaces/msg/Handposture "posture_num: 1<br>block: true" |
| Return value | Successful return: true; failure returns: false, the driver terminal returns an error code. |
| Return example | ros2 topic echo /rm_driver/set_hand_posture_result |
#### Set_the_dexterous_hand_action_sequence_number
| Function description | Set the dexterous hand action sequence number |
| :---: | :---- |
| Parameter description | Handseq.msg<br>uint16 seq_num：the serial number of the action sequence pre-saved in the dexterous hand, ranging from 1 to 40.<br>bool data: whether it is a blocking mode, bool type, true: blocking, false: non-blocking. |
| Command example | ros2 topic pub --once /rm_driver/set_hand_seq_cmd rm_ros_interfaces/msg/Handseq "seq_num: 1
block: true" |
| Return value | Successful return: true; failure returns: false, the driver terminal returns an error code. |
| Return example | ros2 topic echo /rm_driver/set_hand_seq_result |
#### Setting_the_angles_of_various_degrees_of_freedom_for_the_dexterous_hand
| Function description | Setting the angles of various degrees of freedom for the dexterous hand |
| :---: | :---- |
| Parameter description | Handangle.msg<br>int16[6] hand_angle：hand angle array, the range is 0 to 1000, and -1 represents that no operation is performed on this degree of freedom and the current state remains.<br>bool data: whether it is a blocking mode, bool type, true: blocking, false: non-blocking. |
| Command example | ros2 topic pub --once /rm_driver/set_hand_angle_cmd rm_ros_interfaces/msg/Handangle "hand_angle:<br>- 0<br>- 0<br>- 0<br>- 0<br>- 0<br>- 0<br>block: true" |
| Return value | Successful return: true; failure returns: false, the driver terminal returns an error code. |
| Return example | ros2 topic echo /rm_driver/set_hand_angle_result |
#### Setting_the_dexterous_hand_speed
| Function description | Setting the dexterous hand speed |
| :---: | :---- |
| Parameter description | Handspeed.msg<br>uint16 hand_speed：hand speed, range: 1-1000.<br>bool data: whether it is a blocking mode, bool type, true: blocking, false: non-blocking. |
| Command example | ros2 topic pub --once /rm_driver/set_hand_speed_cmd rm_ros_interfaces/msg/Handspeed "hand_speed: 200<br>block: true" |
| Return value | Successful return: true; failure returns: false, the driver terminal returns an error code. |
| Return example | ros2 topic echo /rm_driver/set_hand_speed_result |
#### Setting_the_force_threshold_of_the_dexterous_hand
| Function description | Setting the force threshold of the dexterous hand |
| :---: | :---- |
| Parameter description | Handforce.msg<br>uint16 hand_force：hand force, range: 1-1000.<br>bool data: whether it is a blocking mode, bool type, true: blocking, false: non-blocking. |
| Command example | ros2 topic pub --once /rm_driver/set_hand_force_cmd rm_ros_interfaces/msg/Handforce "hand_force: 200<br>block: true" |
| Return value | Successful return: true; failure returns: false, the driver terminal returns an error code. |
| Return example | ros2 topic echo /rm_driver/set_hand_force_result |
### Lifting_mechanism
The RealMan robotic arm can be integrated with the self-developed lifting mechanism.
#### Speed_open-loop_control_of_the_lifting_mechanism
| Function description | Speed open-loop control of the lifting mechanism |
| :---: | :---- |
| Parameter description | Liftspeed.msg<br>int16 speed：speed percentage, -100-100, Speed < 0: the lifting mechanism moves downward, Speed > 0: the lifting mechanism moves upward, Speed = 0: the lifting mechanism stops.<br>bool data: whether it is a blocking mode, bool type, true: blocking, false: non-blocking. |
| Command example | ros2 topic pub /rm_driver/set_lift_speed_cmd rm_ros_interfaces/msg/Liftspeed "speed: 100" |
| Return value | Successful return: true; failure returns: false, the driver terminal returns an error code. |
| Return example | ros2 topic echo /rm_driver/set_lift_speed_result |
#### Position_closed-loop_control_of_the_lifting_mechanism
| Function description | Position closed-loop control of the lifting mechanism |
| :---: | :---- |
| Parameter description | Liftheight.msg<br>uint16 height： target height, unit: mm, range: 0-2600.<br>uint16 speed：speed percentage, 1-100.<br>bool data: whether it is a blocking mode, bool type, true: blocking, false: non-blocking. |
| Command example | ros2 topic pub --once /rm_driver/set_lift_speed_cmd rm_ros_interfaces/msg/Liftspeed "speed: 100" |
| Return value | Successful return: true; failure returns: false, the driver terminal returns an error code. |
| Return example | ros2 topic echo /rm_driver/set_lift_height_result |
#### Get_the_lifting_mechanism_state
| Function description | Get the lifting mechanism state |
| :---: | :---- |
| Parameter description | Liftstate.msg<br>int16 height：current height.<br>int16 current：current current.<br>uint16 err_flag：drive error code. |
| Command example | ros2 topic pub /rm_driver/get_lift_state_cmd std_msgs/msg/Empty "{}" |
| Return value | Successful return: current state of the lifting mechanism; Failure return: the driver terminal returns an error code. |
| Return example | ros2 topic echo /rm_driver/get_lift_state_result |
### Functions_related_to_the_transmissive_force-position_compensation_Mode
For the RealMan robotic arm with one-axis force and six-axis force versions, the user can not only directly use the teaching device to call the underlying force-position mixing control module but also combine the custom trajectory with the underlying force-position mixing control algorithm in the form of periodic transmission to compensate.
If force data calibration has not been completed before the force operations, the zero position can be calibrated using the one-axis force and six-axis force data clear interfaces.
#### Starting_the_transmissive_force-position_mixing_control_compensation_mode
| Function description | starting the transmissive force-position mixing control compensation mode |
| :---: | :---- |
| Parameter description | std_msgs::msg::Empty |
| Command example | ros2 topic pub /rm_driver/start_force_position_move_cmd std_msgs/msg/Empty "{}" |
| Return value | Successful return: true; failure returns: false, the driver terminal returns an error code. |
| Return example | ros2 topic echo /rm_driver/start_force_position_move_result |
#### Stopping_the_transmissive_force-position_mixing_control_compensation_mode
| Function description | Stopping the transmissive force-position mixing control compensation mode |
| :---: | :---- |
| Parameter description | std_msgs::msg::Empty |
| Command example | ros2 topic pub /rm_driver/stop_force_position_move_cmd std_msgs/msg/Empty "{}" |
| Return value | Successful return: true; failure returns: false, the driver terminal returns an error code. |
| Return example | ros2 topic echo /rm_driver/stop_force_position_move_result |
#### Transmissive_force-position_mixing_control_compensation-joint
| Function description | Transmissive force-position mixing control compensation (joint) |
| :---: | :---- |
| Parameter description | Forcepositionmovejoint.msg<br>float32[6] joint： target joint radian<br>uint8 sensor：type of sensor used, 0 - one-axis force, 1 - six-axis force<br>uint8 mode, 0 - along the base coordinate system, 1 - along the tool coordinate system.<br>int16 dir：force control direction, 0-5 represent X/Y/Z/Rx/Ry/Rz respectively, where the default direction is Z direction for one-axis force type<br>float32 force：force value, unit: 0.1 N.<br>bool follow：whether high follow, true: high follow, false: low follow.<br>uint8 dof:degree of freedom of the robotic arm |
| Command example | It needs to be a large number (10 or more) of continuous position points, with more than 2ms period continuous release.<br>ros2 topic pub /rm_driver/force_position_move_joint_cmd rm_ros_interfaces/msg/Forcepositionmovejoint " joint: [0, 0, 0, 0, 0, 0]<br>sensor: 0<br>mode: 0<br>dir: 0<br>force: 0.0<br>follow: false<br>dof: 6 |
| Return value | Success: no return; Failure return: false, and the driver terminal returns an error code. |
#### Transmissive_force-position_mixing_control_compensation-pose
| Function description | Transmissive force-position mixing control compensation (pose) |
| :---: | :---- |
| Parameter description | Forcepositionmovepose.msg<br>geometry_msgs/Pose pose：target pose, x, y, z coordinates (float type, unit: m) + quaternion.<br>uint8 sensor： type of sensor used, 0 - one-axis force, 1 - six-axis force.<br>uint8 mode：mode, 0 - along the base coordinate system, 1 - along the tool coordinate system<br>int16 dir：force control direction, 0-5 represent X/Y/Z/Rx/Ry/Rz respectively, where the default direction is Z direction for one-axis force type.<br>float32 force：force value,unit:0.1 N.<br>bool follow：whether high follow, true: high follow, false: low follow. |
| Command example | It needs to be a large number (10 or more) of continuous position points, with more than 2ms period continuous release.<br>ros2 topic pub /rm_driver/force_position_move_pose_cmd rm_ros_interfaces/msg/Forcepositionmovepose "pose:<br>  position:<br>    x: 0.0<br>    y: 0.0<br>    z: 0.0<br>  orientation:<br>    x: 0.0<br>    y: 0.0<br>    z: 0.0<br>    w: 1.0<br>sensor: 0<br>mode: 0<br>dir: 0<br>force: 0<br>follow: false" |
| Return value | Success: no return; Failure return: false, and the driver terminal returns an error code.
### Robotic_arm_state_active_reporting
#### Setting_UDP_robotic_arm_state_active_reporting_configuration
| Function description | Set UDP robotic arm state active reporting configuration |
| :---: | :---- |
| Parameter description | Setrealtimepush.msg<br>uint16 cycle：set the broadcast cycle, which is a multiple of 5ms (default 1 i.e. 1 * 5 = 5 ms, 200 Hz).<br>uint16 port：set the broadcast port number (default 8089).<br>uint16 force_coordinate：set the coordinate system of force data outside the system (only supported by the arm with force sensors).<br>string ip：set the custom reporting target IP address (default 192.168.1.10). |
| Command example | ros2 topic pub --once /rm_driver/set_realtime_push_cmd rm_ros_interfaces/msg/Setrealtimepush "cycle: 1<br>port: 8089<br>force_coordinate: 0<br>ip: '192.168.1.10'" |
| Return value | Successful return: true; failure returns: false, the driver terminal returns an error code. |
| Return example | ros2 topic echo /rm_driver/set_realtime_push_result |
#### Getting_UDP_robotic_arm_state_active_reporting_configuration
| Function description | Get UDP robotic arm state active reporting configuration |
| :---: | :---- |
| Parameter description | Setrealtimepush.msg<br>uint16 cycle：set the broadcast cycle, which is a multiple of 5ms (default 1 i.e. 1 * 5 = 5 ms, 200 Hz).<br>uint16 port：set the broadcast port number (default 8089).<br>uint16 force_coordinate：force_coordinate: set the coordinate system of force data outside the system (only supported by the arm with force sensors).<br>string ip：set the custom reporting target IP address (default 192.168.1.10). |
| Command example | ros2 topic pub --once /rm_driver/get_realtime_push_cmd std_msgs/msg/Empty "{}" |
| Return value | Successfully set information; Failure return: the driver terminal returns an error code. |
| Return example | ros2 topic echo /rm_driver/get_realtime_push_result |
#### UDP_robotic_arm_state_active_reporting

* Six-axis force

| Function description | Six-axis force |
| :---: | :---- |
| Parameter description | Sixforce.msg<br>float32 force_fx：the force along the x-axis direction.<br>float32 force_fy：the force along the y-axis direction.<br>float32 force_fz：the force along the z-axis direction.<br>float32 force_mx：the force when rotating along the x-axis direction.<br>float32 force_my：the force when rotating along the y-axis direction.<br>float32 force_mz：the force when rotating along the z-axis direction. |
| Subscription command | ros2 topic echo /rm_driver/udp_six_force |

* One-axis force

| Function description | One-axis force |
| :---: | :---- |
| Parameter description | Sixforce.msg<br>float32 force_fx：the force along the x-axis direction.<br>float32 force_fy：the force along the y-axis direction.<br>float32 force_fz：the force along the z-axis direction.(only this value is valid)<br>float32 force_mx：the force when rotating along the x-axis direction.<br>float32 force_my：the force when rotating along the y-axis direction.<br>float32 force_mz：the force when rotating along the z-axis direction. |
| Subscription command | ros2 topic echo /rm_driver/udp_one_force |

* Robotic arm error

| Function description | Robotic arm error |
| :---: | :---- |
| Parameter description | std_msgs::msg::UInt16<br>uint16 data：the robotic arm error message. |
| Subscription command | ros2 topic echo /rm_driver/udp_arm_err |

* System error

| Function description | System error |
| :---: | :---- |
| Parameter description | std_msgs::msg::UInt16<br>uint16 data：the system error message. |
| Subscription command | ros2 topic echo /rm_driver/udp_sys_err |

* Joint error

| Function description | Joint error |
| :---: | :---- |
| Parameter description | Jointerrorcode.msg<br>uint16[] joint_error：the error message for each joint.<br>Uint8 dof： the arm degree of freedom message. |
| Subscription command | ros2 topic echo /rm_driver/udp_joint_error_code |

* The robot arm radians data

| Function description | The robot arm radians data |
| :---: | :---- |
| Parameter description | sensor_msgs::msg::JointState<br>	builtin_interfaces/Time stamp<br>		int32 sec：time message, unit: second.<br>uint32 nanosec：time message, unit: nanosecond.<br>string frame_id：coordinate system name.<br>string[] name：joint name.<br>float64[] position：joint radian message.<br>float64[] velocity： joint speed message. (not used yet)<br>float64[] effort： joint force message. (not used yet) |
| Subscription command | ros2 topic echo /joint_states |

* Pose information

| Function description | Pose information |
| :---: | :---- |
| Parameter description | geometry_msgs::msg::Pose<br>Point position：the robotic arm current coordinate information.<br>	float64 x<br>	float64 y<br>	float64 z<br>Quaternion orientation：the robotic arm current pose  information.<br>	float64 x 0<br>	float64 y 0<br>	float64 z 0<br>	float64 w 1 |
| Subscription command | ros2 topic echo /rm_driver/udp_arm_position |

* Current external force data of the six-axis force sensor system

| Function description | Current external force data of the six-axis force sensor system |
| :---: | :---- |
| Parameter description | Sixforce.msg<br>float32 force_fx：the force forced on the current sensor along the x-axis direction.<br>float32 force_fy：the force forced on the current sensor along the y-axis direction.<br>float32 force_fz：the force forced on the current sensor along the z-axis direction.<br>float32 force_mx：the force forced on the current sensor when rotating along the x-axis direction.<br>float32 force_my：the force forced on the current sensor when rotating along the y-axis direction.<br>float32 force_mz：the force forced on the current sensor when rotating along the z-axis direction. |
| Subscription command | ros2 topic echo /rm_driver/udp_six_zero_force |

* Current external force data of the one-axis force sensor system

| Function description | Current external force data of the one-axis force sensor system |
| :---: | :---- |
| Parameter description | Sixforce.msg<br>float32 force_fx：the force forced on the current sensor along the x-axis direction.<br>float32 force_fy：the force forced on the current sensor along the y-axis direction.<br>float32 force_fz：the force forced on the current sensor along the z-axis direction. (only this data is valid)<br>float32 force_mx：the force forced on the current sensor when rotating along the x-axis direction.<br>float32 force_my：the force forced on the current sensor when rotating along the y-axis direction.<br>float32 force_mz：the force forced on the current sensor when rotating along the z-axis direction. |
| Subscription command | ros2 topic echo /rm_driver/udp_one_zero_force |

* Reference coordinate system for external force data of the system

| Function description | Reference coordinate system for external force data of the system |
| :----: | :---- |
| Parameter description | std_msgs::msg::UInt16<br>uint16 data：: coordinate system for external force data of the system, where 0 is the sensor coordinate system, 1 is the current work coordinate system, and 2 is the current tool coordinate system This data affects the reference coordinate system for external force data of one-axis and six axis force sensor systems. |
| Subscription command | ros2 topic echo /rm_driver/udp_arm_coordinate |

