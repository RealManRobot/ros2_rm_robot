<div align="right">
  
[中文简体](https://github.com/RealManRobot/ros2_rm_robot/blob/humble/rm_ros_interfaces/README_CN.md)|
[English](https://github.com/RealManRobot/ros2_rm_robot/blob/humble/rm_ros_interfaces/README.md)

</div>

<div align="center">

# RealMan Robot rm_ros_interface User Manual V1.2


 


RealMan Intelligent Technology (Beijing) Co., Ltd.
 
Revision History-

| No. | Date| Comment |
| -----| -----| -----|
|V1.0 | 2-18-2024 | Draft |
|V1.1 | 7-8-2024  | Amend(Add teaching message) |
|V1.2 | 12-25-2024  | Amend(Add UDP report message) |
|V1.3 | 04-07-2025  | Amend(API2 Adaptation) |


</div>

## 目录
* 1[rm_ros_interface Package Description](#rm_ros_interface_Package_Description)
* 2[rm_ros_interface Package Use](#rm_ros_interface_Package_Use)
* 3[rm_ros_interface Package Architecture Description](#rm_ros_interface_Package_Architecture_Description)
* 3.1[Overview of Package Files](#Overview_of_Package_Files)
* 4[rm_ros_interface message description](#rm_ros_interface_message_description)
* 4.1[Joint error code-Jointerrorcode_msg](#Joint_error_code-Jointerrorcode_msg)
* 4.2[Clearing the joint's error code-Jointerrclear_msg](#Clearing_the_joint's_error_code-Jointerrclear_msg)
* 4.3[All coordinate system names-Getallframe_msg](#All_coordinate_system_names-Getallframe_msg)
* 4.4[Joine motion-Movej_msg](#Joine_motion-Movej_msg)
* 4.5[Linear motion-Movel_msg](#Linear_motion-Movel_msg)
* 4.6[Circular motion-Movec_msg](#Circular_motion-Movec_msg)
* 4.7[Joint space planning to target pose-Movejp_msg](#Joint_space_planning_to_target_pose-Movejp_msg)
* 4.8[Joint teaching-Jointteach_msg-Jointteach_msg](#Joint_teaching-Jointteach_msg)
* 4.9[Position teaching-Posteach_msg](#Position_teaching-Posteach_msg)
* 4.10[Attitude teaching-Ortteach_msg](#Attitude_teaching-Ortteach_msg)
* 4.11[Joint transmission-Jointpos_msg](#Joint_transmission-Jointpos_msg)
* 4.12[Pose transmission-Cartepos_msg](#Pose_transmission-Cartepos_msg)
* 4.13[Current robotic arm state Angle and Euler angle-Armoriginalstate_msg](#Current_robotic_arm_state_Angle_and_Euler_angle-Armoriginalstate_msg)
* 4.14[Current arm state radians and quaternion-Armstate_msg](#Current_arm_state_radians_and_quaternion-Armstate_msg)
* 4.15[Gripper's pick-Gripperpick_msg](#Gripper's_pick-Gripperpick_msg)
* 4.16[Gripper's pick gripper's pick-on-Gripperpick_msg](#Gripper's_pick_gripper's_pick-on-Gripperpick_msg)
* 4.17[Gripper reaching the given position-Gripperset_msg](#Gripper_reaching_the_given_position-Gripperset_msg)
* 4.18[Force-position mixing control-Setforceposition_msg](#Force-position_mixing_control-Setforceposition_msg)
* 4.19[Six-axis force data-Sixforce_msg](#Six-axis_force_data-Sixforce_msg)
* 4.20[Setting the dexterous hand posture-Handposture_msg](#Setting_the_dexterous_hand_posture-Handposture_msg)
* 4.21[Setting the dexterous hand action sequence-Handseq_msg](#Seting_the_dexterous_hand_action_sequence-Handseq_msg)
* 4.22[Setting the angles of various degrees of freedom for the dexterous hand-Handangle_msg](#Setting_the_angles_of_various_degrees_of_freedom_for_the_dexterous_hand-Handangle_msg)
* 4.23[Setting the dexterous hand action sequence-Handspeed_msg](#Setting_the_dexterous_hand_action_sequence-Handspeed_msg)
* 4.24[Setting the force threshold for the dexterous hand-Handforce_msg](#Setting_the_force_threshold_for_the_dexterous_hand-Handforce_msg)
* 4.25[Transmissive force-position mixing control compensation-angle-Forcepositionmovejoint_msg](#Transmissive_force-position_mixing_control_compensation-angle-Forcepositionmovejoint_msg)
* 4.26[Transmissive force-position mixing control compensation-pose-Forcepositionmovejoint_msg](#Transmissive_force-position_mixing_control_compensation-pose-Forcepositionmovejoint_msg)
* 4.27[Speed open loop control-lifting mechanism-Liftspeed_msg](#Speed_open_loop_control-lifting_mechanism-Liftspeed_msg)
* 4.28[Position closed-loop control-lifting mechanism-Lift height_msg](#Position_closed-loop_control-lifting_mechanism-Lift_height_msg)
* 4.29[Getting the state of the lifting mechanism-Liftstate_msg](#Getting_the_state_of_the_lifting_mechanism-Liftstate_msg)
* 4.30[Getting or setting UDP active reporting configuration-Setrealtimepush_msg](#Getting_or_setting_UDP_active_reporting_configuration-Setrealtimepush_msg)
* 4.31[UDP manipulator status report Armcurrentstatus_msg](#UDP_manipulator_status_report-Armcurrentstatus_msg)
* 4.32[UDP joint current report Jointcurrent_msg](#UDP_joint_current_report-Jointcurrent_msg)
* 4.33[UDP joint enabling status report Jointenflag_msg](#UDP_joint_enabling_status_report-Jointenflag_msg)
* 4.34[UDP manipulator Euler's angular pose is reported to Jointposeeuler_msg](#UDP_manipulator_Eulers_angular_pose_is_reported_to-Jointposeeuler_msg)
* 4.35[UDP joint speed report Jointspeed_msg](#UDP_joint_speed_report_Jointspeed_msg)
* 4.36[UDP joint temperature report Jointtemperature_msg](#UDP_joint_temperature_report_Jointtemperature_msg)
* 4.37[UDP joint voltage report Jointvoltage_msg](#UDP_joint_voltage_report_Jointvoltage_msg)
* 4.38[System error code_Rmerr_msg](#System_error_code_Rmerr_msg)
* 4.39[Basic information of the end-effector device_Rmplusbase_msg](#Basic_information_of_the_end_effector_device_Rmplusbase_msg)
* 4.40[Real time information of the end_effector device-Rmplusstate_msg](#Real_time_information_of_the_end_effector_deviceRmplusstate_msg)
* 4.41[Customize high following mode joint transmission-Jointposcustom_msg](#Customize_high_following_mode_joint_transmission_Jointposcustom_msg)
* 4.42[Customize high following mode pose transmission-Carteposcustom_msg](#Customize_high_following_mode_pose_transmission_Carteposcustom_msg)

## rm_ros_interface_Package_Description
The main function of the rm_ros_interface package is to provide necessary message files for the robotic arm to run under the framework of ROS2. In the following text, we will provide a detailed introduction to this package through the following aspects.
* 1.Package use.  
* 2.Package architecture description.  
* 3.Package topic description.  
Through the introduction of the three parts, it can help you-  
* 1.Understand the package use.  
* 2.Familiar with the file structure and function of the package.  
* 3.Familiar with the topic related to the package for easy development and use.  
## rm_ros_interface_Package_Use
This package does not have any executable commands, but it is used to provide the necessary message files for other packages.
## rm_ros_interface_Package_Architecture_Description
### Overview_of_Package_Files
```
├── CMakeLists.txt                # compilation rule file
├── include                       # dependency header file folder
│   └── rm_ros_interfaces
├── msg                          # current message file (see below for details)
│   ├── Armcurrentstatus.msg
│   ├── Armoriginalstate.msg
│   ├── Armstate.msg
│   ├── Cartepos.msg
│   ├── Carteposcustom.msg
│   ├── Forcepositionmovejoint.msg
│   ├── Forcepositionmovepose.msg
│   ├── Force_Position_State.msg
│   ├── Getallframe.msg
│   ├── GetArmState_Command.msg
│   ├── Gripperpick.msg
│   ├── Gripperset.msg
│   ├── Handangle.msg
│   ├── Handforce.msg
│   ├── Handposture.msg
│   ├── Handseq.msg
│   ├── Handspeed.msg
│   ├── Handstatus.msg
│   ├── Jointcurrent.msg
│   ├── Jointenflag.msg
│   ├── Jointerrclear.msg
│   ├── Jointerrorcode.msg
│   ├── Jointposeeuler.msg
│   ├── Jointpos.msg
│   ├── Jointposcustom.msg
│   ├── Jointspeed.msg
│   ├── Jointteach.msg
│   ├── Jointtemperature.msg
│   ├── Jointvoltage.msg
│   ├── Liftheight.msg
│   ├── Liftspeed.msg
│   ├── Liftstate.msg
│   ├── Movec.msg
│   ├── Movej.msg
│   ├── Movejp.msg
│   ├── Movel.msg
│   ├── Ortteach.msg
│   ├── Posteach.msg
│   ├── Setforceposition.msg
│   ├── Setrealtimepush.msg
│   ├── Sixforce.msg
│   └── Stop.msg
├── package.xml                                      # dependency declaration file
└── src
```
## rm_ros_interface_message_description
### Joint_error_code-Jointerrorcode_msg
```
uint16[] joint_error  
uint8 dof  
```
__msg member__  
__uint16[] joint_error__  
Error message for each joint.  
__uint8 dof__  
Degree of freedom message of the robotic arm.  
### Clearing_the_joint's_error_code-Jointerrclear_msg
```
uint8 joint_num    
```
__msg member__  
__joint_num__  
the corresponding joint number, from the base to the robotic arm gripper, the number is 1-6 or 1-7.    
### All_coordinate_system_names-Getallframe_msg
```
string[10] frame_name  
```
__msg member__  
__frame_name__  
The array of work coordinate system names returned  
### Joine_motion-Movej_msg
```
float32[] joint  
uint8 speed  
bool block  
uint8 trajectory_connect
uint8 dof  
```
__msg member__  
__joint__  
Joint angle, float type, unit-radians.  
__speed__  
Speed percentage ratio coefficient, 0-100.  
__trajectory_connect__
Is the trajectory plan now. 1.wait 0.plan now.
__block__  
whether it is a blocking mode, bool type, true-blocking, false-non-blocking.  
__dof__  
Degree of freedom message of the robotic arm.  
### Linear_motion-Movel_msg
```
geometry_msgs/Pose pose  
uint8 speed  
uint8 trajectory_connect  
bool block  
```
__msg member__  
__pose__  
Robotic arm pose-geometry_msgs/Pose type, x, y, z coordinates (float type, unit-m) + quaternion (float type).  
__speed__  
Speed percentage ratio coefficient, 0-100.  
__trajectory_connect__
Is the trajectory plan now. 1.wait 0.plan now.
__block__  
whether it is a blocking mode, bool type, true-blocking, false-non-blocking.  
### Circular_motion-Movec_msg
```
geometry_msgs/Pose pose_mid  
geometry_msgs/Pose pose_end  
uint8 speed  
uint8 trajectory_connect  
bool block  
uint8 loop
```
__msg member__  
__pose_mid__  
Middle pose: geometry_msgs/Pose type, x, y, z coordinates (float type, unit: m) + quaternion.  
__pose_end__  
Target pose: geometry_msgs/Pose type, x, y, z coordinates (float type, unit: m) + quaternion.  
__speed__  
Speed percentage ratio coefficient, 0-100.  
__trajectory_connect__
Is the trajectory plan now. 1.wait 0.plan now.
__block__  
whether it is a blocking mode, bool type, true-blocking, false-non-blocking.  
__loop__
Number of cycles
### Joint_space_planning_to_target_pose-Movejp_msg
```
geometry_msgs/Pose pose  
uint8 speed  
uint8 trajectory_connect  
bool block 
```  
__msg member__ 
__pose__  
Target pose: geometry_msgs/Pose type, x, y, z coordinates (float type, unit: m) + quaternion.  
__speed__  
Speed percentage ratio coefficient, 0-100.  
__trajectory_connect__
Is the trajectory plan now. 1.wait 0.plan now.
__block__  
whether it is a blocking mode, bool type, true-blocking, false-non-blocking.  
### Joint_teaching-Jointteach_msg
```
uint8 num
uint8 direction
uint8 speed
```  
__msg member__  
__num__  
joint num，1~7.
__direction__  
teach direction，0-negative direction，1-positive direction.
__speed__
speed:speed percentage ratio coefficient, 0-100.  
### Position_teaching-Posteach_msg
```
uint8 type
uint8 direction
uint8 speed
```  
__msg member__  
__type__  
Teaching demonstration type: input0:X-axis direction、1:Y-axis direction、2:Z-axis direction.
__direction__  
teach direction，0-negative direction，1-positive direction.
__speed__
speed:speed percentage ratio coefficient, 0-100.    
### Attitude_teaching-Ortteach_msg
```
uint8 type
uint8 direction
uint8 speed
```  
__msg member__  
__type__  
Teaching demonstration type: input0:RX-axis direction、1:RY-axis direction、2:RZ-axis direction
__direction__  
teach direction，0-negative direction，1-positive direction.
__speed__
speed:speed percentage ratio coefficient, 0-100.  
### Joint_transmission-Jointpos_msg
```
float32[] joint  
bool follow  
float32 expand  
uint8 dof
```
__msg member__  
__joint__
Joint angle, float type, unit: radians.  
__follow__  
Follow state, bool type, true: high follow, false: low follow, default high follow if not set.  
__expand__  
Expand joint, float type, unit: radians.  
__dof__  
Degree of freedom message of the robotic arm.  
### Pose_transmission-Cartepos_msg
```
geometry_msgs/Pose pose  
bool follow  
```
__msg member__  
__pose__  
Robotic arm poses geometry_msgs/Pose type, x, y, z coordinates (float type, unit: m) + quaternion.  
__follow__  
Follow state, bool type, true: high follow, false: low follow, default high follow if not set.  
### Current_robotic_arm_state_Angle_and_Euler_angle-Armoriginalstate_msg
```
float32[] joint  
float32[6] pose  
uint16 arm_err  
uint16 sys_err  
uint8 dof
```  
__msg member__  
__joint__  
Joint angle, float type, unit: °.  
__pose__  
Current pose of the robotic arm, float type, x, y, z coordinates, unit: m, x, y, z Euler angle, unit: degree.  
__arm_err__  
Robotic arm running error code, unsigned int type.  
__arm_err__
Controller error code, unsigned int type.  
__dof__  
Degree of freedom message of the robotic arm.  
### Current_arm_state_radians_and_quaternion-Armstate_msg
```
float32[] joint  
geometry_msgs/Pose pose  
uint16 arm_err  
uint16 sys_err  
uint8 dof  
```
__msg member__  
__joint__  
Joint angle, float type, unit: radians.  
__pose__  
Current pose of the robotic arm, float type, x, y, z coordinates, unit: m, x, y, z, w quaternion.  
__arm_err__  
Robotic arm running error code, unsigned int type.  
__arm_err__  
Controller error code, unsigned int type.  
__dof__  
Degree of freedom message of the robotic arm.  
### Gripper_pick-Gripperpick_msg
```
uint16 speed  
uint16 force  
bool block 
uint16 timeout 
```
__msg member__  
__speed__  
Gripper pick speed, unsigned int type, range: 1-1000.  
__force__  
Gripper pick torque threshold, unsigned int type, range: 50-1000.  
__block__  
whether it is a blocking mode, bool type, true-blocking, false-non-blocking.  
__timeout__ 
Set the timeout for the return, and the blocking mode will take effect (in seconds).
### Gripper_pick_gripper_pick-on-Gripperpick_msg
```
uint16 speed  
uint16 force  
bool block  
uint16 timeout 
```
__msg member__  
__speed__  
Gripper pick speed, unsigned int type, range: 1-1000.  
__force__  
Gripper picks torque threshold, unsigned int type, range: 50-1000.  
__block__  
whether it is a blocking mode, bool type, true-blocking, false-non-blocking.  
__timeout__ 
Set the timeout for the return, and the blocking mode will take effect (in seconds).
### Gripper_reaching_the_given_position-Gripperset_msg
```
uint16 position  
bool block 
uint16 timeout 
``` 
__msg member__  
__position__  
Gripper target position, unsigned int type, range: 1-1000, representing the degree of opening of the gripper: 0-70 mm.  
__block__  
whether it is a blocking mode, bool type, true-blocking, false-non-blocking.  
__timeout__ 
Set the timeout for the return, and the blocking mode will take effect (in seconds).
### Force-position_mixing_control-Setforceposition_msg
```
uint8 sensor  
uint8 mode  
uint8 direction  
int16 n  
```  
__msg member__  
__sensor__  
Sensor; 0 - One-axis force; 1 - Six-axis force.  
__mode__  
Mode: 0 - Base coordinate system force control; 1 - Tool coordinate system force control.  
__Direction__  
Force control direction; 0 - Along the X-axis; 1 - Along the Y-axis; 2 - Along the Z-axis; 3 - Along the RX posture direction; 4 - Along the RY posture direction; 5 - Along the RZ posture direction.  
__n__  
Force value, unit: 0.1 N.    
### Six-axis_force_data-Sixforce_msg
```
float32 force_fx  
float32 force_fy  
float32 force_fz  
float32 force_mx  
float32 force_my  
float32 force_mz
```  
__msg member__  
__force_fx__  
the force along the x-axis direction.  
__force_fy__  
the force along the y-axis direction.  
__force_fz__
the force along the z-axis direction.  
__force_mx__  
the force when rotating along the x-axis direction.  
__force_my__  
the force when rotating along the y-axis direction.  
__force_mz__  
the force when rotating along the z-axis direction.  
### Setting_the_dexterous_hand_posture-Handposture_msg
```
uint16 posture_num  
bool block 
uint16 timeout  
``` 
__msg member__  
__posture_num__  
The serial number of the posture pre-saved in the dexterous hand, ranges from 1 to 40.  
__block__  
whether it is a blocking mode, bool type, true-blocking, false-non-blocking.  
__timeout__
The timeout setting for blocking mode, unit: seconds.    
### Setting_the_dexterous_hand_action_sequence-Handseq_msg
```
uint16 seq_num  
bool block
unint16 timeout  
```  
__msg member__  
__seq_num__	  
The serial number of the sequence pre-saved in the dexterous hand, ranging from 1 to 40.  
__block__  
whether it is a blocking mode, bool type, true-blocking, false-non-blocking.  
__timeout__
The timeout setting for blocking mode, unit: seconds.
### Setting_the_angles_of_various_degrees_of_freedom_for_the_dexterous_hand-Handangle_msg
```
int16[6] hand_angle   
bool block
```  
__msg member__  
__hand_angle__  
Hand angle array, range: 0-1000. And -1 represents that no operation is performed on this degree of freedom and the current state remains.  
__block__  
whether it is a blocking mode, bool type, true-blocking, false-non-blocking.  
### Setting_the_dexterous_hand_action_sequence-Handspeed_msg
```
uint16 hand_speed  
```  
__msg member__  
__hand_speed__
Hand speed, range: 1-1000.    
### Setting_the_force_threshold_for_the_dexterous_hand-Handforce_msg
```
uint16 hand_force  
```  
__msg member__  
__hand_force__  
Hand force, range: 1-1000.    
### Transmissive_force-position_mixing_control_compensation-angle-Forcepositionmovejoint_msg
```
float32[] joint  
uint8 sensor  
uint8 mode  
int16 dir  
float32 force  
bool follow  
uint8 dof
```  
__msg member__  
__joint__  
Angle force-position mixing transmission, unit: radians.  
__sensor__  
Type of sensor used, 0 - One-axis force, 1 - Six-axis force.  
__mode__  
Mode, 0 - Along the work coordinate system, 1 - Along the tool end coordinate system.  
__dir__  
Force control direction, 0 to 5 represent X/Y/Z/Rx/Ry/Rz respectively, and the default direction for one-axis force type is the Z direction.  
__force__  
Force value, accuracy: 0.1 N or 0.1 Nm.  
__block__  
whether it is a blocking mode, bool type, true-blocking, false-non-blocking.  
dof  
Degree of freedom message of the robotic arm.  
### Transmissive_force-position_mixing_control_compensation-pose-Forcepositionmovejoint_msg
```
geometry_msgs/Pose pose  
uint8 sensor  
uint8 mode  
int16 dir  
float32 force  
bool follow
```  
__msg member__  
__pose__  
Robotic arm pose message, x, y, z position message + quaternion posture message.  
__sensor__  
Type of sensor used, 0 - One-axis force, 1 - Six-axis force.  
__mode__  
Mode, 0 - Along the work coordinate system, 1 - Along the tool end coordinate system.  
__dir__  
Force control direction, 0 to 5 represent X/Y/Z/Rx/Ry/Rz respectively, and the default direction for one-axis force type is the Z direction.  
__force__  
Force value, accuracy: 0.1 N or 0.1 Nm.  
__block__  
whether it is a blocking mode, bool type, true-blocking, false-non-blocking.  
### Speed_open_loop_control-lifting_mechanism-Liftspeed_msg
```
int16 speed  
bool block
```  
__msg member__  
__speed__  
Speed percentage, -100-100. Speed < 0: the lifting mechanism moves downward; Speed > 0: the lifting mechanism moves upward; Speed = 0: the lifting mechanism stops.  
__block__  
whether it is a blocking mode, bool type, true-blocking, false-non-blocking.  
### Position_closed-loop_control-lifting_mechanism-Lift_height_msg
```
uint16 height  
uint16 speed  
bool block
```  
__msg member__  
__height__  
Target height, unit: mm, accuracy: 0-2600.  
__speed__  
Speed percentage, 1-100.  
__block__  
whether it is a blocking mode, bool type, true-blocking, false-non-blocking.  
### Getting_the_state_of_the_lifting_mechanism-Liftstate_msg
```
int16 height   
int16 current  
uint16 err_flag
```  
__msg member__  
__height__  
Current lifting mechanism height, unit: mm, accuracy: 1mm, range: 0-2300.  
__current__  
Lifting drive error code, error code type refers to joint error code.  
### Getting_or_setting_UDP_active_reporting_configuration-Setrealtimepush_msg
```
uint16 cycle  
uint16 port  
uint16 force_coordinate  
string ip
bool aloha_state_enable
bool arm_current_status_enable
bool expand_state_enable
bool hand_enable
bool joint_speed_enable
bool lift_state_enable
bool plus_base_enable
bool plus_state_enable
```  
__msg member__  
__cycle__  
Set the broadcast cycle, which is a multiple of 5ms.  
__port__  
Set the port number for broadcasting.  
__force_coordinate__  
Coordinate system for external force data of the system, where 0 is the sensor coordinate system, 1 is the current work coordinate system, and 2 is the current tool coordinate system.  
__ip__  
Customized reporting target IP address. 
__aloha_state_enable__
aloha master arm state enable.
__arm_current_status_enable__
arm_current_status enable.
__expand_state_enable__
Expansion joint-related data enable.
__hand_enable__
Dexterous hand state enable.
__joint_speed_enable__
Current joint speed, with an accuracy of 0.02 rpm.
__lift_state_enable__
Lifting joint data enable.
__plus_base_enable__
Basic information of the end-effector device enable.
__plus_state_enable__
Real-time information of the end-effector device enable. 

### UDP_manipulator_status_report-Armcurrentstatus_msg
```
uint16 arm_current_status
```
__msg member__   
__arm_current_status__  
Mechanical arm status,
0-RM_IDLE_E // Enabled but idle state
1-RM_MOVE_L_E // move L state in motion
2-RM_MOVE_J_E // move J in motion
3-RM_MOVE_C_E // move C state in motion
4-RM_MOVE_S_E // move S state in motion
5-RM_MOVE_THROUGH_JOINT_E // Angle transparent state
6-RM_MOVE_THROUGH_POSE_E // Pose transparent state
7-RM _ move _ through _ force _ pose _ e//Force control transparent transmission state
8-RM_MOVE_THROUGH_CURRENT_E // Current loop transparent state
9-RM_STOP_E // Emergency stop status
10-RM_SLOW_STOP_E // slow stop status
11-RM_PAUSE_E // Pause status
12-RM_CURRENT_DRAG_E // Current loop drag status
13-RM_SENSOR_DRAG_E // Six-axis force drag state
14-RM_TECH_DEMONSTRATION_E // Teaching Status
### UDP_joint_current_report-Jointcurrent_msg
```
float32[] joint_current
```
__msg member__    
__joint_current__   
Current joint current with an accuracy of 0.001mA.
 ### UDP_joint_enabling_status_report-Jointenflag_msg
```
bool[] joint_en_flag
```
__msg member__   
__joint_en_flag__   
Current joint enabling state, 1 is up enabling and 0 is down enabling.
### UDP_manipulator_Eulers_angular_pose_is_reported_to-Jointposeeuler_msg
```
float32[3] euler
float32[3] position
```
__msg member__    
__euler__   
Euler angle of current waypoint attitude, with an accuracy of 0.001rad.   
__position__   
The current waypoint position has an accuracy of 0.000001M.
### UDP_joint_speed_report_Jointspeed_msg
```
float32[] joint_speed
```
__msg member__    
__joint_speed__    
Current joint speed, accuracy 0.02RPM.
### UDP_joint_temperature_report_Jointtemperature_msg
```
float32[] joint_temperature
```
__msg member__   
__joint_temperature__   
Current joint temperature, with an accuracy of 0.001℃.
### UDP_joint_voltage_report_Jointvoltage_msg
```
float32[] joint_voltage
```
__msg member__     
__joint_voltage__   
Current joint voltage, with an accuracy of 0.001V V.

### System_error_code_Rmerr_msg
```
uint8 err_len
int32[] err
```  
__msg成员__  
__err_len__  
uint8。
__err__  
int32。

### Basic_information_of_the_end_effector_device_Rmplusbase_msg
```
string manu              # Device manufacturer;
int8 type                # Device type, 1 - Two-finger gripper, 2 - Five-finger dexterous hand, 3 - Three-finger gripper;
string hv                # Hardware version;
string sv                # Software version;
string bv                # Bootloader version;
int32 id                 # Device ID;
int8 dof                 # Degrees of freedom;
int8 check               # Self-check switch;
int8 bee                 # Beeper switch;
bool force               # Force control support;
bool touch               # Tactile support;
int8 touch_num           # Number of tactile sensors;
int8 touch_sw            # Tactile switch;
int8 hand                # Hand orientation, 1 - Left hand, 2 - Right hand;
int32[12] pos_up         # Position upper limit 
int32[12] pos_low        # Position lower limit 
int32[12] angle_up       # Angle upper limit 0.01°
int32[12] angle_low      # Angle lower limit 0.01°
int32[12] speed_up       # Speed upper limit 
int32[12] speed_low      # Speed lower limit 
int32[12] force_up       # Force upper limit 0.001N 
int32[12] force_low      # Force lower limit 0.001N 
```  
### Real_time_information_of_the_end_effector_deviceRmplusstate_msg
```
int32 sys_state                   #System status
int32[12] dof_state               #Current status of each degree of freedom (DoF)
int32[12] dof_err                 #Error information of each DoF
int32[12] pos                     #Current position of each DoF
int32[12] speed                   #Current Speed of Each Degree of each DoF
int32[12] angle                   #Current Angle of Each Degree of each DoF
int32[12] current                 #Current Current of Each Degree of Freedom
int32[18] normal_force            #Normal Force of Tactile Three-Dimensional Force of Each Degree of Freedom
int32[18] tangential_force        #Tangential Force of Tactile Three-Dimensional Force of Each Degree of Freedom
int32[18] tangential_force_dir    #Direction of Tangential Force of Tactile Three-Dimensional Force of Each Degree of Freedom
uint32[12] tsa                    #Tactile Self-Approach of Each Degree of Freedom
uint32[12] tma                    #Tactile Mutual Approach of Each Degree of Freedom
int32[18] touch_data              #Raw Data of Tactile Sensors
int32[12] force                   #Torque of Each Degree of Freedom
```  


### Customize_high_following_mode_joint_transmission_Jointposcustom_msg
```
float32[] joint  
bool follow  
float32 expand  
uint8 dof
uint8 trajectory_mode
uint8 radio
```
__msg member__     
__joint__  
Joint angle, float type, unit: radians.  
__follow__    
Follow state, bool type, true: high follow, false: low follow, default high follow if not set.  
__expand__  
Expand joint, float type, unit: radians.  
__dof__  
Degree of freedom message of the robotic arm.   
__trajectory_mode__  
When the high following mode is set, multiple modes are supported, including 0- complete transparent transmission mode, 1- curve fitting mode and 2- filtering mode.  
__radio__  
Set the smoothing coefficient in curve fitting mode (range 0-100) or the filter parameter in filtering mode (range 0-1000). The higher the value, the better the smoothing effect.  

### Customize_high_following_mode_pose_transmission_Carteposcustom_msg
```
geometry_msgs/Pose pose  
bool follow  
uint8 trajectory_mode
uint8 radio
```
__msg member__  
__pose__  
Robotic arm poses geometry_msgs/Pose type, x, y, z coordinates (float type, unit: m) + quaternion.  
__follow__  
Follow state, bool type, true: high follow, false: low follow, default high follow if not set. 
__trajectory_mode__  
When the high following mode is set, multiple modes are supported, including 0- complete transparent transmission mode, 1- curve fitting mode and 2- filtering mode.  
__radio__  
Set the smoothing coefficient in curve fitting mode (range 0-100) or the filter parameter in filtering mode (range 0-1000). The higher the value, the better the smoothing effect.  

It is mainly for the application of API to achieve some of the robotic arm functions; for a more complete introduction and use, please see the special document "[RealMan Robotic Arm ROS2 Topic Detailed Description](https://github.com/kaola-zero/ros2_rm_robot/blob/main/rm_driver/doc/RealMan%20Robotic%20Arm%20rm_driver%20Topic%20Detailed%20Description%20(ROS2).md)".