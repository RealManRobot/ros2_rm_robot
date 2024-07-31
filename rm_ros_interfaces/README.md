<div align="right">
  
[中文简体](https://github.com/RealManRobot/ros2_rm_robot/blob/foxy/rm_ros_interfaces/README_CN.md)|
[English](https://github.com/RealManRobot/ros2_rm_robot/blob/foxy/rm_ros_interfaces/README.md)

</div>

<div align="center">

# RealMan Robot rm_ros_interface User Manual V1.1


 


RealMan Intelligent Technology (Beijing) Co., Ltd.
 
Revision History-

| No. | Date| Comment |
| -----| -----| -----|
|V1.0 | 2-18-2024 | Draft |
|V1.1 | 7-8-2024  | Amend(Add teaching message) |

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
* 4.15[Getting the software version-Armsoftversion_msg](#Getting_the_software_version-Armsoftversion_msg)
* 4.16[Gripper's pick-Gripperpick_msg](#Gripper's_pick-Gripperpick_msg)
* 4.17[Gripper's pick gripper's pick-on-Gripperpick_msg](#Gripper's_pick_gripper's_pick-on-Gripperpick_msg)
* 4.18[Gripper reaching the given position-Gripperset_msg](#Gripper_reaching_the_given_position-Gripperset_msg)
* 4.19[Force-position mixing control-Setforceposition_msg](#Force-position_mixing_control-Setforceposition_msg)
* 4.20[Six-axis force data-Sixforce_msg](#Six-axis_force_data-Sixforce_msg)
* 4.21[Setting the dexterous hand posture-Hand posture_msg](#Setting_the_dexterous_hand_posture-Hand_posture_msg)
* 4.22[Setting the dexterous hand action sequence-Handseq_msg](#Setting_the_dexterous_hand_action_sequence-Handseq_msg)
* 4.23[Setting the angles of various degrees of freedom for the dexterous hand-Handangle_msg](#Setting_the_angles_of_various_degrees_of_freedom_for_the_dexterous_hand-Handangle_msg)
* 4.24[Setting the dexterous hand action sequence-Handspeed_msg](#Setting_the_dexterous_hand_action_sequence-Handspeed_msg)
* 4.25[Setting the force threshold for the dexterous hand-Handforce_msg](#Setting_the_force_threshold_for_the_dexterous_hand-Handforce_msg)
* 4.26[Transmissive force-position mixing control compensation-angle-Forcepositionmovejoint_msg](#Transmissive_force-position_mixing_control_compensation-angle-Forcepositionmovejoint_msg)
* 4.27[Transmissive force-position mixing control compensation-pose-Forcepositionmovejoint_msg](#Transmissive_force-position_mixing_control_compensation-pose-Forcepositionmovejoint_msg)
* 4.28[Speed open loop control-lifting mechanism-Liftspeed_msg](#Speed_open_loop_control-lifting_mechanism-Liftspeed_msg)
* 4.29[Position closed-loop control-lifting mechanism-Lift height_msg](#Position_closed-loop_control-lifting_mechanism-Lift_height_msg)
* 4.30[Getting the state of the lifting mechanism-Liftstate_msg](#Getting_the_state_of_the_lifting_mechanism-Liftstate_msg)
* 4.31[Getting or setting UDP active reporting configuration-Setrealtimepush_msg](#Getting_or_setting_UDP_active_reporting_configuration-Setrealtimepush_msg)


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
│   ├── Armoriginalstate_msg
│   ├── Armsoftversion_msg
│   ├── Armstate_msg
│   ├── Cartepos_msg
│   ├── Forcepositionmovejoint75_msg
│   ├── Forcepositionmovejoint_msg
│   ├── Forcepositionmovepose_msg
│   ├── Force_Position_State_msg
│   ├── Getallframe_msg
│   ├── GetArmState_Command_msg
│   ├── Gripperpick_msg
│   ├── Gripperset_msg
│   ├── Handangle_msg
│   ├── Hand force_msg
│   ├── Handposture_msg
│   ├── Handseq_msg
│   ├── Handspeed_msg
│   ├── Jointerrclear_msg
│   ├── Jointerrorcode75_msg
│   ├── Jointerrorcode_msg
│   ├── Jointpos75_msg
│   ├── Jointpos_msg
│   ├── Lift height_msg
│   ├── Liftspeed_msg
│   ├── Liftstate_msg
│   ├── Movec_msg
│   ├── Movej75_msg
│   ├── Movej_msg
│   ├── Movejp_msg
│   ├── Movel_msg
│   ├── Setforceposition_msg
│   ├── Setrealtimepush_msg
│   ├── Sixforce_msg
│   └── Stop_msg
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
bool block  
```
__msg member__  
__joint_num__  
the corresponding joint number, from the base to the robotic arm gripper, the number is 1-6 or 1-7.  
__block__  
whether it is a blocking mode, bool type, true-blocking, false-non-blocking.  
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
bool block
```  
__msg member__  
__num__  
joint num，1~7.
__direction__  
teach direction，0-negative direction，1-positive direction.
__speed__
speed:speed percentage ratio coefficient, 0-100. 
__block__  
whether it is a blocking mode, bool type, true: blocking, false: non-blocking. 
### Position_teaching-Posteach_msg
```
uint8 type
uint8 direction
uint8 speed
bool block
```  
__msg member__  
__type__  
Teaching demonstration type: input0:X-axis direction、1:Y-axis direction、2:Z-axis direction.
__direction__  
teach direction，0-negative direction，1-positive direction.
__speed__
speed:speed percentage ratio coefficient, 0-100.   
__block__  
whether it is a blocking mode, bool type, true: blocking, false: non-blocking. 
### Attitude_teaching-Ortteach_msg
```
uint8 type
uint8 direction
uint8 speed
bool block
```  
__msg member__  
__type__  
Teaching demonstration type: input0:RX-axis direction、1:RY-axis direction、2:RZ-axis direction
__direction__  
teach direction，0-negative direction，1-positive direction.
__speed__
speed:speed percentage ratio coefficient, 0-100.  
__block__  
whether it is a blocking mode, bool type, true: blocking, false: non-blocking.
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
### Getting_the_software_version-Armsoftversion_msg
```
string planversion  
string ctrlversion  
string kernal1  
string kernal2  
string productversion  
```
__msg member__  
__planversion__  
The read user interface kernel version number, string type.  
__ctrlversion__  
Real-time kernel version number, string type.  
__kernal1__  
The version number of sub-core 1 of the real-time kernel, string type.  
__kernal2__  
The version number of sub-core 2 of the real-time kernel, string type.  
__productversion__  
Robotic arm model, string type.  
### Gripper's_pick-Gripperpick_msg
```
uint16 speed  
uint16 force  
bool block  
```
__msg member__  
__speed__  
Gripper pick speed, unsigned int type, range: 1-1000.  
__force__  
Gripper pick torque threshold, unsigned int type, range: 50-1000.  
__block__  
whether it is a blocking mode, bool type, true-blocking, false-non-blocking.  
### Gripper's_pick_gripper's_pick-on-Gripperpick_msg
```
uint16 speed  
uint16 force  
bool block  
```
__msg member__  
__speed__  
Gripper pick speed, unsigned int type, range: 1-1000.  
__force__  
Gripper picks torque threshold, unsigned int type, range: 50-1000.  
__block__  
whether it is a blocking mode, bool type, true-blocking, false-non-blocking.  
### Gripper_reaching_the_given_position-Gripperset_msg
```
uint16 position  
bool block 
``` 
__msg member__  
__position__  
Gripper target position, unsigned int type, range: 1-1000, representing the degree of opening of the gripper: 0-70 mm.  
__block__  
whether it is a blocking mode, bool type, true-blocking, false-non-blocking.  
### Force-position_mixing_control-Setforceposition_msg
```
uint8 sensor  
uint8 mode  
uint8 direction  
int16 n  
bool block
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
__block__  
whether it is a blocking mode, true: blocking, false: non-blocking.  
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
### Setting_the_dexterous_hand_posture-Hand_posture_msg
```
uint16 posture_num  
bool block 
``` 
__msg member__  
__posture_num__  
The serial number of the posture pre-saved in the dexterous hand, ranges from 1 to 40.  
__block__  
whether it is a blocking mode, bool type, true-blocking, false-non-blocking.  
### Setting_the_dexterous_hand_action_sequence-Handseq_msg
```
uint16 seq_num  
bool block
```  
__msg member__  
__seq_num__	  
The serial number of the sequence pre-saved in the dexterous hand, ranging from 1 to 40.  
__block__  
whether it is a blocking mode, bool type, true-blocking, false-non-blocking.  
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
bool block
```  
__msg member__  
__hand_speed__
Hand speed, range: 1-1000.  
__block__  
whether it is a blocking mode, bool type, true-blocking, false-non-blocking.  
### Setting_the_force_threshold_for_the_dexterous_hand-Handforce_msg
```
uint16 hand_force  
bool block
```  
__msg member__  
__hand_force__  
Hand force, range: 1-1000.  
__block__  
whether it is a blocking mode, bool type, true-blocking, false-non-blocking.  
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

It is mainly for the application of API to achieve some of the robotic arm functions; for a more complete introduction and use, please see the special document "[RealMan Robotic Arm ROS2 Topic Detailed Description](https://github.com/kaola-zero/ros2_rm_robot/blob/main/rm_driver/doc/RealMan%20Robotic%20Arm%20rm_driver%20Topic%20Detailed%20Description%20(ROS2).md)".
