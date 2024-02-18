<div align="right">
  
[中文简体](https://github.com/kaola-zero/ros2_rm_robot/blob/main/rm_driver/doc/%E7%9D%BF%E5%B0%94%E6%9B%BC%E6%9C%BA%E6%A2%B0%E8%87%82ROS2rm_driver%E8%AF%9D%E9%A2%98%E8%AF%A6%E7%BB%86%E8%AF%B4%E6%98%8E.md)|
[English](https://github.com/kaola-zero/ros2_rm_robot/blob/main/rm_driver/doc/RealMan%20Robotic%20Arm%20rm_driver%20Topic%20Detailed%20Description%20(ROS2).md)

</div>

<div align="center">

# RealMan Robot rm_ros_interface User Manual V1.0


 


RealMan Intelligent Technology (Beijing) Co., Ltd.
 
Revision History:

| No. | Date| Comment |
| :---: | :---- | :---: |
|V1.0 | 2-18-2024 | Draft |
		
</div>

## 目录
* 1[rm_ros_interface Package Description](#rm_ros_interface_Package_Description)
* 2[rm_ros_interface Package Use](#rm_ros_interface_Package_Use)
* 3[rm_ros_interface Package Architecture Description](#rm_ros_interface_Package_Architecture_Description)
* 3.1[Overview of Package Files](#Overview_of_Package_Files)
* 4[rm_ros_interface message description](#rm_ros_interface_message_description)
* 4.1[Joint error code: Jointerrorcode.msg](#Joint_error_code:Jointerrorcode.msg)
* 4.2[Clearing the joint's error code: Jointerrclear.msg](#Clearing_the_joint's_error_code:Jointerrclear.msg)
* 4.3[All coordinate system names: Getallframe.msg](#All_coordinate_system_names:Getallframe.msg)
* 4.4[Joine motion: Movej.msg](#Joine_motion:Movej.msg)
* 4.5[Linear motion: Movel.msg](#Linear_motion:Movel.msg)
* 4.6[Circular motion: Movec.msg](#Circular_motion:Movec.msg)
* 4.7[Joint space planning to target pose: Movejp.msg](#Joint_space_planning_to_target_pose:Movejp.msg)
* 4.8[Joint transmission: Jointpos.msg](#Joint_transmission:Jointpos.msg)
* 4.9[Pose transmission: Cartepos.msg](#Pose_transmission:Cartepos.msg)
* 4.10[Current robotic arm state Angle and Euler angle: Armoriginalstate.msg](#Current_robotic_arm_state_Angle_and_Euler_angle:Armoriginalstate.msg)
* 4.11[Current arm state radians and quaternion: Armstate.msg](#Current_arm_state_radians_and_quaternion:Armstate.msg)
* 4.12[Getting the software version: Armsoftversion.msg](#Getting_the_software_version:Armsoftversion.msg)
* 4.13[Gripper's pick: Gripperpick.msg](#Gripper's_pick:Gripperpick.msg)
* 4.14[Gripper's pick gripper's pick-on: Gripperpick.msg](#Gripper's_pick_gripper's_pick-on:Gripperpick.msg)
* 4.15[Gripper reaching the given position: Gripperset.msg](#Gripper_reaching_the_given_position:Gripperset.msg)
* 4.16[Force-position mixing control: Setforceposition.msg](#Force-position_mixing_control:Setforceposition.msg)
* 4.17[Six-axis force data: Sixforce.msg](#Six-axis_force_data:Sixforce.msg)
* 4.18[Setting the dexterous hand posture: Hand posture.msg](#Setting_the_dexterous_hand_posture:Hand_posture.msg)
* 4.19[Setting the dexterous hand action sequence: Handseq.msg](#Setting_the_dexterous_hand_action_sequence:Handseq.msg)
* 4.20[Setting the angles of various degrees of freedom for the dexterous hand: Handangle.msg](#Setting_the_angles_of_various_degrees_of_freedom_for_the_dexterous_hand:Handangle.msg)
* 4.21[Setting the dexterous hand action sequence: Handspeed.msg](#Setting_the_dexterous_hand_action_sequence:Handspeed.msg)
* 4.22[Setting the force threshold for the dexterous hand: Handforce.msg](#Setting_the_force_threshold_for_the_dexterous_hand:Handforce.msg)
* 4.23[Transmissive force-position mixing control compensation-angle: Forcepositionmovejoint.msg](#Transmissive_force-position_mixing_control_compensation-angle:Forcepositionmovejoint.msg)
* 4.24[Transmissive force-position mixing control compensation-pose: Forcepositionmovejoint.msg](#Transmissive_force-position_mixing_control_compensation-pose:Forcepositionmovejoint.msg)
* 4.25[Speed open loop control-lifting mechanism:Liftspeed.msg](#Speed_open_loop_control-lifting_mechanism:Liftspeed.msg)
* 4.26[Position closed-loop control-lifting mechanism:Lift height.msg](#Position_closed-loop_control-lifting_mechanism:Lift_height.msg)
* 4.27[Getting the state of the lifting mechanism: Liftstate.msg](#Getting_the_state_of_the_lifting_mechanism:Liftstate.msg)
* 4.28[Getting/setting UDP active reporting configuration: Setrealtimepush.msg](Getting/setting_UDP_active_reporting_configuration:Setrealtimepush.msg)


## rm_ros_interface_Package_Description
The main function of the rm_ros_interface package is to provide necessary message files for the robotic arm to run under the framework of ROS2. In the following text, we will provide a detailed introduction to this package through the following aspects.
* 1.Package use.  
* 2.Package architecture description.  
* 3.Package topic description.  
Through the introduction of the three parts, it can help you:   
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
│   ├── Armoriginalstate.msg
│   ├── Armsoftversion.msg
│   ├── Armstate.msg
│   ├── Cartepos.msg
│   ├── Forcepositionmovejoint75.msg
│   ├── Forcepositionmovejoint.msg
│   ├── Forcepositionmovepose.msg
│   ├── Force_Position_State.msg
│   ├── Getallframe.msg
│   ├── GetArmState_Command.msg
│   ├── Gripperpick.msg
│   ├── Gripperset.msg
│   ├── Handangle.msg
│   ├── Hand force.msg
│   ├── Handposture.msg
│   ├── Handseq.msg
│   ├── Handspeed.msg
│   ├── Jointerrclear.msg
│   ├── Jointerrorcode75.msg
│   ├── Jointerrorcode.msg
│   ├── Jointpos75.msg
│   ├── Jointpos.msg
│   ├── Lift height.msg
│   ├── Liftspeed.msg
│   ├── Liftstate.msg
│   ├── Movec.msg
│   ├── Movej75.msg
│   ├── Movej.msg
│   ├── Movejp.msg
│   ├── Movel.msg
│   ├── Setforceposition.msg
│   ├── Setrealtimepush.msg
│   ├── Sixforce.msg
│   └── Stop.msg
├── package.xml                                      # dependency declaration file
└── src
```
## rm_ros_interface_message_description
### Joint_error_code:Jointerrorcode.msg
uint16[] joint_error  
uint8 dof  
msg member  
uint16[] joint_error  
Error message for each joint.  
uint8 dof  
Degree of freedom message of the robotic arm.  
### Clearing_the_joint's_error_code:Jointerrclear.msg
uint8 joint_num  
bool block  
msg member  
joint_num  
the corresponding joint number, from the base to the robotic arm gripper, the number is 1-6 or 1-7.  
block  
whether it is a blocking mode, bool type, true: blocking, false: non-blocking.  
### All_coordinate_system_names:Getallframe.msg
string[10] frame_name  
msg member  
frame_name  
The array of work coordinate system names returned  
### Joine_motion:Movej.msg
float32[] joint  
uint8 speed  
bool block  
uint8 dof  
msg member  
joint  
Joint angle, float type, unit: radians.  
speed  
Speed percentage ratio coefficient, 0-100.  
block  
whether it is a blocking mode, bool type, true: blocking, false: non-blocking.  
dof  
Degree of freedom message of the robotic arm.  
### Linear_motion:Movel.msg
geometry_msgs/Pose pose  
uint8 speed  
bool block  
msg member  
pose  
Robotic arm pose: geometry_msgs/Pose type, x, y, z coordinates (float type, unit: m) + quaternion (float type).  
speed  
Speed percentage ratio coefficient, 0-100.  
block  
whether it is a blocking mode, bool type, true: blocking, false: non-blocking.  
### 圆弧运动Movec.msg
geometry_msgs/Pose pose_mid  
geometry_msgs/Pose pose_end  
uint8 speed  
bool block  
msg member  
pose_mid  
中间位姿，geometry_msgs/Pose类型，x、y、z坐标（float类型，单位：m）+四元数。  
pose_end  
目标位姿，geometry_msgs/Pose类型，x、y、z坐标（float类型，单位：m）+四元数。  
speed  
速度百分比例系数，0~100。  
block  
whether it is a blocking mode, bool type, true: blocking, false: non-blocking.  
### 关节空间规划到目标位姿Movejp.msg
geometry_msgs/Pose pose  
uint8 speed  
bool block  
msg member  
pose  
目标位姿，geometry_msgs/Pose类型，x、y、z坐标（float类型，单位：m）+四元数。  
speed  
速度百分比例系数，0~100。  
block  
whether it is a blocking mode, bool type, true: blocking, false: non-blocking.  
### 角度透传Jointpos.msg
float32[] joint  
bool follow  
float32 expand  
uint8 dof  
msg member  
joint
关节角度，float类型，单位：弧度。  
follow  
跟随状态，bool类型，true高跟随，false低跟随，不设置默认高跟随。  
expand  
拓展关节，float类型，单位：弧度。  
dof  
机械臂自由度信息。  
### 位姿透传Cartepos.msg
geometry_msgs/Pose pose  
bool follow  
msg member  
pose  
机械臂位姿，geometry_msgs/Pose类型，x、y、z坐标（float类型，单位：m）+四元数。  
follow  
跟随状态，bool类型，true高跟随，false低跟随，不设置默认高跟随。  
### 机械臂当前状态（角度+欧拉角）Armoriginalstate.msg
float32[] joint  
float32[6] pose  
uint16 arm_err  
uint16 sys_err  
uint8 dof  
msg member  
joint  
关节角度，float类型，单位：度。  
pose  
机械臂当前位姿，float类型，x、y、z坐标，单位：m，x、y、z欧拉角，单位：度。  
arm_err  
机械臂运行错误代码，unsigned int类型。  
arm_err
控制器错误代码，unsigned int类型。  
dof  
机械臂自由度信息。  
### 机械臂当前状态（弧度+四元数）Armstate.msg
float32[] joint  
geometry_msgs/Pose pose  
uint16 arm_err  
uint16 sys_err  
uint8 dof  
msg member  
joint  
关节角度，float类型，单位：弧度。  
pose  
机械臂当前位姿，float类型，x、y、z坐标，单位：m，x、y、z、w四元数。  
arm_err  
机械臂运行错误代码，unsigned int类型。  
arm_err  
控制器错误代码，unsigned int类型。  
dof  
机械臂自由度信息。  
### 读取软件版本号Armsoftversion.msg
string planversion  
string ctrlversion  
string kernal1  
string kernal2  
string productversion  
msg member  
planversion  
读取到的用户接口内核版本号，string类型。  
ctrlversion  
实时内核版本号，string类型。  
kernal1  
实时内核子核心 1 版本号，string类型。  
kernal2  
实时内核子核心 2 版本号，string类型。  
productversion  
机械臂型号，string类型。  
### 手爪力控夹取Gripperpick.msg
uint16 speed  
uint16 force  
bool block  
msg member  
speed  
手爪力控夹取速度，unsigned int类型，范围：1~1000。  
force  
手爪夹取力矩阈值，unsigned int类型，范围 ：50~1000。  
block  
whether it is a blocking mode, bool type, true: blocking, false: non-blocking.  
### 手爪力控夹取（持续力控夹取）Gripperpick.msg
uint16 speed  
uint16 force  
bool block  
msg member  
speed  
手爪力控夹取速度，unsigned int类型，范围：1~1000。  
force  
手爪夹取力矩阈值，unsigned int类型，范围 ：50~1000。  
block  
whether it is a blocking mode, bool type, true: blocking, false: non-blocking.  
### 手爪到达指定位置Gripperset.msg
uint16 position  
bool block  
msg member  
position  
手爪目标位置，unsigned int类型，范围：1～1000,代表手爪开口度：0～70mm。  
block  
whether it is a blocking mode, bool type, true: blocking, false: non-blocking.  
### 力位混合控制Setforceposition.msg
uint8 sensor  
uint8 mode  
uint8 direction  
int16 n  
bool block  
msg member  
sensor  
传感器；0-一维力；1-六维力。  
mode  
Mode：0-工作坐标系力控； 1-工具坐标系力控。  
Direction  
力控方向；0-沿X 轴；1-沿Y 轴；2-沿 Z 轴；3-沿RX 姿态方向；4-沿 RY 姿态方向；5-沿 RZ 姿态方向。  
n  
力的大小，单位 0.1N。  
block  
是否阻塞，true:阻塞，false:非阻塞。  
### 六维力数据Sixforce.msg
float32 force_fx  
float32 force_fy  
float32 force_fz  
float32 force_mx  
float32 force_my  
float32 force_mz  
msg member  
force_fx  
沿x轴方向受力大小。  
force_fy  
沿y轴方向受力大小。  
force_fz
沿z轴方向受力大小。  
force_mx  
沿x轴方向转动受力大小。  
force_my  
沿y轴方向转动受力大小。  
force_mz  
沿z轴方向转动受力大小。  
### 设置灵巧手手势Handposture.msg
uint16 posture_num  
bool block  
msg member  
posture_num  
预先保存在灵巧手内的手势序号，范围：1~40。  
block  
whether it is a blocking mode, bool type, true: blocking, false: non-blocking.  
### 设置灵巧手动作序列Handseq.msg
uint16 seq_num  
bool block  
msg member  
seq_num	  
预先保存在灵巧手内的序列序号，范围：1~40。  
block  
whether it is a blocking mode, bool type, true: blocking, false: non-blocking.  
### 设置灵巧手各自由度角度Handangle.msg
int16[6] hand_angle   
bool block  
msg member  
hand_angle  
手指角度数组，范围：0~1000。另外，-1 代表该自由度不执行任何操作，保持当前状态。  
block  
whether it is a blocking mode, bool type, true: blocking, false: non-blocking.  
### 设置灵巧手速度Handspeed.msg
uint16 hand_speed  
bool block  
msg member  
hand_speed
手指速度，范围：1~1000。  
block  
whether it is a blocking mode, bool type, true: blocking, false: non-blocking.  
### 设置灵巧手力阈值Handforce.msg
uint16 hand_force  
bool block  
msg member  
hand_force  
手指力，范围：1~1000。  
block  
whether it is a blocking mode, bool type, true: blocking, false: non-blocking.  
### 透传力位混合补偿（角度）Forcepositionmovejoint.msg
float32[] joint  
uint8 sensor  
uint8 mode  
int16 dir  
float32 force  
bool follow  
uint8 dof  
msg member  
joint  
角度力位混合透传，单位：弧度。  
sensor  
所使用传感器类型，0-一维力，1-六维力。  
mode  
模式，0-沿工作坐标系，1-沿工具端坐标系。  
dir  
力控方向，0~5 分别代表 X/Y/Z/Rx/Ry/Rz，其中一维力类型时默认方向为Z 方向。  
force  
力的大小，精度 0.1N 或者 0.1Nm。  
block  
whether it is a blocking mode, bool type, true: blocking, false: non-blocking.  
dof  
机械臂自由度信息。  
### 透传力位混合补偿（位姿）Forcepositionmovejoint.msg
geometry_msgs/Pose pose  
uint8 sensor  
uint8 mode  
int16 dir  
float32 force  
bool follow  
msg member  
pose  
机械臂位姿信息，x、y、z位置信息+四元数姿态信息。  
sensor  
所使用传感器类型，0-一维力，1-六维力。  
mode  
模式，0-沿工作坐标系，1-沿工具端坐标系。  
dir  
力控方向，0~5 分别代表 X/Y/Z/Rx/Ry/Rz，其中一维力类型时默认方向为Z 方向。  
force  
力的大小，精度 0.1N 或者 0.1Nm。  
block  
whether it is a blocking mode, bool type, true: blocking, false: non-blocking.  
### 速度开环控制（升降机构）Liftspeed.msg
int16 speed  
bool block  
msg member  
speed  
速度百分比，-100~100。Speed < 0:升降机构向下运动；Speed > 0:升降机构向上运动；Speed = 0:升降机构停止运动。  
block  
whether it is a blocking mode, bool type, true: blocking, false: non-blocking.  
### 位置闭环控制（升降机构）Liftheight.msg
uint16 height  
uint16 speed  
bool block  
msg member  
height  
目标高度，单位 mm，范围：0~2600。  
speed  
速度百分比，1~100。  
block  
whether it is a blocking mode, bool type, true: blocking, false: non-blocking.  
### 获取升降机构状态（升降机构）Liftstate.msg
int16 height   
int16 current  
uint16 err_flag  
msg member  
height  
当前升降机构高度，单位：mm，精度：1mm，范围：0~2300。  
current  
升降驱动错误代码，错误代码类型参考关节错误代码。  
### 查询（设置）UDP 机械臂状态主动上报配置Setrealtimepush.msg
uint16 cycle  
uint16 port  
uint16 force_coordinate  
string ip  
msg member  
cycle  
设置广播周期，为5ms的倍数。  
port  
设置广播的端口号。  
force_coordinate  
系统外受力数据的坐标系，0 为传感器坐标系 1 为当前工作坐标系 2 为当前工具坐标系。  
ip  
自定义的上报目标IP 地址。  

主要为套用API实现的一些机械臂本体的功能，其详细介绍和使用在此不详细展开，可以通过专门的文档《[睿尔曼机械臂ROS2话题详细说明](https://github.com/kaola-zero/ros2_rm_robot/blob/main/rm_driver/doc/%E7%9D%BF%E5%B0%94%E6%9B%BC%E6%9C%BA%E6%A2%B0%E8%87%82ROS2rm_driver%E8%AF%9D%E9%A2%98%E8%AF%A6%E7%BB%86%E8%AF%B4%E6%98%8E.md)》进行查看。
