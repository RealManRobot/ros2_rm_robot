<div align="right">
  
[中文简体](https://github.com/kaola-zero/ros2_rm_robot/blob/main/rm_driver/doc/%E7%9D%BF%E5%B0%94%E6%9B%BC%E6%9C%BA%E6%A2%B0%E8%87%82ROS2rm_driver%E8%AF%9D%E9%A2%98%E8%AF%A6%E7%BB%86%E8%AF%B4%E6%98%8E.md)|
[English](https://github.com/kaola-zero/ros2_rm_robot/blob/main/rm_driver/doc/RealMan%20Robotic%20Arm%20rm_driver%20Topic%20Detailed%20Description%20(ROS2).md)

</div>

<div align="center">

# 睿尔曼机器人rm_ros_interface使用说明书V1.0


 


睿尔曼智能科技(北京)有限公司
 
文件修订记录：

|版本号 | 时间 | 备注 |
| :---: | :---- | :---: |
|V1.0 | 2024-2-18 | 拟制|
		
</div>

## 目录
1[rm_ros_interface功能包说明](#rm_ros_interface功能包说明)
2[rm_ros_interface功能包使用](#rm_ros_interface功能包使用)
3[rm_ros_interface功能包架构说明](#rm_ros_interface功能包架构说明)
3.1[功能包文件总览](#功能包文件总览)
4[rm_ros_interface消息说明](#rm_ros_interface消息说明)
4.1[关节错误代码Jointerrorcode.msg](#关节错误代码Jointerrorcode.msg)
4.2[清除关节错误代码Jointerrclear.msg](#清除关节错误代码Jointerrclear.msg)
4.3[所有坐标系名称Getallframe.msg](#所有坐标系名称Getallframe.msg)
4.4[关节运动Movej.msg](#关节运动Movej.msg)
4.5[直线运动Movel.msg](#直线运动Movel.msg)
4.6[圆弧运动Movec.msg](#圆弧运动Movec.msg)
4.7[关节空间规划到目标位姿Movejp.msg](#关节空间规划到目标位姿Movejp.msg)
4.8[角度透传Jointpos.msg](#角度透传Jointpos.msg)
4.9[位姿透传Cartepos.msg](#位姿透传Cartepos.msg)
4.10[机械臂当前状态（角度+欧拉角）Armoriginalstate.msg](#机械臂当前状态（角度+欧拉角）Armoriginalstate.msg)
4.11[机械臂当前状态（弧度+四元数）Armstate.msg](#机械臂当前状态（弧度+四元数）Armstate.msg)
4.12[读取软件版本号Armsoftversion.msg](#读取软件版本号Armsoftversion.msg)
4.13[手爪力控夹取Gripperpick.msg](#手爪力控夹取Gripperpick.msg)
4.14[手爪力控夹取-持续力控夹取Gripperpick.msg](#手爪力控夹取-持续力控夹取Gripperpick.msg)
4.15[手爪到达指定位置Gripperset.msg](#手爪到达指定位置Gripperset.msg)
4.16[力位混合控制Setforceposition.msg](#力位混合控制Setforceposition.msg)
4.17[六维力数据Sixforce.msg](#六维力数据Sixforce.msg)
4.18[设置灵巧手手势Handposture.msg](#设置灵巧手手势Handposture.msg)
4.19[设置灵巧手动作序列Handseq.msg](#设置灵巧手动作序列Handseq.msg)
4.20[设置灵巧手各自由度角度Handangle.msg](#设置灵巧手各自由度角度Handangle.msg)
4.21[设置灵巧手速度Handspeed.msg](#设置灵巧手速度Handspeed.msg)
4.22[设置灵巧手力阈值Handforce.msg](#设置灵巧手力阈值Handforce.msg)
4.23[透传力位混合补偿-角度Forcepositionmovejoint.msg](#透传力位混合补偿-角度Forcepositionmovejoint.msg)
4.24[透传力位混合补偿-位姿Forcepositionmovejoint.msg](#透传力位混合补偿-位姿Forcepositionmovejoint.msg)
4.25[速度开环控制-升降机构Liftspeed.msg](#速度开环控制-升降机构Liftspeed.msg)
4.26[位置闭环控制-升降机构Liftheight.msg](#位置闭环控制-升降机构Liftheight.msg)
4.27[获取升降机构状态-升降机构Liftstate.msg](#获取升降机构状态-升降机构Liftstate.msg)


## rm_ros_interface功能包说明
rm_ros_interface功能包的主要作用为为机械臂在ROS2的框架下运行提供必要的 消息文件，在下文中将通过以下几个方面详细介绍该功能包。
* 1.功能包使用。  
* 2.功能包架构说明。  
* 3.功能包话题说明。  
通过这三部分内容的介绍可以帮助大家：  
* 1.了解该功能包的使用。
* 2.熟悉功能包中的文件构成及作用。
* 3.熟悉功能包相关的话题，方便开发和使用。
## rm_ros_interface功能包使用
该功能包并没有可执行的使用命令，其主要作用为为其他功能包提供必须的消息文件。
## rm_ros_interface功能包架构说明
### 功能包文件总览
```
当前rm_driver功能包的文件构成如下。
├── CMakeLists.txt                #编译规则文件
├── include                       #依赖头文件文件夹
│   └── rm_ros_interfaces
├── msg                          #当前的消息文件（详细请看下方介绍）
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
│   ├── Handforce.msg
│   ├── Handposture.msg
│   ├── Handseq.msg
│   ├── Handspeed.msg
│   ├── Jointerrclear.msg
│   ├── Jointerrorcode75.msg
│   ├── Jointerrorcode.msg
│   ├── Jointpos75.msg
│   ├── Jointpos.msg
│   ├── Liftheight.msg
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
├── package.xml                       #依赖声明文件
└── src
```
## rm_ros_interface消息说明
### 关节错误代码Jointerrorcode.msg
uint16[] joint_error  
uint8 dof  
msg成员  
uint16[] joint_error  
每个关节报错信息。  
uint8 dof  
机械臂自由度信息。  
### 清除关节错误代码Jointerrclear.msg
uint8 joint_num  
bool block  
msg成员  
joint_num  
对应关节序号，从基座到机械臂手爪端，序号依次为1～6或1~7。  
block  
是否为阻塞模式，bool类型，true:阻塞，false:非阻塞。  
### 所有坐标系名称Getallframe.msg
string[10] frame_name  
msg成员  
frame_name  
返回的工作坐标系的名称数组。  
### 关节运动Movej.msg
float32[] joint  
uint8 speed  
bool block  
uint8 dof  
msg成员  
joint  
关节角度，float类型，单位：弧度。  
speed  
速度百分比例系数，0~100。  
block  
是否为阻塞模式，bool类型，true:阻塞，false:非阻塞。  
dof  
机械臂自由度信息。  
### 直线运动Movel.msg
geometry_msgs/Pose pose  
uint8 speed  
bool block  
msg成员  
pose  
机械臂位姿，geometry_msgs/Pose类型，x、y、z坐标（float类型，单位：m）+四元数（float类型）。  
speed  
速度百分比例系数，0~100。  
block  
是否为阻塞模式，bool类型，true:阻塞，false:非阻塞。  
### 圆弧运动Movec.msg
geometry_msgs/Pose pose_mid  
geometry_msgs/Pose pose_end  
uint8 speed  
bool block  
msg成员  
pose_mid  
中间位姿，geometry_msgs/Pose类型，x、y、z坐标（float类型，单位：m）+四元数。  
pose_end  
目标位姿，geometry_msgs/Pose类型，x、y、z坐标（float类型，单位：m）+四元数。  
speed  
速度百分比例系数，0~100。  
block  
是否为阻塞模式，bool类型，true:阻塞，false:非阻塞。  
### 关节空间规划到目标位姿Movejp.msg
geometry_msgs/Pose pose  
uint8 speed  
bool block  
msg成员  
pose  
目标位姿，geometry_msgs/Pose类型，x、y、z坐标（float类型，单位：m）+四元数。  
speed  
速度百分比例系数，0~100。  
block  
是否为阻塞模式，bool类型，true:阻塞，false:非阻塞。  
### 角度透传Jointpos.msg
float32[] joint  
bool follow  
float32 expand  
uint8 dof  
msg成员  
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
msg成员  
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
msg成员  
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
msg成员  
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
msg成员  
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
msg成员  
speed  
手爪力控夹取速度，unsigned int类型，范围：1~1000。  
force  
手爪夹取力矩阈值，unsigned int类型，范围 ：50~1000。  
block  
是否为阻塞模式，bool类型，true:阻塞，false:非阻塞。  
### 手爪力控夹取（持续力控夹取）Gripperpick.msg
uint16 speed  
uint16 force  
bool block  
msg成员  
speed  
手爪力控夹取速度，unsigned int类型，范围：1~1000。  
force  
手爪夹取力矩阈值，unsigned int类型，范围 ：50~1000。  
block  
是否为阻塞模式，bool类型，true:阻塞，false:非阻塞。  
### 手爪到达指定位置Gripperset.msg
uint16 position  
bool block  
msg成员  
position  
手爪目标位置，unsigned int类型，范围：1～1000,代表手爪开口度：0～70mm。  
block  
是否为阻塞模式，bool类型，true:阻塞，false:非阻塞。  
### 力位混合控制Setforceposition.msg
uint8 sensor  
uint8 mode  
uint8 direction  
int16 n  
bool block  
msg成员  
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
msg成员  
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
msg成员  
posture_num  
预先保存在灵巧手内的手势序号，范围：1~40。  
block  
是否为阻塞模式，bool类型，true:阻塞，false:非阻塞。  
### 设置灵巧手动作序列Handseq.msg
uint16 seq_num  
bool block  
msg成员  
seq_num	  
预先保存在灵巧手内的序列序号，范围：1~40。  
block  
是否为阻塞模式，bool类型，true:阻塞，false:非阻塞。  
### 设置灵巧手各自由度角度Handangle.msg
int16[6] hand_angle   
bool block  
msg成员  
hand_angle  
手指角度数组，范围：0~1000。另外，-1 代表该自由度不执行任何操作，保持当前状态。  
block  
是否为阻塞模式，bool类型，true:阻塞，false:非阻塞。  
### 设置灵巧手速度Handspeed.msg
uint16 hand_speed  
bool block  
msg成员  
hand_speed
手指速度，范围：1~1000。  
block  
是否为阻塞模式，bool类型，true:阻塞，false:非阻塞。  
### 设置灵巧手力阈值Handforce.msg
uint16 hand_force  
bool block  
msg成员  
hand_force  
手指力，范围：1~1000。  
block  
是否为阻塞模式，bool类型，true:阻塞，false:非阻塞。  
### 透传力位混合补偿（角度）Forcepositionmovejoint.msg
float32[] joint  
uint8 sensor  
uint8 mode  
int16 dir  
float32 force  
bool follow  
uint8 dof  
msg成员  
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
是否为阻塞模式，bool类型，true:阻塞，false:非阻塞。  
dof  
机械臂自由度信息。  
### 透传力位混合补偿（位姿）Forcepositionmovejoint.msg
geometry_msgs/Pose pose  
uint8 sensor  
uint8 mode  
int16 dir  
float32 force  
bool follow  
msg成员  
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
是否为阻塞模式，bool类型，true:阻塞，false:非阻塞。  
### 速度开环控制（升降机构）Liftspeed.msg
int16 speed  
bool block  
msg成员  
speed  
速度百分比，-100~100。Speed < 0:升降机构向下运动；Speed > 0:升降机构向上运动；Speed = 0:升降机构停止运动。  
block  
是否为阻塞模式，bool类型，true:阻塞，false:非阻塞。  
### 位置闭环控制（升降机构）Liftheight.msg
uint16 height  
uint16 speed  
bool block  
msg成员  
height  
目标高度，单位 mm，范围：0~2600。  
speed  
速度百分比，1~100。  
block  
是否为阻塞模式，bool类型，true:阻塞，false:非阻塞。  
### 获取升降机构状态（升降机构）Liftstate.msg
int16 height   
int16 current  
uint16 err_flag  
msg成员  
height  
当前升降机构高度，单位：mm，精度：1mm，范围：0~2300。  
current  
升降驱动错误代码，错误代码类型参考关节错误代码。  
### 查询（设置）UDP 机械臂状态主动上报配置Setrealtimepush.msg
uint16 cycle  
uint16 port  
uint16 force_coordinate  
string ip  
msg成员  
cycle  
设置广播周期，为5ms的倍数。  
port  
设置广播的端口号。  
force_coordinate  
系统外受力数据的坐标系，0 为传感器坐标系 1 为当前工作坐标系 2 为当前工具坐标系。  
ip  
自定义的上报目标IP 地址。  

主要为套用API实现的一些机械臂本体的功能，其详细介绍和使用在此不详细展开，可以通过专门的文档《[睿尔曼机械臂ROS2话题详细说明](https://github.com/kaola-zero/ros2_rm_robot/blob/main/rm_driver/doc/%E7%9D%BF%E5%B0%94%E6%9B%BC%E6%9C%BA%E6%A2%B0%E8%87%82ROS2rm_driver%E8%AF%9D%E9%A2%98%E8%AF%A6%E7%BB%86%E8%AF%B4%E6%98%8E.md)》进行查看。
