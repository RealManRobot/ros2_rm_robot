<div align="right">
  
[中文简体](https://github.com/RealManRobot/ros2_rm_robot/blob/foxy/rm_ros_interfaces/README_CN.md)|
[English](https://github.com/RealManRobot/ros2_rm_robot/blob/foxy/rm_ros_interfaces/README.md)

</div>

<div align="center">

# 睿尔曼机器人rm_ros_interface使用说明书V1.1


 


睿尔曼智能科技(北京)有限公司
 
文件修订记录：

|版本号 | 时间 | 备注 |
| :---: | :---- | :---: |
|V1.0 | 2024-2-18 | 拟制 |
|V1.1 | 2024-7-8  | 修订(添加示教消息) |

</div>

## 目录
* 1[rm_ros_interface功能包说明](#rm_ros_interface功能包说明)
* 2[rm_ros_interface功能包使用](#rm_ros_interface功能包使用)
* 3[rm_ros_interface功能包架构说明](#rm_ros_interface功能包架构说明)
* 3.1[功能包文件总览](#功能包文件总览)
* 4[rm_ros_interface消息说明](#rm_ros_interface消息说明)
* 4.1[关节错误代码Jointerrorcode_msg](#关节错误代码Jointerrorcode_msg)
* 4.2[清除关节错误代码Jointerrclear_msg](#清除关节错误代码Jointerrclear_msg)
* 4.3[所有坐标系名称Getallframe_msg](#所有坐标系名称Getallframe_msg)
* 4.4[关节运动Movej_msg](#关节运动Movej_msg)
* 4.5[直线运动Movel_msg](#直线运动Movel_msg)
* 4.6[圆弧运动Movec_msg](#圆弧运动Movec_msg)
* 4.7[关节空间规划到目标位姿Movejp_msg](#关节空间规划到目标位姿Movejp_msg)
* 4.8[关节示教Jointteach.msg](#关节示教Jointteach_msg)
* 4.9[位置示教Posteach.msg](#位置示教Posteach_msg)
* 4.10[姿态示教Ortteach.msg](#姿态示教Ortteach_msg)
* 4.11[角度透传Jointpos_msg](#角度透传Jointpos_msg)
* 4.12[位姿透传Cartepos_msg](#位姿透传Cartepos_msg)
* 4.13[机械臂当前状态-角度和欧拉角Armoriginalstate_msg](#机械臂当前状态-角度和欧拉角Armoriginalstate_msg)
* 4.14[机械臂当前状态-弧度和四元数Armstate_msg](#机械臂当前状态-弧度和四元数Armstate_msg)
* 4.15[读取软件版本号Armsoftversion_msg](#读取软件版本号Armsoftversion_msg)
* 4.16[手爪力控夹取Gripperpick_msg](#手爪力控夹取Gripperpick_msg)
* 4.17[手爪力控夹取-持续力控夹取Gripperpick_msg](#手爪力控夹取-持续力控夹取Gripperpick_msg)
* 4.18[手爪到达指定位置Gripperset_msg](#手爪到达指定位置Gripperset_msg)
* 4.19[力位混合控制Setforceposition_msg](#力位混合控制Setforceposition_msg)
* 4.20[六维力数据Sixforce_msg](#六维力数据Sixforce_msg)
* 4.21[设置灵巧手手势Handposture_msg](#设置灵巧手手势Handposture_msg)
* 4.22[设置灵巧手动作序列Handseq_msg](#设置灵巧手动作序列Handseq_msg)
* 4.23[设置灵巧手各自由度角度Handangle_msg](#设置灵巧手各自由度角度Handangle_msg)
* 4.24[设置灵巧手速度Handspeed_msg](#设置灵巧手速度Handspeed_msg)
* 4.25[设置灵巧手力阈值Handforce_msg](#设置灵巧手力阈值Handforce_msg)
* 4.26[透传力位混合补偿-角度Forcepositionmovejoint_msg](#透传力位混合补偿-角度Forcepositionmovejoint_msg)
* 4.27[透传力位混合补偿-位姿Forcepositionmovejoint_msg](#透传力位混合补偿-位姿Forcepositionmovejoint_msg)
* 4.28[速度开环控制-升降机构Liftspeed_msg](#速度开环控制-升降机构Liftspeed_msg)
* 4.29[位置闭环控制-升降机构Liftheight_msg](#位置闭环控制-升降机构Liftheight_msg)
* 4.30[获取升降机构状态-升降机构Liftstate_msg](#获取升降机构状态-升降机构Liftstate_msg)
* 4.31[查询或设置UDP机械臂状态主动上报配置Setrealtimepush_msg](#查询或设置UDP机械臂状态主动上报配置Setrealtimepush_msg)

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
│   ├── Handforce_msg
│   ├── Handposture_msg
│   ├── Handseq_msg
│   ├── Handspeed_msg
│   ├── Jointerrclear_msg
│   ├── Jointerrorcode75_msg
│   ├── Jointerrorcode_msg
│   ├── Jointpos75_msg
│   ├── Jointpos_msg
│   ├── Liftheight_msg
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
├── package.xml                       #依赖声明文件
└── src
```
## rm_ros_interface消息说明
### 关节错误代码Jointerrorcode_msg
```
uint16[] joint_error  
uint8 dof
```  
__msg成员__  
__uint16[] joint_error__  
每个关节报错信息。  
__uint8 dof__  
机械臂自由度信息。  
### 清除关节错误代码Jointerrclear_msg
```
uint8 joint_num  
bool block
```  
__msg成员__  
__joint_num__  
对应关节序号，从基座到机械臂手爪端，序号依次为1-6或1-7。  
__block__  
是否为阻塞模式，bool类型，true:阻塞，false:非阻塞。  
### 所有坐标系名称Getallframe_msg
```
string[10] frame_name
```  
__msg成员__  
__frame_name__  
返回的工作坐标系的名称数组。  
### 关节运动Movej_msg
```
float32[] joint  
uint8 speed  
bool block  
uint8 trajectory_connect  
uint8 dof 
``` 
__msg成员__  
__joint__  
关节角度，float类型，单位：弧度。  
__speed__  
速度百分比例系数，0-100。  
__block__  
是否为阻塞模式，bool类型，true:阻塞，false:非阻塞。  
__trajectory_connect__
#0 代表立即规划，1 代表和下一条轨迹一起规划，当为 1 时，轨迹不会立即执行  
__dof__  
机械臂自由度信息。  
### 直线运动Movel_msg
```
geometry_msgs/Pose pose  
uint8 speed  
uint8 trajectory_connect  
bool block
```  
__msg成员__  
__pose__  
机械臂位姿，geometry_msgs/Pose类型，x、y、z坐标（float类型，单位：m）+四元数（float类型）。  
__speed__  
速度百分比例系数，0-100。  
__trajectory_connect__
#0 代表立即规划，1 代表和下一条轨迹一起规划，当为 1 时，轨迹不会立即执行  
__block__  
是否为阻塞模式，bool类型，true:阻塞，false:非阻塞。  
### 圆弧运动Movec_msg
```
geometry_msgs/Pose pose_mid  
geometry_msgs/Pose pose_end  
uint8 speed  
uint8 trajectory_connect  
bool block  
uint8 loop
```  
__msg成员__  
__pose_mid__  
中间位姿，geometry_msgs/Pose类型，x、y、z坐标（float类型，单位：m）+四元数。  
__pose_end__  
目标位姿，geometry_msgs/Pose类型，x、y、z坐标（float类型，单位：m）+四元数。  
__speed__  
速度百分比例系数，0-100。  
__trajectory_connect__
#0 代表立即规划，1 代表和下一条轨迹一起规划，当为 1 时，轨迹不会立即执行
__block__  
是否为阻塞模式，bool类型，true:阻塞，false:非阻塞。  
__loop__
循环次数
### 关节空间规划到目标位姿Movejp_msg
```
geometry_msgs/Pose pose  
uint8 speed  
uint8 trajectory_connect
bool block
```  
__msg成员__  
__pose__  
目标位姿，geometry_msgs/Pose类型，x、y、z坐标（float类型，单位：m）+四元数。  
__speed__  
速度百分比例系数，0-100。  
__trajectory_connect__
#0 代表立即规划，1 代表和下一条轨迹一起规划，当为 1 时，轨迹不会立即执行  
__block__  
是否为阻塞模式，bool类型，true:阻塞，false:非阻塞。  
### 关节示教Jointteach_msg
```
uint8 num
uint8 direction
uint8 speed
bool block
```  
__msg成员__  
__num__  
示教关节的序号，1~7。
__direction__  
示教方向，0-负方向，1-正方向。
__speed__
速度比例1~100，即规划速度和加速度占关节最大线转速和加速度的百分比。  
__block__  
是否为阻塞模式，bool类型，true:阻塞，false:非阻塞。 
### 位置示教Posteach_msg
```
uint8 type
uint8 direction
uint8 speed
bool block
```  
__msg成员__  
__type__  
示教类型 输入0：X轴方向 | 1：Y轴方向 | 2：Z轴方向。
__direction__  
示教方向，0-负方向，1-正方向。
__speed__
速度比例1~100，即规划速度和加速度占关节最大线转速和加速度的百分比。  
__block__  
是否为阻塞模式，bool类型，true:阻塞，false:非阻塞。 
### 姿态示教Ortteach_msg
```
uint8 type
uint8 direction
uint8 speed
bool block
```  
__msg成员__  
__type__  
示教类型 输入0：RX轴方向 | 1：RY轴方向 | 2：RZ轴方向。
__direction__  
示教方向，0-负方向，1-正方向。
__speed__
速度比例1~100，即规划速度和加速度占关节最大线转速和加速度的百分比。  
__block__  
是否为阻塞模式，bool类型，true:阻塞，false:非阻塞。 
### 角度透传Jointpos_msg
```
float32[] joint  
bool follow  
float32 expand  
uint8 dof
```  
__msg成员__  
__joint__
关节角度，float类型，单位：弧度。  
__follow__  
跟随状态，bool类型，true高跟随，false低跟随，不设置默认高跟随。  
__expand__  
拓展关节，float类型，单位：弧度。  
__dof__  
机械臂自由度信息。  
### 位姿透传Cartepos_msg
```
geometry_msgs/Pose pose  
bool follow
```  
__msg成员__  
__pose__  
机械臂位姿，geometry_msgs/Pose类型，x、y、z坐标（float类型，单位：m）+四元数。  
__follow __ 
跟随状态，bool类型，true高跟随，false低跟随，不设置默认高跟随。  
### 机械臂当前状态-角度和欧拉角Armoriginalstate_msg
```
float32[] joint  
float32[6] pose  
uint16 arm_err  
uint16 sys_err  
uint8 dof
```  
__msg成员__  
__joint__  
关节角度，float类型，单位：度。  
__pose__  
机械臂当前位姿，float类型，x、y、z坐标，单位：m，x、y、z欧拉角，单位：度。  
__arm_err__  
机械臂运行错误代码，unsigned int类型。  
__arm_err__
控制器错误代码，unsigned int类型。  
__dof__  
机械臂自由度信息。  
### 机械臂当前状态-弧度和四元数Armstate_msg
```
float32[] joint  
geometry_msgs/Pose pose  
uint16 arm_err  
uint16 sys_err  
uint8 dof
```  
__msg成员__  
__joint__  
关节角度，float类型，单位：弧度。  
__pose__  
机械臂当前位姿，float类型，x、y、z坐标，单位：m，x、y、z、w四元数。  
__arm_err__  
机械臂运行错误代码，unsigned int类型。  
__arm_err__  
控制器错误代码，unsigned int类型。  
__dof__  
机械臂自由度信息。  
### 读取软件版本号Armsoftversion_msg
```
string planversion  
string ctrlversion  
string kernal1  
string kernal2  
string productversion
```  
__msg成员__  
__planversion__  
读取到的用户接口内核版本号，string类型。  
__ctrlversion__  
实时内核版本号，string类型。  
__kernal1__  
实时内核子核心 1 版本号，string类型。  
__kernal2__  
实时内核子核心 2 版本号，string类型。  
__productversion__  
机械臂型号，string类型。  
### 手爪力控夹取Gripperpick_msg
```
uint16 speed  
uint16 force  
bool block
```  
__msg成员__  
__speed__  
手爪力控夹取速度，unsigned int类型，范围：1-1000。  
__force__  
手爪夹取力矩阈值，unsigned int类型，范围 ：50-1000。  
__block__  
是否为阻塞模式，bool类型，true:阻塞，false:非阻塞。  
### 手爪力控夹取-持续力控夹取Gripperpick_msg
```
uint16 speed  
uint16 force  
bool block
```  
__msg成员__  
__speed__  
手爪力控夹取速度，unsigned int类型，范围：1-1000。  
__force__  
手爪夹取力矩阈值，unsigned int类型，范围 ：50-1000。  
__block__  
是否为阻塞模式，bool类型，true:阻塞，false:非阻塞。  
### 手爪到达指定位置Gripperset_msg
```
uint16 position  
bool block
```  
__msg成员__  
__position__  
手爪目标位置，unsigned int类型，范围：1-1000,代表手爪开口度：0-70mm。  
__block__  
是否为阻塞模式，bool类型，true:阻塞，false:非阻塞。  
### 力位混合控制Setforceposition_msg
```
uint8 sensor  
uint8 mode  
uint8 direction  
int16 n  
bool block
```  
__msg成员__  
__sensor__  
传感器；0-一维力；1-六维力。  
__mode__  
Mode：0-工作坐标系力控； 1-工具坐标系力控。  
__Direction__  
力控方向；0-沿X 轴；1-沿Y 轴；2-沿 Z 轴；3-沿RX 姿态方向；4-沿 RY 姿态方向；5-沿 RZ 姿态方向。  
__n__  
力的大小，单位 0.1N。  
__block__  
是否阻塞，true:阻塞，false:非阻塞。  
### 六维力数据Sixforce_msg
```
float32 force_fx  
float32 force_fy  
float32 force_fz  
float32 force_mx  
float32 force_my  
float32 force_mz
```  
__msg成员__  
__force_fx__  
沿x轴方向受力大小。  
__force_fy__  
沿y轴方向受力大小。  
__force_fz__
沿z轴方向受力大小。  
__force_mx__  
沿x轴方向转动受力大小。  
__force_my__  
沿y轴方向转动受力大小。  
__force_mz__  
沿z轴方向转动受力大小。  
### 设置灵巧手手势Handposture_msg
```
uint16 posture_num  
bool block
```  
__msg成员__  
__posture_num__  
预先保存在灵巧手内的手势序号，范围：1-40。  
__block__  
是否为阻塞模式，bool类型，true:阻塞，false:非阻塞。  
### 设置灵巧手动作序列Handseq_msg
```
uint16 seq_num  
bool block
```  
__msg成员__  
__seq_num__	  
预先保存在灵巧手内的序列序号，范围：1-40。  
__block__  
是否为阻塞模式，bool类型，true:阻塞，false:非阻塞。  
### 设置灵巧手各自由度角度Handangle_msg
```
int16[6] hand_angle   
bool block
```  
__msg成员__  
__hand_angle__  
手指角度数组，范围：0-1000。另外，-1 代表该自由度不执行任何操作，保持当前状态。  
__block__  
是否为阻塞模式，bool类型，true:阻塞，false:非阻塞。  
### 设置灵巧手速度Handspeed_msg
```
uint16 hand_speed  
bool block
```  
__msg成员__  
__hand_speed__
手指速度，范围：1-1000。  
__block__  
是否为阻塞模式，bool类型，true:阻塞，false:非阻塞。  
### 设置灵巧手力阈值Handforce_msg
```
uint16 hand_force  
bool block
```  
__msg成员__  
__hand_force__  
手指力，范围：1-1000。  
__block__  
是否为阻塞模式，bool类型，true:阻塞，false:非阻塞。  
### 透传力位混合补偿-角度Forcepositionmovejoint_msg
```
float32[] joint  
uint8 sensor  
uint8 mode  
int16 dir  
float32 force  
bool follow  
uint8 dof
```  
__msg成员__  
__joint__  
角度力位混合透传，单位：弧度。  
__sensor__  
所使用传感器类型，0-一维力，1-六维力。  
__mode__  
模式，0-沿工作坐标系，1-沿工具端坐标系。  
__dir__  
力控方向，0-5 分别代表 X/Y/Z/Rx/Ry/Rz，其中一维力类型时默认方向为Z 方向。  
__force__  
力的大小，精度 0.1N 或者 0.1Nm。  
__block__  
是否为阻塞模式，bool类型，true:阻塞，false:非阻塞。  
__dof__  
机械臂自由度信息。  
### 透传力位混合补偿-位姿Forcepositionmovejoint_msg
```
geometry_msgs/Pose pose  
uint8 sensor  
uint8 mode  
int16 dir  
float32 force  
bool follow
```  
__msg成员__  
__pose__  
机械臂位姿信息，x、y、z位置信息+四元数姿态信息。  
__sensor__  
所使用传感器类型，0-一维力，1-六维力。  
__mode__  
模式，0-沿工作坐标系，1-沿工具端坐标系。  
__dir__  
力控方向，0-5 分别代表 X/Y/Z/Rx/Ry/Rz，其中一维力类型时默认方向为Z 方向。  
__force__  
力的大小，精度 0.1N 或者 0.1Nm。  
__block__  
是否为阻塞模式，bool类型，true:阻塞，false:非阻塞。  
### 速度开环控制-升降机构Liftspeed_msg
```
int16 speed  
bool block
```  
__msg成员__  
__speed__  
速度百分比，-100-100。Speed < 0:升降机构向下运动；Speed > 0:升降机构向上运动；Speed = 0:升降机构停止运动。  
__block__  
是否为阻塞模式，bool类型，true:阻塞，false:非阻塞。  
### 位置闭环控制-升降机构Liftheight_msg
```
uint16 height  
uint16 speed  
bool block
```  
__msg成员__  
__height__  
目标高度，单位 mm，范围：0-2600。  
__speed__  
速度百分比，1-100。  
__block__  
是否为阻塞模式，bool类型，true:阻塞，false:非阻塞。  
### 获取升降机构状态-升降机构Liftstate_msg
```
int16 height   
int16 current  
uint16 err_flag
```  
__msg成员__  
__height__  
当前升降机构高度，单位：mm，精度：1mm，范围：0-2300。  
__current__  
升降驱动错误代码，错误代码类型参考关节错误代码。  
### 查询或设置UDP机械臂状态主动上报配置Setrealtimepush_msg
```
uint16 cycle  
uint16 port  
uint16 force_coordinate  
string ip
```  
__msg成员__  
__cycle__  
设置广播周期，为5ms的倍数。  
__port__  
设置广播的端口号。  
__force_coordinate__  
系统外受力数据的坐标系，0 为传感器坐标系 1 为当前工作坐标系 2 为当前工具坐标系。  
__ip__  
自定义的上报目标IP 地址。  

主要为套用API实现的一些机械臂本体的功能，其详细介绍和使用在此不详细展开，可以通过专门的文档《[睿尔曼机械臂ROS2话题详细说明](https://github.com/kaola-zero/ros2_rm_robot/blob/main/rm_driver/doc/%E7%9D%BF%E5%B0%94%E6%9B%BC%E6%9C%BA%E6%A2%B0%E8%87%82ROS2rm_driver%E8%AF%9D%E9%A2%98%E8%AF%A6%E7%BB%86%E8%AF%B4%E6%98%8E.md)》进行查看。
