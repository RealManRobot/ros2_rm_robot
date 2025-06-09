<div align="right">
  
[中文简体](https://github.com/RealManRobot/ros2_rm_robot/blob/humble/rm_driver/doc/%E7%9D%BF%E5%B0%94%E6%9B%BC%E6%9C%BA%E6%A2%B0%E8%87%82ROS2rm_driver%E8%AF%9D%E9%A2%98%E8%AF%A6%E7%BB%86%E8%AF%B4%E6%98%8E.md)|
[English](https://github.com/RealManRobot/ros2_rm_robot/blob/humble/rm_driver/doc/RealMan%20Robotic%20Arm%20rm_driver%20Topic%20Detailed%20Description%20(ROS2).md)

</div>

<div align="center">

# 睿尔曼机械臂接口函数说明(ROS2)V1.1.5


 


睿尔曼智能科技(北京)有限公司
 
文件修订记录：

|版本号 | 时间 | 备注 |
| :---: | :---- | :---: |
|V1.0 | 2024-2-18 | 拟制 |
|V1.1 | 2024-7-8  | 修订（添加示教指令3.6） |
|V1.1.1| 2024-8-13| 修订（添加查询六维力数据）|
|V1.1.2| 2024-9-25| 修订（修正坐标系话题描述错误）|
|V1.1.3| 2024-10-31|修订（添加灵巧手UDP功能，跟随功能）|
|V1.1.4| 2024-12-25|修订（修改UDP上报内容）|
|V1.1.5| 2025-02-19|修订（适配API2、添加末端生态协议接口、更新UDP接口）|


</div>

## 目录
* 1[简介](#简介)
* 2[报错说明](#报错说明)
* 2.1[控制器错误类型](#控制器错误类型)
* 2.2[关节错误类型](#关节错误类型)
* 2.3[API错误类型](#API错误类型)
* 3[ROS功能包机械臂相关指令使用说明](#ROS功能包机械臂相关指令使用说明)
* 3.1[关节配置](#关节配置)
* 3.1.1[清除关节错误代码](#清除关节错误代码)
* 3.2[工作坐标系设置](#工作坐标系设置)
* 3.2.1[切换当前工作坐标系](#切换当前工作坐标系)
* 3.3[坐标系查询](#坐标系查询)
* 3.3.1[查询当前工具坐标系](#查询当前工具坐标系)
* 3.3.2[查询所有工具坐标系名称](#查询所有工具坐标系名称)
* 3.3.3[查询当前工作坐标系](#查询当前工作坐标系)
* 3.3.4[查询所有工作坐标系](#查询所有工作坐标系)
* 3.4[机械臂状态查询](#机械臂状态查询)
* 3.4.1[获取机械臂当前状态-返回关节角度+欧拉角](#获取机械臂当前状态-返回各关节角度和欧拉角)
* 3.4.2[获取机械臂当前状态-返回各关节弧度+四元数](#获取机械臂当前状态-返回各关节弧度和四元数)
* 3.5[机械臂运动规划](#机械臂运动规划)
* 3.5.1[关节空间运动](#关节空间运动)
* 3.5.2[笛卡尔空间直线运动](#笛卡尔空间直线运动)
* 3.5.3[笛卡尔空间圆弧运动](#笛卡尔空间圆弧运动)
* 3.5.4[关节角度CANFD透传](#关节角度CANFD透传)
* 3.5.5[自定义高跟随模式关节角度CANFD透传](#自定义高跟随关节角度CANFD透传)
* 3.5.6[位姿CANFD透传](#位姿CANFD透传)
* 3.5.7[自定义高跟随模式位姿CANFD透传](#自定义高跟随模式位姿CANFD透传)
* 3.5.8[关节空间规划到目标位姿](#关节空间规划到目标位姿)
* 3.5.9[轨迹急停](#轨迹急停)
* 3.6[示教指令](#示教指令)
* 3.6.1[关节示教](#关节示教)
* 3.6.2[位置示教](#位置示教)
* 3.6.3[姿态示教](#姿态示教)
* 3.6.4[示教停止](#示教停止)
* 3.7[末端工具IO配置](#末端工具IO配置)
* 3.7.1[设置工具端电源输出](#设置工具端电源输出)
* 3.8[末端手爪控制](#末端手爪控制)
* 3.8.1[设置夹爪力控夹取](#设置夹爪力控夹取)
* 3.8.2[设置夹爪持续力控夹取](#设置夹爪持续力控夹取)
* 3.8.3[夹爪到达指定位置](#夹爪到达指定位置)
* 3.9[拖动示教及轨迹复现](#拖动示教及轨迹复现)
* 3.9.1[设置力位混合控制](#设置力位混合控制)
* 3.9.2[结束力位混合控制](#结束力位混合控制)
* 3.10[末端六维力传感器的使用](#末端六维力传感器的使用)
* 3.10.1[查询六维力数据](#查询六维力数据)
* 3.10.2[清空六维力数据](#清空六维力数据)
* 3.11[末端五指灵巧手控制](#末端五指灵巧手控制)
* 3.11.1[设置灵巧手手势序号](#设置灵巧手手势序号)
* 3.11.2[设置灵巧手动作序列](#设置灵巧手动作序列)
* 3.11.3[设置灵巧手各自由度角度](#设置灵巧手各自由度角度)
* 3.11.4[设置灵巧手速度](#设置灵巧手速度)
* 3.11.5[设置灵巧手力阈值](#设置灵巧手力阈值)
* 3.11.6[设置灵巧手角度跟随](#设置灵巧手角度跟随)
* 3.11.7[设置灵巧手姿态跟随](#设置灵巧手姿态跟随)
* 3.12[升降机构](#升降机构)
* 3.12.1[升降机构速度开环控制](#升降机构速度开环控制)
* 3.12.2[升降机构位置闭环控制](#升降机构位置闭环控制)
* 3.12.3[获取升降机构状态](#获取升降机构状态)
* 3.13[末端生态协议](#末端生态协议)
* 3.13.1[设置末端生态协议模式](#设置末端生态协议模式)
* 3.13.2[查询末端生态协议模式](#查询末端生态协议模式)
* 3.13.3[设置触觉传感器模式](#设置触觉传感器模式)
* 3.13.4[获取触觉传感器模式](#获取触觉传感器模式)
* 3.14[透传力位混合控制补偿](#透传力位混合控制补偿)
* 3.14.1[开启透传力位混合控制补偿模式](#开启透传力位混合控制补偿模式)
* 3.14.2[关闭透传力位混合控制补偿模式](#关闭透传力位混合控制补偿模式)
* 3.14.3[透传力位混合补偿-关节](#透传力位混合补偿-关节)
* 3.14.4[透传力位混合补偿-位姿](#透传力位混合补偿-位姿)
* 3.15[机械臂状态主动上报](#机械臂状态主动上报)
* 3.15.1[设置UDP机械臂状态主动上报配置](#设置UDP机械臂状态主动上报配置)
* 3.15.2[查询UDP机械臂状态主动上报配置](#查询UDP机械臂状态主动上报配置)
* 3.15.3[UDP机械臂状态主动上报](#UDP机械臂状态主动上报)

 
## 简介
为了方便用户通过ROS2控制机械臂，睿尔曼提供了基于API的ROS2功能包，有关机械臂的控制细节想要了解的话也可以参考API的相关文档和说明，在实际使用机械臂时，用户可通过以太网口与机械臂建立通信，并控制机械臂。
## 报错说明
### 控制器错误类型
| 序号 | 错误代码(16进制) | 错误内容 |
| :---: | :---- | :---: |
| 1 | 0x0000 | 系统正常 |
| 2 | 0x1001 | 关节通信异常 |
| 3 | 0x1002 | 目标角度超过限位 |
| 4 | 0x1003 | 该处不可达，为奇异点 |
| 5 | 0x1004 | 实时内核通信错误 |
| 6 | 0x1005 | 关节通信总线错误 |
| 7 | 0x1006 | 规划层内核错误 |
| 8 | 0x1007 | 关节超速 |
| 9 | 0x1008 | 末端接口板无法连接 |
| 10 | 0x1009 | 超速度限制 |
| 11 | 0x100A | 超加速度限制 |
| 12 | 0x100B | 关节抱闸未打开 |
| 13 | 0x100C | 拖动示教时超速 |
| 14 | 0x100D | 机械臂发生碰撞 |
| 15 | 0x100E | 无该工作坐标系 |
| 16 | 0x100F | 无该工具坐标系 |
| 17 | 0x1010 | 关节发生掉使能错误 |
### 关节错误类型

| 序号 | 错误代码(16进制) | 错误内容 |
| :---: | :---- | :---: |
| 1 | 0x0000 | 关节正常 |
| 2 | 0x0001 | FOC错误 |
| 3 | 0x0002 | 过压 |
| 4 | 0x0004 | 欠压 |
| 5 | 0x0008 | 过温 |
| 6 | 0x0010 | 启动失败 |
| 7 | 0x0020 | 编码器错误 |
| 8 | 0x0040 | 过流 |
| 9 | 0x0080 | 软件错误 |
| 10 | 0x0100 | 温度传感器错误 |
| 11 | 0x0200 | 位置超限错误 |
| 12 | 0x0400 | 关节ID非法 |
| 13 | 0x0800 | 位置跟踪错误 |
| 14 | 0x1000 | 电流检测错误 |
| 15 | 0x2000 | 抱闸打开失败 |
| 16 | 0x4000 | 位置指令阶跃警告 |
| 17 | 0x8000 | 多圈关节丢圈数 |
| 18 | 0xF000 | 通信丢帧 |
### API错误类型

| 错误代码(int) | 说明 | 处理建议 |
| :---: | :---- | :--- |
| 0 | 系统运行正常 | - |
| 1 | 消息请求返回FALSE | - 校验JSON指令：<br>①启用API的DEBUG日志，捕获原始JSON数据。<br>②检查JSON语法：确保括号、引号、逗号等格式正确（可借助JSON校验工具）。<br>③对照API文档，验证参数名称、数据类型及取值范围是否符合规范。<br>④修正问题后重新发送指令，检查控制器返回的状态码及业务数据是否正常。<br>- 检查机械臂状态：<br>①查看机械臂控制器或日志中的实时报错信息（如硬件故障、超限等），根据提示复位、校准或排查硬件问题。<br>②修正问题后重新发送指令，检查控制器返回的状态码及业务数据是否正常。|
| -1 | 数据发送失败，通信过程中出现问题 | 检查网络连通性：<br>使用ping/telnet等工具检测与控制器的通信链路是否正常。|
| -2 | 数据接收失败，通信过程中出现问题或者控制器超时没有返回。|- 检查网络连通性：<br>使用ping/telnet等工具检测与控制器的通信链路是否正常。<br>- 校验版本兼容性：<br>①核对控制器固件版本是否支持当前API功能，具体版本配套关系请参考版本变更说明。<br>②若版本过低需升级控制器或使用适配的API版本。<br>- 调用ModbusTCP接口：仅在读写控制器ModbusTCP设备时适用，创建机械臂控制句柄后，必须调用rm_set_modbustcp_mode()接口，否则无法接收到返回值。|
| -3 | 返回值解析失败，接收到的数据格式不正确或不完整。|校验版本兼容性：<br>①核对控制器固件版本是否支持当前API功能，具体版本配套关系请参考[版本变更说明](#https://develop.realman-robotics.com/robot/releaseNotes/releaseNotes/)。<br>②若版本过低需升级控制器或使用适配的API版本。|
| -4 | 当前到位设备校验失败，即当前到位设备不为关节/升降机构/夹爪/灵巧手。| - 检测多设备并发控制：检查是否有其他设备给机械臂发送运动指令：包括机械臂、夹爪、灵巧手、升降机的运动；<br>- 实时监听指令事件：注册回调函数 rm_get_arm_event_call_back：<br>①捕获设备到位事件（如运动完成、超时等）；<br>②通过回调参数 device 判断触发事件的具体设备类型 |
| -5 | 单线程阻塞模式超时未接收到返回，请确保超时时间设置合理。| - 检查超时时长设置：单线程阻塞模式下，支持配置等待设备运动完成的超时时间，务必确保设置超时时间大于设备运动时间；<br>- 检查网络连通性：<br>使用ping/telnet等工具检测与控制器的通信链路是否正常。|
## ROS功能包机械臂相关指令使用说明
该部分介绍如何来通过ROS话题查询和控制机械臂。
### 关节配置
#### 清除关节错误代码

| 功能描述 | 清除关节错误代码 |
| :---: | :---- |
| 参数说明 | Jointerrclear.msg<br>uint8 joint_num：对应关节序号，从基座到机械臂手爪端，六自由度序号依次为1～6，七自由度序号依次为1～7。 |
| 命令示例 | ros2 topic pub /rm_driver/set_joint_err_clear_cmd rm_ros_interfaces/msg/Jointerrclear "joint_num: 1 " |
| 返回值 | true-设置成功，false-设置失败 |
| 返回查询示例 | ros2 topic echo /rm_driver/set_joint_err_clear_result |

### 工作坐标系设置
#### 切换当前工作坐标系
| 功能描述 | 切换当前工作坐标系 |
| :---: | :---- |
| 参数说明 | ROS自带msg std_msgs::msg::String |
| 命令示例 | ros2 topic pub /rm_driver/change_work_frame_cmd std_msgs/msg/String "data: 'Base'" |
| 返回值 | true-设置成功，false-设置失败 |
| 返回查询示例 | ros2 topic echo /rm_driver/change_work_frame_result |
### 坐标系查询
#### 查询当前工具坐标系
| 功能描述 | 查询当前工具坐标系 |
| :---: | :---- |
| 参数说明 | ROS自带msg std_msgs::msg::Empty |
| 命令示例 | ros2 topic pub --once /rm_driver/get_current_tool_frame_cmd std_msgs/msg/Empty "{}" |
| 返回值 | 当前工具坐标系名称 |
| 返回查询示例 | ros2 topic echo /rm_driver/get_current_tool_frame_result |
#### 查询所有工具坐标系名称
| 功能描述 | 查询所有工具坐标系 |
| :---: | :---- |
| 参数说明 | ROS自带msg std_msgs::msg::Empty |
| 命令示例 | ros2 topic pub /rm_driver/get_all_tool_frame_cmd std_msgs/msg/Empty "{}" |
| 返回值 | 当前工具坐标系所有名称 |
| 返回查询示例 | ros2 topic echo /rm_driver/get_all_tool_frame_result |
#### 查询当前工作坐标系
| 功能描述 | 查询当前工作坐标系 |
| :---: | :---- |
| 参数说明 | ROS自带msg std_msgs::msg::Empty |
| 命令示例 | ros2 topic pub --once /rm_driver/get_curr_workFrame_cmd std_msgs/msg/Empty "{}" |
| 返回值 | true-设置成功，false-设置失败 |
| 返回查询示例 | ros2 topic echo /rm_driver/get_curr_workFrame_result |
#### 查询所有工作坐标系
| 功能描述 | 查询所有工作坐标系 |
| :---: | :---- |
| 参数说明 | ROS自带msg std_msgs::msg::Empty |
| 命令示例 | ros2 topic pub --once /rm_driver/get_all_work_frame_cmd std_msgs/msg/Empty "{}" |
| 返回值 | 所有工作坐标系名称 |
| 返回查询示例 | ros2 topic echo /rm_driver/get_all_work_frame_result |
### 机械臂状态查询
#### 获取机械臂当前状态-返回各关节角度和欧拉角
| 功能描述 | 获取机械臂当前状态 |
| :---: | :---- |
| 参数说明 | ROS自带msg std_msgs::msg::Empty |
| 命令示例 | ros2 topic pub --once /rm_driver/get_current_arm_state_cmd std_msgs/msg/Empty "{}" |
| 返回值 | 当前机械臂关节状态(角度)+位姿信息(欧拉角)+报错信息 |
| 返回查询示例 | ros2 topic echo /rm_driver/get_current_arm_original_state_result |
#### 获取机械臂当前状态-返回各关节弧度和四元数
| 功能描述 | 获取机械臂当前状态 |
| :---: | :---- |
| 参数说明 | ROS自带msg std_msgs::msg::Empty |
| 命令示例 | ros2 topic pub --once /rm_driver/get_current_arm_state_cmd std_msgs/msg/Empty "{}" |
| 返回值 | 当前机械臂关节状态(弧度)+位姿信息(四元数)+报错信息 |
| 返回查询示例 | ros2 topic echo /rm_driver/get_current_arm_state_result |
### 机械臂运动规划
#### 关节空间运动
| 功能描述 | 关节空间运动MOVEJ |
| :---: | :---- |
| 参数说明 | Movej.msg<br>float32[6] joint：关节角度，单位：弧度。<br>uint8 speed：速度百分比例系数，0~100。<br>bool block：是否为阻塞模式，bool类型，true:阻塞，false:非阻塞。 |
| 命令示例 | 六自由度<br>ros2 topic pub --once /rm_driver/movej_cmd rm_ros_interfaces/msg/Movej "joint: [0, 0, 0, 0, 0, 0]<br>speed: 20<br>block: true <br>dof: 6"<br>七自由度<br>ros2 topic pub --once /rm_driver/movej_cmd rm_ros_interfaces/msg/Movej "joint: [0, 0, 0, 0, 0, 0, 0]<br>speed: 20<br>block: true<br>trajectory_connect: 0<br>dof: 7" |
| 返回值 | 成功返回：true；失败返回：false，driver终端返回错误码。 |
| 返回查询示例 | ros2 topic echo /rm_driver/movej_result |
#### 笛卡尔空间直线运动
| 功能描述 | 笛卡尔空间直线运动MOVEL |
| :---: | :---- |
| 参数说明 | Movel.msg<br>geometry_msgs/Pose pose：机械臂位姿，geometry_msgs/Pose类型，x、y、z坐标(float类型，单位：m)+四元数。<br>uint8 speed：速度百分比例系数，0~100。<br>bool block：是否为阻塞模式，bool类型，true:阻塞，false:非阻塞。 |
| 命令示例 | 先使用MoveJP<br>ros2 topic pub --once /rm_driver/movej_p_cmd rm_ros_interfaces/msg/Movejp "pose:<br>  position:<br>    x: -0.317239<br>    y: 0.120903<br>    z: 0.255765<br>  orientation:<br>    x: -0.983404<br>    y: -0.178432<br>    z: 0.032271<br>    w: 0.006129<br>speed: 20<br>block: true"<br>后使用MoveL<br>ros2 topic pub --once /rm_driver/movel_cmd rm_ros_interfaces/msg/Movel "pose:<br>  position:<br>    x: -0.317239<br>    y: 0.120903<br>    z: 0.295765<br>  orientation:<br>    x: -0.983404<br>    y: -0.178432<br>    z: 0.032271<br>    w: 0.006129<br>speed: 20<br>trajectory_connect: 0<br>block: true" |
| 返回值 | 成功返回：true；失败返回：false，driver终端返回错误码。 |
| 返回查询示例 | ros2 topic echo /rm_driver/movel_result |
#### 笛卡尔空间圆弧运动
| 功能描述 | 笛卡尔空间圆弧运动MOVEC |
| :---: | :---- |
| 参数说明 | Movec.msg<br>geometry_msgs/Pose pose_mid：中间位姿，geometry_msgs/Pose类型，x、y、z坐标(float类型，单位：m)+四元数。<br>geometry_msgs/Pose pose_end：终点位姿，geometry_msgs/Pose类型，x、y、z坐标(float类型，单位：m)+四元数。<br>uint8 speed：速度百分比例系数，0~100。<br>bool block：是否为阻塞模式，bool类型，true:阻塞，false:非阻塞。 |
| 命令示例 | 首先使用movej_p到达指定位置<br>ros2 topic pub --once /rm_driver/movej_p_cmd rm_ros_interfaces/msg/Movejp "pose:<br>  position:<br>    x: 0.274946<br>    y: -0.058786<br>    z: 0.299028<br>  orientation:<br>    x: 0.7071<br>    y: -0.7071<br>    z: 0.0<br>w: 0.0<br>speed: 0<br>block: true"<br>使用movec到达指定位置<br>ros2 topic pub --once /rm_driver/movec_cmd rm_ros_interfaces/msg/Movec "pose_mid:<br>  position:<br>    x: 0.324946<br>    y: -0.008786<br>    z: 0.299028<br>  orientation:<br>    x: 0.7071<br>    y: -0.7071<br>    z: 0.0<br>    w: 0.0<br>pose_end:<br>  position:<br>    x: 0.274946<br>    y: 0.041214<br>    z: 0.299028<br>  orientation:<br>    x: 0.7071<br>    y: -0.7071<br>    z: 0.0<br>    w: 0.0<br>speed: 20<br>trajectory_connect: 0<br>block: false<br>loop: 0" |
| 返回值 | 成功返回：true；失败返回：false，driver终端返回错误码。 |
| 返回查询示例 | ros2 topic echo /rm_driver/movec_result |
#### 关节角度CANFD透传
| 功能描述 | 关节角度CANFD透传 |
| :---: | :---- |
| 参数说明 | Jointpos.msg<br>float32[6] joint：关节角度，单位：弧度。<br>bool follow：跟随状态，true高跟随，false低跟随，不设置默认高跟随。<br>float32 expand：拓展关节，单位：弧度。 |
| 命令示例 | 透传需要连续发送多个连续的点实现，单纯靠以下命令并不能实现功能，当前moveit2控制使用了角度透传的控制方式。<br>ros2 topic pub /rm_driver/movej_canfd_cmd rm_ros_interfaces/msg/Jointpos "joint: [0, 0, 0, 0, 0, 0]<br>follow: false<br>expand: 0.0<br>dof: 6" |
| 返回值 | 成功：无返回值；失败返回：driver终端返回错误码。 |
#### 自定义高跟随模式关节角度CANFD透传
| 功能描述 | 自定义高跟随模式关节角度CANFD透传 |
| :---: | :---- |
| 参数说明 | Jointposcustom.msg<br>float32[6] joint：关节角度，单位：弧度。<br>bool follow：跟随状态，true高跟随，false低跟随，不设置默认高跟随。<br>float32 expand：拓展关节，单位：弧度。<br>uint8 trajectory_mode: 高跟随模式下，支持多种模式，0-完全透传模式、1-曲线拟合模式、2-滤波模式。<br>uint8 radio: 设置曲线拟合模式下平滑系数（范围0-100）或者滤波模式下的滤波参数（范围0-1000），数值越大表示平滑效果越好 |
| 命令示例 | 透传需要连续发送多个连续的点实现，单纯靠以下命令并不能实现功能，当前moveit2控制使用了角度透传的控制方式。<br>ros2 topic pub /rm_driver/movej_canfd_custom_cmd rm_ros_interfaces/msg/Jointposcustom "joint: [0, 0, 0, 0, 0, 0]<br>follow: false<br>expand: 0.0<br>trajectory_mode: 0<br>radio: 0<br>dof: 6" |
| 返回值 | 成功：无返回值；失败返回：driver终端返回错误码。 |
	
#### 位姿CANFD透传
| 功能描述 | 位姿CANFD透传 |
| :---: | :---- |
| 参数说明 | Cartepos.msg<br>geometry_msgs/Pose pose：透传位姿，geometry_msgs/Pose类型，x、y、z坐标(float类型，单位：m)+四元数。<br>bool follow：跟随状态，true高跟随，false低跟随，不设置默认高跟随。 |
| 命令示例 | 需要是大量(10个以上)位置连续 的点，单纯靠以下命令并不能实现功能，以2ms以上的周期持续发布。<br>ros2 topic pub /rm_driver/movep_canfd_cmd rm_ros_interfaces/msg/Cartepos "pose:<br>  position:<br>    x: 0.0<br>    y: 0.0<br>    z: 0.0<br>  orientation:<br>    x: 0.0<br>    y: 0.0<br>    z: 0.0<br>    w: 1.0<br>follow: false" |
| 返回值 | 成功：无返回值；失败返回：driver终端返回错误码。 |
#### 自定义高跟随模式位姿CANFD透传
| 功能描述 | 自定义高跟随模式位姿CANFD透传 |
| :---: | :---- |
| 参数说明 | Carteposcustom.msg<br>geometry_msgs/Pose pose：透传位姿，geometry_msgs/Pose类型，x、y、z坐标(float类型，单位：m)+四元数。<br>bool follow：跟随状态，true高跟随，false低跟随，不设置默认高跟随。 <br>uint8 trajectory_mode: 高跟随模式下，支持多种模式，0-完全透传模式、1-曲线拟合模式、2-滤波模式。<br>uint8 radio: 设置曲线拟合模式下平滑系数（范围0-100）或者滤波模式下的滤波参数（范围0-1000），数值越大表示平滑效果越好|
| 命令示例 | 需要是大量(10个以上)位置连续 的点，单纯靠以下命令并不能实现功能，以2ms以上的周期持续发布。<br>ros2 topic pub /rm_driver/movep_canfd_custom_cmd rm_ros_interfaces/msg/Carteposcustom  <br>"pose:<br>  position:<br>    x: 0.0<br>    y: 0.0<br>    z: 0.0<br>  orientation:<br>    x: 0.0<br>    y: 0.0<br>    z: 0.0<br>    w: 1.0<br>follow: false<br> trajectory_mode: 0<br>radio: 0" |
| 返回值 | 成功：无返回值；失败返回：driver终端返回错误码。 |
#### 关节空间规划到目标位姿
| 功能描述 | 关节空间规划到目标位姿MOVEJP |
| :---: | :---- |
| 参数说明 | Movejp.msg<br>geometry_msgs/Pose pose：目标位姿，x、y、z坐标(float类型，单位：m)+四元数。<br>uint8 speed：速度百分比例系数，0~100。<br>bool block：是否为阻塞模式，true:阻塞，false:非阻塞。 |
| 命令示例 | ros2 topic pub --once /rm_driver/movej_p_cmd rm_ros_interfaces/msg/Movejp "pose:<br>  position:<br>    x: -0.317239<br>    y: 0.120903<br>    z: 0.255765<br>  orientation:<br>    x: -0.983404<br>    y: -0.178432<br>    z: 0.032271<br>    w: 0.006129<br>speed: 20<br>block: true" |
| 返回值 | 成功返回：true；失败返回：false，driver终端返回错误码。 |
| 返回查询示例 | ros2 topic echo /rm_driver/movej_p_result |
#### 轨迹急停
| 功能描述 | 运动规划轨迹急停 |
| :---: | :---- |
| 参数说明 | ROS官方msg std_msgs::msg::Empty |
| 命令示例 | ros2 topic pub /rm_driver/move_stop_cmd std_msgs/msg/Empty "{}" |
| 返回值 | 成功返回：true；失败返回：false，driver终端返回错误码。 |
| 返回查询示例 | ros2 topic echo /rm_driver/move_stop_result |
### 示教指令
#### 关节示教
| 功能描述 | 关节示教 |
| :---: | :---- |
| 参数说明 | Jointteach.msg<br>uint8 num:示教关节的序号，1~7<br>uint8 direction:示教方向，0-负方向，1-正方向<br>uint8 speed:速度百分比例系数，0~100 |
| 命令示例 | ros2 topic pub /rm_driver/set_joint_teach_cmd rm_ros_interfaces/msg/Jointteach "num: 1<br>direction: 0<br>speed: 10" |
| 返回值 | 成功返回：true；失败返回：false，driver终端返回错误码。 |
| 返回查询示例 | ros2 topic echo /rm_driver/set_joint_teach_result |
#### 位置示教
| 功能描述 | 位置示教 |
| :---: | :---- |
| 参数说明 | Posteach.msg<br>uint8 type：示教类型 输入0X轴方向、1Y轴方向、2Z轴方向<br>uint8 direction:示教方向，0-负方向，1-正方向<br>uint8 speed:速度百分比例系数，0~100。|
| 命令示例 | ros2 topic pub /rm_driver/set_pos_teach_cmd rm_ros_interfaces/msg/Posteach "type: 2<br>direction: 0<br>speed: 10" |
| 返回值 | 成功返回：true；失败返回：false，driver终端返回错误码。 |
| 返回查询示例 | ros2 topic echo /rm_driver/set_pos_teach_result |
#### 姿态示教
| 功能描述 | 姿态示教 |
| :---: | :---- |
| 参数说明 | Ortteach.msg.msg<br>uint8 type：示教类型 输入0RX轴方向、1RY轴方向、2RZ轴方向<br>uint8 direction:示教方向，0-负方向，1-正方向<br>uint8 speed:速度百分比例系数，0~100。|
| 命令示例 | ros2 topic pub /rm_driver/set_ort_teach_cmd rm_ros_interfaces/msg/Ortteach "type: 2<br>direction: 0<br>speed: 10" |
| 返回值 | 成功返回：true；失败返回：false，driver终端返回错误码。 |
| 返回查询示例 | ros2 topic echo /rm_driver/set_ort_teach_result |
#### 示教停止
| 功能描述 | 示教停止 |
| :---: | :---- |
| 参数说明 | ROS官方msg std_msgs::msg::Empty|
| 命令示例 | ros2 topic pub /rm_driver/set_stop_teach_cmd std_msgs/msg/Empty "{}" |
| 返回值 | 成功返回：true；失败返回：false，driver终端返回错误码。 |
| 返回查询示例 | ros2 topic echo /rm_driver/set_stop_teach_result |
### 末端工具IO配置
#### 设置工具端电源输出
| 功能描述 | 设置工具端电源输出 |
| :---: | :---- |
| 参数说明 | ROS自带消息文件：std_msgs::msg::UInt16<br>uint16 data：电源输出类型，范围：0~3   0-0V，1-5V，2-12V，3-24V |
| 命令示例 | ros2 topic pub --once /rm_driver/set_tool_voltage_cmd std_msgs/msg/UInt16 "data: 0" |
| 返回值 | 成功返回：true；失败返回：false，driver终端返回错误码。 |
| 返回查询示例 | ros2 topic echo /rm_driver/set_tool_voltage_result |
### 末端手爪控制
睿尔曼机械臂末端配备了因时机器人公司的EG2-4C2手爪，为了便于用户操作手爪，机械臂控制器对用户适配了手爪的ROS控制方式
#### 设置夹爪力控夹取
| 功能描述 | 设置夹爪力控夹取 |
| :---: | :---- |
| 参数说明 | Gripperpick.msg<br>uint16 speed：1～1000,代表手爪开合速度，无量纲。<br>uint16 force：1～1000,代表手爪夹持力，最大1.5kg。<br>bool block：是否为阻塞模式，true:阻塞，false:非阻塞。 |
| 命令示例 | ros2 topic pub --once /rm_driver/set_gripper_pick_cmd rm_ros_interfaces/msg/Gripperpick "speed: 200<br>force: 200<br>block: true<br>timeout: 1000" |
| 返回值 | 成功返回：true；失败返回：false，driver终端返回错误码。 |
| 返回查询示例 | ros2 topic echo /rm_driver/set_gripper_pick_result |
#### 设置夹爪持续力控夹取
| 功能描述 | 设置夹爪持续力控夹取 |
| :---: | :---- |
| 参数说明 | Gripperpick.msg<br>uint16 speed：1～1000,代表手爪开合速度，无量纲。<br>uint16 force：1～1000,代表手爪夹持力，最大1.5kg。<br>bool block：是否为阻塞模式，true:阻塞，false:非阻塞。 |
| 命令示例 | ros2 topic pub --once /rm_driver/set_gripper_pick_on_cmd rm_ros_interfaces/msg/Gripperpick "speed: 200<br>force: 200<br>block: true<br>timeout: 1000" |
| 返回值 | 成功返回：true；失败返回：false，driver终端返回错误码。 |
| 返回查询示例 | ros2 topic echo /rm_driver/set_gripper_pick_on_result |
#### 夹爪到达指定位置
| 功能描述 | 夹爪到达指定位置 |
| :---: | :---- |
| 参数说明 | Gripperset.msg<br>uint16 position：手爪目标位置，范围：1～1000,代表手爪开口度：0～70mm<br>bool block：是否为阻塞模式，true:阻塞，false:非阻塞。 |
| 命令示例 | ros2 topic pub --once /rm_driver/set_gripper_position_cmd rm_ros_interfaces/msg/Gripperset "position: 500<br>block: true<br>timeout: 1000" |
| 返回值 | 成功返回：true；失败返回：false，driver终端返回错误码。 |
| 返回查询示例 | ros2 topic echo /rm_driver/set_gripper_position_result |
### 拖动示教及轨迹复现
#### 设置力位混合控制
| 功能描述 | 设置力位混合控制 |
| :---: | :---- |
| 参数说明 | Setforceposition.msg<br>uint8 sensor: 0-一维力；1-六维力<br>uint8 mode: 0-基坐标系力控；1-工具坐标系力控<br>uint8 direction: 力控方向；0-沿X轴；1-沿Y轴；2-沿Z轴；3-沿RX姿态方向；4-沿RY姿态方向；5-沿RZ姿态方向<br>int16 n: 力的大小，单位N，精确到0.1N |
| 命令示例 | ros2 topic pub --once /rm_driver/set_force_postion_cmd rm_ros_interfaces/msg/Setforceposition "sensor: 1<br>mode: 0<br>direction: 2<br>n: 3" |
| 返回值 | 成功返回：true；失败返回：false，driver终端返回错误码。 |
| 返回查询示例 | ros2 topic echo /rm_driver/set_force_postion_result |
#### 结束力位混合控制
| 功能描述 | 结束力位混合控制 |
| :---: | :---- |
| 参数说明 | std_msgs::msg::Empty |
| 命令示例 | ros2 topic pub /rm_driver/stop_force_postion_cmd std_msgs/msg/Empty "{}" |
| 返回值 | 成功返回：true；失败返回：false，driver终端返回错误码。 |
| 返回查询示例 | ros2 topic echo /rm_driver/clear_force_data_result |
### 末端六维力传感器的使用
睿尔曼RM-65F机械臂末端配备集成式六维力传感器，无需外部走线，用户可直接通过ROS话题对六维力进行操作。
#### 查询六维力数据
| 功能描述 | 查询六维力数据 |
| :---: | :---- |
| 参数说明 | std_msgs::msg::Empty |
| 命令示例 | ros2 topic pub rm_driver/get_force_data_cmd std_msgs/msg/Empty "{}" |
| 返回值 | 成功返回对应坐标系六维力数据。 |
| 返回查询示例 | ros2 topic echo /rm_driver/get_force_data_result原始数据<br>ros2 topic echo /rm_driver/get_zero_force_data_result系统受力数据<br>ros2 topic echo /rm_driver/get_work_force_data_result工作坐标系受力数据<br>ros2 topic echo /rm_driver/get_tool_force_data_result工具坐标系受力数据 |
#### 清空六维力数据
| 功能描述 | 清空六维力数据 |
| :---: | :---- |
| 参数说明 | std_msgs::msg::Empty |
| 命令示例 | ros2 topic pub /rm_driver/clear_force_data_cmd std_msgs/msg/Empty "{}" |
| 返回值 | 成功返回：true；失败返回：false，driver终端返回错误码。 |
| 返回查询示例 | ros2 topic echo /rm_driver/clear_force_data_result |
### 末端五指灵巧手控制
睿尔曼RM-65机械臂末端配备了五指灵巧手，可通过ROS对灵巧手进行设置。
#### 设置灵巧手手势序号
| 功能描述 | 设置灵巧手手势序号 |
| :---: | :---- |
| 参数说明 | Handposture.msg<br>uint16 posture_num：预先保存在灵巧手内的手势序号，范围：1~40。<br>bool data：是否为阻塞模式，true:阻塞，false:非阻塞。<br>uint16 timeout :  timeout 阻塞模式下超时时间设置，单位：秒 |
| 命令示例 | ros2 topic pub --once /rm_driver/set_hand_posture_cmd rm_ros_interfaces/msg/Handposture "posture_num: 1<br>block: true<br>timeout: 1000" |
| 返回值 | 成功返回：true；失败返回：false，driver终端返回错误码。 |
| 返回查询示例 | ros2 topic echo /rm_driver/set_hand_posture_result |
#### 设置灵巧手动作序列
| 功能描述 | 设置灵巧手动作序列 |
| :---: | :---- |
| 参数说明 | Handseq.msg<br>uint16 seq_num：预先保存在灵巧手内的序列序号，范围：1~40。<br>bool data：是否为阻塞模式，true:阻塞，false:非阻塞。 <br>uint16 timeout :  timeout 阻塞模式下超时时间设置，单位：秒  |
| 命令示例 | ros2 topic pub --once /rm_driver/set_hand_seq_cmd rm_ros_interfaces/msg/Handseq "seq_num: 1<br>block: true<br>timeout: 1000" |
| 返回值 | 成功返回：true；失败返回：false，driver终端返回错误码。 |
| 返回查询示例 | ros2 topic echo /rm_driver/set_hand_seq_result |
#### 设置灵巧手各自由度角度
| 功能描述 | 设置灵巧手各自由度角度 |
| :---: | :---- |
| 参数说明 | Handangle.msg<br>int16[6] hand_angle：手指角度数组，范围：0~1000.另外，-1 代表该自由度不执行任何操作，保持当前状态。<br>bool data：是否为阻塞模式，true:阻塞，false:非阻塞。 |
| 命令示例 | ros2 topic pub --once /rm_driver/set_hand_angle_cmd rm_ros_interfaces/msg/Handangle "hand_angle:<br>- 0<br>- 0<br>- 0<br>- 0<br>- 0<br>- 0<br>block: true" |
| 返回值 | 成功返回：true；失败返回：false，driver终端返回错误码。 |
| 返回查询示例 | ros2 topic echo /rm_driver/set_hand_angle_result |
#### 设置灵巧手速度
| 功能描述 | 设置灵巧手速度 |
| :---: | :---- |
| 参数说明 | Handspeed.msg<br>uint16 hand_speed：手指速度，范围：1~1000。 |
| 命令示例 | ros2 topic pub --once /rm_driver/set_hand_speed_cmd rm_ros_interfaces/msg/Handspeed "hand_speed: 200" |
| 返回值 | 成功返回：true；失败返回：false，driver终端返回错误码。 |
| 返回查询示例 | ros2 topic echo /rm_driver/set_hand_speed_result |
#### 设置灵巧手力阈值
| 功能描述 | 设置灵巧手力阈值 |
| :---: | :---- |
| 参数说明 | Handforce.msg<br>uint16 hand_force：手指力，范围：1~1000。 |
| 命令示例 | ros2 topic pub --once /rm_driver/set_hand_force_cmd rm_ros_interfaces/msg/Handforce "hand_force: 200" |
| 返回值 | 成功返回：true；失败返回：false，driver终端返回错误码。 |
| 返回查询示例 | ros2 topic echo /rm_driver/set_hand_force_result |
#### 设置灵巧手角度跟随
| 功能描述 | 设置灵巧手角度跟随 |
| :---: | :---- |
| 参数说明 | Handangle.msg<br>int16[6] hand_angle：手指角度数组，范围(根据实际设备属性，以下为因时参考)：0~2000.另外，-1 代表该自由度不执行任何操作，保持当前状态。<br>bool data：是否为阻塞模式，true:阻塞，false:非阻塞。 |
| 命令示例 | ros2 topic pub --once /rm_driver/set_hand_follow_angle_cmd rm_ros_interfaces/msg/Handangle "hand_angle:<br>- 0<br>- 0<br>- 0<br>- 0<br>- 0<br>- 0<br>block: true" |
| 返回值 | 成功返回：true；失败返回：false，driver终端返回错误码。 |
| 返回查询示例 | ros2 topic echo /rm_driver/set_hand_follow_angle_result |
#### 设置灵巧手姿态跟随
| 功能描述 | 设置灵巧手姿态跟随 |
| :---: | :---- |
| 参数说明 | Handangle.msg<br>int16[6] hand_angle：手指姿态数组，范围(根据实际设备属性，以下为因时参考)：0~1000.另外，-1 代表该自由度不执行任何操作，保持当前状态。<br>bool data：是否为阻塞模式，true:阻塞，false:非阻塞。 |
| 命令示例 | ros2 topic pub --once /rm_driver/set_hand_follow_pos_cmd rm_ros_interfaces/msg/Handangle "hand_angle:<br>- 0<br>- 0<br>- 0<br>- 0<br>- 0<br>- 0<br>block: true" |
| 返回值 | 成功返回：true；失败返回：false，driver终端返回错误码。 |
| 返回查询示例 | ros2 topic echo /rm_driver/set_hand_follow_pos_result |
### 升降机构
睿尔曼机械臂可集成自主研发升降机构。
#### 升降机构速度开环控制
| 功能描述 | 升降机构速度开环控制 |
| :---: | :---- |
| 参数说明 | Liftspeed.msg<br>int16 speed：速度百分比，-100~100，Speed < 0:升降机构向下运动，Speed > 0:升降机构向上运动，Speed = 0:升降机构停止运动。<br>bool data：是否为阻塞模式，true:阻塞，false:非阻塞。 |
| 命令示例 | ros2 topic pub /rm_driver/set_lift_speed_cmd rm_ros_interfaces/msg/Liftspeed "speed: 100" |
| 返回值 | 成功返回：true；失败返回：false，driver终端返回错误码。 |
| 返回查询示例 | ros2 topic echo /rm_driver/set_lift_speed_result |
#### 升降机构位置闭环控制
| 功能描述 | 升降机构位置闭环控制 |
| :---: | :---- |
| 参数说明 | Liftheight.msg<br>uint16 height：目标高度，单位 mm，范围：0-2600。<br>uint16 speed：速度百分比，1-100。<br>bool data：是否为阻塞模式，true:阻塞，false:非阻塞。 |
| 命令示例 | ros2 topic pub --once /rm_driver/set_lift_height_cmd rm_ros_interfaces/msg/Liftheight "height: 0<br>speed: 10<br>block: true" |
| 返回值 | 成功返回：true；失败返回：false，driver终端返回错误码。 |
| 返回查询示例 | ros2 topic echo rm_driver/set_lift_height_result |
#### 获取升降机构状态
| 功能描述 | 获取升降机构状态 |
| :---: | :---- |
| 参数说明 | Liftstate.msg<br>int16 height：当前高度。<br>int16 current：当前电流。<br>uint16 err_flag：驱动错误代码。 |
| 命令示例 | ros2 topic pub /rm_driver/get_lift_state_cmd std_msgs/msg/Empty "{}" |
| 返回值 | 成功返回：升降机构当前状态；失败返回：driver终端返回错误码。 |
| 返回查询示例 | ros2 topic echo /rm_driver/get_lift_state_result |
### 末端生态协议
末端生态协议支持下的末端设备基础信息与实时信息的读取。
#### 设置末端生态协议模式
| 功能描述 | 设置末端生态协议模式 |
| :---: | :---- |
| 参数说明 | std_msgs::msg::Int32 <br>0-禁用协议；<br>9600-开启协议（波特率9600）；<br>115200-开启协议（波特率115200）；<br>256000-开启协议（波特率256000）；<br>460800-开启协议（波特率460800）。|
| 命令示例 | ros2 topic pub /rm_driver/set_rm_plus_mode_cmd std_msgs/msg/Int32 "data: 0" |
| 返回值 | 成功返回：true；失败返回：false，driver终端返回错误码。 |
| 返回查询示例 | ros2 topic echo /rm_driver/set_rm_plus_mode_result |
#### 查询末端生态协议模式
| 功能描述 | 查询末端生态协议模式 |
| :---: | :---- |
| 参数说明 | std_msgs::msg::Empty |
| 命令示例 | ros2 topic pub /rm_driver/get_rm_plus_mode_cmd std_msgs/msg/Empty "{}" |
| 返回值 | 0-禁用协议；<br>9600-开启协议（波特率9600）；<br>115200-开启协议（波特率115200）；<br>256000-开启协议（波特率256000）；<br>460800-开启协议（波特率460800）。 |
| 返回查询示例 | ros2 topic echo /rm_driver/get_rm_plus_mode_result |
#### 设置触觉传感器模式
| 功能描述 | 设置触觉传感器模式 |
| :---: | :---- |
| 参数说明 | std_msgs::msg::Int32<br>0-关闭触觉传感器；<br>1-打开触觉传感器（返回处理后数据）；<br>2-打开触觉传感器（返回原始数据）。 |
| 命令示例 | ros2 topic pub /rm_driver/set_rm_plus_touch_cmd std_msgs::msg::Int32 "data: 0" |
| 返回值 | 成功无返回；失败返回：false，driver终端返回错误码。 |
| 返回查询示例 | ros2 topic echo /rm_driver/set_rm_plus_touch_result |
#### 获取触觉传感器模式
| 功能描述 | 获取触觉传感器模式 |
| :---: | :---- |
| 参数说明 | std_msgs::msg::Empty |
| 命令示例 | ros2 topic pub /rm_driver/get_rm_plus_touch_cmd std_msgs::msg::Empty "{}" |
| 返回值 | std_msgs::msg::Int32<br>0-关闭触觉传感器；<br>1-打开触觉传感器（返回处理后数据）；<br>2-打开触觉传感器（返回原始数据）。 |
| 返回查询示例 | ros2 topic echo /rm_driver/get_rm_plus_touch_result |
### 透传力位混合控制补偿
针对睿尔曼带一维力和六维力版本的机械臂，用户除了可直接使用示教器调用底层的力位混合控制模块外，还可以将自定义的轨迹以周期性透传的形式结合底层的力位混合控制算法进行补偿。
在进行力的操作之前，如果未进行力数据标定，可使用清空一维力、六维力数据接口对零位进行标定。
#### 开启透传力位混合控制补偿模式
| 功能描述 | 开启透传力位混合控制补偿模式 |
| :---: | :---- |
| 参数说明 | std_msgs::msg::Empty |
| 命令示例 | ros2 topic pub /rm_driver/start_force_position_move_cmd std_msgs/msg/Empty "{}" |
| 返回值 | 成功返回：true；失败返回：false，driver终端返回错误码。 |
| 返回查询示例 | ros2 topic echo /rm_driver/start_force_position_move_result |
#### 关闭透传力位混合控制补偿模式
| 功能描述 | 关闭透传力位混合控制补偿模式 |
| :---: | :---- |
| 参数说明 | std_msgs::msg::Empty |
| 命令示例 | ros2 topic pub /rm_driver/stop_force_position_move_cmd std_msgs/msg/Empty "{}" |
| 返回值 | 成功返回：true；失败返回：false，driver终端返回错误码。 |
| 返回查询示例 | ros2 topic echo /rm_driver/stop_force_position_move_result |
#### 透传力位混合补偿-关节
| 功能描述 | 透传力位混合补偿(关节) |
| :---: | :---- |
| 参数说明 | Forcepositionmovejoint.msg<br>float32[6] joint：目标关节弧度<br>uint8 sensor：所使用传感器类型，0-一维力，1-六维力<br>uint8 mode：模式，0-沿基坐标系，1-沿工具端坐标系<br>int16 dir：力控方向，0~5分别代表X/Y/Z/Rx/Ry/Rz，其中一维力类型时默认方向为Z方向<br>float32 force：力的大小 单位0.1N<br>bool follow：是否高跟随，true:高跟随，false：低跟随。<br>uint8 dof：机械臂自由度 |
| 命令示例 | 需要是大量(10个以上)位置连续的点，以2ms以上的周期持续发布。<br>ros2 topic pub /rm_driver/force_position_move_joint_cmd rm_ros_interfaces/msg/Forcepositionmovejoint " joint: [0, 0, 0, 0, 0, 0]<br>sensor: 0<br>mode: 0<br>dir: 0<br>force: 0.0<br>follow: false<br>dof: 6 |
| 返回值 | 成功无返回；失败返回：false，driver终端返回错误码。 |
#### 透传力位混合补偿-位姿
| 功能描述 | 透传力位混合补偿(位姿) |
| :---: | :---- |
| 参数说明 | Forcepositionmovepose.msg<br>geometry_msgs/Pose pose：目标位姿，x、y、z坐标(float类型，单位：m)+四元数。<br>uint8 sensor：所使用传感器类型，0-一维力，1-六维力<br>uint8 mode：模式，0-沿基坐标系，1-沿工具端坐标系<br>int16 dir：力控方向，0~5分别代表X/Y/Z/Rx/Ry/Rz，其中一维力类型时默认方向为Z方向<br>float32 force：力的大小 单位0.1N<br>bool follow：是否高跟随，true:高跟随，false：低跟随。 |
| 命令示例 | 需要是大量(10个以上)位置连续 的点，以2ms以上的周期持续发布。<br>ros2 topic pub /rm_driver/force_position_move_pose_cmd rm_ros_interfaces/msg/Forcepositionmovepose "pose:<br>  position:<br>    x: 0.0<br>    y: 0.0<br>    z: 0.0<br>  orientation:<br>    x: 0.0<br>    y: 0.0<br>    z: 0.0<br>    w: 1.0<br>sensor: 0<br>mode: 0<br>dir: 0<br>force: 0<br>follow: false" |
| 返回值 | 成功无返回；失败返回：false，driver终端返回错误码。
### 机械臂状态主动上报
#### 设置UDP机械臂状态主动上报配置
| 功能描述 | 设置UDP 机械臂状态主动上报配置 |
| :---: | :---- |
| 参数说明 | Setrealtimepush.msg<br>uint16 cycle：设置广播周期，为5ms的倍数(默认1即1*5=5ms,200Hz)。<br>uint16 port：设置广播的端口号(默认8089)。<br>uint16 force_coordinate：设置系统外受力数据的坐标系(仅带有力传感器的机械臂支持)。<br>string ip：设置自定义的上报目标IP 地址(默认192.168.1.10)<br>bool hand_enable：是否使能灵巧手状态上报，true使能，false不使能。<br>aloha_state_enable: 是否使能aloha主臂状态上报，true使能，false不使能。<br>arm_current_status_enable: 是否使能机械臂状态上报，true使能，false不使能。<br>expand_state_enable: 是否使能扩展关节相关数据上报，true使能，false不使能。<br>joint_speed_enable: 是否使能关节速度上报，true使能，false不使能。<br>lift_state_enable: 是否使能升降关节数据上报，true使能，false不使能。<br>plus_base_enable: 末端设备基础信息上报，true使能，false不使能。<br>plus_state_enable: 末端设备实时信息上报，true使能，false不使能。 |
| 命令示例 | ros2 topic pub --once /rm_driver/set_realtime_push_cmd rm_ros_interfaces/msg/Setrealtimepush "cycle: 1<br>port: 8089<br>force_coordinate: 0<br>ip: '192.168.1.10'<br>hand_enable: false<br>aloha_state_enable: false<br>arm_current_status_enable: false<br>expand_state_enable: false<br>joint_speed_enable: false<br>lift_state_enable: false<br>plus_base_enable: false<br>plus_state_enable: false" |
| 返回值 | 成功返回：true；失败返回：false，driver终端返回错误码。 |
| 返回查询示例 | ros2 topic echo /rm_driver/set_realtime_push_result |
#### 查询UDP机械臂状态主动上报配置
| 功能描述 | 查询UDP 机械臂状态主动上报配置 |
| :---: | :---- |
| 参数说明 | Setrealtimepush.msg<br>uint16 cycle：设置广播周期，为5ms的倍数(默认1即1*5=5ms,200Hz)。<br>uint16 port：设置广播的端口号(默认8089)。<br>uint16 force_coordinate：设置系统外受力数据的坐标系(仅带有力传感器的机械臂支持)。<br>string ip：设置自定义的上报目标IP 地址(默认192.168.1.10)<br>bool hand_enable：是否使能灵巧手状态上报，true使能，false不使能。<br>aloha_state_enable: 是否使能aloha主臂状态上报，true使能，false不使能。<br>arm_current_status_enable: 是否使能机械臂状态上报，true使能，false不使能。<br>expand_state_enable: 是否使能扩展关节相关数据上报，true使能，false不使能。<br>joint_speed_enable: 是否使能关节速度上报，true使能，false不使能。<br>lift_state_enable: 是否使能升降关节数据上报，true使能，false不使能。<br>plus_base_enable: 末端设备基础信息上报，true使能，false不使能。<br>plus_state_enable: 末端设备实时信息上报，true使能，false不使能。 |
| 命令示例 | ros2 topic pub --once /rm_driver/get_realtime_push_cmd std_msgs/msg/Empty "{}" |
| 返回值 | 成功设置信息；失败返回：driver终端返回错误码。 |
| 返回查询示例 | ros2 topic echo /rm_driver/get_realtime_push_result |
#### UDP机械臂状态主动上报

* 六维力

| 功能描述 | 六维力 |
| :---: | :---- |
| 参数说明 | Sixforce.msg<br>float32 force_fx：沿x轴方向受力大小。<br>float32 force_fy：沿y轴方向受力大小。<br>float32 force_fz：沿z轴方向受力大小。<br>float32 force_mx：沿x轴方向转动受力大小。<br>float32 force_my：沿y轴方向转动受力大小。<br>float32 force_mz：沿z轴方向转动受力大小。 |
| 查询示例 | ros2 topic echo /rm_driver/udp_six_force |

* 一维力

| 功能描述 | 一维力 |
| :---: | :---- |
| 参数说明 | Sixforce.msg<br>float32 force_fx：沿x轴方向受力大小。<br>float32 force_fy：沿y轴方向受力大小。<br>float32 force_fz：沿z轴方向受力大小。(仅该数值有效)<br>float32 force_mx：沿x轴方向转动受力大小。<br>float32 force_my：沿y轴方向转动受力大小。<br>float32 force_mz：沿z轴方向转动受力大小。 |
| 查询示例 | ros2 topic echo /rm_driver/udp_one_force |

* 机械臂错误

| 功能描述 | 机械臂错误 |
| :---: | :---- |
| 参数说明 | std_msgs::msg::UInt16<br>uint16 data：机械臂报错信息。 |
| 查询示例 | ros2 topic echo /rm_driver/udp_arm_err |

* 系统错误

| 功能描述 | 系统错误 |
| :---: | :---- |
| 参数说明 | std_msgs::msg::UInt16<br>uint16 data：系统报错信息。 |
| 查询示例 | ros2 topic echo /rm_driver/udp_sys_err |

* 关节错误

| 功能描述 | 关节错误 |
| :---: | :---- |
| 参数说明 | Jointerrorcode.msg<br>uint16[] joint_error：每个关节报错信息。<br>Uint8 dof：机械臂自由度信息。 |
| 查询示例 | ros2 topic echo /rm_driver/udp_joint_error_code |

* 机械臂弧度数据

| 功能描述 | 机械臂弧度数据 |
| :---: | :---- |
| 参数说明 | sensor_msgs::msg::JointState<br>	builtin_interfaces/Time stamp<br>		int32 sec：时间信息，秒。<br>		uint32 nanosec：时间信息，纳秒。<br>	string frame_id：坐标系名称。<br>string[] name：关节名称。<br>float64[] position：关节弧度信息。<br>float64[] velocity：关节速度信息。(暂未使用)<br>float64[] effort：关节受力信息。(暂未使用) |
| 查询示例 | ros2 topic echo /joint_states |

* 位姿信息

| 功能描述 | 位姿信息 |
| :---: | :---- |
| 参数说明 | geometry_msgs::msg::Pose<br>Point position：机械臂当前坐标信息。<br>	float64 x<br>	float64 y<br>	float64 z<br>Quaternion orientation：机械臂当前姿态信息。<br>	float64 x 0<br>	float64 y 0<br>	float64 z 0<br>	float64 w 1 |
| 查询示例 | ros2 topic echo /rm_driver/udp_arm_position |

* 当前六维力传感器系统外受力数据

| 功能描述 | 当前六维力传感器系统外受力数据 |
| :---: | :---- |
| 参数说明 | Sixforce.msg<br>float32 force_fx：当前传感器沿x轴方向受外力大小。<br>float32 force_fy：当前传感器沿y轴方向受外力大小。<br>float32 force_fz：当前传感器沿z轴方向受外力大小。<br>float32 force_mx：当前传感器沿x轴方向转动受外力大小。<br>float32 force_my：当前传感器沿y轴方向转动受外力大小。<br>float32 force_mz：当前传感器沿z轴方向转动受外力大小。 |
| 查询示例 | ros2 topic echo /rm_driver/udp_six_zero_force |

* 当前一维力传感器系统外受力数据

| 功能描述 | 当前一维力传感器系统外受力数据 |
| :---: | :---- |
| 参数说明 | Sixforce.msg<br>float32 force_fx：当前传感器沿x轴方向受外力大小。<br>float32 force_fy：当前传感器沿y轴方向受外力大小。<br>float32 force_fz：当前传感器沿z轴方向受外力大小。(仅该数据有效)<br>float32 force_mx：当前传感器沿x轴方向转动受外力大小。<br>float32 force_my：当前传感器沿y轴方向转动受外力大小。<br>float32 force_mz：当前传感器沿z轴方向转动受外力大小。 |
| 查询示例 | ros2 topic echo /rm_driver/udp_one_zero_force |

* 系统外受力数据参考坐标系

| 功能描述 | 系统外受力数据参考坐标系 |
| :----: | :---- |
| 参数说明 | std_msgs::msg::UInt16<br>uint16 data：系统外受力数据的坐标系，0 为传感器坐标系 1 为当前工作坐标系 2 为当前工具坐标系。该数据会影响一维力和六维力传感器系统外受力数据的参考坐标系 |
| 查询示例 | ros2 topic echo /rm_driver/udp_arm_coordinate |

* 灵巧手力当前状态

| 功能描述 | 获取灵巧手力当前状态 |
| :----: | :---- |
| 参数说明 | rm_ros_interfaces::msg::Handstatus.msg<br>uint16[6] hand_angle：#手指角度数组，范围：0~2000.<br>uint16[6] hand_pos：#手指位置数组，范围：0~1000.<br>uint16[6] hand_state：#手指状态,0正在松开，1正在抓取，2位置到位停止，3力到位停止，5电流保护停止，6电缸堵转停止，7电缸故障停止.<br>uint16[6] hand_force：#灵巧手自由度电流，单位mN.<br>uint16  hand_err：#灵巧手系统错误，1表示有错误，0表示无错误. |
| 查询示例 | ros2 topic echo /rm_driver/udp_hand_status |
* 机械臂当前状态

| 功能描述 | 获取机械臂当前状态 |
| :----: | :---- |
| 参数说明 | rm_ros_interfaces::msg::Armcurrentstatus.msg<br>uint16 arm_current_status：机械臂状态<br>0 - RM_IDLE_E // 使能但空闲状态<br>1 - RM_MOVE_L_E // move L运动中状态<br>2 - RM_MOVE_J_E // move J运动中状态 <br>3 - RM_MOVE_C_E // move C运动中状态 <br>4 - RM_MOVE_S_E // move S运动中状态 <br>5 - RM_MOVE_THROUGH_JOINT_E // 角度透传状态 <br>6 - RM_MOVE_THROUGH_POSE_E  // 位姿透传状态 <br>7 - RM_MOVE_THROUGH_FORCE_POSE_E // 力控透传状态 <br>8 - RM_MOVE_THROUGH_CURRENT_E // 电流环透传状态 <br>9 - RM_STOP_E             // 急停状态 <br>10 - RM_SLOW_STOP_E        // 缓停状态 <br>11 - RM_PAUSE_E            // 暂停状态 <br>12 - RM_CURRENT_DRAG_E     // 电流环拖动状态 <br>13 - RM_SENSOR_DRAG_E      // 六维力拖动状态 <br>14 - RM_TECH_DEMONSTRATION_E // 示教状态 |
| 查询示例 | ros2 topic echo /rm_driver/udp_arm_current_status |

* 当前关节电流

| 功能描述 | 当前关节电流 |
| :----: | :---- |
| 参数说明 | rm_ros_interfaces::msg::Jointcurrent.msg<br>float32[] joint_current: 当前关节电流，精度 0.001mA |
| 查询示例 | ros2 topic echo /rm_driver/udp_joint_current |

* 当前关节使能状态

| 功能描述 | 当前关节使能状态 |
| :----: | :---- |
| 参数说明 | rm_ros_interfaces::msg::Jointenflag.msg<br>bool[] joint_en_flag: 当前关节使能状态 ，1 为上使能，0 为掉使能|
| 查询示例 | ros2 topic echo /rm_driver/udp_joint_en_flag |

* 机械臂欧拉角位姿

| 功能描述 | 机械臂欧拉角位姿 |
| :----: | :---- |
| 参数说明 | rm_ros_interfaces::msg::Jointposeeuler.msg<br>float32[3] euler: 当前路点姿态欧拉角，精度 0.001rad<br>float32[3] position：当前路点位置，精度 0.000001M|
| 查询示例 | ros2 topic echo /rm_driver/udp_joint_pose_euler |

* 当前关节速度

| 功能描述 | 当前关节速度 |
| :----: | :---- |
| 参数说明 | rm_ros_interfaces::msg::Jointspeed.msg<br>float32[] joint_speed: 当前关节速度，精度0.02RPM。|
| 查询示例 | ros2 topic echo /rm_driver/udp_joint_speed |

* 当前关节温度

| 功能描述 | 当前关节温度 |
| :----: | :---- |
| 参数说明 | rm_ros_interfaces::msg::Jointtemperature.msg<br>float32[] joint_temperature: 当前关节温度，精度 0.001℃|
| 查询示例 | ros2 topic echo /rm_driver/udp_joint_temperature |
* 当前关节电压

| 功能描述 | 当前关节电压 |
| :----: | :---- |
| 参数说明 | rm_ros_interfaces::msg::Jointvoltage.msg<br>float32[] joint_voltage: 当前关节电压，精度 0.001V|
| 查询示例 | ros2 topic echo /rm_driver/udp_joint_voltage |
* 末端设备基础信息

| 功能描述 | 末端设备基础信息(末端生态协议支持) |
| :----: | :---- |
| 参数说明 | rm_ros_interfaces::msg::Rmplusbase.msg<br>string manu:# 设备厂家.<br>int8 type:设备类型 1：两指夹爪 2：五指灵巧手 3：三指夹爪<br>string hv:硬件版本<br>string sv:软件版本<br>string bv:boot版本<br>int32 id:设备ID<br>int8 dof:自由度<br>int8 check:自检开关<br>int8 bee:蜂鸣器开关<br>bool force:力控支持<br>bool touch:触觉支持<br>int8 touch_num:触觉个数<br>int8 touch_sw:触觉开关<br>int8 hand:手方向 1 ：左手 2： 右手<br>int32[12] pos_up:位置上限,单位：无量纲<br>int32[12] pos_low:位置下限,单位：无量纲<br>int32[12] angle_up:角度上限,单位：0.01度<br>int32[12] angle_low:角度下限,单位：0.01度<br>int32[12] speed_up:速度上限,单位：无量纲<br>int32[12] speed_low:速度下限,单位：无量纲<br>int32[12] force_up:力上限,单位：0.001N<br>int32[12] force_low:力下限,单位：0.001N|
| 查询示例 | ros2 topic echo /rm_driver/udp_rm_plus_base |
* 末端设备实时信息

| 功能描述 | 末端设备实时信息(末端生态协议支持) |
| :----: | :---- |
| 参数说明 | rm_ros_interfaces::msg::Rmplusstate.msg<br>int32 sys_state:系统状态.<br>int32[12] dof_state:各自由度当前状态<br>int32[12] dof_err:各自由度错误信息<br>int32[12] pos:各自由度当前位置<br>int32[12] speed:各自由度当前速度<br>int32[12] angle:各自由度当前角度<br>int32[12] current:各自由度当前电流<br>int32[18] normal_force:自由度触觉三维力的法向力<br>int32[18] tangential_force:自由度触觉三维力的切向力<br>int32[18] tangential_force_dir:自由度触觉三维力的切向力方向<br>uint32[12] tsa:自由度触觉自接近<br>uint32[12] tma:自由度触觉互接近<br>int32[18] touch_data:触觉传感器原始数据<br>int32[12] force:自由度力矩|
| 查询示例 | ros2 topic echo /rm_driver/udp_rm_plus_state |