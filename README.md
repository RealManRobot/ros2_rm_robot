# ros2_rm_robot
该功能包的主要作用为提供机械臂的ROS2支持，以下为使用环境。
* 当前支持的机械臂有rm_65系列、rm_75系列、rm_eco65系列、rm_63系列，详细可参考网址 [RealMan robots](http://www.realman-robotics.com/)。
* 基于的Ubuntu版本为22.04。
* ROS2版本为humble。

下面为功能包安装使用教程。
## 1.搭建环境
---
在使用功能包之前我们首先需要进行如下操作。
* 1.安装ROS2
* 2.安装Moveit2
* 3.配置功能包环境
* 4.编译
### 1.1安装ROS2
----
我们提供了ROS2的安装脚本ros2_install.sh，该脚本位于rm_install功能包中的scripts文件夹下，在实际使用时我们需要移动到该路径执行如下指令。
```
sudo bash ros2_install.sh
```
如果不想使用脚本安装也可以参考网址 [ROS2_INSTALL](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)。
### 1.2安装Moveit2
----
我们提供了ROS2的安装脚本moveit2_install.sh，该脚本位于rm_install功能包中的scripts文件夹下，在实际使用时我们需要移动到该路径执行如下指令。
```
sudo bash moveit2_install.sh
```
如果不想使用脚本安装也可以参考网址 [Moveit2_INSTALL](https://moveit.ros.org/install-moveit2/binary/)进行安装。
### 1.3配置功能包环境
----
该脚本位于rm_driver功能包中的lib文件夹下，在实际使用时我们需要移动到该路径执行如下指令。
```
sudo bash lib_install.sh
```
### 1.4编译
----
以上执行成功后，可以执行如下指令进行功能包编译，首先需要构建工作空间，并将功能包文件导入工作空间下的src文件夹下,之后使用colcon build指令进行编译。
```
mkdir -p ~/ros2_ws/src
cp -r ros2_rm_robot ~/ros2_ws/src
cd ~/ros2_ws
colocn build
```
编译完成后即可进行功能包的运行操作。


## 2.功能运行
---
功能包简介
1.	安装与环境配置(rm_install)
2.	硬件驱动(rm_driver)
3.	启动(rm_bringup)
4.	模型描述(rm_description)
5.	ROS消息接口(rm_ros_interfaces)
6.	Moveit2配置(rm_moveit_config)
7.	Moveit2与硬件驱动通信连接(rm_config)
8.	Gazebo仿真机械臂控制(rm_gazebo)
9.	使用案例(rm_examples)
10.	技术文档(rm_docs)
以上为当前的十个功能包，每个功能包都有其独特的作用，详情请参考rm_doc功能包下的doc文件夹中的文档进行详细了解。
### 2.1运行虚拟机械臂
----
使用如下指令可以启动gazebo显示仿真机械臂，并同时启动moveit2进行仿真机械臂的规划操控。
```
source ~/ros2_ws/install/setup.bash
ros2 launch rm_bringup rm_<arm_type>_gazebo.launch.py
```
<arm_type>需要使用65、75、eco65、63字符进行代替，如使用rm_65机械臂时，命令如下。
```
ros2 launch rm_bringup rm_65_gazebo.launch.py
```
启动成功后即可使用moveit2进行虚拟机械臂的控制。
### 2.2控制真实机械臂
----
使用如下指令可以启动机械臂硬件驱动，并同时启动moveit2进行机械臂的规划操控。
```
source ~/ros2_ws/install/setup.bash
ros2 launch rm_bringup rm_<arm_type>_bringup.launch.py
```
<arm_type>需要使用65、75、eco65、63字符进行代替，如使用rm_65机械臂时，命令如下。
```
ros2 launch rm_bringup rm_65_bringup.launch.py
```
启动成功后即可使用moveit2进行真实机械臂的控制。

