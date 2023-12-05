# ros2_rm_robot
该功能包的主要作用为提供机械臂的ROS2支持，以下为使用环境。
* 当前支持的机械臂有rm_65系列、rm_75系列、rm_eco65系列、rm_63系列，详细可参考网址（http://www.realman-robotics.com/）。
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
如果不想使用脚本安装也可以参考网址（https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html）进行安装。
### 1.2安装Moveit2
----
我们提供了ROS2的安装脚本moveit2_install.sh，该脚本位于rm_install功能包中的scripts文件夹下，在实际使用时我们需要移动到该路径执行如下指令。
```
sudo bash moveit2_install.sh
```
如果不想使用脚本安装也可以参考网址（https://moveit.ros.org/install-moveit2/binary/）进行安装。
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
colocn build
```
编译完成后即可进行功能包的运行操作。
## 2.功能运行
---
