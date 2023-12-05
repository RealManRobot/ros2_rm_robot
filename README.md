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
bash ros2_install.sh
```
如果不想使用脚本安装也可以参考网址（）进行安装。
