<div align="right">
 
[简体中文](https://github.com/RealManRobot/ros2_rm_robot/blob/humble/rm_gazebo/README_CN.md)|[English](https://github.com/RealManRobot/ros2_rm_robot/blob/humble/rm_gazebo/README.md)

</div>

<div align="center">

# 睿尔曼机器人rm_gazebo使用说明书V1.5
 
睿尔曼智能科技（北京）有限公司 
文件修订记录：

| 版本号| 时间   | 备注  | 
| :---: | :-----: | :---: |
|V1.0    |2024-2-19  |拟制 |
|V1.1    |2024-7-8   |修订（添加gen72相关适配文件） |
|V1.1.1  |2024-8-13  |修订（添加机械臂型号适配说明） |
|V1.2    |2024-9-10  |修订（添加eco63相关适配文件） |
|V1.3    |2024-12-25 |修订(添加了63、65、75、ECO65的六维力适配文件，以及63、65、75、ECO63、ECO65的一体化六维力适配文件) |
|V1.4    |2025-4-3 |修订(添加了Gen72_II型适配文件) |
|V1.5    |2025-11-13 |修订(添加了RML63_III型适配文件) |

</div>

## 目录
* 1.[rm_gazebo功能包说明](#rm_gazebo功能包说明)
* 2.[rm_gazebo功能包运行](#rm_gazebo功能包运行)
* 2.1[控制仿真机械臂](#控制仿真机械臂)
* 3.[rm_gazebo功能包架构说明](#rm_gazebo功能包架构说明)
* 3.1[功能包文件总览](#功能包文件总览)

## rm_gazebo功能包说明
rm_gazebo的主要作用为帮助我们实现机械臂Moveit2规划的仿真功能，我们将在gazebo的仿真环境中搭建一个虚拟机械臂，然后通过Moveit2控制gazebo中的虚拟机械臂。
* 1.功能包使用。
* 2.功能包架构说明。
通过这三部分内容的介绍可以帮助大家：  
* 1.了解该功能包的使用。
* 2.熟悉功能包中的文件构成及作用。
### 控制仿真机械臂
在完成环境安装和功能包安装后，我们可以进行rm_gazebo功能包的运行。  
使用如下指令启动gazebo虚拟空间和虚拟机械臂。
```
rm@rm-desktop:~$ ros2 launch rm_gazebo gazebo_<arm_type>_demo.launch.py
```
启动六维力版本机械臂的命令为(注意：eco63不可用)：
```
rm@rm-desktop:~$ ros2 launch rm_gazebo gazebo_<arm_type>_6f_demo.launch.py
```
启动一体化六维力版本机械臂的命令为：
```
rm@rm-desktop:~$ ros2 launch rm_gazebo gazebo_<arm_type>_6fb_demo.launch.py
```
在实际使用时需要将以上的<arm_type>更换为实际的机械臂型号，可选择的机械臂型号有65、63、 63_III、eco65、eco63、75、gen72、gen72_II，运行成功后将弹出如下界面。  
![image](doc/rm_gazebo1.png)
之后我们使用如下指令启动moveit2控制gazebo中的仿真机械臂。
```
rm@rm-desktop:~$ ros2 launch rm_<arm_type>_config gazebo_moveit_demo.launch.py
```
启动六维力版本机械臂的命令为(注意：eco63不可用)：
```
rm@rm-desktop:~$ ros2 launch rm_<arm_type>_config gazebo_moveit_demo_6f.launch.py
```
启动一体化六维力版本机械臂的命令为：
```
rm@rm-desktop:~$ ros2 launch rm_<arm_type>_config gazebo_moveit_demo_6fb.launch.py
```
在实际使用时需要将以上的<arm_type>更换为实际的机械臂型号，可选择的机械臂型号有65、63、 63_III、eco65、eco63、75、gen72、gen72_II，运行成功后弹出rviz2的控制界面后就可以进行moveit2和gazebo的仿真控制了。
![image](doc/rm_gazebo2.png)
## rm_gazebo功能包架构说明
### 功能包文件总览
当前rm_gazebo功能包的文件构成如下。
```
├── CMakeLists.txt                              #编译规则文件
├── config
│   ├── gazebo_63_6fb_description.urdf.xacro    #RML63一体化六维力gazebo模型描述文件
│   ├── gazebo_63_III_6fb_description.urdf.xacro#RML63_III一体化六维力gazebo模型描述文件
│   ├── gazebo_65_6fb_description.urdf.xacro    #RM65一体化六维力gazebo模型描述文件
│   ├── gazebo_75_6fb_description.urdf.xacro    #RM75一体化六维力gazebo模型描述文件
│   ├── gazebo_eco63_6fb_description.urdf.xacro #ECO63一体化六维力gazebo模型描述文件
│   ├── gazebo_eco65_6fb_description.urdf.xacro #ECO65一体化六维力gazebo模型描述文件
│   ├── gazebo_63_description.urdf.xacro        #RML63gazebo模型描述文件
│   ├── gazebo_65_description.urdf.xacro        #RM65gazebo模型描述文件
│   ├── gazebo_75_description.urdf.xacro        #RM75gazebo模型描述文件
│   ├── gazebo_eco65_description.urdf.xacro     #ECO65gazebo模型描述文件
│   ├── gazebo_eco63_description.urdf.xacro     #ECO63gazebo模型描述文件
│   ├── gazebo_gen72_description.urdf.xacro     #GEN72gazebo模型描述文件
│   └── gazebo_gen72_II_description.urdf.xacro  #GEN72_IIgazebo模型描述文件
├── doc
│   ├── rm_gazebo1.png
│   └── rm_gazebo2.png
├── include
│   └── rm_gazebo
├── launch
│   ├── gazebo_63_6fb_demo.launch.py       #RML63一体化六维力gazebo启动文件
│   ├── gazebo_63_6f_demo.launch.py        #RML63六维力gazebo启动文件
│   ├── gazebo_63_demo.launch.py           #RML63gazebo启动文件
│   ├── gazebo_63_III_6fb_demo.launch.py   #RML63_III 一体化六维力gazebo启动文件
│   ├── gazebo_63_III_demo.launch.py       #RML63_III gazebo启动文件
│   ├── gazebo_65_6fb_demo.launch.py       #RM65一体化六维力gazebo启动文件
│   ├── gazebo_65_6f_demo.launch.py        #RM65六维力gazebo启动文件
│   ├── gazebo_65_demo.launch.py           #RM65gazebo启动文件
│   ├── gazebo_75_6fb_demo.launch.py       #RM75一体化六维力gazebo启动文件
│   ├── gazebo_75_6f_demo.launch.py        #RM75六维力gazebo启动文件
│   ├── gazebo_75_demo.launch.py           #RM75gazebo启动文件
│   ├── gazebo_eco63_6fb_demo.launch.py    #ECO63一体化六维力gazebo启动文件
│   ├── gazebo_eco63_demo.launch.py        #ECO63gazebo启动文件
│   ├── gazebo_eco65_6fb_demo.launch.py    #ECO63一体化六维力gazebo启动文件
│   ├── gazebo_eco65_6f_demo.launch.py     #ECO63六维力gazebo启动文件
│   ├── gazebo_eco65_demo.launch.py        #ECO63gazebo启动文件
│   ├── gazebo_gen72_demo.launch.py        #GEN72gazebo启动文件
│   └── gazebo_gen72_II_demo.launch.py     #GEN72_IIgazebo启动文件
├── package.xml
├── README_CN.md
└── README.md
```