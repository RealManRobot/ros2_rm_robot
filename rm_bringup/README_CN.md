<div align="right">
 
[简体中文](https://github.com/RealManRobot/ros2_rm_robot/tree/humble/rm_bringup/README_CN.md)|[English](https://github.com/RealManRobot/ros2_rm_robot/tree/humble/rm_bringup/README.md)

</div>

<div align="center">

# 睿尔曼机器人rm_bringup使用说明书V1.5
 
睿尔曼智能科技（北京）有限公司 
文件修订记录：

| 版本号| 时间   | 备注  | 
| :---: | :-----: | :---: |
|V1.0    |2024-2-19  |拟制 |
|V1.1    |2024-7-8   |修订(添加GEN72适配文件) |
|V1.2    |2024-9-10  |修订(添加ECO63适配文件) |
|V1.3    |2024-12-25 |修订(添加了63、65、75、ECO65的六维力适配文件，以及63、65、75、ECO63、ECO65的一体化六维力适配文件) |
|V1.4    |2025-4-7 |修订(添加了GEN72_II适配文件) |
|V1.5    |2025-11-13 |修订(添加了RML63_III适配文件) |

</div>

## 目录
* 1.[rm_bringup功能包说明](#rm_bringup功能包说明)
* 2.[rm_bringup功能包使用](#rm_bringup功能包使用)
* 2.1[moveit2控制真实机械臂](#moveit2控制真实机械臂)
* 2.2[控制gazebo仿真机械臂](#控制gazebo仿真机械臂)
* 3.[rm_bringup功能包架构说明](#rm_bringup功能包架构说明)
* 3.1[功能包文件总览](#rm_bringup功能包架构说明)
* 4.[rm_bringup话题说明](#rm_bringup话题说明)

## rm_bringup功能包说明
rm_bringup功能包为实现多个launch文件同时运行所设计的功能包，使用该功能包可用一条命令实现多个节点结合的复杂功能的启动。
* 1.功能包使用。
* 2.功能包架构说明。
* 3.功能包话题说明。  
通过这三部分内容的介绍可以帮助大家：
* 1.了解该功能包的使用。
* 2.熟悉功能包中的文件构成及作用。
* 3.熟悉功能包相关的话题，方便开发和使用
## rm_bringup功能包使用
### moveit2控制真实机械臂
首先配置好环境完成连接后我们可以通过以下命令直接启动节点，运行rm_bringup功能包中的launch.py文件。
```
rm@rm-desktop:~$ ros2 launch rm_bringup rm_<arm_type>_bringup.launch.py
```
在实际使用时需要将以上的<arm_type>更换为实际的机械臂型号，可选择的机械臂型号有65、63、63_III、eco65、eco63、75、gen72、gen72_II。

启动六维力版本机械臂的命令为(注意：eco63不可用)：
```
rm@rm-desktop:~$ ros2 launch rm_bringup rm_<arm_type>_6f_bringup.launch.py
```
启动一体化六维力版本机械臂的命令为：
```
rm@rm-desktop:~$ ros2 launch rm_bringup rm_<arm_type>_6fb_bringup.launch.py
```
例如65机械臂的启动命令：
```
rm@rm-desktop:~$ ros2 launch rm_bringup rm_65_bringup.launch.py
```
节点启动成功后，将弹出以下画面。
![image](doc/rm_bringup1.png)  
实际该launch文件启动的为moveit2控制真实机械臂的功能下面就可以使用控制球规划控制机械臂运动，详细可查看《[rm_moveit2_config详解]((https://github.com/RealManRobot/ros2_rm_robot/blob/main/rm_moveit2_config/README_CN.md))》相关内容。
### 控制gazebo仿真机械臂
我们可以通过以下命令运行rm_bringup功能包中的launch.py文件，直接启动其中的gzaebo仿真节点。
```
rm@rm-desktop:~$ ros2 launch rm_bringup rm_<arm_type>_gazebo.launch.py
```
在实际使用时需要将以上的<arm_type>更换为实际的机械臂型号，可选择的机械臂型号有65、63、63_III、eco65、eco63、75、gen72、gen72_II。

启动六维力版本机械臂的命令为(注意：eco63不可用)：
```
rm@rm-desktop:~$ ros2 launch rm_bringup rm_<arm_type>_6f_gazebo.launch.py
```
启动一体化六维力版本机械臂的命令为：
```
rm@rm-desktop:~$ ros2 launch rm_bringup rm_<arm_type>_6fb_gazebo.launch.py
```
例如65机械臂的启动命令：
```
rm@rm-desktop:~$ ros2 launch rm_bringup rm_65_gazebo.launch.py
```
节点启动成功后，将弹出以下画面。
![image](doc/rm_bringup2.png)  
之后我们使用如下指令启动moveit2控制gazebo中的仿真机械臂。
![image](doc/rm_bringup3.png)
## rm_bringup功能包架构说明
### 功能包文件总览
当前rm_bringup功能包的文件构成如下。
```
├── CMakeLists.txt                      #编译规则文件
├── doc                                 #辅助文档、图片存放文件夹
│   ├── rm_bringup1.png                 #图片1
│   ├── rm_bringup2.png                 #图片2
│   └── rm_bringup3.png                 #图片3
├── launch                              #启动文件
│   ├── rm_63_6f_bringup.launch.py      #63臂六维力moveit2启动文件
│   ├── rm_63_6f_gazebo.launch.py       #63臂六维力gazebo启动文件
│   ├── rm_63_6fb_bringup.launch.py     #63臂一体化六维力moveit2启动文件
│   ├── rm_63_6fb_gazebo.launch.py      #63臂一体化六维力gazebo启动文件
│   ├── rm_63_bringup.launch.py         #63臂moveit2启动文件
│   ├── rm_63_gazebo.launch.py          #63臂gazebo启动文件
│   ├── rm_63_III_bringup.launch.py     #63_III臂moveit2启动文件
│   ├── rm_63_III_gazebo.launch.py      #63_III臂gazebo启动文件
│   ├── rm_63_III_6fb_bringup.launch.py #63_III臂六维力moveit2启动文件
│   ├── rm_63_III_6fb_gazebo.launch.py  #63_III臂六维力gazebo启动文件
│   ├── rm_65_6f_bringup.launch.py      #65臂六维力moveit2启动文件
│   ├── rm_65_6f_gazebo.launch.py       #65臂六维力gazebo启动文件
│   ├── rm_65_6fb_bringup.launch.py     #65臂一体化六维力moveit2启动文件
│   ├── rm_65_6fb_gazebo.launch.py      #65臂一体化六维力gazebo启动文件
│   ├── rm_65_bringup.launch.py         #65臂moveit2启动文件
│   ├── rm_65_gazebo.launch.py          #65臂gazebo启动文件
│   ├── rm_75_6f_bringup.launch.py      #75臂六维力moveit2启动文件
│   ├── rm_75_6f_gazebo.launch.py       #75臂六维力gazebo启动文件
│   ├── rm_75_6fb_bringup.launch.py     #75臂一体化六维力moveit2启动文件
│   ├── rm_75_6fb_gazebo.launch.py      #75臂一体化六维力gazebo启动文件
│   ├── rm_75_bringup.launch.py         #75臂moveit2启动文件
│   ├── rm_75_gazebo.launch.py          #75臂gazebo启动文件
│   ├── rm_eco63_6fb_bringup.launch.py  #eco63臂一体化六维力moveit2启动文件
│   ├── rm_eco63_6fb_gazebo.launch.py   #eco63臂一体化六维力gazebo启动文件
│   ├── rm_eco63_bringup.launch.py      #eco63臂moveit2启动文件
│   ├── rm_eco63_gazebo.launch.py       #eco63臂gazebo启动文件
│   ├── rm_eco65_6f_bringup.launch.py   #eco65臂六维力moveit2启动文件
│   ├── rm_eco65_6f_gazebo.launch.py    #eco65臂六维力gazebo启动文件
│   ├── rm_eco65_6fb_bringup.launch.py  #eco65臂一体化六维力moveit2启动文件
│   ├── rm_eco65_6fb_gazebo.launch.py   #eco65臂一体化六维力gazebo启动文件
│   ├── rm_eco65_bringup.launch.py      #eco65臂moveit2启动文件
│   ├── rm_eco65_gazebo.launch.py       #eco65臂gazebo启动文件
│   ├── rm_gen72_bringup.launch.py      #gen72臂moveit2启动文件
│   ├── rm_gen72_gazebo.launch.py       #gen72臂gazebo启动文件
│   ├── rm_gen72_II_bringup.launch.py   #gen72_II臂moveit2启动文件
│   └── rm_gen72_II_gazebo.launch.py    #gen72_II臂gazebo启动文件
├── package.xml                         #依赖说明文件
├── README_CN.md                        #中文说明文档
└── README.md                           #英文说明文档
```
## rm_bringup话题说明
该功能包当前并没有本身的话题，主要为调用其他功能包的话题实现，关于moveit2相关话题可查看《[rm_moveit2_config详解](https://github.com/RealManRobot/ros2_rm_robot/blob/main/rm_moveit2_config/README_CN.md)》相关内容。
