<div align="right">
 
[简体中文](README_CN.md)|[English](README.md)

</div>

<div align="center">

# 睿尔曼机器人rm_gazebo使用说明书V1.7
 
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
|V1.6    |2026-4-16 |修订(添加ECO62、RX75适配文件，更新Gazebo版本) |
|V1.7    |2026-8-11 |修订(增加统一Gazebo启动入口，旧入口保留兼容) |

</div>

## 启动示例

构建并 source 工作区后，以一体化六维力 RM65 的 Gazebo 仿真为例：

```bash
# 仿真（三代 / 四代均沿用 6fb 模型命名，在使用仿真是目前不专门做三四代的区别）
ros2 launch rm_gazebo rm_gazebo.launch.py \
  arm_type:=65-6fb
```

本包启动 Gazebo 仿真。仿真不连接控制器，也不受控制器软件版本限制；四代软件从 `1.0.6` 起将原 `6fb` 末端命名为 `6f`，仿真同一结构仍使用 `65-6fb`。完整型号见[三、四代型号表](../rm_bringup/README_CN.md#统一启动入口)。

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

推荐通过统一入口启动Gazebo虚拟空间和虚拟机械臂：
```
rm@rm-desktop:~$ ros2 launch rm_gazebo rm_gazebo.launch.py arm_type:=65-6fb
```
统一入口支持以下参数：

* `arm_type`：必填，直接选择型号和末端版本，如 `65`、`65-6fb`、`rx75-6fb-v`；不带末端后缀时选择标准版。完整可选值见 [统一入口支持表](../rm_bringup/README_CN.md#统一启动入口)。
* `model`：默认 `auto`，有配套资源的型号使用彩色 GLB，其余使用原 STL；`stl` 切回原网格，`glb` 强制使用 GLB。仅替换外观，保留原碰撞和物理参数。
* `start_gazebo`：是否启动Gazebo，默认 `true`。仅当需要复用已经运行且 world 名为 `empty` 的 Gazebo 时设置为 `false`；此时只跳过 Gazebo 进程，模型创建、`/world/empty/clock` bridge 和控制器启动仍会执行。
* `joint_states_topic`：当前仅支持 `auto`；普通机械臂解析为 `/joint_states`，RX75解析为 `/joint_state_broadcaster/joint_states`。
* `use_sim_time`：`robot_state_publisher`是否使用仿真时间，默认 `true`。

能力表使用 10 个系列、21 个完整 `arm_type`。原有 21 个 `gazebo_*_demo.launch.py` 入口直接传递完整型号，文件名、参数与默认值继续保留。例如：
```
rm@rm-desktop:~$ ros2 launch rm_gazebo gazebo_<arm_type>_demo.launch.py
```
启动六维力版本机械臂的命令为（当前支持63、65、75、eco65）：
```
rm@rm-desktop:~$ ros2 launch rm_gazebo gazebo_<arm_type>_6f_demo.launch.py
```
启动一体化六维力版本机械臂的命令为（当前支持63、63_III、65、75、eco63、eco65、rx75）：
```
rm@rm-desktop:~$ ros2 launch rm_gazebo gazebo_<arm_type>_6fb_demo.launch.py
```
启动带视觉方案的六维力版本机械臂的命令为（当前仅支持rx75）：
```
rm@rm-desktop:~$ ros2 launch rm_gazebo gazebo_<arm_type>_6fb_v_demo.launch.py
```
在实际使用时需要将以上的<arm_type>更换为实际的机械臂型号，可选择的机械臂型号有65、63、63_III、75、eco62、eco63、eco65、gen72、gen72_II、rx75。对于 RX75，请使用 `gazebo_rx75_6fb_demo.launch.py` 启动 RX75-6FB，使用 `gazebo_rx75_6fb_v_demo.launch.py` 启动 RX75-6FB-V，运行成功后将弹出如下界面。  
![image](doc/rm_gazebo1.png)
对于63、65、75、eco62、eco63、eco65和gen72的标准版本，使用如下指令启动MoveIt 2：
```
rm@rm-desktop:~$ ros2 launch rm_<arm_type>_config gazebo_moveit_demo.launch.py
```
启动六维力版本机械臂的命令为（当前支持63、65、75、eco65）：
```
rm@rm-desktop:~$ ros2 launch rm_<arm_type>_config gazebo_moveit_demo_6f.launch.py
```
对于63、65、75、eco63和eco65的一体化六维力版本，使用：
```
rm@rm-desktop:~$ ros2 launch rm_<arm_type>_config gazebo_moveit_demo_6fb.launch.py
```

RML63_III和GEN72_II复用基础型号的MoveIt配置包，不能直接替换到 `rm_<arm_type>_config` 中。请使用以下准确命令：

```bash
ros2 launch rm_63_config gazebo_moveit_demo_III.launch.py
ros2 launch rm_63_config gazebo_moveit_demo_III_6fb.launch.py
ros2 launch rm_gen72_config gazebo_moveit_demo_II.launch.py
```

RX75使用独立MoveIt配置包 `rm_rx75_config`：RX75-6FB-V请执行 `ros2 launch rm_rx75_config gazebo_moveit_demo_6fb_v.launch.py`，RX75-6FB请执行 `ros2 launch rm_rx75_config gazebo_moveit_demo_6fb.launch.py`。运行成功并弹出RViz 2控制界面后，即可进行MoveIt 2和Gazebo仿真控制。
![image](doc/rm_gazebo2.png)
## rm_gazebo功能包架构说明
### 功能包文件总览
当前rm_gazebo功能包的文件构成如下。
```
├── CMakeLists.txt                              #编译规则文件
├── config
│   ├── gazebo_63_6fb_description.urdf.xacro    #RML63一体化六维力gazebo模型描述文件
│   ├── gazebo_63_III_description.urdf.xacro    #RML63_III一体化六维力gazebo模型描述文件
│   ├── gazebo_65_6fb_description.urdf.xacro    #RM65一体化六维力gazebo模型描述文件
│   ├── gazebo_75_6fb_description.urdf.xacro    #RM75一体化六维力gazebo模型描述文件
│   ├── gazebo_eco62_description.urdf.xacro     #ECO62gazebo模型描述文件
│   ├── gazebo_eco63_6fb_description.urdf.xacro #ECO63一体化六维力gazebo模型描述文件
│   ├── gazebo_eco65_6fb_description.urdf.xacro #ECO65一体化六维力gazebo模型描述文件
│   ├── gazebo_63_description.urdf.xacro        #RML63gazebo模型描述文件
│   ├── gazebo_65_description.urdf.xacro        #RM65gazebo模型描述文件
│   ├── gazebo_75_description.urdf.xacro        #RM75gazebo模型描述文件
│   ├── gazebo_eco65_description.urdf.xacro     #ECO65gazebo模型描述文件
│   ├── gazebo_eco63_description.urdf.xacro     #ECO63gazebo模型描述文件
│   ├── gazebo_gen72_description.urdf.xacro     #GEN72gazebo模型描述文件
│   ├── gazebo_gen72_II_description.urdf.xacro  #GEN72_IIgazebo模型描述文件
│   ├── gazebo_rx75_6fb_description.urdf.xacro  #RX75-6FB双臂gazebo模型描述文件
│   ├── gazebo_rx75_6fb_v_description.urdf.xacro  #RX75-6FB-V双臂gazebo模型描述文件
│   └── gazebo_rx75_description.urdf.xacro        #RX75双臂gazebo辅助描述文件
├── doc
│   ├── rm_gazebo1.png
│   └── rm_gazebo2.png
├── launch
│   ├── rm_gazebo.launch.py                #统一入口，覆盖10个系列及21个完整arm_type
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
│   ├── gazebo_eco62_demo.launch.py        #ECO62gazebo启动文件
│   ├── gazebo_eco63_6fb_demo.launch.py    #ECO63一体化六维力gazebo启动文件
│   ├── gazebo_eco63_demo.launch.py        #ECO63gazebo启动文件
│   ├── gazebo_eco65_6fb_demo.launch.py    #ECO65一体化六维力gazebo启动文件
│   ├── gazebo_eco65_6f_demo.launch.py     #ECO65六维力gazebo启动文件
│   ├── gazebo_eco65_demo.launch.py        #ECO65gazebo启动文件
│   ├── gazebo_gen72_demo.launch.py        #GEN72gazebo启动文件
│   ├── gazebo_gen72_II_demo.launch.py     #GEN72_IIgazebo启动文件
│   ├── gazebo_rx75_6fb_demo.launch.py     #RX75-6FB双臂gazebo启动文件
│   ├── gazebo_rx75_6fb_v_demo.launch.py   #RX75-6FB-V双臂gazebo启动文件
│   └── gz_demo_common.py                  #Gazebo公共编排与旧入口兼容辅助脚本
├── package.xml
├── README_CN.md
└── README.md
```
