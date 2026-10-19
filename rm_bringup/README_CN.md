<div align="right">
 
[简体中文](README_CN.md)|[English](README.md)

</div>

<div align="center">

# 睿尔曼机器人rm_bringup使用说明书V1.7
 
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
|V1.6    |2026-4-16 |修订(添加ECO62、RX75适配文件) |
|V1.7    |2026-8-7 |修订(添加统一启动入口) |

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
### 统一启动入口
推荐使用 `rm_bringup.launch.py`，通过参数选择机械臂、末端版本和运行模式；原有按型号拆分的 launch 命令继续兼容。

```bash
# 仿真（沿用三代 / 四代旧软件的 6fb 模型命名）
ros2 launch rm_bringup rm_bringup.launch.py \
  arm_type:=65-6fb mode:=gazebo

# 控制真实机械臂（四代控制器软件 > 1.0.5）
ros2 launch rm_bringup rm_bringup.launch.py \
  arm_type:=65-6f

# 注：四代控制器软件从 1.0.6 起将原 6fb 末端统一命名为 6f，四代真机按 6f 选择。
```

仿真不连接控制器，同一结构始终使用 `6fb` 模型名。三代或四代旧软件的真机使用 `arm_type:=65-6fb`；`mode` 默认 `real`，`allow_trajectory_execution` 默认 `true`。

| 参数 | 默认值 | 说明 |
| :--- | :--- | :--- |
| `arm_type` | 无，必须指定 | 型号及末端版本，如 `65`、`65-6f`、`eco63-6fb`；完整可选值见下表 |
| `mode` | `real` | `real` 或 `gazebo` |
| `model` | `auto` | `auto/stl/glb`；`auto` 优先使用配套 GLB，其余变体使用 STL |
| `allow_trajectory_execution` | `true` | 是否允许 MoveIt 执行轨迹；设为 `false` 时仅规划，不执行 |
| `use_moveit` | `true` | 是否启动 MoveIt；为 `false` 时还需设置 `use_rviz:=false` |
| `use_rviz` | `true` | 是否启动 MoveIt RViz |
| `driver_config` | `auto` | 真机单臂 driver YAML；RX75 请使用左右两项 |
| `left_driver_config` | `auto` | RX75 左臂 driver YAML |
| `right_driver_config` | `auto` | RX75 右臂 driver YAML |
| `follow` | `auto` | 真机控制跟随模式：`auto/true/false`；`auto` 对所有型号均为 `false`，可手动设为 `true` |
| `joint_states_topic` | `auto` | 当前仅支持 `auto`，按型号和模式选择默认 joint states Topic |
| `start_gazebo` | `true` | Gazebo 模式下是否启动仿真进程 |

以下为 10 个系列的统一真机入口选择。四代新命名适用于 四代控制器软件 > `1.0.5`；旧软件继续使用 `6fb/6fb-v/6f`。不带后缀时选择标准版：

| 型号 | 三代 arm_type 可选值 | 四代新命名 arm_type 可选值 |
| :--- | :--- | :--- |
| `63` | `63`, `63-6f`, `63-6fb` | `63`, `63-6f`；也可以使用 `63-6fb` |
| `63_iii` | `63_iii`, `63_iii-6fb` | `63_iii`, `63_iii-6f`；也可以使用 `63_iii-6fb` |
| `65` | `65`, `65-6f`, `65-6fb` | `65`, `65-6f`；也可以使用 `65-6fb` |
| `75` | `75`, `75-6f`, `75-6fb` | `75`, `75-6f`；也可以使用 `75-6fb` |
| `eco62` | `eco62` | `eco62` |
| `eco63` | `eco63`, `eco63-6fb` | `eco63`, `eco63-6f`；也可以使用 `eco63-6fb` |
| `eco65` | `eco65`, `eco65-6f`, `eco65-6fb` | `eco65`, `eco65-6f`；也可以使用 `eco65-6fb` |
| `gen72` | `gen72` | `gen72` |
| `gen72_ii` | `gen72_ii` | `gen72_ii` |
| `rx75` | `rx75-6fb`, `rx75-6fb-v` | `rx75-6f`, `rx75-6f-v`；也可以使用 `rx75-6fb`, `rx75-6fb-v` |

三代的 `-6f` 表示旧六维力结构；四代同名 `-6f` 表示原一体化 `-6fb` 结构。
单独显示和 Gazebo 不读取代际，仍按原模型名称选择；查看或仿真四代同款一体化结构时使用 `65-6fb`，不要用 `65-6f` 代替。
不支持的组合会报错，例如 ECO62 不提供 `6fb`。`allow_trajectory_execution` 默认 `true`；需要禁止执行轨迹时，可手动设为 `false`。

统一入口直接组合 `rm_driver`、`rm_description`、`rm_control` 和
`rm_gazebo` 的通用 launch；各组件接收解析后的完整 `arm_type`，例如 `65-6fb`。
Driver/Control 在内部选择基础型号的 YAML、硬件代码和跟随默认值。
原有 42 个按型号入口已改成薄兼容 wrapper，旧命令及 RX75 的
`use_moveit_rviz` 参数继续可用。Gazebo 仍按历史行为在 8 秒后启动
MoveIt；controller readiness 时序优化将在后续独立实施。

### 模型选择

`mode:=real` 默认 `model:=auto`，有配套 GLB 的 14 个变体使用彩色模型，TF 与 MoveIt
读取相同的机器人描述。`model:=stl` 显式选择原 STL，`model:=glb` 强制使用 GLB；
缺少该变体的 GLB 时会在启动节点前报错。

`mode:=gazebo` 下，有配套 GLB 的 14 个型号/末端组合默认在 Gazebo 与 RViz2 中使用彩色 GLB；其余型号使用原 STL。添加 `model:=stl` 可切回原网格，碰撞和物理参数保持不变。

### moveit2控制真实机械臂
首先配置好环境完成连接后我们可以通过以下命令直接启动节点，运行rm_bringup功能包中的launch.py文件。
```
rm@rm-desktop:~$ ros2 launch rm_bringup rm_<arm_type>_bringup.launch.py
```
在实际使用时需要将以上的<arm_type>更换为实际的机械臂型号，可选择的机械臂型号有65、63、63_III、75、eco62、eco63、eco65、gen72、gen72_II、rx75。

启动六维力版本机械臂的命令为（当前支持63、65、75、eco65）：
```
rm@rm-desktop:~$ ros2 launch rm_bringup rm_<arm_type>_6f_bringup.launch.py
```
启动一体化六维力版本机械臂的命令为（当前支持63、63_III、65、75、eco63、eco65、rx75）：
```
rm@rm-desktop:~$ ros2 launch rm_bringup rm_<arm_type>_6fb_bringup.launch.py
```
启动带视觉方案的六维力版本机械臂的命令为（当前仅支持rx75）：
```
rm@rm-desktop:~$ ros2 launch rm_bringup rm_<arm_type>_6fb_v_bringup.launch.py
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
在实际使用时需要将以上的<arm_type>更换为实际的机械臂型号，可选择的机械臂型号有65、63、63_III、75、eco62、eco63、eco65、gen72、gen72_II、rx75。

启动六维力版本机械臂的命令为（当前支持63、65、75、eco65）：
```
rm@rm-desktop:~$ ros2 launch rm_bringup rm_<arm_type>_6f_gazebo.launch.py
```
启动一体化六维力版本机械臂的命令为（当前支持63、63_III、65、75、eco63、eco65、rx75）：
```
rm@rm-desktop:~$ ros2 launch rm_bringup rm_<arm_type>_6fb_gazebo.launch.py
```
启动带视觉方案的六维力版本机械臂的命令为（当前仅支持rx75）：
```
rm@rm-desktop:~$ ros2 launch rm_bringup rm_<arm_type>_6fb_v_gazebo.launch.py
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
│   ├── rm_bringup.launch.py             #所有支持型号、末端版本和运行模式的统一启动入口
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
│   ├── rm_eco62_bringup.launch.py      #eco62臂moveit2启动文件
│   ├── rm_eco62_gazebo.launch.py       #eco62臂gazebo启动文件
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
│   ├── rm_gen72_II_gazebo.launch.py    #gen72_II臂gazebo启动文件
│   ├── rm_rx75_6fb_bringup.launch.py   #RX75-6FB双臂moveit2启动文件
│   ├── rm_rx75_6fb_gazebo.launch.py    #RX75-6FB双臂gazebo启动文件
│   ├── rm_rx75_6fb_v_bringup.launch.py #RX75-6FB-V双臂moveit2启动文件
│   └── rm_rx75_6fb_v_gazebo.launch.py  #RX75-6FB-V双臂gazebo启动文件
├── rm_bringup                         #统一启动入口使用的Python模块
│   ├── __init__.py                      #Python包标记文件
│   ├── legacy_bringup.py                #42个旧入口的兼容wrapper工厂
│   ├── real_moveit.py                   #真机 GLB MoveIt 节点构造
│   └── variant_catalog.py               #完整arm_type能力表、硬件profile和组件映射
├── package.xml                         #依赖说明文件
├── README_CN.md                        #中文说明文档
└── README.md                           #英文说明文档
```
## rm_bringup话题说明
该功能包当前并没有本身的话题，主要为调用其他功能包的话题实现，关于moveit2相关话题可查看《[rm_moveit2_config详解](https://github.com/RealManRobot/ros2_rm_robot/blob/main/rm_moveit2_config/README_CN.md)》相关内容。
