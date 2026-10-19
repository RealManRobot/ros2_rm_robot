<div align="right">

[简体中文](README_CN.md)|[English](README.md)

</div>

<div align="center">

# 睿尔曼机械臂rm_moveit2使用说明书V1.1

睿尔曼智能科技（北京）有限公司
文件修订记录：

| 版本号 |   时间   | 备注 |
| :----: | :-------: | :--: |
|  V1.0  | 2026-4-16 | 拟制 |
|  V1.1  | 2026-8-13 | 修订（同步运动示例） |

</div>

## 启动示例

构建并 source 工作区后，以本包提供的标准版 RM75 运动规划示例为例：

```bash
# 仿真 / 三代或四代真机：先启动对应的 RM75 标准版 MoveIt 环境，再在新终端运行
ros2 launch rm_moveit2 moveit_rm75.launch.py
```

本包提供 ECO63、RM75 和 RX75 的运动规划示例，通过 launch 文件选择型号，不提供 `arm_type` 或 `mode` 参数。仿真与真机使用同一示例入口，由预先启动的 MoveIt 环境决定控制对象；本例使用标准版 RM75。四代 `6f`（原 `6fb`）的末端选择见统一入口型号表。完整型号见[三、四代型号表](../rm_bringup/README_CN.md#统一启动入口)。

## 目录

* 1.[rm_moveit2功能包说明](#rm_moveit2功能包说明)
* 2.[rm_moveit2功能包使用](#rm_moveit2功能包使用)
* 2.1[基础功能包使用](#基础功能包使用)
* 2.2[高级功能包使用](#高级功能包使用)
* 3.[rm_moveit2功能包架构说明](#rm_moveit2功能包架构说明)
* 3.1[功能包文件总览](#功能包文件总览)

## rm_moveit2功能包说明

rm_moveit2是睿尔曼机械臂的MoveIt2控制示例功能包，目前提供ECO63、RM75和RX75双臂运动规划示例的启动入口。

* 1.功能包使用。
* 2.功能包架构说明。
  通过这两部分内容的介绍可以帮助大家：
* 1.了解该功能包的使用。
* 2.熟悉功能包中的文件构成及作用。

## rm_moveit2功能包使用

### 基础功能包使用

运行rm_moveit2之前，需要先启动对应的rm_driver、rm_description、rm_control以及MoveIt配置功能包节点。
对于ECO63和RM75，可通过对应的rm_<arm_type>_config功能包启动MoveIt环境。
如需启动其它机械臂的moveit可以通过更改launch中的moveit_config参数实现。

使用以下命令启动ECO63示例。

```
rm@rm-desktop:~$ ros2 launch rm_moveit2 moveit_eco63.launch.py
```

使用以下命令启动RM75示例。

```
rm@rm-desktop:~$ ros2 launch rm_moveit2 moveit_rm75.launch.py
```

使用以下命令启动RX75双臂示例。

```
rm@rm-desktop:~$ ros2 launch rm_moveit2 moveit_rx75.launch.py
```

启动RX75示例前，默认使用RX75-6FB的MoveIt环境：

```
rm@rm-desktop:~$ ros2 launch rm_rx75_config demo_6fb.launch.py
```

仅在使用RX75-6FB-V时改用 `demo_6fb_v.launch.py`。

### 高级功能包使用

以下为 `moveit_eco63.launch.py`、`moveit_rm75.launch.py` 和 `moveit_rx75.launch.py` 的启动参数：

| 参数 | 默认值 | 说明 |
| :--- | :--- | :--- |
| `planning_group` | `rm_group`（ECO63/RM75）；`right_arm`（RX75） | MoveIt 规划组；ECO63/RM75 使用 `rm_group`，RX75 默认 `right_arm`，可改为 `left_arm` |
| `current_state_wait_sec` | `10.0` | 等待当前机器人状态的时间，单位 s |
| `velocity_scaling` | `0.3` | 运动速度缩放系数，范围 `(0, 1]` |
| `acceleration_scaling` | `0.3` | 运动加速度缩放系数，范围 `(0, 1]` |
| `planning_time` | `5.0` | MoveIt 规划时间，单位 s |
| `home_named_target` | `forward` | 初始姿态的命名目标，需在对应规划组的 SRDF 中定义 |
| `enable_pose_target` | `false` | 是否直接规划并执行 `pose_target_csv` 给定的目标位姿 |
| `pose_target_csv` | 空字符串 | 目标位姿，按 `x,y,z,rx,ry,rz` 填写六个值 |
| `pose_target_position_in_mm` | `true` | 目标位置是否以 mm 输入；`false` 时使用 m |
| `pose_target_rpy_in_degrees` | `false` | 目标姿态是否以度输入；`false` 时使用 rad |
| `pose_reference_frame` | 空字符串 | 目标位姿参考坐标系；空字符串使用 MoveIt 规划坐标系 |
| `prefer_named_start` | `true` | 仅 RX75 launch：是否优先采用命名起始姿态 |
| `enable_cartesian_demo` | `false` | 仅 RX75 launch：是否执行圆弧与直线示例 |

## rm_moveit2功能包架构说明

### 功能包文件总览

当前rm_moveit2功能包的文件构成如下。

```
├── CMakeLists.txt
├── launch
│   ├── moveit_eco63.launch.py              # ECO63启动文件
│   ├── moveit_rm75.launch.py               # RM75启动文件
│   └── moveit_rx75.launch.py               # RX75双臂启动文件
├── package.xml
├── README_CN.md
├── README.md
└── src
    ├── linear_motion_node.cpp              # MoveIt运动辅助文件
    └── simple_moveit_node.cpp              # 主节点文件
```
