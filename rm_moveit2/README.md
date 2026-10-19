<div align="right">

[简体中文](README_CN.md)|[English](README.md)

</div>

<div align="center">

# RealMan Robotic Arm rm_moveit2 User Manual V1.1

RealMan Intelligent Technology (Beijing) Co., Ltd.
Revision History:

|No.	  | Date   |	Comment |
| :---: | :----: | :---:   |
|V1.0    | 2026-4-16 | Draft |
|V1.1    | 2026-8-13 | Amend(Synchronize the motion examples and use RX75-6FB as the RX75 default) |

</div>

## Launch examples

After building and sourcing the workspace, run the standard RM75 motion-planning example provided by this package:

```bash
# Simulation or generation 3/4 real robot: start the matching standard RM75 MoveIt environment, then run in another terminal
ros2 launch rm_moveit2 moveit_rm75.launch.py
```

This package provides motion-planning examples for ECO63, RM75, and RX75. The launch filename selects the model; these examples do not expose `arm_type` or `mode`. Simulation and real robots use the same example entry, with the running MoveIt environment determining the target. This example uses the standard RM75. For generation 4 `6f` (formerly `6fb`) end-link selection, see the unified-entry model table. See the [generation-specific model table](../rm_bringup/README.md#unified-launch-entry).

## Content
* 1.[rm_moveit2 Package Description](#rm_moveit2_Package_Description)
* 2.[rm_moveit2 Package Use](#rm_moveit2_Package_Use)
* 2.1[Basic Package Use](#Basic_Package_Use)
* 2.2[Advanced Package Use](#Advanced_Package_Use)
* 3.[rm_moveit2 Package Architecture Description](#rm_moveit2_Package_Architecture_Description)
* 3.1[Overview of Package Files](#Overview_of_Package_Files)

## rm_moveit2_Package_Description
rm_moveit2 is a MoveIt2 control example package for RealMan robotic arms. It currently provides launch entries for ECO63, RM75, and RX75 dual-arm motion planning demos.
* 1.Package use.
* 2.Package architecture description.
Through the introduction of these two parts, it can help you:
* 1.Understand the package use.
* 2.Familiar with the file structure and function of the package.

## rm_moveit2_Package_Use
### Basic_Package_Use
Before running rm_moveit2, first start the corresponding rm_driver, rm_description, rm_control, and MoveIt configuration package nodes.
For ECO63 and RM75, the MoveIt environment can be started with the corresponding rm_<arm_type>_config package.
For RX75 dual-arm, please use the dedicated rm_rx75_config package.

Start the ECO63 example with the following command.
```
rm@rm-desktop:~$ ros2 launch rm_moveit2 moveit_eco63.launch.py
```
Start the RM75 example with the following command.
```
rm@rm-desktop:~$ ros2 launch rm_moveit2 moveit_rm75.launch.py
```
Start the RX75 dual-arm example with the following command.
```
rm@rm-desktop:~$ ros2 launch rm_moveit2 moveit_rx75.launch.py
```
Before starting the RX75 dual-arm example, launch the default RX75-6FB MoveIt environment:
```
rm@rm-desktop:~$ ros2 launch rm_rx75_config demo_6fb.launch.py
```
Use `demo_6fb_v.launch.py` only when running the RX75-6FB-V variant.

### Advanced_Package_Use
Launch arguments for `moveit_eco63.launch.py`, `moveit_rm75.launch.py` and `moveit_rx75.launch.py`:

| Argument | Default | Description |
| :--- | :--- | :--- |
| `planning_group` | `rm_group` (ECO63/RM75); `right_arm` (RX75) | MoveIt planning group: `rm_group` for ECO63/RM75; RX75 defaults to `right_arm` and also supports `left_arm` |
| `current_state_wait_sec` | `10.0` | Time to wait for the current robot state, in seconds |
| `velocity_scaling` | `0.3` | Motion velocity scaling factor, in `(0, 1]` |
| `acceleration_scaling` | `0.3` | Motion acceleration scaling factor, in `(0, 1]` |
| `planning_time` | `5.0` | MoveIt planning time, in seconds |
| `home_named_target` | `forward` | Named target for the initial pose, defined in the planning group SRDF |
| `enable_pose_target` | `false` | Plan and execute the target pose supplied by `pose_target_csv` |
| `pose_target_csv` | Empty string | Target pose as six values: `x,y,z,rx,ry,rz` |
| `pose_target_position_in_mm` | `true` | Interpret target position in millimetres; `false` uses metres |
| `pose_target_rpy_in_degrees` | `false` | Interpret target orientation in degrees; `false` uses radians |
| `pose_reference_frame` | Empty string | Target pose reference frame; an empty string uses the MoveIt planning frame |
| `prefer_named_start` | `true` | RX75 launch only: prefer the named initial pose |
| `enable_cartesian_demo` | `false` | RX75 launch only: execute the arc and straight-line examples |

## rm_moveit2_Package_Architecture_Description
### Overview_of_Package_Files
The current rm_moveit2 package is composed of the following files.
```
├── CMakeLists.txt
├── launch
│   ├── moveit_eco63.launch.py              # ECO63 launch file
│   ├── moveit_rm75.launch.py               # RM75 launch file
│   └── moveit_rx75.launch.py               # RX75 dual-arm launch file
├── package.xml
├── README.md
└── src
    ├── linear_motion_node.cpp              # MoveIt motion helper file
    └── simple_moveit_node.cpp              # main node file
```
