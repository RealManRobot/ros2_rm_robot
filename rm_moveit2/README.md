<div align="right">

[简体中文](https://github.com/RealManRobot/ros2_rm_robot/blob/humble/rm_moveit2/README_CN.md)|[English](https://github.com/RealManRobot/ros2_rm_robot/blob/humble/rm_moveit2/README.md)
 
</div>

<div align="center">

# RealMan Robotic Arm rm_moveit2 User Manual V1.0
 
RealMan Intelligent Technology (Beijing) Co., Ltd. 
Revision History:

|No.	  | Date   |	Comment |
| :---: | :----: | :---:   |
|V1.0    | 2026-4-16 | Draft |

</div>

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
Before starting the RX75 dual-arm example, use the following command to launch the corresponding MoveIt environment.
```
rm@rm-desktop:~$ ros2 launch rm_rx75_config demo_6fb_v.launch.py
```

### Advanced_Package_Use
The common parameters of rm_moveit2 are configured in the launch files.  
Parameter planning_group: the planning group used by MoveIt. The default value is rm_group for ECO63 and RM75, and right_arm for RX75.  
Parameter current_state_wait_sec: the waiting time for reading the current state.  
Parameter velocity_scaling: the motion velocity scaling factor.  
Parameter acceleration_scaling: the motion acceleration scaling factor.  
Parameter planning_time: the planning time of MoveIt.  
Parameter home_named_target: the named target used when moving to the initial posture.  
Parameter enable_pose_target: whether to directly plan to the pose target given by pose_target_csv.  
Parameter pose_target_csv: the pose target in x,y,z,rx,ry,rz format.  
Parameter pose_reference_frame: the reference frame used for the pose target.  
Parameter prefer_named_start: this parameter is only used by the RX75 dual-arm launch file to prefer the named start posture.  
Parameter enable_cartesian_demo: this parameter is only used by the RX75 dual-arm launch file to control whether the Cartesian demo is executed.  
For the RX75 dual-arm example, planning_group can be set to left_arm or right_arm according to the selected arm side.

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
