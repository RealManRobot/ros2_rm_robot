<div align="right">

[简体中文](README_CN.md)|[English](README.md)
 
</div>

<div align="center">

# RealMan Robotic Arm rm_gazebo User Manual V1.7

RealMan Intelligent Technology (Beijing) Co., Ltd. 

Revision History:

|No.	  | Date   |	Comment |
| :---: | :----: | :---:   |
|V1.0	  | 2024-2-19 | Draft |
|V1.1	  | 2024-7-8 | Amend(Add gen72 related adapter files) |
|V1.1.1   | 2024-8-13 | Amend(Add robotic arm model adaptation description) |
|V1.2     | 2024-9-10 | Amend(Add eco63 related adapter files) |
|V1.3     | 2024-12-25 | Amend(Add 63, 65, 75, ECO65 six-axis force adapter files and 63, 65, 75, ECO63, ECO65 integrated six-axis force adapter files) |
|V1.4    | 2025-4-3 | Amend(Add Gen72_II adapter files) |
|V1.5    | 2025-11-13 | Amend(Add RML63_III adapter files) |
|V1.6    | 2026-4-16 | Amend(Add ECO62 and RX75 adapter files, update Gazebo version) |
|V1.7    | 2026-8-14 | Amend(Add the Foxy Gazebo Classic unified entry and synchronized simulation models) |

</div>

## Launch examples

After building and sourcing the workspace, simulate an RM65 with integrated force sensing in Gazebo:

```bash
# Simulation (generation 3 and 4 both retain the 6fb model name)
ros2 launch rm_gazebo rm_gazebo.launch.py \
  arm_type:=65-6fb
```

This package starts Gazebo simulation, which does not connect to a controller or depend on its software version. Generation 4 software 1.0.6 and later names the former `6fb` end link `6f`; keep `65-6fb` when simulating the same geometry. See the [generation-specific model table](../rm_bringup/README.md#unified-launch-entry).

## Content
* 1.[rm_gazebo Package Description](#rm_gazebo_Package_Description)
* 2.[rm_gazebo Package Running](#rm_gazebo_Package_Running)
* 2.1[Control of the simulation robotic arm](#Control_of_the_simulation_robotic_arm)
* 3.[rm_gazebo Package Architecture Description](#rm_gazebo_Package_Architecture_Description)
* 3.1[Overview of Package Files](#Overview_of_Package_Files)

## rm_gazebo_Package_Description
rm_gazebo is mainly used for realizing the simulation function of robot arm Moveit2 planning. We build a virtual robotic arm in the simulation environment of Gazebo, and then control the virtual robot arm in Gazebo through Moveit2. This package is introduced in detail in the following aspects.
* 1.Package use.
* 2.Package architecture description.
Through the introduction of this part, it can help you:
* 1.Understand the package use.
* 2.Familiar with the file structure and function of the package.
Source code address: https://github.com/RealManRobot/ros2_rm_robot.git。
## rm_gazebo_Package_Running
The unified entry point is recommended:

```bash
ros2 launch rm_gazebo rm_gazebo.launch.py \
  arm_type:=65-6f
```

This Foxy branch deliberately uses Gazebo Classic (`gazebo_ros` and
`gazebo_ros2_control`), not the newer `ros_gz` stack. `arm_type` is required;
Select the model directly with `arm_type`, such as `65-6f`.

| Argument | Default | Description |
| :--- | :--- | :--- |
| `arm_type` | Required | Model and end-link version, e.g. `65-6f` or `rx75-6fb` |
| `model` | `auto` | `auto` prefers matching DAE and falls back to STL; `stl` selects the original meshes, `dae` requires matching DAE |
| `start_gazebo` | `true` | Start Gazebo Classic; `false` connects to an existing Gazebo world |
| `use_gazebo_gui` | `true` | Open the graphical interface when starting Gazebo |
| `clock_topic` | `auto` | ROS simulation clock topic; `auto` uses `/clock` |
| `joint_states_topic` | `auto` | Currently accepts only `auto`, selecting the default joint-state topic for the model |
| `use_sim_time` | `true` | Use the Gazebo simulation clock in robot_state_publisher |
| `spawn_entity_timeout` | `120` | Timeout for waiting for the Gazebo spawn service, in seconds |

The 14 model/end-link combinations with DAE assets default to colored DAE in Gazebo Classic and RViz2; other models keep STL. Only visuals change; collision and physics stay unchanged.

Controller startup is sequenced after successful entity spawn,
and a failed entity/controller process requests launch shutdown. The 21 old
Gazebo files remain compatibility wrappers.

### Control_of_the_simulation_robotic_arm
After the installation of the environment and the package, we can run the rm_gazebo package.  
Use the following command to launch the Gazebo virtual space and the virtual robotic arm.
```
rm@rm-desktop:~$ ros2 launch rm_gazebo gazebo_65_demo.launch.py
```
The command to start the six-axis force version is currently available for 63, 65, 75, and eco65:
```
rm@rm-desktop:~$ ros2 launch rm_gazebo gazebo_<arm_type>_6f_demo.launch.py
```
The command to start the integrated six-axis force version is currently available for 63, 63_III, 65, 75, eco63, eco65, and rx75:
```
rm@rm-desktop:~$ ros2 launch rm_gazebo gazebo_<arm_type>_6fb_demo.launch.py
```
The command to start the vision-enabled integrated six-axis force version is currently only available for rx75:
```
rm@rm-desktop:~$ ros2 launch rm_gazebo gazebo_<arm_type>_6fb_v_demo.launch.py
```
In practice, the above <arm_type> needs to be replaced by the actual model of the robotic arm. The available models are 65, 63, 63_III, 75, eco62, eco63, eco65, gen72, gen72_II, and rx75. For RX75, use `gazebo_rx75_6fb_demo.launch.py` for RX75-6FB and `gazebo_rx75_6fb_v_demo.launch.py` for RX75-6FB-V. The interface displays as follows after successful running.  
![image](doc/rm_gazebo1.png)
Then, we use the following command to launch moveit2 to control the simulation robot arm in Gazebo.
```
rm@rm-desktop:~$ ros2 launch rm_<arm_type>_config gazebo_moveit_demo.launch.py
```
The command to start the six-axis force version is currently available for 63, 65, 75, and eco65:
```
rm@rm-desktop:~$ ros2 launch rm_<arm_type>_config gazebo_moveit_demo_6f.launch.py
```
The command to start the integrated six-axis force version is currently available for 63, 63_III, 65, 75, eco63, and eco65:
```
rm@rm-desktop:~$ ros2 launch rm_<arm_type>_config gazebo_moveit_demo_6fb.launch.py
```
In practice, the above <arm_type> needs to be replaced by the actual model of the robotic arm. The available models are 65, 63, 63_III, 75, eco62, eco63, eco65, gen72, and gen72_II. RX75 uses the dedicated MoveIt package `rm_rx75_config`: run `ros2 launch rm_rx75_config gazebo_moveit_demo_6fb_v.launch.py` for RX75-6FB-V and `ros2 launch rm_rx75_config gazebo_moveit_demo_6fb.launch.py` for RX75-6FB.   
After the control interface of rviz2 pops up, you can perform the simulation control of moveit2 and Gazebo.
![image](doc/rm_gazebo2.png)
## rm_gazebo_Package_Architecture_Description
## Overview_of_Package_Files
The current rm_gazebo package is composed of the following files.
```
├── CMakeLists.txt                # compilation rule file
├── config
│   ├── gazebo_63_6fb_description.urdf.xacro    #RML63 integrated six-axis force gazebo launch file
│   ├── gazebo_63_III_description.urdf.xacro    #RML63_III integrated gazebo launch file
│   ├── gazebo_65_6fb_description.urdf.xacro    #RM65 integrated six-axis force gazebo launch file
│   ├── gazebo_75_6fb_description.urdf.xacro    #RM75 integrated six-axis force gazebo launch file
│   ├── gazebo_eco62_description.urdf.xacro     #ECO62 gazebo model description file
│   ├── gazebo_eco63_6fb_description.urdf.xacro #ECO63 integrated six-axis force gazebo launch file
│   ├── gazebo_eco65_6fb_description.urdf.xacro #ECO65 integrated six-axis force gazebo launch file
│   ├── gazebo_63_description.urdf.xacro     #63gazebo model description file
│   ├── gazebo_65_description.urdf.xacro     #65gazebo model description file
│   ├── gazebo_75_description.urdf.xacro     #75gazebo model description file
│   ├── gazebo_eco65_description.urdf.xacro  #eco65gazebo model description file
│   ├── gazebo_eco63_description.urdf.xacro  #eco63gazebo model description file
│   ├── gazebo_gen72_II_description.urdf.xacro #gen72_IIgazebo model description file
│   ├── gazebo_gen72_description.urdf.xacro  #gen72gazebo model description file
│   ├── gazebo_rx75_6fb_description.urdf.xacro #RX75-6FB dual-arm gazebo model description file
│   ├── gazebo_rx75_6fb_v_description.urdf.xacro #RX75-6FB-V dual-arm gazebo model description file
│   └── gazebo_rx75_description.urdf.xacro       #RX75 dual-arm gazebo helper file
├── doc
│   ├── rm_gazebo1.png
│   └── rm_gazebo2.png
├── launch
│   ├── rm_gazebo.launch.py                 # unified Gazebo Classic entry
│   ├── gazebo_63_6fb_demo.launch.py       #63 integrated six-axis force gazebo launch file
│   ├── gazebo_63_6f_demo.launch.py        #63 six-axis force gazebo launch file
│   ├── gazebo_63_demo.launch.py           #63 gazebo launch file
│   ├── gazebo_63_III_6fb_demo.launch.py   #63_III integrated six-axis force gazebo launch file
│   ├── gazebo_63_III_demo.launch.py       #63_III gazebo launch file
│   ├── gazebo_65_6fb_demo.launch.py       #RM65 integrated six-axis force gazebo launch file
│   ├── gazebo_65_6f_demo.launch.py        #RM65 six-axis force gazebo launch file
│   ├── gazebo_65_demo.launch.py           #RM65 gazebo launch file
│   ├── gazebo_75_6fb_demo.launch.py       #RM75 integrated six-axis force gazebo launch file
│   ├── gazebo_75_6f_demo.launch.py        #RM75 six-axis force gazebo launch file
│   ├── gazebo_75_demo.launch.py           #RM75 gazebo launch file
│   ├── gazebo_eco62_demo.launch.py        #ECO62 gazebo launch file
│   ├── gazebo_eco63_6fb_demo.launch.py    #ECO63 integrated six-axis force gazebo launch file
│   ├── gazebo_eco63_demo.launch.py        #ECO63 gazebo launch file
│   ├── gazebo_eco65_6fb_demo.launch.py    #ECO65 integrated six-axis force gazebo launch file
│   ├── gazebo_eco65_6f_demo.launch.py     #ECO65 six-axis force gazebo launch file
│   ├── gazebo_eco65_demo.launch.py        #ECO65 gazebo launch file
│   ├── gazebo_gen72_II_demo.launch.py     #gen72_IIgazebo launch file
│   ├── gazebo_gen72_demo.launch.py        #gen72gazebo launch file
│   ├── gazebo_rx75_6fb_demo.launch.py     #RX75-6FB dual-arm gazebo launch file
│   ├── gazebo_rx75_6fb_v_demo.launch.py   #RX75-6FB-V dual-arm gazebo launch file
│   └── gz_demo_common.py                  #shared Gazebo launch helper
├── package.xml
├── README_CN.md
└── README.md
```
