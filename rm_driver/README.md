<div align="right">

[简体中文](README_CN.md)|[English](README.md)
 
</div>

<div align="center">

# RealMan Robot rm_driver User Manual V1.7.1

RealMan Intelligent Technology (Beijing) Co., Ltd. 

Revision History:

|No.	  | Date   |	Comment |
| :---: | :----: | :---:   |
|V1.0	  | 2024-2-7 | Draft |
|V1.1	  | 2024-7-8 | Amend(Add GEN72 adapter files) |
|V1.2	  | 2024-9-10 | Amend(Add ECO63 adapter files) |
|V1.2.1 | 2024-10-31 | Amend(Add dexterous hand high-speed adaptation) |
|V1.3   | 2024-12-25 | Amend(Add UDP reporting adaptation) |
|V1.4   | 2024-12-25 | Amend(Add UDP reporting adaptation) |
|V1.5   | 2025-5-29 | Amend(Adapt to fourth-generation controllers, add a version query interface, Cartesian space linear offset motion interface, Modbus interface, and trajectory list interface; see the topic interface description document for details) |
|V1.6   | 2025-11-13 | Amend(Add basic UDP enable configurations) |
|V1.7   | 2026-4-16 | Amend(Add ECO62 and RX75 adapter files) |
|V1.7.1 | 2026-8-14 | Amend(Add SDK V1.1.6, unified launch, Modbus timeouts, endpoint ownership, and signal-safe cleanup for Foxy) |

</div>

 

## Launch examples

After building and sourcing the workspace, for an RM65 with integrated force sensing:

```bash
# Connect to the real robot (generation 3 and 4 both use the base model)
ros2 launch rm_driver rm_driver.launch.py \
  arm_type:=65

# Note: Generation 4 software 1.0.6 and later names the former 6fb end link 6f.
# rm_driver and rm_control only need a base model, such as 63, 65, 75, eco63 or eco65.
# The naming difference does not change these launch arguments.
```

This package is part of the real-robot control chain; Gazebo uses simulation controllers. `arm_type` selects the base-family configuration. See the [generation-specific model table](../rm_bringup/README.md#unified-launch-entry).

## Content
* 1.rm_driver package description
* 2.rm_driver package use
* 2.1 Basic use of the package
* 2.2 Advanced use of the package
* 3.rm_driver package architecture description
* 3.1 Overview of Package Files
* 4.rm_driver topic description



 
## 1. rm_driver package description
rm_driver package is very important in the ROS2 robotic arm package. This package realizes the function of controlling the robotic arm through communication between ROS and the robotic arm. The package will be introduced in detail in the following text through the following aspects:
* 1.Package use.
* 2.Package architecture description.
* 3.Package topic description.

Through the introduction of the three parts, it can help you:
* 1.Understand the package use.
* 2.Familiar with the file structure and function of the package.
* 3.Familiar with the topic related to the package for easy development and use.
Source code address:https://github.com/RealManRobot/ros2_rm_robot.git.
## 2. rm_driver package use
### 2.1 Basic use of the package
The unified entry point is recommended:

```bash
ros2 launch rm_driver rm_driver.launch.py arm_type:=65
```

| Argument | Default | Description |
| :--- | :--- | :--- |
| `arm_type` | Required | Model with an optional end-link suffix, e.g. `65`, `65-6fb` or `rx75`; see the [model table](../rm_bringup/README.md). Driver configuration is selected by the base model |
| `driver_config` | `auto` | Single-arm driver YAML path; `auto` selects the model default |
| `left_driver_config` | `auto` | RX75 only: left-arm driver YAML path; `auto` selects the default left-arm configuration |
| `right_driver_config` | `auto` | RX75 only: right-arm driver YAML path; `auto` selects the default right-arm configuration |

Invalid model/config combinations and missing YAML files fail before a driver
node starts. RX75 launches two namespaced driver nodes. The eight historical
model launch files remain supported as compatibility wrappers.

The driver takes a host-local non-blocking lock on `arm_ip:tcp_port` before SDK
initialization. A second process using the same endpoint exits non-zero; a
different endpoint may run concurrently. SIGINT, SIGTERM, and SIGHUP converge
on the same idempotent SDK cleanup path. This cleanup is best effort and does
not replace a controller watchdog or hardware emergency stop for SIGKILL,
power loss, or process crashes.

Modbus V1.7.1 message additions:

- `RS485params.timeout`: third-generation Modbus RTU timeout in 100 ms units;
  non-positive input uses the compatible default `5` (500 ms).
- `Modbustcpmasterinfo.timeout`: third-generation Modbus TCP timeout in ms;
  non-positive input uses the compatible default `2000` ms.
- Fourth-generation query responses report `timeout=0` because that SDK path
  does not expose a timeout value.

First, after configuring the environment and completing the connection, we can directly start the node and control the robotic arm through the following command.
The current control is based on the fact that we have not changed the IP of the robotic arm, which is still 192.168.1.18.
rm@rm-desktop:~$ ros2 launch rm_driver rm_<arm_type>_driver.launch.py
In practice, the above <arm_type> needs to be replaced by the actual model of the robotic arm. The available models of the robotic arm are 65, 63, eco65, eco63, eco62, gen72, 75, and rx75.
The following screen will appear if the underlying driver is successfully started.
![image](doc/rm_driver1.png)  
### 2.2 Advanced use of the package
When our robotic arm's IP is changed, our start command is invalid. If we use the above command directly, we cannot successfully connect to the robotic arm. We can re-establish the connection by modifying the following configuration file.
The configuration file is located in the config folder under our rm_driver package.
![image](doc/rm_driver4.png)  

The contents of the configuration file are as follows:

```
rm_driver: 
  ros__parameters:
    #robot param
    arm_ip: "192.168.1.18"        # Set the IP address for the TCP connection
    tcp_port: 8080#               # Set the port for the TCP connection
    
    arm_type: "RM_65"             # set the robotic arm model       
    arm_dof: 6                    # Set the degree of freedom of the robotic arm

    udp_ip: "192.168.1.10"        # set the udp active reporting IP address
    udp_cycle: 5                  # the active reporting cycle of UDP, which needs to be a multiple of 5.  
    udp_port: 8089                # Set the udp active reporting port   
    udp_force_coordinate: 0       # Set the base coordinate of the six-axis force when the system is forced, where 0 is the sensor coordinate system, 1 is the current work coordinate system, and 2 is the current tool coordinate system
    udp_hand: false               # Set the udp hand reporting enable
    udp_plus_base: false          # Set the udp plus base reporting enable
    udp_plus_state: false         # Set the udp plus state reporting enable
    udp_joint_speed_state: true   # Set the udp joint speed reporting enable
    udp_lift_state: true          # Set the udp lift state reporting enable
    udp_expand_state: false       # Set the udp expand state reporting enable
    udp_arm_current_status: true  # Set the udp arm_current status reporting enable
    udp_aloha_state: true         # Set the udp plus base reporting enable
    trajectory_mode: 0            #When the high following mode is set, multiple modes are supported, including 0- complete transparent transmission mode, 1- curve fitting mode and 2- filtering mode.
    radio: 0                     #Set the smoothing coefficient in curve fitting mode (range 0-100) or the filter parameter in filtering mode (range 0-1000). The higher the value, the better the smoothing effect.
    arm_joints: ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
```

There are mainly the following parameters.
* arm_ip: This parameter represents the current IP of the robotic arm
* tcp_port: set the port when TCP is connected.
* arm_type: This parameter represents the current model of the robotic arm. The parameters that can be selected are RM_65 (65 series), RM_eco65 (ECO65 series), RM_eco63 (ECO63 series), RM_eco62 (ECO62 series), RM_63 (63 series), GEN_72 (GEN72 series), and RM_75 (75 series; the RX75 dual-arm configuration also uses this model type for each arm).
* arm_dof: set the degree of freedom of the robotic arm. 6 is 6 degrees of freedom, and 7 is 7 degrees of freedom.
* udp_ip: set the udp active reporting IP address.
* udp_cycle: the active reporting cycle of UDP, which needs to be a multiple of 5.
* udp_port: set the udp active reporting port.
* udp_force_coordinate: set the base coordinate of the six-axis force when the system is forced, where 0 is the sensor coordinate system (original data), 1 is the current work coordinate system, and 2 is the current tool coordinate system.
* udp_hand: false               # Set the udp hand reporting enable
* udp_plus_base: false          # Set the udp plus base reporting enable
* udp_plus_state: false         # Set the udp plus state reporting enable
* udp_joint_speed_state: true   # Set the udp joint speed reporting enable
* udp_lift_state: true          # Set the udp lift state reporting enable
* udp_expand_state: false       # Set the udp expand state reporting enable
* udp_arm_current_status: true  # Set the udp arm_current status reporting enable
* udp_aloha_state: true         # Set the udp plus base reporting enable
* trajectory_mode：When the high following mode is set, multiple modes are supported, including 0- complete transparent transmission mode, 1- curve fitting mode and 2- filtering mode.
* radio：Set the smoothing coefficient in curve fitting mode (range 0-100) or the filter parameter in filtering mode (range 0-1000). The higher the value, the better the smoothing effect.
* In practice, we choose the corresponding launch file to start, which will automatically select the correct model. If there are special requirements, you can modify the corresponding parameters here. After modification, recompile the configuration in the workspace directory, and then the modified configuration will take effect.
Run the colcon build command in the workspace directory.

```
rm@rm-desktop: ~/ros2_ws$ colcon build
```

After successful compilation, follow the above commands to start the package.
## 3. rm_driver package architecture description
### 3.1 Overview of package files
The current rm_driver package is composed of the following files.

```
├── CMakeLists.txt                # compilation rule file
├── config                        # config folder
│   ├── rm_63_config.yaml         # 63 configuration file
│   ├── rm_65_config.yaml         # 65 configuration file
│   ├── rm_75_config.yaml         # 75 configuration file
│   ├── rm_eco62_config.yaml      # eco62 configuration file
│   ├── rm_eco65_config.yaml      # eco65 configuration file
│   ├── rm_eco63_config.yaml      # eco63 configuration file
│   ├── rm_gen72_config.yaml      # gen72 configuration file
│   ├── rm_rx75_left_config.yaml  # RX75 left-arm configuration file
│   └── rm_rx75_right_config.yaml # RX75 right-arm configuration file
├── doc
│   ├── Description of Function Expansion for Realman Robotic Arm ROS Driver Package.md
│   ├── RealMan Robotic Arm rm_driver Topic Detailed Description (ROS2).md
│   ├── rm_driver1.png
│   ├── rm_driver2.png
│   ├── rm_driver3.png
│   ├── rm_driver4.png
│   ├── rm_driver5.png
│   ├── rm_driver6.png
│   ├── rm_driver7.png
│   ├── rm_driver8.png
│   ├── rm_driver9.png
│   ├── rm_driver10.png
│   ├── rm_driver11.png
│   ├── rm_driver12.png
│   ├── rm_driver13.png
│   ├── 睿尔曼机械臂ROS2rm_driver话题详细说明.md
│   └── 睿尔曼机械臂ROS驱动包功能拓展说明.md
├── include                       # dependency header file folder
│   └── rm_driver/
│       ├── rm_define.h                               # SDK data types, enums and constants
│       ├── rm_driver.h                               # ROS 2 driver nodes, callbacks and runtime state
│       ├── rm_interface.h                            # SDK C interface declarations
│       ├── rm_interface_global.h                     # SDK symbol export macros
│       ├── rm_service.h                              # SDK C++ RM_Service class interface
│       └── rm_version.h                              # SDK version definition
├── launch
│   ├── rm_driver.launch.py        # unified driver launch entry
│   ├── rm_63_driver.launch.py     # 63 launch file
│   ├── rm_65_driver.launch.py     # 65 launch file
│   ├── rm_75_driver.launch.py     # 75 launch file
│   ├── rm_eco62_driver.launch.py  # eco62 launch file
│   ├── rm_eco65_driver.launch.py  # eco65 launch file
│   ├── rm_eco63_driver.launch.py  # eco63 launch file
│   ├── rm_gen72_driver.launch.py  # gen72 launch file
│   └── rm_rx75_driver.launch.py   # RX75 launch file
├── lib
│   ├── lib_install.sh                                # Select and install the SDK library for the host architecture
│   ├── libapi_cpp.so                                 # SDK shared library linked and installed by CMake
│   ├── linux_arm64_c++_v1.1.7/
│   │   └── libapi_cpp.so                             # ARM64 SDK shared library
│   └── linux_x86_c++_v1.1.7/
│       └── libapi_cpp.so                             # x86_64 SDK shared library
├── package.xml                                           # dependency declaration file
├── README_CN.md
├── README.md
└── src
    └── rm_driver.cpp                                     # driver code source file
```

### 4. rm_driver topic description
rm_driver has many topics, and you can learn about the topic information through the following commands.
![image](doc/rm_driver2.png)
![image](doc/rm_driver3.png)  
It is mainly for the application of API to achieve some of the robotic arm functions; for a more complete introduction and use, please see the special document "[RealMan Robotic Arm ROS2 Topic Detailed Description](https://github.com/RealManRobot/ros2_rm_robot/blob/foxy/rm_driver/doc/RealMan%20Robotic%20Arm%20rm_driver%20Topic%20Detailed%20Description%20(ROS2).md)".
