<div align="right">
 
[简体中文](https://github.com/RealManRobot/ros2_rm_robot/blob/humble/rm_driver/doc/%E7%9D%BF%E5%B0%94%E6%9B%BC%E6%9C%BA%E6%A2%B0%E8%87%82ROS%E9%A9%B1%E5%8A%A8%E5%8C%85%E5%8A%9F%E8%83%BD%E6%8B%93%E5%B1%95%E8%AF%B4%E6%98%8E.md)|[English](https://github.com/RealManRobot/ros2_rm_robot/blob/humble/rm_driver/doc/Description%20of%20Function%20Expansion%20for%20Realman%20Robotic%20Arm%20ROS%20Driver%20Package.md)

</div>

<div align="center">

# RealMan Robotic Arm ROS Driver Package Function Extension Instructions V1.0
 
RealMan Intelligent Technology (Beijing) Co., Ltd.
Document Revision Record:
|Version | Date | Remarks |
| :---: | :---- | :---: |
|V1.0 | 2025-11-14 | Drafted |

</div>

## Table of Contents
* 1[Introduction](#introduction)
* 2[Main Environment Overview](#main-environment-overview)
* 3[Instructions for Adding ROS Robot Arm Driver Package Functions](#instructions-for-adding-ros-robot-arm-driver-package-functions)
* 4[Adding Topics to the Robot Arm Driver Package](#adding-topics-to-the-robot-arm-driver-package)
* 4.1[Defining msg Files](#defining-msg-files)
* 4.2[Adding Definitions and Inclusions in rm_driver.h](#adding-definitions-and-inclusions-in-rm_driverh)
* 4.3[Adding Related Implementations in rm_driver.cpp](#adding-related-implementations-in-rm_drivercpp)

## Introduction

Since the current ROS development package is mainly created as a functional and usage example for everyone, there may be topics you want to use that are not included in the rm_driver package. To facilitate usage, this document is added to introduce how to add topics that are not originally present in rm_driver. (It is hereby noted that this document is only for convenience and does not take responsibility for issues encountered during secondary development.)

## Main Environment Overview

* System
ROS2 currently has two versions, Foxy and Humble. Foxy needs to run on Ubuntu 20.04, and Humble needs to run on Ubuntu 22.04.
* Reference Documents 
ROS2 is currently strongly dependent on APIs for design. During development, we need to refer to the API documents. 
API document for the 3rd generation controller[link](https://develop.realman-robotics.com/robot/apic/getStarted/)；  
API document for the 4th generation controller[link](https://develop.realman-robotics.com/robot4th/apic/getStarted/)。

## Instructions for Adding ROS Robot Arm Driver Package Functions

**The main steps to add a topic in rm_driver are as follows**
* 1.Define the required msg file.
* 2.Add the msg header file inclusion in the rm_robot.h file.
* 3.Declare subscribers and publishers in the rm_robot.h file.
* 4.Initialize subscribers and publishers in the rm_driver.cpp file.
* 5.Implement callback functions in the rm_driver.cpp file.

## Adding Topics to the Robot Arm Driver Package

Let's perform an actual topic addition operation. Here, we take joint teaching as an example.

### Define msg
* Create a new msg file
First, we need to define the msg file. The variables in the msg file depend on the interface's structure in the API.
![image](rm_driver5.png)   
As shown in the figure above, the function has the following elements, and each element is explained as follows.
![image](rm_driver6.png)   
Next, in the msg folder of the rm_ros_interfaces package, we add the Jointteach.msg file and define the variables as follows.
![image](rm_driver7.png)   
* Modify CMakeLists.txt
After creating the new file, we need to add it in CMakeLists.txt. The specific content to add is as follows.
![image](rm_driver13.png)  
Through the above operations, we have defined a new msg file, which can be called through the header file later.
### Adding Definitions and Inclusions in rm_driver.h
We need to add the following content in rm_driver.h
* 1.First, add the header file inclusion
For example, the header file inclusion for the newly defined Jointteach.msg message file is as follows:
#include "rm_ros_interfaces/msg/jointteach.hpp"  
![image](rm_driver8.png)
* 2.Next, add the definitions of subscribers and publishers
//Publisher for joint teaching results
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Set_Joint_Teach_Cmd_Result;  
//Subscriber for joint teaching commands 
rclcpp::Subscription<rm_ros_interfaces::msg::Jointteach>::SharedPtr Set_Joint_Teach_Cmd;  
![image](rm_driver9.png)
* 3.Add the declaration of the callback function  
There is no requirement for the callback function name; currently, the naming format with the first letter capitalized is uniformly adopted.
void Set_Joint_Teach_Callback(rm_ros_interfaces::msg::Jointteach::SharedPtr msg);                       //Joint teaching  
![image](rm_driver10.png)
### Adding Related Implementations in rm_driver.cpp
We need to add the following content in rm_driver.cpp
* 1.Initialize subscribers and publishers
It is necessary to initialize the previously declared subscribers and publishers in the RmArm constructor and define their topic names.

    ``` C++
    /*********************************************************Joint Teaching*****************************************************************/
    Set_Joint_Teach_Cmd_Result = this->create_publisher<std_msgs::msg::Bool>("rm_driver/set_joint_teach_result", rclcpp::ParametersQoS());
    Set_Joint_Teach_Cmd = this->create_subscription<rm_ros_interfaces::msg::Jointteach>("rm_driver/set_joint_teach_cmd",rclcpp::ParametersQoS(),
        std::bind(&RmArm::Set_Joint_Teach_Callback,this,std::placeholders::_1),
        sub_opt4);
    ```
    As above, the topic name of the regularly published publisher is rm_driver/set_joint_teach_result, and the name of the subscriber is rm_driver/set_joint_teach_cmd.
    The main function of the subscriber is to receive external topic commands; as above, it is to receive external joint teaching commands and move according to the parameter information of the teaching commands.
    The main function of the publisher is to publish the movement result to the user after completing the movement, and the user needs to subscribe to this topic to know whether the movement is successful.
    The Set_Joint_Teach_Callback function is its callback function, and the name must be consistent with that defined in rm_driver.h earlier.
    ![image](rm_driver11.png)
* 2.Callback function implementation
The specific function of the callback function needs to be implemented:

    ``` C++
    void RmArm::Set_Joint_Teach_Callback(rm_ros_interfaces::msg::Jointteach::SharedPtr msg)
    {
        int num;
        int direction;
        int v;
        // bool block;
        int32_t res;
        std_msgs::msg::UInt32 joint_teach_data;
        std_msgs::msg::Bool joint_teach_result;

        num = msg->num;
        direction = msg->direction;
        v = msg->speed;
        // block = msg->block;

        // res = Rm_Api.Service_Joint_Teach_Cmd(m_sockhand, num, direction, v , block);
        res = Rm_Api.rm_set_joint_teach(robot_handle, num, direction, v);
        joint_teach_data.data = res;
        if(joint_teach_data.data == 0)
        {
            joint_teach_result.data = true;
            this->Set_Joint_Teach_Cmd_Result->publish(joint_teach_result);
        }
        else
        {
            joint_teach_result.data = false;
            this->Set_Joint_Teach_Cmd_Result->publish(joint_teach_result);
            RCLCPP_INFO (this->get_logger(),"Joint_Teach error code is %d\n",joint_teach_data.data);
        }
    }
    ```
    As shown above, the specific implementation of the callback function assigns the received msg parameters to the corresponding variables and then to the rm_set_joint_teach API function, and publishes the corresponding result according to its return value.
    ![image](rm_driver12.png)
