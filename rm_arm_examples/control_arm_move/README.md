# 机械臂运动控制`ARM_MOVE_DEMO`

---

## **1.项目介绍**

本项目是一个基于RM65、RM75机械臂和ROS功能包实现MoveJ、MoveJ_P、MoveL、MoveC规划运动功能，在程序执行时将依次执行关节运动MoveJ指令，位姿运动MoveJ_P指令、直线运动MoveL指令，圆弧运动MoveC指令，在执行成功或失败时终端都会收到相关提示，目的是使ROS开发者迅速掌握并灵活运用机械臂。

## **2. 代码结构**

```
├── CMakeLists.txt                           <-CMake编译文件
├── launch                                   <-启动文件夹
│   ├── rm_65_move.launch.py                 <-启动文件
│   └── rm_75_move.launch.py                 <-启动文件
├── LICENSE                                  <-版本说明
├── package.xml                              <-依赖描述文件夹
├── README.md                                <-说明文档
└── src                                      <-C++源码文件夹
    └── api_Move_demo.cpp                    <-源码文件
```
## **3.项目下载**

通过项目链接下载本项目工程文件到本地：[ros2_rm_robot](https://github.com/RealManRobot/ros2_rm_robot/tree/foxy)

## **4.环境配置**

| 项目 | 内容 |
| :-- | :-- |
| 系统 | Ubuntu20.04 |
| ROS版本 | foxy |
| 依赖 | 机械臂的ROS2-foxy功能包 |

**配置过程**

1. 首先需要准备好Ubuntu20.04操作系统的虚拟机或其他设备。
2. 安装ROS2环境[foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html),也可参考ROS2-foxy功能包中的安装说明进行安装。
3. ROS2-Foxy功能包安装  

    新建工作空间和src文件
    ```
    mkdir -p ~/ros2_ws/src
    ```

    将ROS2文件复制到src文件夹中
    ```
    cp -r ros2_rm_robot ~/ros2_ws/src
    ```

    进入工作空间
    ```
    cd ~/ros2_ws
    ```

    编译rm_ros_interfaces功能包
    ```
    colcon build --packages-select rm_ros_interfaces
    ```

    声明环境变量
    ```
    source ./install/setup.bash
    ```

    编译所有功能包
    ```
    colcon build
    ```
    
    再次声明环境变量
    ```
    source ./install/setup.bash
    ```

## **5. 使用指南**

* **命令行使用**：

    我们需要在一个终端中启动机械臂的rm_driver功能包。
    ```
    ros2 launch rm_driver rm_<arm_type>_driver.launch.py
    ```
    <arm_type>可以为65、63、eco65、75、gen72，可对照自己使用的设备进行实际选择
    若我们的机械臂为RM65机械臂时应使用如下启动指令。
    ```
    ros2 launch control_arm_move rm_65_move.launch.py
    ```
    若为RM75机械臂时应使用如下启动指令。
    ```
    ros2 launch control_arm_move rm_75_move.launch.py
    ```
    若非RM65、RM75机械臂可能会出现无法到达点位的情况，为正常现象。
* **返回信息**：

    在程序成功运行时将会出现以下提示信息。
    ```
    [move_demo-1] [INFO] [1722921182.157890965] [Move_demo_pub_node]: arm_dof is 7             //当前控制机械臂自由度提示
    [move_demo-1]  
    [move_demo-1] [INFO] [1722921188.120872813] [Move_demo_sub_node]: *******Movej succeeded   //MoveJ运动成功时的提示信息
    [move_demo-1]  
    [move_demo-1] [INFO] [1722921191.708291782] [Move_demo_sub_node]: *******Movej_p succeeded //MoveJ_P运动成功时的提示信息
    [move_demo-1]  
    [move_demo-1] [INFO] [1722921193.977513811] [Move_demo_sub_node]: *******MoveL succeeded   //MoveL运动成功时的提示信息
    [move_demo-1]  
    [move_demo-1] [INFO] [1722921194.075138936] [Move_demo_sub_node]: *******MoveC succeeded   //MoveC运动成功时的提示信息
    ```
### 关键代码说明：

下面是 `api_Move_demo.cpp` 文件的主要功能：

- **初始化**
相关发布订阅信息初始化
    
    ```ROS
    rclcpp::Publisher<rm_ros_interfaces::msg::Movejp>::SharedPtr movej_p_publisher_;       //声明movej_p发布器
    rclcpp::Publisher<rm_ros_interfaces::msg::Movel>::SharedPtr movel_publisher_;          //声明movel发布器
    rclcpp::Publisher<rm_ros_interfaces::msg::Movej>::SharedPtr movej_publisher_;          //声明movej发布器
    rclcpp::Publisher<rm_ros_interfaces::msg::Movec>::SharedPtr movec_publisher_;          //声明movec发布器
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr movej_p_subscription_;            //声明movej_p订阅器
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr movel_subscription_;              //声明movel订阅器
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr movej_subscription_;              //声明movej订阅器
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr movec_subscription_;              //声明movec订阅器
    ```
- **回调函数**
接收机械臂版本信息，进入消息回调函数

    ```ROS
    MoveDemoSub();                                                                                 //构造函数
    void MoveJPDemo_Callback(const std_msgs::msg::Bool::SharedPtr msg);                            //movej_p结果回调函数
    void MoveJDemo_Callback(const std_msgs::msg::Bool::SharedPtr msg);                             //movel结果回调函数
    void MoveLDemo_Callback(const std_msgs::msg::Bool::SharedPtr msg);                             //movej结果回调函数
    void MoveCDemo_Callback(const std_msgs::msg::Bool::SharedPtr msg);                             //movec结果回调函数
    ```
- **发布MoveJ指令**
发布MoveJ指令使机械臂运动到目标位姿。

    ```ROS
    this->movej_publisher_->publish(movej_way);
    ```

- **发布MoveJ_P指令**
发布MoveJ_P指令使机械臂运动到目标位姿。

    ```ROS
    this->movej_p_publisher_->publish(moveJ_P_TargetPose);
    ```

- **发布MoveL指令**
发布MoveL指令使机械臂运动到目标位姿。

    ```ROS
    this->movel_publisher_->publish(moveL_TargetPose);
    ```

- **发布MoveC指令**
发布MoveC指令使机械臂运动到目标位姿。

    ```ROS
    this->movec_publisher_->publish(moveC_TargetPose);
    ```
* **支持渠道**：

    自定义锚点： # 自定义锚点名称 {#my-anchor}

## **6. 许可证信息**

* 具体许可证内容请参见`LICENSE`文件。

## **7. 常见问题解答（FAQ）**

- **Q1：机械臂连接失败**

  答案：修改过机械臂IP

- **Q2：UDP数据推送接口收不到数据**

  答案：检查线程模式、是否使能推送数据、IP以及防火墙
