# 获取机械臂状态`GET_ARM_STATE_DEMO`

---

## **1.项目介绍**

本项目是一个基于机械臂机械臂本体和ROS功能包实现获取机械臂当前控制器版本、关节状态、位姿状态、六维力信息功能，程序将依次执行获取控制器版本信息，获取机械臂状态，获取六维力数据信息的指令，程序执行时，对应的数据信息会打印在终端中，该示例目的是使ROS开发者迅速掌握并灵活运用机械臂。

## **2. 代码结构**

```
├── CMakeLists.txt                           <-CMake编译文件
├── launch                                   <-启动文件夹
│   └── get_arm_state_demo.launch.py         <-启动文件
├── LICENSE                                  <-版本说明
├── package.xml                              <-依赖描述文件夹
├── README.md                                <-说明文档
└── src                                      <-C++源码文件夹
    └── api_Get_Arm_State_demo.cpp           <-源码文件
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

    新建工作空间和src文件，
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
    我们需要在另一个终端中启动机械臂的get_arm_state功能包。
    ```
    ros2 launch get_arm_state get_arm_state_demo.launch.py
    ```
* **返回信息**：

    在程序成功运行时将会出现以下提示信息。
    ```
    [rm_get_arm_state_demo-1] [INFO] [1722912567.024745517] [get_state]: joint state is: [0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000]       //机械臂角度信息
    [rm_get_arm_state_demo-1] 
    [rm_get_arm_state_demo-1] [INFO] [1722912567.024994866] [get_state]: pose state is: [0.000000, 0.000000, 0.534000, 3.141000, 0.000000, 0.000000]                //机械臂位姿信息（欧拉角）
    [rm_get_arm_state_demo-1] 
    [rm_get_arm_state_demo-1] [INFO] [1722912567.025200125] [get_state]: joint state is: [0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000]      //机械臂弧度信息
    [rm_get_arm_state_demo-1] 
    [rm_get_arm_state_demo-1] [INFO] [1722912567.025321800] [get_state]: pose state is:   //机械臂位姿信息（四元数）
    [rm_get_arm_state_demo-1] [position.x = 0.000000,
    [rm_get_arm_state_demo-1]  position.y = 0.000000,
    [rm_get_arm_state_demo-1]  position.z = 0.534000,
    [rm_get_arm_state_demo-1]  orientation.x = 1.000000,
    [rm_get_arm_state_demo-1]  orientation.y = 0.000000,
    [rm_get_arm_state_demo-1]  orientation.z = 0.000000,
    [rm_get_arm_state_demo-1]  orientation.w = 0.000296]
    [rm_get_arm_state_demo-1] 
    [rm_get_arm_state_demo-1] [INFO] [1722912567.025408782] [get_state]: Planversion is 7B0156       //机械臂控制器版本信息（7代表自由度，b代表没有六维力，156为控制器程序版本为1.5.6）
    [rm_get_arm_state_demo-1]  
    [rm_get_arm_state_demo-1] [INFO] [1722912567.025463071] [get_state]: Productversion is GEN72-BI  //机械臂设备型号息信息
    [rm_get_arm_state_demo-1] [INFO] [1723550015.625603508] [get_state]: 
    [rm_get_arm_state_demo-1] [INFO] [1723550325.782782033] [get_state]:                            // 六维力传感器原始数据
    [rm_get_arm_state_demo-1] force-force_fx is 0.000000
    [rm_get_arm_state_demo-1] force-force_fy is 0.000000
    [rm_get_arm_state_demo-1] force-force_fz is 0.000000
    [rm_get_arm_state_demo-1] force-force_mx is 0.000000
    [rm_get_arm_state_demo-1] force-force_my is 0.000000
    [rm_get_arm_state_demo-1] force-force_mz is 0.000000
    [rm_get_arm_state_demo-1]  
    [rm_get_arm_state_demo-1] [INFO] [1723550325.782929425] [get_state]:                             // 六维力传感器系统受力数据
    [rm_get_arm_state_demo-1] zero-force_fx is 0.000000
    [rm_get_arm_state_demo-1] zero-force_fy is 0.000000
    [rm_get_arm_state_demo-1] zero-force_fz is 0.000000
    [rm_get_arm_state_demo-1] zero-force_mx is 0.000000
    [rm_get_arm_state_demo-1] zero-force_my is 0.000000
    [rm_get_arm_state_demo-1] zero-force_mz is 0.000000
    [rm_get_arm_state_demo-1] 
    [rm_get_arm_state_demo-1] [INFO] [1723550325.782964040] [get_state]:                              // 六维力传感器工作坐标系受力数据
    [rm_get_arm_state_demo-1] work-zero_fx is 0.000000
    [rm_get_arm_state_demo-1] work-zero_fy is 0.000000
    [rm_get_arm_state_demo-1] work-zero_fz is 0.000000
    [rm_get_arm_state_demo-1] work-zero_mx is 0.000000
    [rm_get_arm_state_demo-1] work-zero_my is 0.000000
    [rm_get_arm_state_demo-1] work-zero_mz is 0.000000
    [rm_get_arm_state_demo-1]  
    [rm_get_arm_state_demo-1] [INFO] [1723550325.782985859] [get_state]:                              // 六维力传感器工具坐标系受力数据
    [rm_get_arm_state_demo-1] tool-zero_fx is 0.000000
    [rm_get_arm_state_demo-1] tool-zero_fy is 0.000000
    [rm_get_arm_state_demo-1] tool-zero_fz is 0.000000
    [rm_get_arm_state_demo-1] tool-zero_mx is 0.000000
    [rm_get_arm_state_demo-1] tool-zero_my is 0.000000
    [rm_get_arm_state_demo-1] tool-zero_mz is 0.000000

    ```
### 关键代码说明：

下面是 `api_Get_Arm_State_demo.cpp` 文件的主要功能：

- **初始化**
相关发布订阅信息初始化
    
    ```ROS
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr arm_state_publisher_;                                 //机械臂关节状态发布器
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr arm_software_version_publisher_;                      //机械臂软件版本发布器
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr get_force_data_publisher_;                            //机械臂六维力状态发布器
    rclcpp::Subscription<rm_ros_interfaces::msg::Armoriginalstate>::SharedPtr subscription_original_state;   //关节原始状态订阅器
    rclcpp::Subscription<rm_ros_interfaces::msg::Armstate>::SharedPtr subscription_arm_state;                //关节状态订阅器
    rclcpp::Subscription<rm_ros_interfaces::msg::Armsoftversion>::SharedPtr subscription_software_version;   //控制器版本订阅器
    rclcpp::Subscription<rm_ros_interfaces::msg::Sixforce>::SharedPtr subscription_force_data;               //六维力传感器状态订阅器
    rclcpp::Subscription<rm_ros_interfaces::msg::Sixforce>::SharedPtr subscription_zero_force_data;          //六维力系统受力状态订阅器
    rclcpp::Subscription<rm_ros_interfaces::msg::Sixforce>::SharedPtr subscription_work_force_data;          //六维力工作坐标系受力状态订阅器
    rclcpp::Subscription<rm_ros_interfaces::msg::Sixforce>::SharedPtr subscription_tool_force_data;          //六维力工具坐标系受力状态订阅器
    ```
- **回调函数**
接收机械臂版本信息，进入消息回调函数

    ```ROS
    void get_arm_state();                                                                                    //改变工作坐标系函数
    void GetArmOriginalState_Callback(const rm_ros_interfaces::msg::Armoriginalstate::SharedPtr msg);        //机械臂状态原始结果回调函数
    void GetArmState_Callback(const rm_ros_interfaces::msg::Armstate::SharedPtr msg);                        //机械臂状态结果回调函数
    void GetArmSoftwareVersion_Callback(const rm_ros_interfaces::msg::Armsoftversion::SharedPtr msg);        //控制器版本结果回调函数
    void ForceData_Callback(const rm_ros_interfaces::msg::Sixforce::SharedPtr msg);                          //六维力传感器结果回调函数
    void ZeroForceData_Callback(const rm_ros_interfaces::msg::Sixforce::SharedPtr msg);                      //六维力结果回调函数
    void WorkForceData_Callback(const rm_ros_interfaces::msg::Sixforce::SharedPtr msg);                      //六维力工作坐标结果回调函数
    void ToolForceData_Callback(const rm_ros_interfaces::msg::Sixforce::SharedPtr msg);                      //六维力工具坐标结果回调函数
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
