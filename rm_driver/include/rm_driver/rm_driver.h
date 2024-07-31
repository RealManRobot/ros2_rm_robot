// Copyright (c) 2023  RealMan Intelligent Ltd
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp" 
#include <memory>
#include <string>
#include <thread>
#include <chrono>
#include <functional>
#include <unistd.h>             
#include <signal.h>             
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>


#include <sys/ioctl.h>          // 设置非阻塞需要用到的头文件
#include <sys/time.h>
#include <sys/select.h>         //使用fd_set结构体时使用。
#include <fcntl.h>  

#include "rm_service.h"
#include "rm_ros_interfaces/msg/movej.hpp"
#include "rm_ros_interfaces/msg/movel.hpp"
#include "rm_ros_interfaces/msg/movec.hpp"
#include "rm_ros_interfaces/msg/movejp.hpp"
#include "rm_ros_interfaces/msg/jointteach.hpp"
#include "rm_ros_interfaces/msg/ortteach.hpp"
#include "rm_ros_interfaces/msg/posteach.hpp"
#include "rm_ros_interfaces/msg/setrealtimepush.hpp"
#include "rm_ros_interfaces/msg/armsoftversion.hpp"
#include "rm_ros_interfaces/msg/sixforce.hpp"
#include "rm_ros_interfaces/msg/jointerrorcode.hpp"
#include "rm_ros_interfaces/msg/forcepositionmovejoint.hpp"
#include "rm_ros_interfaces/msg/forcepositionmovepose.hpp"
#include "rm_ros_interfaces/msg/setforceposition.hpp"
#include "rm_ros_interfaces/msg/jointpos.hpp"
#include "rm_ros_interfaces/msg/cartepos.hpp"
#include "rm_ros_interfaces/msg/jointerrclear.hpp"
#include "rm_ros_interfaces/msg/gripperset.hpp"
#include "rm_ros_interfaces/msg/gripperpick.hpp"
#include "rm_ros_interfaces/msg/handangle.hpp"
#include "rm_ros_interfaces/msg/handforce.hpp"
#include "rm_ros_interfaces/msg/handposture.hpp"
#include "rm_ros_interfaces/msg/handseq.hpp"
#include "rm_ros_interfaces/msg/handspeed.hpp"
#include "rm_ros_interfaces/msg/armstate.hpp"
#include "rm_ros_interfaces/msg/armoriginalstate.hpp"
#include "rm_ros_interfaces/msg/getallframe.hpp"
#include "rm_ros_interfaces/msg/liftspeed.hpp"
#include "rm_ros_interfaces/msg/liftstate.hpp"
#include "rm_ros_interfaces/msg/liftheight.hpp"
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>

#define RAD_DEGREE 57.295791433
#define DEGREE_RAD 0.01745
using namespace std::chrono_literals;
//udp数据处理函数
void Udp_RobotStatuscallback(RobotStatus Udp_RM_Callback);
//ctrl+c执行程序
static void my_handler(int sig);
//机械臂型号信息
int realman_arm;
//tcp ip
char* tcp_ip;
//tcp port
int tcp_port;
//udp hz
int udp_cycle_g = 5;
//arm dof
int arm_dof_g = 6;
//ctrl+c触发信号
bool ctrl_flag = false;
//api类
RM_Service Rm_Api;
//机械臂TCp网络通信套接字
SOCKHANDLE m_sockhand = -1;
//机械臂状态参数
typedef struct
{
    float    joint[7];                 //关节角度
    uint16_t err_flag[7];              //关节错误代码
    uint16_t sys_err;                  //系统错误代码
    uint16_t arm_err;                  //机械臂错误代码
    float    one_force;                //一维力传感器原始数据0.001N或0.001Nm
    float    six_force[6];             //六维力数据
    float    joint_current[7];         //机械臂电流数据
    bool     en_flag[7];               //当前关节使能状态 ，1为上使能，0为掉使能
    float    joint_position[3];        //当前末端关节位置，精度0.001°
    float    joint_temperature[7];     //当前关节温度，精度0.001℃
    float    joint_voltage[7];         //当前关节电压，精度0.001V
    float    joint_euler[3];           //欧拉角
    float    joint_quat[4];            //四元数
    float    zero_force[6];            //当前力传感器系统外受力数据0.001N或0.001Nm
    float    work_zero_force[6];       //当前工作坐标系下系统受到的外力数据
    float    tool_zero_force[6];       //当前该工具坐标系下系统受到的外力数据
    float    one_zero_force;           //一维力基准坐标系下系统受力数据
    uint16_t control_version;          //版本信息
    uint16_t coordinate;               //当前六维力传感器的基准坐标
} JOINT_STATE_VALUE;
JOINT_STATE_VALUE Udp_RM_Joint;

std_msgs::msg::UInt16 sys_err_;                                     //系统错误信息
std_msgs::msg::UInt16 arm_err_;                                     //机械臂错误信息
std_msgs::msg::UInt16 arm_coordinate_;                              //六维力基准坐标系
sensor_msgs::msg::JointState udp_real_joint_;                       //关节角度
geometry_msgs::msg::Pose udp_arm_pose_;                             //位姿
rm_ros_interfaces::msg::Sixforce udp_sixforce_;                     //六维力传感器原始数据
rm_ros_interfaces::msg::Sixforce udp_zeroforce_;                    //六维力传感器转化后数据
rm_ros_interfaces::msg::Sixforce udp_oneforce_;                     //一维力传感器原始数据
rm_ros_interfaces::msg::Sixforce udp_onezeroforce_;                 //一维力传感器转化后数据
rm_ros_interfaces::msg::Jointerrorcode udp_joint_error_code_;       //关节报错数据
rm_ros_interfaces::msg::Armoriginalstate Arm_original_state;        //机械臂原始数据（角度+欧拉角）
rm_ros_interfaces::msg::Armstate Arm_state;                         //机械臂数据（弧度+四元数）

class RmArm: public rclcpp::Node
{
public:
    RmArm();
    ~RmArm();

/**********************************************初始化需要用到的回调函数***********************************************/
    void Get_Arm_Version();                                                                                 //获取版本信息
    void Set_UDP_Configuration(int udp_cycle, int udp_port, int udp_force_coordinate, std::string udp_ip);  //设置udp主动上报配置
    /*******************************运动控制回调函数******************************/
    // void Arm_MoveJ_75_Callback(rm_ros_interfaces::msg::Movej75::SharedPtr msg);                          //75角度控制
    void Arm_MoveJ_Callback(rm_ros_interfaces::msg::Movej::SharedPtr msg);                                  //角度控制
    void Arm_MoveL_Callback(rm_ros_interfaces::msg::Movel::SharedPtr msg);                                  //直线运动控制
    void Arm_MoveC_Callback(rm_ros_interfaces::msg::Movec::SharedPtr msg);                                  //圆弧运动控制
    void Arm_Movej_CANFD_Callback(rm_ros_interfaces::msg::Jointpos::SharedPtr msg);                         //角度透传控制
    void Arm_Movep_CANFD_Callback(rm_ros_interfaces::msg::Cartepos::SharedPtr msg);                         //位姿透传控制
    void Arm_MoveJ_P_Callback(rm_ros_interfaces::msg::Movejp::SharedPtr msg);                               //位姿运动控制
    void Arm_Move_Stop_Callback(std_msgs::msg::Bool::SharedPtr msg);                                        //轨迹急停控制
    /**************************************************************************/
    void Set_Joint_Teach_Callback(rm_ros_interfaces::msg::Jointteach::SharedPtr msg);                       //关节示教
    void Set_Pos_Teach_Callback(rm_ros_interfaces::msg::Posteach::SharedPtr msg);                           //位置示教
    void Set_Ort_Teach_Callback(rm_ros_interfaces::msg::Ortteach::SharedPtr msg);                           //姿态示教
    void Set_Stop_Teach_Callback(const std_msgs::msg::Bool::SharedPtr msg);                                //停止示教

    /*******************************主动上报回调函数******************************/
    void Arm_Get_Realtime_Push_Callback(const std_msgs::msg::Empty::SharedPtr msg);                         //获取主动上报配置
    void Arm_Set_Realtime_Push_Callback(const rm_ros_interfaces::msg::Setrealtimepush::SharedPtr msg);      //设置主动上报配置参数
    /*******************************固件版本回调函数******************************/
    void Arm_Get_Arm_Software_Version_Callback(const std_msgs::msg::Empty::SharedPtr msg);                  //获取机械臂固件版本
    /*******************************力位混合回调函数******************************/
    void Arm_Start_Force_Position_Move_Callback(const std_msgs::msg::Empty::SharedPtr msg);                 //力位混合开始
    void Arm_Stop_Force_Position_Move_Callback(const std_msgs::msg::Empty::SharedPtr msg);                  //力位混合结束
    void Arm_Force_Position_Move_Joint_Callback(const rm_ros_interfaces::msg::Forcepositionmovejoint::SharedPtr msg);       //力位混合透传（角度）
    void Arm_Force_Position_Move_Pose_Callback(const rm_ros_interfaces::msg::Forcepositionmovepose::SharedPtr msg);         //力位混合透传（位姿）
    void Arm_Set_Force_Postion_Callback(const rm_ros_interfaces::msg::Setforceposition::SharedPtr msg);                     //使能力位混合透传
    void Arm_Stop_Force_Postion_Callback(const std_msgs::msg::Bool::SharedPtr msg);                                         //结束力位混合透传
    /*******************************坐标系回调函数******************************/
    void Arm_Change_Work_Frame_Callback(const std_msgs::msg::String::SharedPtr msg);                        //更改工作坐标系
    void Arm_Get_Curr_WorkFrame_Callback(const std_msgs::msg::Empty::SharedPtr msg);                        //查询工作坐标系
    void Arm_Get_Current_Tool_Frame_Callback(const std_msgs::msg::Empty::SharedPtr msg);                    //查询工具坐标系
    void Arm_Get_All_Tool_Frame_Callback(const std_msgs::msg::Empty::SharedPtr msg);                        //查询所有工具坐标系
    void Arm_Get_All_Work_Frame_Callback(const std_msgs::msg::Empty::SharedPtr msg);                        //查询所有工作坐标系
    /*******************************设置工具端电压*********************************/
    void Arm_Set_Tool_Voltage_Callback(const std_msgs::msg::UInt16::SharedPtr msg);                         
    /*******************************清除错误码回调函数****************************/
    void Arm_Set_Joint_Err_Clear_Callback(const rm_ros_interfaces::msg::Jointerrclear::SharedPtr msg);
    /*********************************夹爪回调函数******************************/
    void Arm_Set_Gripper_Pick_On_Callback(const rm_ros_interfaces::msg::Gripperpick::SharedPtr msg);        //持续力控夹取
    void Arm_Set_Gripper_Pick_Callback(const rm_ros_interfaces::msg::Gripperpick::SharedPtr msg);           //力控夹取
    void Arm_Set_Gripper_Position_Callback(const rm_ros_interfaces::msg::Gripperset::SharedPtr msg);        //移动到固定位置
    /*********************************灵巧手回调函数******************************/
    void Arm_Set_Hand_Posture_Callback(const rm_ros_interfaces::msg::Handposture::SharedPtr msg);           //设置灵巧手手势
    void Arm_Set_Hand_Seq_Callback(const rm_ros_interfaces::msg::Handseq::SharedPtr msg);                   //设置灵巧手动作序列
    void Arm_Set_Hand_Angle_Callback(const rm_ros_interfaces::msg::Handangle::SharedPtr msg);               //设置灵巧手角度
    void Arm_Set_Hand_Speed_Callback(const rm_ros_interfaces::msg::Handspeed::SharedPtr msg);               //设置灵巧手速度
    void Arm_Set_Hand_Force_Callback(const rm_ros_interfaces::msg::Handforce::SharedPtr msg);               //设置灵巧手力控
    /*********************************升降机构回调函数******************************/
    void Arm_Set_Lift_Speed_Callback(const rm_ros_interfaces::msg::Liftspeed::SharedPtr msg);               //升降机构速度开环控制
    void Arm_Set_Lift_Height_Callback(const rm_ros_interfaces::msg::Liftheight::SharedPtr msg);             //升降机构位置闭环控制
    void Arm_Get_Lift_State_Callback(const std_msgs::msg::Empty::SharedPtr msg);                            //获取升降机构状态
    /*******************************机械臂状态回调函数****************************/
    void Arm_Get_Current_Arm_State_Callback(const std_msgs::msg::Empty::SharedPtr msg);
    /*********************************六维力数据清零******************************/
    void Arm_Clear_Force_Data_Callback(const std_msgs::msg::Bool::SharedPtr msg);
    /*********************************六维力数据获取******************************/
    void Arm_Get_Force_Data_Callback(const std_msgs::msg::Empty::SharedPtr msg);
    
/***************************************************************end******************************************************/
private:
    // int Arm_Start(void);        //TCP连接函数
    // void Arm_Close();           //TCP断连函数

/************************************************************变量信息******************************************************/
    std_msgs::msg::Empty::SharedPtr copy;                               //闲置
    // std_msgs::msg::UInt16 sys_err_;                                     //系统错误信息
    // std_msgs::msg::UInt16 arm_err_;                                     //机械臂错误信息
    // std_msgs::msg::UInt16 arm_coordinate_;                              //六维力基准坐标系
    // sensor_msgs::msg::JointState udp_real_joint_;                       //关节角度
    // geometry_msgs::msg::Pose udp_arm_pose_;                             //位姿
    // rm_ros_interfaces::msg::Sixforce udp_sixforce_;                     //六维力传感器原始数据
    // rm_ros_interfaces::msg::Sixforce udp_zeroforce_;                    //六维力传感器转化后数据
    // rm_ros_interfaces::msg::Sixforce udp_oneforce_;                     //一维力传感器原始数据
    // rm_ros_interfaces::msg::Sixforce udp_onezeroforce_;                 //一维力传感器转化后数据
    // rm_ros_interfaces::msg::Jointerrorcode udp_joint_error_code_;       //关节报错数据
    // rm_ros_interfaces::msg::Armoriginalstate Arm_original_state;        //机械臂原始数据（角度+欧拉角）
    // rm_ros_interfaces::msg::Armstate Arm_state;                         //机械臂数据（弧度+四元数）

    /****************************************udp主动上报配置查询发布器*************************************/
    rclcpp::Publisher<rm_ros_interfaces::msg::Setrealtimepush>::SharedPtr Get_Realtime_Push_Result;
    /****************************************udp主动上报配置查询订阅器*************************************/
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr Get_Realtime_Push_Cmd;
    /******************************************udp主动上报配置发布器***************************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Set_Realtime_Push_Result;
    /******************************************udp主动上报配置发布器***************************************/
    rclcpp::Subscription<rm_ros_interfaces::msg::Setrealtimepush>::SharedPtr Set_Realtime_Push_Cmd;
    /**********************************************************end******************************************************/

    /********************************************************运动配置******************************************************/
    /****************************************MoveJ运动控制结果发布器*************************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr MoveJ_Cmd_Result;
    /******************************************75MoveJ运动控制订阅器***************************************/
    // rclcpp::Subscription<rm_ros_interfaces::msg::Movej75>::SharedPtr MoveJ_75_Cmd;
    /******************************************MoveJ运动控制订阅器***************************************/
    rclcpp::Subscription<rm_ros_interfaces::msg::Movej>::SharedPtr MoveJ_Cmd;
    /****************************************MoveL运动控制结果发布器*************************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr MoveL_Cmd_Result;
    /*******************************************MoveL运动控制订阅器*************************************/
    rclcpp::Subscription<rm_ros_interfaces::msg::Movel>::SharedPtr MoveL_Cmd;
    /****************************************MoveC运动控制结果发布器*************************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr MoveC_Cmd_Result;
    /*******************************************MoveC运动控制订阅器*************************************/
    rclcpp::Subscription<rm_ros_interfaces::msg::Movec>::SharedPtr MoveC_Cmd;
    /*******************************************角度透传运动控制订阅器*************************************/
    rclcpp::Subscription<rm_ros_interfaces::msg::Jointpos>::SharedPtr Movej_CANFD_Cmd;
    /*******************************************角度透传运动控制订阅器*************************************/
    // rclcpp::Subscription<rm_ros_interfaces::msg::Jointpos75>::SharedPtr Movej_CANFD_75_Cmd;
    /*******************************************位姿透传运动控制订阅器*************************************/
    rclcpp::Subscription<rm_ros_interfaces::msg::Cartepos>::SharedPtr Movep_CANFD_Cmd;
    /****************************************MoveJ_P运动控制结果发布器*************************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr MoveJ_P_Cmd_Result;
    /*******************************************MoveJ_P运动控制订阅器*************************************/
    rclcpp::Subscription<rm_ros_interfaces::msg::Movejp>::SharedPtr MoveJ_P_Cmd;
    /********************************************轨迹急停结果发布器*****************************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Move_Stop_Cmd_Result;
    /***********************************************轨迹急停控制订阅器*************************************/
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr Move_Stop_Cmd;
    /********************************************************end******************************************************/

    /*******************************************************关节示教***************************************************/
    /****************************************关节示教结果发布器*************************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Set_Joint_Teach_Cmd_Result;
    /*******************************************关节示教订阅器*************************************/
    rclcpp::Subscription<rm_ros_interfaces::msg::Jointteach>::SharedPtr Set_Joint_Teach_Cmd;
    /****************************************位置示教结果发布器*************************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Set_Pos_Teach_Cmd_Result;
    /*******************************************位置示教订阅器*************************************/
    rclcpp::Subscription<rm_ros_interfaces::msg::Posteach>::SharedPtr Set_Pos_Teach_Cmd;
    /****************************************姿态示教结果发布器*************************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Set_Ort_Teach_Cmd_Result;
    /*******************************************姿态示教订阅器*************************************/
    rclcpp::Subscription<rm_ros_interfaces::msg::Ortteach>::SharedPtr Set_Ort_Teach_Cmd;
    /****************************************停止示教结果发布器*************************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Set_Stop_Teach_Cmd_Result;
    /*******************************************停止示教订阅器*************************************/
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr Set_Stop_Teach_Cmd;
    /********************************************************end******************************************************/

    /********************************************************固件版本***************************************************/
    /*************************************************查询固件版本发布器****************************************/
    rclcpp::Publisher<rm_ros_interfaces::msg::Armsoftversion>::SharedPtr Get_Arm_Software_Version_Result;
    /*************************************************查询固件版本订阅器****************************************/
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr Get_Arm_Software_Version_Cmd;
    /********************************************************end******************************************************/

    /*************************************************************力位混合**************************************************/
    /****************************************开启力位混合透传结果发布器*************************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Start_Force_Position_Move_Result;
    /***********************************************开启力位混合订阅器************************************************/
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr Start_Force_Position_Move_Cmd;
    /****************************************关闭力位混合透传结果发布器*************************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Stop_Force_Position_Move_Result;
    /***********************************************关闭力位混合订阅器************************************************/
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr Stop_Force_Position_Move_Cmd;
    /********************************************设置力位混合控制结果发布器**************************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Set_Force_Postion_Result;
    /************************************************设置力位混合控制************************************************/
    rclcpp::Subscription<rm_ros_interfaces::msg::Setforceposition>::SharedPtr Set_Force_Postion_Cmd;
    /********************************************结束力位混合控制结果发布器*******************************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Stop_Force_Postion_Result;
    /************************************************结束力位混合控制************************************************/
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr Stop_Force_Postion_Cmd;
    
    /**************************************************75力位混合角度透传订阅器***************************************/
    // rclcpp::Subscription<rm_ros_interfaces::msg::Forcepositionmovejoint75>::SharedPtr Force_Position_Move_Joint_75_Cmd;
    /**************************************************力位混合角度透传订阅器***************************************/
    rclcpp::Subscription<rm_ros_interfaces::msg::Forcepositionmovejoint>::SharedPtr Force_Position_Move_Joint_Cmd;
    /***********************************************力位混合位姿透传订阅器*********************************************/
    rclcpp::Subscription<rm_ros_interfaces::msg::Forcepositionmovepose>::SharedPtr Force_Position_Move_Pose_Cmd;
    /**********************************************************************end************************************************************/

    /**************************************************************坐标系指令发布器**********************************************************/
    /****************************************切换工作坐标系发布器*************************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Change_Work_Frame_Result;
    /*****************************************切换工作坐标系订阅器************************************/
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr Change_Work_Frame_Cmd;
    /****************************************获取工作坐标系发布器*************************************/
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr Get_Curr_WorkFrame_Result;
    /*****************************************获取工作坐标系订阅器************************************/
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr Get_Curr_WorkFrame_Cmd;
    /****************************************获取工具坐标系发布器*************************************/
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr Get_Current_Tool_Frame_Result;
    /*****************************************获取工具坐标系订阅器************************************/
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr Get_Current_Tool_Frame_Cmd;
    /****************************************获取所有工具坐标系发布器**********************************/
    rclcpp::Publisher<rm_ros_interfaces::msg::Getallframe>::SharedPtr Get_All_Tool_Frame_Result;
    /****************************************获取所有工具坐标系订阅器**********************************/
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr Get_All_Tool_Frame_Cmd;
    /****************************************获取所有工作坐标系发布器**********************************/
    rclcpp::Publisher<rm_ros_interfaces::msg::Getallframe>::SharedPtr Get_All_Work_Frame_Result;
    /****************************************获取所有工作坐标系订阅器**********************************/
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr Get_All_Work_Frame_Cmd;
    /*****************************************************************end******************************************************************/

    /****************************************机械臂工具端电源输出发布器**********************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Set_Tool_Voltage_Result;
    /****************************************机械臂工具端电源输出订阅器**********************************/
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr Set_Tool_Voltage_Cmd;

    /****************************************清除关节错误代码结果发布器**********************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Set_Joint_Err_Clear_Result;
    /******************************************清除关节错误代码订阅器**********************************/
    rclcpp::Subscription<rm_ros_interfaces::msg::Jointerrclear>::SharedPtr Set_Joint_Err_Clear_Cmd;

/**************************************************************末端工具-手爪控制**********************************************************/
    /****************************************手爪持续力控夹取结果发布器**********************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Set_Gripper_Pick_On_Result;
    /******************************************手爪持续力控夹取订阅器**********************************/
    rclcpp::Subscription<rm_ros_interfaces::msg::Gripperpick>::SharedPtr Set_Gripper_Pick_On_Cmd;
    /******************************************手爪力控夹取结果发布器**********************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Set_Gripper_Pick_Result;
    /*********************************************手爪力控夹取订阅器**********************************/
    rclcpp::Subscription<rm_ros_interfaces::msg::Gripperpick>::SharedPtr Set_Gripper_Pick_Cmd;
    /****************************************手爪到达指定位置结果发布器**********************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Set_Gripper_Position_Result;
    /******************************************手爪到达指定位置夹取订阅器*********************************/
    rclcpp::Subscription<rm_ros_interfaces::msg::Gripperset>::SharedPtr Set_Gripper_Position_Cmd;
/*****************************************************************end******************************************************************/

/**************************************************************末端工具-五指灵巧手控制******************************************************/
    /****************************************设置灵巧手手势序号发布器**********************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Set_Hand_Posture_Result;
    /****************************************设置灵巧手手势序号订阅器**********************************/
    rclcpp::Subscription<rm_ros_interfaces::msg::Handposture>::SharedPtr Set_Hand_Posture_Cmd;
    /**************************************设置灵巧手动作序列序号发布器*********************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Set_Hand_Seq_Result;
    /**************************************设置灵巧手动作序列序号订阅器*********************************/
    rclcpp::Subscription<rm_ros_interfaces::msg::Handseq>::SharedPtr Set_Hand_Seq_Cmd;
    /****************************************设置灵巧手角度结果发布器**********************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Set_Hand_Angle_Result;
    /*******************************************设置灵巧手角度订阅器**********************************/
    rclcpp::Subscription<rm_ros_interfaces::msg::Handangle>::SharedPtr Set_Hand_Angle_Cmd;
    /****************************************设置灵巧手关节速度发布器**********************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Set_Hand_Speed_Result;
    /****************************************设置灵巧手关节速度订阅器**********************************/
    rclcpp::Subscription<rm_ros_interfaces::msg::Handspeed>::SharedPtr Set_Hand_Speed_Cmd;
    /**************************************设置灵巧手各关节力阈值发布器*********************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Set_Hand_Force_Result;
    /***************************************设置灵巧手各关节力阈值订阅器********************************/
    rclcpp::Subscription<rm_ros_interfaces::msg::Handforce>::SharedPtr Set_Hand_Force_Cmd;
/*****************************************************************end******************************************************************/

/********************************************************************升降机构***********************************************************/
    /******************************************设置升降机构速度发布器*********************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Set_Lift_Speed_Result;
    /*******************************************设置升降机构速度订阅器********************************/
    rclcpp::Subscription<rm_ros_interfaces::msg::Liftspeed>::SharedPtr Set_Lift_Speed_Cmd;
    /****************************************设置升降机构高度发布器*********************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Set_Lift_Height_Result;
    /***************************************设置升降机构高度订阅器********************************/
    rclcpp::Subscription<rm_ros_interfaces::msg::Liftheight>::SharedPtr Set_Lift_Height_Cmd;
    /****************************************获取升降机构状态发布器*********************************/
    rclcpp::Publisher<rm_ros_interfaces::msg::Liftstate>::SharedPtr Get_Lift_State_Result;
    /***************************************获取升降机构状态订阅器********************************/
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr Get_Lift_State_Cmd;
/********************************************************************end***********************************************************/
 
    /**************************************获取机械臂当前状态发布器*************************************/
    rclcpp::Publisher<rm_ros_interfaces::msg::Armoriginalstate>::SharedPtr Get_Current_Arm_Original_State_Result;
    /**************************************获取机械臂当前状态发布器*************************************/
    rclcpp::Publisher<rm_ros_interfaces::msg::Armstate>::SharedPtr Get_Current_Arm_State_Result;
    /***************************************获取机械臂当前状态订阅器************************************/
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr Get_Current_Arm_State_Cmd;

/********************************************************************六维力***********************************************************/
    /****************************************六维力数据清零发布器***************************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Clear_Force_Data_Result;
    /******************************************六维力数据清零订阅器*************************************/
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr Clear_Force_Data_Cmd;
    /********************************************六维力数据获取发布器****************************************/
    rclcpp::Publisher<rm_ros_interfaces::msg::Sixforce>::SharedPtr Get_Force_Data_Result;
    rclcpp::Publisher<rm_ros_interfaces::msg::Sixforce>::SharedPtr Get_Zero_Force_Result;
    rclcpp::Publisher<rm_ros_interfaces::msg::Sixforce>::SharedPtr Get_Work_Zero_Result;
    rclcpp::Publisher<rm_ros_interfaces::msg::Sixforce>::SharedPtr Get_Tool_Zero_Result;
    /******************************************六维力数据获取订阅器*************************************/
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr Get_Force_Data_Cmd;
/********************************************************************end***********************************************************/

    std::string arm_ip_ = "192.168.1.18";    
    std::string udp_ip_ = "192.168.1.10";
    std::string  arm_type_ = "RM_75";  
    
    
    int tcp_port_ = 8080;  
    int udp_port_ = 8089; 
    int arm_dof_ = 7;                                  //机械臂自由度
    int udp_cycle_ = 5;                                //udp主动上报周期（ms）
    int udp_force_coordinate_ = 0;                     //udp主动上报系统六维力参考坐标系

    rclcpp::CallbackGroup::SharedPtr callback_group_sub1_;
    rclcpp::CallbackGroup::SharedPtr callback_group_sub2_;
    rclcpp::CallbackGroup::SharedPtr callback_group_sub3_;
    rclcpp::CallbackGroup::SharedPtr callback_group_sub4_;
};

class UdpPublisherNode : public rclcpp::Node
{
public:
    UdpPublisherNode();
    /*******************************主动上报定时器数据处理回调函数**************************/
    void udp_timer_callback();
    void heart_timer_callback();
    bool read_data();

private:
    rclcpp::CallbackGroup::SharedPtr callback_group_time1_;
    rclcpp::CallbackGroup::SharedPtr callback_group_time2_;
    rclcpp::CallbackGroup::SharedPtr callback_group_time3_;
    rclcpp::TimerBase::SharedPtr Udp_Timer;                             //UDP定时器
    rclcpp::TimerBase::SharedPtr Heart_Timer;                           //心跳定时器，检查断开情况
    /*****************************************************UDP数据发布话题************************************************/
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr Joint_Position_Result;                                //关节当前状态发布器
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr Arm_Position_Result;                                      //末端位姿当前状态发布器
    rclcpp::Publisher<rm_ros_interfaces::msg::Sixforce>::SharedPtr Six_Force_Result;                                 //六维力发布器
    rclcpp::Publisher<rm_ros_interfaces::msg::Sixforce>::SharedPtr Six_Zero_Force_Result;                            //六维力目标坐标系下系统受力发布器
    rclcpp::Publisher<rm_ros_interfaces::msg::Sixforce>::SharedPtr One_Force_Result;                                 //一维力发布器
    rclcpp::Publisher<rm_ros_interfaces::msg::Sixforce>::SharedPtr One_Zero_Force_Result;                            //一维力目标坐标系下系统受力发布器
    rclcpp::Publisher<rm_ros_interfaces::msg::Jointerrorcode>::SharedPtr Joint_Error_Code_Result;                    //关节报错信息发布器
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr Sys_Err_Result;                                              //系统报错发布器
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr Arm_Err_Result;                                              //机械臂报错发布器
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr Arm_Coordinate_Result;                                       //力传感器基准坐标发布器
    int connect_state = 0;                             //网络连接状态
    int come_time = 0;
    struct sockaddr_in clientAddr;
    socklen_t clientAddrLen = sizeof(clientAddr);
    char udp_socket_buffer[800];

};

