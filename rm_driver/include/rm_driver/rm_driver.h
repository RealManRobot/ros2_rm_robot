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
#include <arpa/inet.h>


#include <sys/ioctl.h>          // 设置非阻塞需要用到的头文件
#include <sys/time.h>
#include <sys/select.h>         //使用fd_set结构体时使用。
#include <fcntl.h>  

#include "rm_service.h"
#include "rm_define.h"
#include "rm_ros_interfaces/msg/movej.hpp"
#include "rm_ros_interfaces/msg/movel.hpp"
#include "rm_ros_interfaces/msg/movec.hpp"
#include "rm_ros_interfaces/msg/movejp.hpp"
#include "rm_ros_interfaces/msg/jointteach.hpp"
#include "rm_ros_interfaces/msg/ortteach.hpp"
#include "rm_ros_interfaces/msg/posteach.hpp"
#include "rm_ros_interfaces/msg/setrealtimepush.hpp"
#include "rm_ros_interfaces/msg/softwarebuildinfo.hpp"
#include "rm_ros_interfaces/msg/armsoftversion.hpp"
#include "rm_ros_interfaces/msg/sixforce.hpp"
#include "rm_ros_interfaces/msg/jointerrorcode.hpp"
#include "rm_ros_interfaces/msg/forcepositionmovejoint.hpp"
#include "rm_ros_interfaces/msg/forcepositionmovepose.hpp"
#include "rm_ros_interfaces/msg/forcepositionmove.hpp"
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
#include "rm_ros_interfaces/msg/udpliftstate.hpp"
#include "rm_ros_interfaces/msg/liftheight.hpp"
#include "rm_ros_interfaces/msg/expandstate.hpp"
#include "rm_ros_interfaces/msg/expandpos.hpp"
#include "rm_ros_interfaces/msg/udpexpandstate.hpp"
#include "rm_ros_interfaces/msg/handstatus.hpp"
#include "rm_ros_interfaces/msg/armcurrentstatus.hpp"
#include "rm_ros_interfaces/msg/jointcurrent.hpp"
#include "rm_ros_interfaces/msg/jointenflag.hpp"
#include "rm_ros_interfaces/msg/jointposeeuler.hpp"
#include "rm_ros_interfaces/msg/jointspeed.hpp"
#include "rm_ros_interfaces/msg/jointtemperature.hpp"
#include "rm_ros_interfaces/msg/jointvoltage.hpp"
#include "rm_ros_interfaces/msg/jointposcustom.hpp"
#include "rm_ros_interfaces/msg/carteposcustom.hpp"
#include "rm_ros_interfaces/msg/rmplusbase.hpp"
#include "rm_ros_interfaces/msg/rmplusstate.hpp"
#include "rm_ros_interfaces/msg/rmerr.hpp"
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
// 适配四代控制器新增
#include "rm_ros_interfaces/msg/robot_info.hpp"
// #include "rm_ros_interfaces/msg/rmversion.hpp"
#include "rm_ros_interfaces/msg/flowchartrunstate.hpp"
#include "rm_ros_interfaces/msg/trajectoryinfo.hpp"
#include "rm_ros_interfaces/msg/trajectorylist.hpp"
#include "rm_ros_interfaces/msg/modbustcpmasterinfo.hpp"
#include "rm_ros_interfaces/msg/modbustcpmasterupdata.hpp"
#include "rm_ros_interfaces/msg/modbustcpmasterlist.hpp"
#include "rm_ros_interfaces/msg/modbustcpreadparams.hpp"
#include "rm_ros_interfaces/msg/modbustcpwriteparams.hpp"
#include "rm_ros_interfaces/msg/modbusrtureadparams.hpp"
#include "rm_ros_interfaces/msg/modbusrtuwriteparams.hpp"
#include "rm_ros_interfaces/msg/programrunstate.hpp"
// #include "rm_ros_interfaces/msg/armsoftversionv3.hpp"
// #include "rm_ros_interfaces/msg/armsoftversionv4.hpp"
#include "rm_ros_interfaces/msg/moveloffset.hpp"
#include "rm_ros_interfaces/msg/jointversion.hpp"
#include "rm_ros_interfaces/msg/stop.hpp"
#include "rm_ros_interfaces/msg/mastername.hpp"
#include "rm_ros_interfaces/msg/getmodbustcpmasterlist.hpp"
#include "rm_ros_interfaces/msg/rs485params.hpp"
#include "rm_ros_interfaces/msg/modbusreaddata.hpp"
#include "rm_ros_interfaces/msg/gettrajectorylist.hpp"
#include "rm_ros_interfaces/msg/sendproject.hpp"
#include "rm_ros_interfaces/msg/toolsoftwareversionv4.hpp"
#include "rm_ros_interfaces/msg/alohastate.hpp"

#define RAD_DEGREE 57.295791433
#define DEGREE_RAD 0.01745
#define TIME_OUT 5
using namespace std::chrono_literals;
//udp数据处理函数
// void Udp_RobotStatuscallback(RobotStatus Udp_RM_Callback);
void Udp_Robot_Status_Callback(rm_realtime_arm_joint_state_t data);
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
// controller verison
int controller_version = 3;
//ctrl+c触发信号
bool ctrl_flag = false;
// 灵巧手数据发布
bool udp_hand_g = false;
// 末端设备基础信息发布
bool rm_plus_base_g = false;
// 末端设备实时信息
bool rm_plus_state_g = false;
// 拓展关节状态信息
bool udp_expand_state_g = true;
// 升降关节状态信息
bool udp_lift_state_g = false;
// 关节速度状态信息
bool udp_joint_speed_state_g = true;
// 机械臂状态信息
bool udp_arm_current_status_state_g = true;
// aloha状态信息
bool udp_aloha_state_g = true;
// 连接状态标志
int connect_state_flag = 0;
//api类
RM_Service Rm_Api;
//机械臂TCp网络通信套接字
// SOCKHANDLE m_sockhand = -1;
//机械臂控制句柄
rm_robot_handle *robot_handle;

//末端设备基础信息
typedef struct{
    char manu[10];          // 设备厂家
    int type;               // 设备类型 1：两指夹爪 2：五指灵巧手 3：三指夹爪
    char hv[10];            // 硬件版本
    char sv[10];            // 软件版本
    char bv[10];            // boot版本
    int id;                 // 设备ID
    int dof;                // 自由度
    int check;              // 自检开关
    int bee;                // 蜂鸣器开关
    bool force;             // 力控支持
    bool touch;             // 触觉支持
    int touch_num;          // 触觉个数
    int touch_sw;           // 触觉开关
    int hand;               // 手方向 1 ：左手 2： 右手
    int pos_up[12];         // 位置上限,单位：无量纲
    int pos_low[12];        // 位置下限,单位：无量纲
    int angle_up[12];       // 角度上限,单位：0.01度
    int angle_low[12];      // 角度下限,单位：0.01度
    int speed_up[12];       // 速度上限,单位：无量纲
    int speed_low[12];      // 速度下限,单位：无量纲
    int force_up[12];       // 力上限,单位：0.001N 
    int force_low[12];      // 力下限,单位：0.001N 
} RM_PLUS_BASE_INFO;

//末端设备实时信息(末端生态协议支持)
typedef struct
{
    int sys_state;                   //系统状态
    int dof_state[12];               //各自由度当前状态
    int dof_err[12];                 //各自由度错误信息
    int pos[12];                     //各自由度当前位置
    int speed[12];                   //各自由度当前速度
    int angle[12];                   //各自由度当前角度
    int current[12];                 //各自由度当前电流
    int normal_force[18];            //自由度触觉三维力的法向力
    int tangential_force[18];        //自由度触觉三维力的切向力
    int tangential_force_dir[18];    //自由度触觉三维力的切向力方向
    uint32_t tsa[12];                //自由度触觉自接近
    uint32_t tma[12];                //自由度触觉互接近
    int touch_data[18];              //触觉传感器原始数据
    int force[12];                   //自由度力矩
} RM_PLUS_STATE_INFO;

typedef struct
{
    uint8_t err_len;
    std::vector<uint32_t> err;
} RM_ERR;

//机械臂状态参数
typedef struct
{
    float    joint[7];                          //关节角度
    uint16_t err_flag[7];                       //关节错误代码
    uint16_t sys_err;                           //系统错误代码
    uint16_t arm_err;                           //机械臂错误代码
    float    one_force;                         //一维力传感器原始数据0.001N或0.001Nm
    float    six_force[6];                      //六维力数据
    float    joint_current[7];                  //机械臂电流数据
    bool     en_flag[7];                        //当前关节使能状态 ，1为上使能，0为掉使能
    float    joint_position[3];                 //当前末端关节位置，精度0.001°
    float    joint_temperature[7];              //当前关节温度，精度0.001℃
    float    joint_voltage[7];                  //当前关节电压，精度0.001V
    float    joint_euler[3];                    //欧拉角
    float    joint_quat[4];                     //四元数
    float    zero_force[6];                     //当前力传感器系统外受力数据0.001N或0.001Nm
    float    work_zero_force[6];                //当前工作坐标系下系统受到的外力数据
    float    tool_zero_force[6];                //当前该工具坐标系下系统受到的外力数据
    float    one_zero_force;                    //一维力基准坐标系下系统受力数据
    uint16_t control_version;                   //版本信息
    uint16_t coordinate;                        //当前六维力传感器的基准坐标
    uint16_t hand_angle[6];                     //手指角度数组，范围：0~2000.
    uint16_t hand_pos[6];                       //手指位置数组，范围：0~1000.
    uint16_t hand_state[6];                     //手指状态,0正在松开，1正在抓取，2位置到位停止，3力到位停止，5电流保护停止，6电缸堵转停止，7电缸故障停止
    uint16_t hand_force[6];                     //灵巧手自由度电流，单位mN
    uint16_t hand_err;                          //灵巧手系统错误，1表示有错误，0表示无错误
    uint16_t arm_current_status;                //当前机械臂状态上报，
    float    joint_speed[7];                    //当前关节速度，精度0.02RPM。
    RM_PLUS_STATE_INFO udp_rm_plus_state_info;  //末端设备实时信息
    RM_PLUS_BASE_INFO udp_rm_plus_base_info;    //末端设备实时信息
    RM_ERR udp_rm_err;
    
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
rm_ros_interfaces::msg::Handstatus udp_hand_status_;
rm_ros_interfaces::msg::Armoriginalstate Arm_original_state;        //机械臂原始数据（角度+欧拉角）
rm_ros_interfaces::msg::Armstate Arm_state;                         //机械臂数据（弧度+四元数）
rm_ros_interfaces::msg::Armcurrentstatus udp_arm_current_status_;   //
rm_ros_interfaces::msg::Jointcurrent udp_joint_current_;            //
rm_ros_interfaces::msg::Jointenflag udp_joint_en_flag_;
rm_ros_interfaces::msg::Jointposeeuler udp_joint_pose_euler_;
rm_ros_interfaces::msg::Jointspeed udp_joint_speed_;
rm_ros_interfaces::msg::Jointtemperature udp_joint_temperature_; 
rm_ros_interfaces::msg::Jointvoltage udp_joint_voltage_;
rm_ros_interfaces::msg::Rmplusbase udp_rm_plus_base_;                //末端设备基础信息
rm_ros_interfaces::msg::Rmplusstate udp_rm_plus_state_;              //末端设备实时信息
rm_ros_interfaces::msg::Rmerr udp_rm_err_;                           //报错信息
rm_ros_interfaces::msg::Udpliftstate udp_lift_data_;                 //升降关节实时信息
rm_ros_interfaces::msg::Udpexpandstate udp_expand_data_;             //拓展关节实时信息
rm_ros_interfaces::msg::Alohastate udp_aloha_data_;                  //aloha实时信息

class RmArm: public rclcpp::Node
{
public:
    RmArm();
    ~RmArm();

/**********************************************初始化需要用到的回调函数***********************************************/
    void Get_Arm_Version();      
    void Get_Controller_Version();                                                                           //获取版本信息
    void Set_UDP_Configuration(int udp_cycle, int udp_port, int udp_force_coordinate, std::string udp_ip,bool hand, bool rm_plus_base, bool rm_plus_state);  //设置udp主动上报配置
    /*******************************运动控制回调函数******************************/
    // void Arm_MoveJ_75_Callback(rm_ros_interfaces::msg::Movej75::SharedPtr msg);                          //75角度控制
    void Arm_MoveJ_Callback(rm_ros_interfaces::msg::Movej::SharedPtr msg);                                  //角度控制
    void Arm_MoveL_Callback(rm_ros_interfaces::msg::Movel::SharedPtr msg);   
    void Arm_MoveL_Offset_Callback(rm_ros_interfaces::msg::Moveloffset::SharedPtr msg);
    void Arm_MoveC_Callback(rm_ros_interfaces::msg::Movec::SharedPtr msg);                                  //圆弧运动控制
    void Arm_Movej_CANFD_Callback(rm_ros_interfaces::msg::Jointpos::SharedPtr msg);                         //角度透传控制
    void Arm_Movej_CANFD_Custom_Callback(rm_ros_interfaces::msg::Jointposcustom::SharedPtr msg);            //角度透传控制高跟随下可自定义模式
    void Arm_Movep_CANFD_Callback(rm_ros_interfaces::msg::Cartepos::SharedPtr msg);                         //位姿透传控制
    void Arm_Movep_CANFD_Custom_Callback(rm_ros_interfaces::msg::Carteposcustom::SharedPtr msg);            //位姿透传控制高跟随下可自定义模式
    void Arm_MoveJ_P_Callback(rm_ros_interfaces::msg::Movejp::SharedPtr msg);                               //位姿运动控制
    void Arm_Move_Stop_Callback(const std_msgs::msg::Empty::SharedPtr msg);                                 //轨迹急停控制
    void Arm_Emergency_Stop_Callback(const rm_ros_interfaces::msg::Stop::SharedPtr msg);                    //设置机械臂急停状态
    void Pause_Callback(const std_msgs::msg::Empty::SharedPtr msg);                                         //轨迹暂停控制
    void Set_Arm_Continue_Callback(const std_msgs::msg::Empty::SharedPtr msg);                              //轨迹暂停后恢复运动
    /**************************************************************************/
    void Set_Joint_Teach_Callback(rm_ros_interfaces::msg::Jointteach::SharedPtr msg);                       //关节示教
    void Set_Pos_Teach_Callback(rm_ros_interfaces::msg::Posteach::SharedPtr msg);                           //位置示教
    void Set_Ort_Teach_Callback(rm_ros_interfaces::msg::Ortteach::SharedPtr msg);                           //姿态示教
    void Set_Stop_Teach_Callback(const std_msgs::msg::Empty::SharedPtr msg);                                //停止示教

    /*******************************主动上报回调函数******************************/
    void Arm_Get_Realtime_Push_Callback(const std_msgs::msg::Empty::SharedPtr msg);                         //获取主动上报配置
    void Arm_Set_Realtime_Push_Callback(const rm_ros_interfaces::msg::Setrealtimepush::SharedPtr msg);      //设置主动上报配置参数
    /*******************************力位混合回调函数******************************/
    void Arm_Start_Force_Position_Move_Callback(const std_msgs::msg::Empty::SharedPtr msg);                 //力位混合开始
    void Arm_Stop_Force_Position_Move_Callback(const std_msgs::msg::Empty::SharedPtr msg);                  //力位混合结束
    void Arm_Force_Position_Move_Joint_Callback(const rm_ros_interfaces::msg::Forcepositionmovejoint::SharedPtr msg);       //力位混合透传（角度）
    void Arm_Force_Position_Move_Pose_Callback(const rm_ros_interfaces::msg::Forcepositionmovepose::SharedPtr msg);         //力位混合透传（位姿）
    void Arm_Force_Position_Move_Callback(const rm_ros_interfaces::msg::Forcepositionmove::SharedPtr msg);                  //力位混合透传（补偿）
    void Arm_Set_Force_Postion_Callback(const rm_ros_interfaces::msg::Setforceposition::SharedPtr msg);                     //使能力位混合透传
    void Arm_Stop_Force_Postion_Callback(const std_msgs::msg::Empty::SharedPtr msg);                                        //结束力位混合透传
    /*******************************坐标系回调函数******************************/
    void Arm_Change_Work_Frame_Callback(const std_msgs::msg::String::SharedPtr msg);                        //更改工作坐标系
    void Arm_Change_Tool_Frame_Callback(const std_msgs::msg::String::SharedPtr msg);                        //更改工具坐标系
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
    void Arm_Set_Hand_Follow_Angle_Callback(const rm_ros_interfaces::msg::Handangle::SharedPtr msg);        //设置灵巧手角度跟随
    void Arm_Set_Hand_Follow_Pos_Callback(const rm_ros_interfaces::msg::Handangle::SharedPtr msg);          //设置灵巧手位置跟随
    /*********************************升降机构回调函数******************************/
    void Arm_Set_Lift_Speed_Callback(const rm_ros_interfaces::msg::Liftspeed::SharedPtr msg);               //升降机构速度开环控制
    void Arm_Set_Lift_Height_Callback(const rm_ros_interfaces::msg::Liftheight::SharedPtr msg);             //升降机构位置闭环控制
    void Arm_Get_Lift_State_Callback(const std_msgs::msg::Empty::SharedPtr msg);                            //获取升降机构状态
    /*********************************拓展关节回调函数******************************/
    void Arm_Set_Expand_Speed_Callback(const std_msgs::msg::Int32::SharedPtr msg);                          //升降机构速度开环控制
    void Arm_Set_Expand_Pos_Callback(const rm_ros_interfaces::msg::Expandpos::SharedPtr msg);               //升降机构位置闭环控制
    void Arm_Get_Expand_State_Callback(const std_msgs::msg::Empty::SharedPtr msg); 
    /*******************************机械臂状态回调函数****************************/
    void Arm_Get_Current_Arm_State_Callback(const std_msgs::msg::Empty::SharedPtr msg);
    /*********************************六维力数据清零******************************/
    void Arm_Clear_Force_Data_Callback(const std_msgs::msg::Empty::SharedPtr msg);
    /*********************************六维力数据获取******************************/
    void Arm_Get_Force_Data_Callback(const std_msgs::msg::Empty::SharedPtr msg);
    /*********************************适配四代控制器******************************/
    /*******************************固件版本回调函数******************************/
    void Arm_Get_Robot_Info_Callback(const std_msgs::msg::Empty::SharedPtr msg);     //获取机械臂固件版本
    void Arm_Get_Arm_Software_Info_Callback(const std_msgs::msg::Empty::SharedPtr msg);       //获取机械臂固件版本
    void Arm_Get_Joint_Software_Version_Callback(const std_msgs::msg::Empty::SharedPtr msg);  //获取机械臂固件版本
    void Arm_Get_Tool_Software_Version_Callback(const std_msgs::msg::Empty::SharedPtr msg);   //获取机械臂固件版本
    
    void Get_Trajectory_File_List_Callback(const rm_ros_interfaces::msg::Gettrajectorylist::SharedPtr msg);   //查询轨迹列表
    void Set_Run_Trajectory_Callback(const std_msgs::msg::String::SharedPtr msg);   //开始运行指定轨迹
    void Delete_Trajectory_File_Callback(const std_msgs::msg::String::SharedPtr msg);   //删除指定轨迹
    void Save_Trajectory_File_Callback(const std_msgs::msg::String::SharedPtr msg);   //保存轨迹到控制机器

    void Arm_Get_Flowchart_Program_Run_State_Callback(const std_msgs::msg::Empty::SharedPtr msg);             //获取机械臂固件版本
    void Add_Modbus_Tcp_Master_Callback(const rm_ros_interfaces::msg::Modbustcpmasterinfo::SharedPtr msg);    //新增Modbus TCP主站
    void Update_Modbus_Tcp_Master_Callback(const rm_ros_interfaces::msg::Modbustcpmasterupdata::SharedPtr msg); //更新Modbus TCP主站
    void Delete_Modbus_Tcp_Master_Callback(const rm_ros_interfaces::msg::Mastername::SharedPtr msg);          //删除Modbus TCP主站 
    void Get_Modbus_Tcp_Master_Callback(const rm_ros_interfaces::msg::Mastername::SharedPtr msg);             //查询指定modbus主站
    void Get_Modbus_Tcp_Master_List_Callback(const rm_ros_interfaces::msg::Getmodbustcpmasterlist::SharedPtr msg);   //查询modbus主站列表
    void Set_Controller_RS485_Mode_Callback(const rm_ros_interfaces::msg::RS485params::SharedPtr msg);   // 设置控制器RS485模式(四代控制器支持)
    void Get_Controller_RS485_Mode_v4_Callback(const std_msgs::msg::Empty::SharedPtr msg);   // 查询控制器RS485模式(四代控制器支持)
    void Set_Tool_RS485_Mode_Callback(const rm_ros_interfaces::msg::RS485params::SharedPtr msg);   // 设置工具端RS485模式(四代控制器支持)
    void Get_Tool_RS485_Mode_v4_Callback(const std_msgs::msg::Empty::SharedPtr msg);   // 查询工具端RS485模式(四代控制器支持)
    void Read_Modbus_RTU_Coils_Callback(const rm_ros_interfaces::msg::Modbusrtureadparams::SharedPtr msg);// Modbus RTU协议读线圈
    void Write_Modbus_RTU_Coils_Callback(const rm_ros_interfaces::msg::Modbusrtuwriteparams::SharedPtr msg);// Modbus RTU协议写线圈
    void Read_Modbus_RTU_Input_Status_Callback(const rm_ros_interfaces::msg::Modbusrtureadparams::SharedPtr msg);// Modbus RTU协议读离散量输入
    void Read_Modbus_RTU_Holding_Registers_Callback(const rm_ros_interfaces::msg::Modbusrtureadparams::SharedPtr msg);// Modbus RTU协议读保持寄存器
    void Write_Modbus_RTU_Registers_Callback(const rm_ros_interfaces::msg::Modbusrtuwriteparams::SharedPtr msg);// Modbus RTU协议写保持寄存器
    void Read_Modbus_RTU_Input_Registers_Callback(const rm_ros_interfaces::msg::Modbusrtureadparams::SharedPtr msg);// Modbus RTU协议读输入寄存器
    void Read_Modbus_TCP_Coils_Callback(const rm_ros_interfaces::msg::Modbustcpreadparams::SharedPtr msg);// Modbus TCP协议读线圈
    void Write_Modbus_TCP_Coils_Callback(const rm_ros_interfaces::msg::Modbustcpwriteparams::SharedPtr msg);// Modbus TCP协议写线圈
    void Read_Modbus_TCP_Input_Status_Callback(const rm_ros_interfaces::msg::Modbustcpreadparams::SharedPtr msg);// Modbus TCP协议读离散量输入
    void Read_Modbus_TCP_Holding_Registers_Callback(const rm_ros_interfaces::msg::Modbustcpreadparams::SharedPtr msg);// Modbus TCP协议读保持寄存器
    void Write_Modbus_TCP_Registers_Callback(const rm_ros_interfaces::msg::Modbustcpwriteparams::SharedPtr msg);// Modbus TCP协议写保持寄存器
    void Read_Modbus_TCP_Input_Registers_Callback(const rm_ros_interfaces::msg::Modbustcpreadparams::SharedPtr msg);// Modbus TCP协议读输入寄存器

    void Close_Controller_Tcp_Modbus_Callback(const std_msgs::msg::Empty::SharedPtr msg); //关闭tcp modbus
    void Set_Controller_Tcp_Mode_Callback(const rm_ros_interfaces::msg::Modbustcpmasterinfo::SharedPtr msg); //设置tcp modbus
    void Close_Controller_RS485_Modbus_Callback(const std_msgs::msg::UInt16::SharedPtr msg);     //关闭485 modbus
    
    void Send_Project_Callback(const rm_ros_interfaces::msg::Sendproject::SharedPtr msg);   //文件下发
    void Get_Program_Run_State_Callback(const std_msgs::msg::Empty::SharedPtr msg);   //查询在线编程运行状态   

    void Arm_Clear_System_Err_Callback(const std_msgs::msg::Empty::SharedPtr msg);          //清除系统错误
    

    
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
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr MoveL_offset_Cmd_Result;
    /*******************************************MoveL运动控制订阅器*************************************/
    rclcpp::Subscription<rm_ros_interfaces::msg::Movel>::SharedPtr MoveL_Cmd;
    rclcpp::Subscription<rm_ros_interfaces::msg::Moveloffset>::SharedPtr MoveL_offset_Cmd;
    /****************************************MoveC运动控制结果发布器*************************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr MoveC_Cmd_Result;
    /*******************************************MoveC运动控制订阅器*************************************/
    rclcpp::Subscription<rm_ros_interfaces::msg::Movec>::SharedPtr MoveC_Cmd;
    /*******************************************角度透传运动控制订阅器*************************************/
    rclcpp::Subscription<rm_ros_interfaces::msg::Jointpos>::SharedPtr Movej_CANFD_Cmd;
    rclcpp::Subscription<rm_ros_interfaces::msg::Jointposcustom>::SharedPtr Movej_CANFD_Custom_Cmd;
    /*******************************************角度透传运动控制订阅器*************************************/
    // rclcpp::Subscription<rm_ros_interfaces::msg::Jointpos75>::SharedPtr Movej_CANFD_75_Cmd;
    /*******************************************位姿透传运动控制订阅器*************************************/
    rclcpp::Subscription<rm_ros_interfaces::msg::Cartepos>::SharedPtr Movep_CANFD_Cmd;
    rclcpp::Subscription<rm_ros_interfaces::msg::Carteposcustom>::SharedPtr Movep_CANFD_Custom_Cmd;
    /****************************************MoveJ_P运动控制结果发布器*************************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr MoveJ_P_Cmd_Result;
    /*******************************************MoveJ_P运动控制订阅器*************************************/
    rclcpp::Subscription<rm_ros_interfaces::msg::Movejp>::SharedPtr MoveJ_P_Cmd;
    /********************************************轨迹急停结果发布器*****************************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Move_Stop_Cmd_Result;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Arm_Emergency_Stop_Cmd_Result;
    /********************************************轨迹暂停结果发布器*****************************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Arm_Pause_Cmd_Result;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr Arm_Pause_Cmd;
    /********************************************轨迹暂停后恢复*****************************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Set_Arm_Continue_Cmd_Result;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr Set_Arm_Continue_Cmd;
    
    /***********************************************轨迹急停控制订阅器*************************************/
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr Move_Stop_Cmd;
    rclcpp::Subscription<rm_ros_interfaces::msg::Stop>::SharedPtr Arm_Emergency_Stop_Cmd;
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
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr Set_Stop_Teach_Cmd;
    /********************************************************end******************************************************/

    /********************************************************适配四代控制器新增***************************************************/
    /********************************************************固件版本***************************************************/
    /*************************************************查询固件版本发布器****************************************/
    // rclcpp::Publisher<rm_ros_interfaces::msg::Armsoftversionv3>::SharedPtr Get_Arm_Software_Version_Result_v3;
    // rclcpp::Publisher<rm_ros_interfaces::msg::Armsoftversionv4>::SharedPtr Get_Arm_Software_Version_Result_v4;
    rclcpp::Publisher<rm_ros_interfaces::msg::Armsoftversion>::SharedPtr Get_Arm_Software_Version_Result;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr Get_Arm_Software_Version_Cmd;
    rclcpp::Publisher<rm_ros_interfaces::msg::RobotInfo>::SharedPtr Get_Robot_Info_Result;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr Get_Robot_Info_Cmd;
    rclcpp::Publisher<rm_ros_interfaces::msg::Jointversion>::SharedPtr Get_Joint_Software_Version_Result;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr Get_Joint_Software_Version_Cmd;
    rclcpp::Publisher<rm_ros_interfaces::msg::Toolsoftwareversionv4>::SharedPtr Get_Tool_Software_Version_Result;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr Get_Tool_Software_Version_Cmd;
    rclcpp::Publisher<rm_ros_interfaces::msg::Flowchartrunstate>::SharedPtr Get_Flowchart_Program_Run_State_Result;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr Get_Flowchart_Program_Run_State_Cmd;
    /*************************************************轨迹列表相关接口****************************************/
    rclcpp::Publisher<rm_ros_interfaces::msg::Trajectorylist>::SharedPtr Get_Trajectory_File_List_Result;
    rclcpp::Subscription<rm_ros_interfaces::msg::Gettrajectorylist>::SharedPtr Get_Trajectory_File_List_Cmd;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Set_Run_Trajectory_Result;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr Set_Run_Trajectory_Cmd;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Delete_Trajectory_File_Result;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr Delete_Trajectory_File_Cmd;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Save_Trajectory_File_Result;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr Save_Trajectory_File_Cmd;

    /*************************************************Modbus相关接口(四代)****************************************/
    /*************************************************新增Modbus TCP主站****************************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Add_Modbus_Tcp_Master_Result;
    rclcpp::Subscription<rm_ros_interfaces::msg::Modbustcpmasterinfo>::SharedPtr Add_Modbus_Tcp_Master_Cmd;    
    /*************************************************更新Modbus TCP主站****************************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Update_Modbus_Tcp_Master_Result;
    rclcpp::Subscription<rm_ros_interfaces::msg::Modbustcpmasterupdata>::SharedPtr Update_Modbus_Tcp_Master_Cmd;
    /*************************************************删除Modbus TCP主站****************************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Delete_Modbus_Tcp_Master_Result;
    rclcpp::Subscription<rm_ros_interfaces::msg::Mastername>::SharedPtr Delete_Modbus_Tcp_Master_Cmd;    
    /*************************************************查询指定modbus主站****************************************/
    rclcpp::Publisher<rm_ros_interfaces::msg::Modbustcpmasterinfo>::SharedPtr Get_Modbus_Tcp_Master_Result;
    rclcpp::Subscription<rm_ros_interfaces::msg::Mastername>::SharedPtr Get_Modbus_Tcp_Master_Cmd;    
    /*************************************************查询modbus主站列表****************************************/
    rclcpp::Publisher<rm_ros_interfaces::msg::Modbustcpmasterlist>::SharedPtr Get_Modbus_Tcp_Master_List_Result;
    rclcpp::Subscription<rm_ros_interfaces::msg::Getmodbustcpmasterlist>::SharedPtr Get_Modbus_Tcp_Master_List_Cmd;
    /*************************************************设置控制器RS485模式(四代三代控制器支持)****************************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Set_Controller_RS485_Mode_Result;
    rclcpp::Subscription<rm_ros_interfaces::msg::RS485params>::SharedPtr Set_Controller_RS485_Mode_Cmd;
    /*************************************************查询控制器RS485模式(四代控制器支持)****************************************/
    rclcpp::Publisher<rm_ros_interfaces::msg::RS485params>::SharedPtr Get_Controller_RS485_Mode_v4_Result;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr Get_Controller_RS485_Mode_v4_Cmd;
    /*************************************************设置工具端RS485模式(四代控制器支持)****************************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Set_Tool_RS485_Mode_Result;
    rclcpp::Subscription<rm_ros_interfaces::msg::RS485params>::SharedPtr Set_Tool_RS485_Mode_Cmd;
    /*************************************************查询工具端RS485模式(四代控制器支持)****************************************/
    rclcpp::Publisher<rm_ros_interfaces::msg::RS485params>::SharedPtr Get_Tool_RS485_Mode_v4_Result;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr Get_Tool_RS485_Mode_v4_Cmd;

    /*************************************************Modbus相关接口(三代)****************************************/
    /*************************************************设置控制器RS485模式****************************************/
    // rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Set_Controller_RS485_Mode_Result;
    // rclcpp::Subscription<rm_ros_interfaces::msg::RS485params>::SharedPtr Set_Controller_RS485_Mode_Cmd;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Close_Controller_RS485_Modbus_Result;
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr Close_Controller_RS485_Modbus_Cmd;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Close_Controller_Tcp_Modbus_Result;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr Close_Controller_Tcp_Modbus_Cmd;
    // rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Close_Controller_Tcp_Modbus_Result;
    rclcpp::Subscription<rm_ros_interfaces::msg::Modbustcpmasterinfo>::SharedPtr Set_Controller_Tcp_Modbus_Cmd;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Set_Controller_Tcp_Modbus_Result;


    /*************************************************Modbus RTU协议读线圈****************************************/
    rclcpp::Publisher<rm_ros_interfaces::msg::Modbusreaddata>::SharedPtr Read_Modbus_RTU_Coils_Result;
    rclcpp::Subscription<rm_ros_interfaces::msg::Modbusrtureadparams>::SharedPtr Read_Modbus_RTU_Coils_Cmd;
    /*************************************************Modbus RTU协议写线圈****************************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Write_Modbus_RTU_Coils_Result;
    rclcpp::Subscription<rm_ros_interfaces::msg::Modbusrtuwriteparams>::SharedPtr Write_Modbus_RTU_Coils_Cmd;
    /*************************************************Modbus RTU协议读离散量输入****************************************/
    rclcpp::Publisher<rm_ros_interfaces::msg::Modbusreaddata>::SharedPtr Read_Modbus_RTU_Input_Status_Result;
    rclcpp::Subscription<rm_ros_interfaces::msg::Modbusrtureadparams>::SharedPtr Read_Modbus_RTU_Input_Status_Cmd;
    /*************************************************Modbus RTU协议读保持寄存器****************************************/
    rclcpp::Publisher<rm_ros_interfaces::msg::Modbusreaddata>::SharedPtr Read_Modbus_RTU_Holding_Registers_Result;
    rclcpp::Subscription<rm_ros_interfaces::msg::Modbusrtureadparams>::SharedPtr Read_Modbus_RTU_Holding_Registers_Cmd;
    /*************************************************Modbus RTU协议写保持寄存器****************************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Write_Modbus_RTU_Registers_Result;
    rclcpp::Subscription<rm_ros_interfaces::msg::Modbusrtuwriteparams>::SharedPtr Write_Modbus_RTU_Registers_Cmd;
    /*************************************************Modbus RTU协议读输入寄存器****************************************/
    rclcpp::Publisher<rm_ros_interfaces::msg::Modbusreaddata>::SharedPtr Read_Modbus_RTU_Input_Registers_Result;
    rclcpp::Subscription<rm_ros_interfaces::msg::Modbusrtureadparams>::SharedPtr Read_Modbus_RTU_Input_Registers_Cmd;

    /*************************************************Modbus TCP协议读线圈****************************************/
    rclcpp::Publisher<rm_ros_interfaces::msg::Modbusreaddata>::SharedPtr Read_Modbus_TCP_Coils_Result;
    rclcpp::Subscription<rm_ros_interfaces::msg::Modbustcpreadparams>::SharedPtr Read_Modbus_TCP_Coils_Cmd;
    /*************************************************Modbus TCP协议写线圈****************************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Write_Modbus_TCP_Coils_Result;
    rclcpp::Subscription<rm_ros_interfaces::msg::Modbustcpwriteparams>::SharedPtr Write_Modbus_TCP_Coils_Cmd;
    /*************************************************Modbus TCP协议读离散量输入****************************************/
    rclcpp::Publisher<rm_ros_interfaces::msg::Modbusreaddata>::SharedPtr Read_Modbus_TCP_Input_Status_Result;
    rclcpp::Subscription<rm_ros_interfaces::msg::Modbustcpreadparams>::SharedPtr Read_Modbus_TCP_Input_Status_Cmd;
    /*************************************************Modbus TCP协议读保持寄存器****************************************/
    rclcpp::Publisher<rm_ros_interfaces::msg::Modbusreaddata>::SharedPtr Read_Modbus_TCP_Holding_Registers_Result;
    rclcpp::Subscription<rm_ros_interfaces::msg::Modbustcpreadparams>::SharedPtr Read_Modbus_TCP_Holding_Registers_Cmd;
    /*************************************************Modbus TCP协议写保持寄存器****************************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Write_Modbus_TCP_Registers_Result;
    rclcpp::Subscription<rm_ros_interfaces::msg::Modbustcpwriteparams>::SharedPtr Write_Modbus_TCP_Registers_Cmd;
    /*************************************************Modbus TCP协议读输入寄存器****************************************/
    rclcpp::Publisher<rm_ros_interfaces::msg::Modbusreaddata>::SharedPtr Read_Modbus_TCP_Input_Registers_Result;
    rclcpp::Subscription<rm_ros_interfaces::msg::Modbustcpreadparams>::SharedPtr Read_Modbus_TCP_Input_Registers_Cmd;

    /*************************************************文件下发****************************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Send_Project_Result;
    rclcpp::Subscription<rm_ros_interfaces::msg::Sendproject>::SharedPtr Send_Project_Cmd;
    /*************************************************查询在线编程运行状态****************************************/
    rclcpp::Publisher<rm_ros_interfaces::msg::Programrunstate>::SharedPtr Get_Program_Run_State_Result;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr Get_Program_Run_State_Cmd;
    
    /*************************************************适配四代控制器新增 end****************************************/
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
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr Stop_Force_Postion_Cmd;
    
    /**************************************************75力位混合角度透传订阅器***************************************/
    // rclcpp::Subscription<rm_ros_interfaces::msg::Forcepositionmovejoint75>::SharedPtr Force_Position_Move_Joint_75_Cmd;
    /**************************************************力位混合角度透传订阅器***************************************/
    rclcpp::Subscription<rm_ros_interfaces::msg::Forcepositionmovejoint>::SharedPtr Force_Position_Move_Joint_Cmd;
    /***********************************************力位混合位姿透传订阅器*********************************************/
    rclcpp::Subscription<rm_ros_interfaces::msg::Forcepositionmovepose>::SharedPtr Force_Position_Move_Pose_Cmd;
    /***********************************************力位混合补偿透传订阅器*********************************************/
    rclcpp::Subscription<rm_ros_interfaces::msg::Forcepositionmove>::SharedPtr Force_Position_Move_Cmd;
    /**********************************************************************end************************************************************/

    /**************************************************************坐标系指令发布器**********************************************************/
    /****************************************切换工作坐标系发布器*************************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Change_Work_Frame_Result;
    /*****************************************切换工作坐标系订阅器************************************/
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr Change_Work_Frame_Cmd;
    /****************************************切换工具坐标系发布器*************************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Change_Tool_Frame_Result;
    /*****************************************切换工具坐标系订阅器************************************/
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr Change_Tool_Frame_Cmd;
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
    /**************************************设置灵巧手各关节角度跟随结果发布器*********************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Set_Hand_Follow_Angle_Result;
    /***************************************设置灵巧手各关节角度设置订阅器********************************/
    rclcpp::Subscription<rm_ros_interfaces::msg::Handangle>::SharedPtr Set_Hand_Follow_Angle_Cmd;
    /**************************************设置灵巧手各关节位置跟随结果发布器*********************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Set_Hand_Follow_Pos_Result;
    /***************************************设置灵巧手各关节位置跟随订阅器********************************/
    rclcpp::Subscription<rm_ros_interfaces::msg::Handangle>::SharedPtr Set_Hand_Follow_Pos_Cmd;
/*****************************************************************end******************************************************************/

/********************************************************************升降机构***********************************************************/
    /******************************************设置升降机构速度发布器****************Publisher*****************/
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
 
/********************************************************************拓展关节***********************************************************/
    /******************************************设置拓展关节速度发布器****************Publisher*****************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Set_Expand_Speed_Result;
    /*******************************************设置拓展关节速度订阅器********************************/
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr Set_Expand_Speed_Cmd;
    /****************************************设置拓展关节高度发布器*********************************/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Set_Expand_Pos_Result;
    /***************************************设置拓展关节高度订阅器********************************/
    rclcpp::Subscription<rm_ros_interfaces::msg::Expandpos>::SharedPtr Set_Expand_Pos_Cmd;
    /****************************************获取拓展关节状态发布器*********************************/
    rclcpp::Publisher<rm_ros_interfaces::msg::Expandstate>::SharedPtr Get_Expand_State_Result;
    /***************************************获取拓展关节状态订阅器********************************/
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr Get_Expand_State_Cmd;
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
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr Clear_Force_Data_Cmd;
    /********************************************六维力数据获取发布器****************************************/
    rclcpp::Publisher<rm_ros_interfaces::msg::Sixforce>::SharedPtr Get_Force_Data_Result;
    rclcpp::Publisher<rm_ros_interfaces::msg::Sixforce>::SharedPtr Get_Zero_Force_Result;
    rclcpp::Publisher<rm_ros_interfaces::msg::Sixforce>::SharedPtr Get_Work_Zero_Result;
    rclcpp::Publisher<rm_ros_interfaces::msg::Sixforce>::SharedPtr Get_Tool_Zero_Result;
    /******************************************六维力数据获取订阅器*************************************/
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr Get_Force_Data_Cmd;
/********************************************************************end***********************************************************/

/********************************************************************系统配置***********************************************************/
    /******************************************清除系统错误****************************************/
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr Clear_System_Err_Cmd;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Clear_System_Err_Result;
/********************************************************************end***********************************************************/
    std::string arm_ip_ = "192.168.1.188";    
    std::string udp_ip_ = "192.168.1.10";
    std::string  arm_type_ = "RM_75";  
    
    
    int tcp_port_ = 8081;  
    int udp_port_ = 9501; 
    int arm_dof_ = 7;                                  //机械臂自由度
    int udp_cycle_ = 5;                                //udp主动上报周期（ms）
    int udp_force_coordinate_ = 0;                     //udp主动上报系统六维力参考坐标系
    bool udp_hand_ = false;                            //灵巧手数据发布，与末端高速不可共用
    bool udp_rm_plus_base_ = false;                    //末端设备基础信息udp发布
    bool udp_rm_plus_state_ = false;                   //末端设备状态信息udp发布
    bool udp_joint_speed_state_= false;                //设置关节速度主动上报
    bool udp_lift_state_= false;                       //设置升降关节主动上报
    bool udp_expand_state_= false;                     //设置拓展关节主动上报
    bool udp_arm_current_status_state_= false;         //设置机械臂状态主动上报
    bool udp_aloha_state_= false;                      //aloha状态主动上报
    int trajectory_mode_ = 0;
    int radio_ = 50; 
    int controller_type = 3;                           //控制器类型
    // bool stop_flag = false;                            //停止信号接收
    std::vector<std::string> arm_joints;

    rclcpp::CallbackGroup::SharedPtr callback_group_sub1_;
    rclcpp::CallbackGroup::SharedPtr callback_group_sub2_;
    rclcpp::CallbackGroup::SharedPtr callback_group_sub3_;
    rclcpp::CallbackGroup::SharedPtr callback_group_sub4_;
    rclcpp::CallbackGroup::SharedPtr callback_group_sub5_;
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
    // rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr Sys_Err_Result;                                              //系统报错发布器
    rclcpp::Publisher<rm_ros_interfaces::msg::Rmerr>::SharedPtr Rm_Err_Result;                                              //机械臂报错发布器
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr Arm_Coordinate_Result;                                       //力传感器基准坐标发布器
    rclcpp::Publisher<rm_ros_interfaces::msg::Handstatus>::SharedPtr Hand_Status_Result;                             //灵巧手数据发布器
    rclcpp::Publisher<rm_ros_interfaces::msg::Rmplusbase>::SharedPtr Rm_Plus_Base_Result;                            //末端设备基础信息udp主动上报发布器
    rclcpp::Publisher<rm_ros_interfaces::msg::Rmplusstate>::SharedPtr Rm_Plus_State_Result;                           //末端设备实时信息udp主动上报发布器
    rclcpp::Publisher<rm_ros_interfaces::msg::Armcurrentstatus>::SharedPtr Arm_Current_Status_Result;                //机械臂当前状态发布器
    rclcpp::Publisher<rm_ros_interfaces::msg::Jointcurrent>::SharedPtr Joint_Current_Result;                         //关节当前电流发布器
    rclcpp::Publisher<rm_ros_interfaces::msg::Jointenflag>::SharedPtr Joint_En_Flag_Result;                          //关节使能状态布器
    rclcpp::Publisher<rm_ros_interfaces::msg::Jointposeeuler>::SharedPtr Joint_Pose_Euler_Result;                    //末端位姿欧拉角形式发布器
    rclcpp::Publisher<rm_ros_interfaces::msg::Jointspeed>::SharedPtr Joint_Speed_Result;                             //关节速度发布器
    rclcpp::Publisher<rm_ros_interfaces::msg::Jointtemperature>::SharedPtr Joint_Temperature_Result;                 //关节温度发布器
    rclcpp::Publisher<rm_ros_interfaces::msg::Jointvoltage>::SharedPtr Joint_Voltage_Result;                         //关节电压发布器
    rclcpp::Publisher<rm_ros_interfaces::msg::Udpliftstate>::SharedPtr Lift_State_Result;                            //升降关节发布器
    rclcpp::Publisher<rm_ros_interfaces::msg::Udpexpandstate>::SharedPtr Expand_State_Result;                         //拓展关节发布器
    rclcpp::Publisher<rm_ros_interfaces::msg::Alohastate>::SharedPtr Aloha_State_Result;                              //aloha状态发布器
    int connect_state = 0;                             //网络连接状态
    int come_time = 0;
    struct sockaddr_in clientAddr;
    socklen_t clientAddrLen = sizeof(clientAddr);
    char udp_socket_buffer[1000];

};

