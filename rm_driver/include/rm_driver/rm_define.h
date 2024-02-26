#ifndef RM_DEFINE_H
#define RM_DEFINE_H
//////////////////////////////////////////////////////////////////////////////////
//睿尔曼智能科技有限公司        Author:Dong Qinpeng
//创建日期:2022/08/23
//版本：V4.0
//版权所有，盗版必究。
//Copyright(C) 睿尔曼智能科技有限公司
//All rights reserved
//文档说明：该文档定义了机械臂接口函数中使用到的结构体和错误代码类型
//////////////////////////////////////////////////////////////////////////////////
#ifdef __cplusplus
extern "C" {
#endif
#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "cJSON.h"


#include "robot_define.h"

#ifdef _WIN32
#define MSG_DONTWAIT 0
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <process.h>
typedef SOCKET  SOCKHANDLE;
#endif

#ifdef __linux
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

typedef int SOCKHANDLE;
#endif

#define  SDK_VERSION (char*)"4.2.8"

typedef unsigned char byte;
typedef unsigned short u16;

////位姿结构体
//typedef struct
//{
//    //位置
//    float px;
//    float py;
//    float pz;
//    //欧拉角
//    float rx;
//    float ry;
//    float rz;
//}POSE;

// 回调函数结构体
typedef struct
{
    int sockhand;       // 返回调用时句柄
    int codeKey;        // 调用接口类型
    int errCode;        // 接口错误码
    Pose pose;          // 位姿信息
    float joint[7];     // 角度信息
    float direction_force[7]; //所有方向的力或力矩
    int nforce;         // 返回力大小
    uint16_t sys_err;   // 返回系统错误
} CallbackData;

//坐标系
typedef struct
{
    char name[12];    //坐标系名称,不超过10个字符
}FRAME_NAME;

//坐标系名称列表-兼容MATLAB API
typedef struct
{
    FRAME_NAME name[10];    //名称列表
}NAMES;

//坐标系
typedef struct
{
    FRAME_NAME frame_name;  //坐标系名称
    Pose pose;              //坐标系位姿
    float payload;     //坐标系末端负载重量
    float x;           //坐标系末端负载位置
    float y;           //坐标系末端负载位置
    float z;           //坐标系末端负载位置
}FRAME;

//机械臂控制模式
typedef enum
{
    None_Mode = 0,     //无规划
    Joint_Mode = 1,    //关节空间规划
    Line_Mode = 2,     //笛卡尔空间直线规划
    Circle_Mode = 3,   //笛卡尔空间圆弧规划
    Replay_Mode = 4,    //拖动示教轨迹复现
}ARM_CTRL_MODES;

//机械臂位置示教模式
typedef enum
{
    X_Dir = 0,       //X轴方向
    Y_Dir = 1,       //Y轴方向
    Z_Dir = 2,       //Z轴方向
}POS_TEACH_MODES;

//机械臂姿态示教模式
typedef enum
{
    RX_Rotate = 0,       //RX轴方向
    RY_Rotate = 1,       //RY轴方向
    RZ_Rotate = 2,       //RZ轴方向
}ORT_TEACH_MODES;

//控制器通讯方式选择
typedef enum
{
    WIFI_AP = 0,       //WIFI AP模式
    WIFI_STA = 1,      //WIFI STA模式
    BlueTeeth = 2,     //蓝牙模式
    USB       = 3,     //通过控制器UART-USB接口通信
    Ethernet  = 4      //以太网口
} ARM_COMM_TYPE;

//机械臂自由度
#define   ARM_DOF               7              //机械臂自由度
//机械臂状态参数
typedef struct
{
    //float joint[ARM_DOF];       // 关节角度
    float temperature[ARM_DOF];   // 关节温度
    float voltage[ARM_DOF];       // 关节电压
    float current[ARM_DOF];       // 关节电流
    byte en_state[ARM_DOF];       // 使能状态
    uint16_t err_flag[ARM_DOF];   // 关节错误代码
    uint16_t sys_err;             // 机械臂系统错误代码
}JOINT_STATE;

//位置
typedef struct
{
    //position
    float px;
    float py;
    float pz;
    //orientation
    float w;
    float x;
    float y;
    float z;
}POSE_QUAT;

//姿态
typedef struct
{
    float rx;
    float ry;
    float rz;
}ORT;
//typedef struct
//{
//    POSE2 pose;
//    ORT ort;
//}KINEMATIC;
//旋转矩阵
//typedef struct
//{
//    int irow;
//    int iline;
//    float data[4][4];
//}_Matrix;

//位置
//typedef struct
//{
//    float x;
//    float y;
//    float z;
//}pos;

////四元数姿态
//typedef struct
//{
//    float w;
//    float x;
//    float y;
//    float z;
//}ort;

////欧拉角姿态
//typedef struct
//{
//    float Phi;
//    float Theta;
//    float Psi;
//}eul;

//机械臂位姿
//typedef struct
//{
//    pos position;    // px
//    ort orientation; // rx
//    eul euler;
//}Pose;

// 无线网络信息结构体
typedef struct{
    int channel;               // 信道 AP模式时存在此字段
    char ip[16];               // IP 地址
    char mac[18];              // MAC 地址
    char mask[16];             // 子网掩码
    char mode[5];              // 模式
    char password[16];         // 密码
    char ssid[32];             // 网络名称 (SSID)
}WiFi_Info;

// 在线编程存储信息
typedef struct  {
    int id;
    int size;
    int speed;
    char trajectory_name[32];
}TrajectoryData;

// 在线编程程序列表
typedef struct{
    int page_num;       // 页码（全部查询时此参数传NULL）
    int page_size;      // 每页大小（全部查询时此参数传NULL）
    int total_size;
    char vague_search[32];  // 模糊搜索 （传递此参数可进行模糊查询）
    TrajectoryData list[100];   // 符合的在线编程列表
}ProgramTrajectoryData;

// 在线编程运行状态结构体
typedef struct{
    int run_state;  // 0 未开始 1运行中 2暂停中
    int id;         // 运行轨迹编号，已存储轨迹 的id，没有存储则为0 ，未运行则不返回
    int plan_num;   // 运行到的行数，未运行则不返回
    int loop_num[10];   // 存在循环指令的行数，未运行则不返回
    int loop_cont[10];  // 循环指令行数对应的运行次数，未运行则不返回
    int step_mode;  // 单步模式，1为单步模式，0为非单步模式，未运行则不返回
    int plan_speed; // 全局规划速度比例 1-100，未运行则不返回
}ProgramRunState;

//扩展关节配置参数
typedef struct{
    int32_t rpm_max;        //  关节的最大速度
    int32_t rpm_acc;        // 最大加速度
    int32_t conversin_coe;  // 减速比,该字段只针对升降关节（指直线运动的关节）有效；如果是旋转关节（指做旋转运动的关节），则不发送该字段，注意参数的设置一定跟电机匹配，避免发生意外
    int32_t limit_min;      // 最小限位，如果是旋转关节，单位为°，精度0.001，如果是升降关节，则单位为mm
    int32_t limit_max;      // 最大限位，如果是旋转关节，单位为°，精度0.001，如果是升降关节，则单位为mm
}ExpandConfig;


//实时机械臂状态上报
// 机械臂关节状态结构体
typedef struct {
    float joint_current[ARM_DOF];
    byte joint_en_flag[ARM_DOF];
    uint16_t joint_err_code[ARM_DOF];
    float joint_position[ARM_DOF];
    float joint_temperature[ARM_DOF];
    float joint_voltage[ARM_DOF];
} JointStatus;

// 力传感器结构体
typedef struct {
    float force[6];
    float zero_force[6];
    int coordinate;         //系统外受力数据的坐标系，0为传感器坐标系 1为当前工作坐标系 2为当前工具坐标系
} ForceData;

// UDP接口实时机械臂状态上报
typedef struct {
    int errCode;        // 接口错误码
    char *arm_ip;
    uint16_t arm_err;
    JointStatus joint_status;
    ForceData force_sensor;
    uint16_t sys_err;
    Pose waypoint;
} RobotStatus;


//电子围栏名称列表
typedef struct
{
    char name[12];    // 电子围栏名称,不超过10个字符
}ElectronicFenceNames;

//所有电子围栏信息
typedef struct
{
    int form;           // 形状，1表示立方体
    char name[12];      // 电子围栏名称，不超过10个字节，支持字母、数字、下划线
    // 立方体
    int32_t x_min_limit;    // 立方体基于世界坐标系 X 方向最小位置，单位 0.001m
    int32_t x_max_limit;    // 立方体基于世界坐标系 X 方向最大位置，单位 0.001m
    int32_t y_min_limit;    // 立方体基于世界坐标系 Y 方向最小位置，单位 0.001m
    int32_t y_max_limit;    // 立方体基于世界坐标系 Y 方向最大位置，单位 0.001m
    int32_t z_min_limit;    // 立方体基于世界坐标系 Z 方向最小位置，单位 0.001m
    int32_t z_max_limit;    // 立方体基于世界坐标系 Z 方向最大位置，单位 0.001m
}ElectronicFenceConfig;

// 电子围栏参数列表-适配matlab
typedef struct
{
    ElectronicFenceConfig config[10];
}ElectronicFenceConfigList;

//夹爪状态
typedef struct
{
    bool enable_state;  // 夹爪使能标志，0 表示未使能，1 表示使能
    bool status;         // 夹爪在线状态，0 表示离线， 1表示在线
    int32_t error;          // 夹爪错误信息，低8位表示夹爪内部的错误信息bit5-7 保留bit4 内部通bit3 驱动器bit2 过流 bit1 过温bit0 堵转
    int32_t mode;           // 当前工作状态：1 夹爪张开到最大且空闲，2 夹爪闭合到最小且空闲，3 夹爪停止且空闲，4 夹爪正在闭合，5 夹爪正在张开，6 夹爪闭合过程中遇到力控停止
    int32_t current_force;  // 夹爪当前的压力，单位g
    int32_t temperature;    // 当前温度，单位℃
    int32_t actpos;         // 夹爪开口度
}GripperState;

typedef struct{
    char build_time[20];
    char version[10];
}CtrlInfo;

typedef struct{
    char model_version[5];
}DynamicInfo;

typedef struct{
    char build_time[20];
    char version[10];
}PlanInfo;

typedef struct {
    char version[10];
}AlgorithmInfo;

// 机械臂软件信息
typedef struct
{
    char product_version[10];
    AlgorithmInfo algorithm_info;
    CtrlInfo ctrl_info;
    DynamicInfo dynamic_info;
    PlanInfo plan_info;
}ArmSoftwareInfo;

typedef void (*RobotStatusListener)(RobotStatus data);
typedef void (*RM_Callback)(CallbackData data);

#define  M_PI_RAD    0.0174533f
#define  MI_PI_ANG   57.2957805f
#define  PI          3.14159f

#define  M_PI		 3.14159265358979323846
#define  DELTA       0.26f   //关节判断角度差
#define  DELTA2      2*PI    //关节运动到该处

// 机械臂型号
#define ARM_65      65
#define ARM_63_1    631
#define ARM_63_2    632
#define ARM_ECO65   651
#define ARM_75      75

// 是否打印日志
#define RM_DISPLAY_LOG 0  // 0 不打印, 1打印
#define RM_NONBLOCK 0   // 非阻塞
#define RM_BLOCK 1      // 阻塞

#define RM_INPUT 0      // 输入
#define RM_OUTPUT 1     // 输出

#define RM_LOW 0        // 低
#define RM_TALL 1       // 高

#define PORT_CONTROLLER  0 // 控制器
#define PORT_ENDMODEL    1 // 末端接口板

#define NAVIGATION_MAGNETIC 0 // 磁条导航
#define OPING_CONTROLLER 1    // 开环控制模式

#define TRAJECTORY_FILE_NAME_MAX_LENGTH 300
//系统初始化错误代码
#define SYS_NORMAL                          0x0000          // 系统运行正常
#define CONTROLLER_DATE_RETURN_FALSE        0x0001          // 消息请求返回FALSE
#define INIT_MODE_ERR                       0x0002          // 机械臂未初始化或输入型号非法
#define INIT_TIME_ERR                       0x0003          // 非法超时时间
#define INIT_SOCKET_ERR                     0x0004          // Socket 初始化失败
#define SOCKET_CONNECT_ERR                  0x0005          // Socket 连接失败
#define SOCKET_SEND_ERR                     0x0006          // Socket 发送失败
#define SOCKET_TIME_OUT                     0x0007          // Socket 通讯超时
#define UNKNOWN_ERR                         0x0008          // 未知错误
#define CONTROLLER_DATA_LOSE_ERR            0x0009          // 数据不完整
#define CONTROLLER_DATE_ARR_NUM_ERR         0x000A          // 数组长度错误
#define WRONG_DATA_TYPE                     0x000B          // 数据类型错误
#define MODEL_TYPE_ERR                      0x000C          // 型号错误
#define CALLBACK_NOT_FIND                   0x000D          // 缺少回调函数
#define ARM_ABNORMAL_STOP                   0x000E          // 机械臂异常停止
#define TRAJECTORY_FILE_LENGTH_ERR          0x000F          // 轨迹文件名称过长
#define TRAJECTORY_FILE_CHECK_ERR           0x0010          // 轨迹文件校验失败
#define TRAJECTORY_FILE_READ_ERR            0x0011          // 轨迹文件读取失败
#define CONTROLLER_BUSY                     0x0012          // 控制器忙,请稍后再试
#define ILLEGAL_INPUT                       0x0013          // 非法输入
#define QUEUE_LENGTH_FULL                   0x0014          // 数据队列已满
#define CALCULATION_FAILED                  0x0015          // 计算失败
#define FILE_OPEN_ERR                       0x0016          // 文件打开失败
#define FORCE_AUTO_STOP                     0x0017          // 力控标定手动停止
#define DRAG_TEACH_FLAG_FALSE               0x0018          // 没有可保存轨迹
#define LISTENER_RUNNING_ERR                0x0019          // UDP监听接口运行报错

// 回调函数对应Code
#define MOVEJ_CANFD_CB                      0x0001          // 角度透传非阻塞标识码
#define MOVEP_CANFD_CB                      0x0002          // 位姿透传非阻塞标识码
#define FORCE_POSITION_MOVE_CB              0x0003          // 力位混合透传

#ifdef __cplusplus
}
#endif
#endif // RM_DEFINE_H
