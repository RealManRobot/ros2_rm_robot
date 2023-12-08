#ifndef RM_PARSER_DATA_H
#define RM_PARSER_DATA_H
//////////////////////////////////////////////////////////////////////////////////
//睿尔曼智能科技有限公司        Author:Dong Qinpeng
//创建日期:2022/08/23
//版本：V4.0
//版权所有，盗版必究。
//Copyright(C) 睿尔曼智能科技有限公司
//All rights reserved
//文档说明：该文档定义了内部数据解析函数
//////////////////////////////////////////////////////////////////////////////////
#include "rm_define.h"

#ifdef __cplusplus
extern "C" {
#endif

// ================================== 接口校验Key get_calibrate_state
#define     GET_JOINT_MAX_SPEED             0x0001
#define     SET_JOINT_MAX_SPEED             0x0002
#define     SET_JOINT_MAX_ACC               0x0003
#define     SET_JOINT_MIN_POS               0x0004
#define     SET_JOINT_MAX_POS               0x0005
#define     SET_JOINT_EN_STATE              0x0006
#define     SET_JOINT_ZERO_POS              0x0007
#define     SET_JOINT_CLEAR_ERR             0x0008
#define     START_CALIBRATE                 0x0009
#define     STOP_CALIBRATE                  0x000A
#define     GET_CALIBRATE_STATE             0x000B
#define     GET_JOINT_MAX_ACC               0x000C
#define     GET_JOINT_MIN_POS               0x000D
#define     GET_JOINT_MAX_POS               0x000E
#define     GET_JOINT_EN_STATE              0x000F
#define     GET_JOINT_ERR_FLAG              0x0010
#define     GET_JOINT_SOFTWARE_VERSION      0x0011
#define     GET_JOINT_SOFTWARE_VERSION1     0x0012
#define     SET_ARM_MAX_LINE_SPEED          0x0013
#define     SET_ARM_MAX_LINE_ACC            0x0014
#define     SET_ARM_MAX_ANGULAR_SPEED       0x0015
#define     SET_ARM_MAX_ANGULAR_ACC         0x0016
#define     GET_ARM_MAX_LINE_SPEED          0x0017
#define     GET_ARM_MAX_LINE_ACC            0x0018
#define     GET_ARM_MAX_ANGULAR_SPEED       0x0019
#define     GET_ARM_MAX_ANGULAR_ACC         0x001A
#define     SET_ARM_INIT                    0x001B
#define     SET_COLLISION_STAGE             0x001C
#define     GET_COLLISION_STAGE             0x001D
#define     SET_DH_DATA                     0x001E
#define     GET_DH_DATA                     0x001F
#define     SET_JOINT_ZERO_OFFSET           0x0020
#define     SET_ARM_DYNAMIC_PARM            0x0021
#define     GET_TOOL_SOFTWARE_VERSION       0x0022
#define     SET_ARM_SERVO                   0x0023
#define     SET_SYSTEM_STATE_SERVO          0x0024
#define     SET_AUTO_TOOL_FRAME             0x0025
#define     GENERATE_AUTO_TOOL_FRAME        0x0026
#define     SET_MANUAL_TOOL_FRAME           0x0027
#define     SET_CHANGE_TOOL_FRAME           0x0028
#define     SET_DELETE_TOOL_FRAME           0x0029
#define     SET_PAYLOAD                     0x002A
#define     SET_NONE_PAYLOAD                0x002B
#define     GET_CURRENT_TOOL_FRAME          0x002C
#define     GET_TOOL_FRAME                  0x002D
#define     GET_TOTAL_TOOL_FRAME            0x002E
#define     SET_AUTO_WORK_FRAME             0x002F
#define     SET_MANUAL_WORK_FRAME           0x0030
#define     SET_CHANGE_WORK_FRAME           0x0031
#define     SET_DELETE_WORK_FRAME           0x0032
#define     GET_CURRENT_WORK_FRAME          0x0033
#define     GET_WORK_FRAME                  0x0034
#define     GET_TOTAL_WORK_FRAME            0x0035
#define     GET_CURRENT_ARM_STATE           0x0036
#define     GET_CURRENT_JOINT_TEMPERATURE   0x0037
#define     GET_CURRENT_JOINT_CURRENT       0x0038
#define     GET_CURRENT_JOINT_VOLTAGE       0x0039
#define     GET_JOINT_DEGREE                0x003A
#define     GET_ARM_ALL_STATE               0x003B
#define     GET_ARM_PLAN_NUM                0x003C
#define     SET_INIT_POSE                   0x003D
#define     GET_INIT_POSE                   0x003E
#define     SET_INSTALL_POSE                0x003F
#define     MOVEJ                           0x0040
#define     MOVEJ_CANFD                     0x0041
#define     MOVEP_CANFD                     0x0042
#define     SET_ARM_STOP                    0x0043
#define     SET_ARM_PAUSE                   0x0044
#define     SET_ARM_CONTONUE                0x0045
#define     SET_DELETE_CURRENT_TRAJECTORY   0x0046
#define     SET_ARM_DELETE_TRAJECTORY       0x0047
#define     GET_ARM_CURRENT_TRAJECTORY      0x0048
#define     ADD_WAYPOINT                    0x0049
#define     START_WAYPOINT                  0x004A
#define     CLEAR_WAYPOINT                  0x004B
#define     STOP_WAYPOINT                   0x004C
#define     TIMER                           0x004D
#define     SET_JOINT_TEACH                 0x004E
#define     SET_POS_TEACH                   0x004F
#define     SET_ORT_TEACH                   0x0050
#define     SET_STOP_TEACH                  0x0051
#define     SET_JOINT_STEP                  0x0052
#define     SET_POS_STEP                    0x0053
#define     SET_ORT_STEP                    0x0054
#define     GET_CONTROLLER_STATE            0x0055
#define     SET_ARM_POWER                   0x0056
#define     GET_ARM_POWER_STATE             0x0057
#define     CLEAR_SYSTEM_ERR                0x0058
#define     GET_ARM_HARDWARE_VERSION        0x0059
#define     GET_ARM_SOFTWARE_VERSION        0x005A
#define     GET_LOG_FILE                    0x005B
#define     GET_SYSTEM_RUNTIME              0x005C
#define     CLEAR_SYSTEM_RUNTIME            0x005D
#define     GET_JOINT_ODOM                  0x005E
#define     CLEAR_JOINT_ODOM                0x005F
#define     SET_HEIGHT_SPEED_ETH            0x0060
#define     SET_DO_STATE                    0x0061
#define     SET_AO_STATE                    0x0062
#define     GET_IO_INPUT                    0x0063
#define     GET_DO_STATE                    0x0064
#define     GET_DI_STATE                    0x0065
#define     GET_AO_STATE                    0x0066
#define     GET_AI_STATE                    0x0067
#define     GET_IO_STATE_NC                 0x0068
#define     GET_IO_OUTPUT                   0x0069
#define     SET_TOOL_DO_STATE               0x006A
#define     SET_TOOL_IO_STATE               0x006B
#define     GET_TOOL_IO_STATE               0x006C
#define     SET_TOOL_VOLTAGE                0x006D
#define     GET_TOOL_VOLTAGE                0x006E
#define     SET_GRIPPER_ROUTE               0x006F
#define     SET_GRIPPER_RELEASE             0x0070
#define     START_DRAG_TEACH                0x0071
#define     STOP_DRAG_TEACH                 0x0072
#define     RUN_DRAG_TRAJECTORY             0x0073
#define     PAUSE_DRAG_TRAJECTORY           0x0074
#define     CONTINUE_DRAG_TRAJECTORY        0x0075
#define     STOP_DRAG_TRAJECTORY            0x0076
#define     DRAG_TRAJECTORY_ORIGIN          0x0077
#define     START_MULTI_DRAG_TEACH          0x0078
#define     SET_FORCE_POSTION               0x0079
#define     STOP_FORCE_POSTION              0x007A
#define     GET_FORCE_DATA                  0x007B
#define     CLEAR_FORCE_DATA                0x007C
#define     SET_FORCE_SENSOR                0x007D
#define     STOP_SET_FORCE_SENSOR           0x007E
#define     SET_HAND_POSTURE                0x007F
#define     SET_HAND_SEQ                    0x0080
#define     SET_HAND_ANGLE                  0x0081
#define     SET_HAND_SPEED                  0x0082
#define     SET_HAND_FORCE                  0x0083
#define     SET_PWM                         0x0084
#define     GET_FZ                          0x0085
#define     CLEAR_FZ                        0x0086
#define     AUTO_SET_FZ                     0x0087
#define     MANUAL_SET_FZ                   0x0088
#define     SET_MODBUS_MODE                 0x0089
#define     CLOSE_MODBUS_MODE               0x008A
#define     READ_COILS                      0x008B
#define     READ_INPUT_STATE                0x008C
#define     READ_HOLDING_REGISTERS          0x008D
#define     READ_INPUT_REGISTERS            0x008E
#define     WRITE_SINGLE_COIL               0x008F
#define     WRITE_COILS                     0x0090
#define     WRITE_SIGNALE_REGISTER          0x0091
#define     WRITE_REGISTERS                 0x0092
#define     SET_VEHICLE                     0x0093
#define     SET_LIFT                        0x0094
#define     SET_LIFT_SPEED                  0x0095
#define     SET_LIFT_HEIGHT                 0x0096
#define     GET_LIFT_STATE                  0x0097
#define     SET_CAR_MOVE                    0x0098
#define     SET_CAR_SPEED                   0x0099
#define     SET_CAR_NAVIGATION_SPEED        0x009A
#define     SET_CAR_NAVIGATION_MODE         0x009B
#define     GET_CAR_STATE                   0x009C
#define     SET_CAR_CHARGE                  0x009D
#define     SET_CAR_STATION_NUM             0x009E
#define     GET_JOINT_DEGREE1               0x009F
#define     START_FORCE_POSITION_MOVE       0x00A0
#define     FORCE_POSITION_MOVE             0x00A1
#define     STOP_FORCE_POSITION_MOVE        0x00A2
#define     RUN_PROJECT                     0x00A3
#define     DOWNLOAD_PROJECT                0x00A4
#define     SET_TEACH_FRAME                 0x00A5
#define     FORCE_POSITION_MOVE_ERR         0x00A6
#define     GET_CONTROLLER_VOLTAGE_NC       0x00A7
#define     SET_ARM_RUM_MODE_NC             0x00A8
#define     GET_ARM_RUM_MODE_NC             0x00A8
#define     SET_IO_MODE_NC                  0x00A9
#define     SET_IO_STATE_NC                 0x00AA
#define     SET_CONTROLLER_VOLTAGE_NC       0x00AB
#define     SET_CONTROLLER_IP_NC            0x00AC
#define     SAVE_TRAJECTORY                 0x00AD
#define     GET_INSTALL_POSE                0x00AE
#define     AUTO_SET_JOINT_LIMIT            0x00AF
#define     SET_POPUP                       0x00B1
#define     SET_ONLINE_START                0x00B2
#define     SET_LIFT_VERSION                0x00B3
#define     SET_SALE_VERSION                0x00B4
#define     TOOL_HARDWARE_VERSION           0x00B5

// 清单长度
#define     MENU_LIST_LENGTH                2000

// ****************************************** 解析函数 ******************************************
// 解析关节设置零位
int Parser_Joint_Zero_Pos_E(char *msg);
// 解析设置关节最大速度
int Parser_Joint_Speed_E(char *msg);
// 解析最大加速度
int Parser_Joint_Acc_E(char *msg);
// 解析设置关节最小限位
int Parser_Joint_Min_Pos_E(char *msg);
// 解析设置关节最大限位
int Parser_Joint_Max_Pos_E(char *msg);
// 解析设置关节使能状态
int Parser_Joint_EN_State_E(char *msg);
// 解析清除关节错误信息
int Parser_Joint_Err_Clear_E(char *msg);
// 解析结束标定
int Parser_Start_Calibrate_E(char *msg);
// 解析删除工作坐标系
int Parser_Delete_Work_Frame_E(char *msg);
// 解析设置末端速度
int Parser_Arm_Line_Speed_E(char *msg);
// 解析设置末端加速度
int Parser_Arm_Line_Acc_E(char *msg);
//解析设置末端角速度
int Parser_Arm_Angular_Speed_E(char *msg);
//解析设置末端角加速度
int Parser_Arm_Angular_Acc_E(char *msg);
// 解析获取末端最大速度
int Parser_Get_Arm_Line_Speed_E(char *msg, float *speed);
// 解析获取末端最大加速度
int Parser_Get_Arm_Line_Acc_E(char *msg, float *acc);
// 解析获取末端角速度
int Parser_Get_Arm_Angular_Speed_E(char *msg, float *speed);
// 解析获取末端角加速度
int Parser_Get_Arm_Angular_Acc_E(char *msg, float *acc);
//解析初始化机械臂末端参数返回值
int Parser_Arm_Tip_Init_E(char *msg);
//解析设置碰撞检测等级
int Parser_Set_Collision_Stage_E(char *msg);
//解析获取碰撞检测等级
int Parser_Get_Collision_Stage_E(int* stage,char *msg);
//解析设置DH参数
int Parser_Set_DH_Data_E(char *msg);
//解析获取DH参数
int Parser_Get_DH_Data_E(char *msg,float* lsb, float* lse, float* lew, float* lwt, float* d3);
//解析设置关节零位补偿
int Parser_Set_Joint_Zero_Offset_E(char *msg);
//解析目标手势
int Parser_Set_Hand_E(char *msg);
//解析查询末端接口板软件版本号
int Parser_Get_Tool_Software_Version_E(int* version,char *msg);
//解析控制器伺服返回值
int Parser_Arm_Servo_State_E(char *msg);
//解析设置系统状态自动回传
int Parser_Set_System_State_Servo_E(char *msg);
//解析自动标定工具坐标系
int Parser_Auto_Set_Tool_Frame_E(char *msg);
//解析手动标定工具坐标系
int Parser_Manual_Set_Tool_Frame_E(char *msg);
//解析切换工具坐标系指令
int Parser_Change_Tool_Frame_E(char *msg);
//解析删除工具坐标系指令
int Parser_Delete_Tool_Frame_E(char *msg);
//解析获取当前工具坐标系
int Parser_Get_Current_Tool_Frame_E(char *msg, FRAME *tool);
//解析指定工具坐标系
int Parser_Get_Given_Tool_Frame_E(char *msg, FRAME *tool);
//解析获取所有工具坐标系的名称
int Parser_Get_All_Tool_Frame_E(char *msg, FRAME_NAME *names, int *len);
//解析自动标定工作坐标系
int Parser_Auto_Set_Work_Frame_E(char *msg);
//解析手动标定工作坐标系
int Parser_Manual_Set_Work_Frame_E(char *msg);
//解析切换工作坐标系指令
int Parser_Change_Work_Frame_E(char *msg);
//解析获取当前工作坐标系
int Parser_Get_Current_Work_Frame_E(char *msg, FRAME *frame);
//解析指定工作坐标系
int Parser_Get_Given_Work_Frame_E(char *msg, Pose *pose);
//解析获取所有工作坐标系的名称
int Parser_Get_All_Work_Frame_E(char *msg, FRAME_NAME *names, int *len);
//解析机械臂当前角度
int Paser_Get_Joint_Degreee_E(char *msg, float *joint);
//解析查询规划计数
int Parser_Get_Arm_Plan_Num_E(int* plan,char* msg);
//解析设置机械臂初始位置返回
int Parser_Set_Arm_Init_Pose_E(char *msg);
// 解析规划是否成功
int Parser_Plan_Feedback_E(char *msg);
// 解析急停反馈指令
int Parser_Move_Stop_E(char *msg);
//解析暂停反馈指令
int Parser_Move_Pause_E(char *msg);
//解析继续反馈指令
int Parser_Move_Continue_E(char *msg);
//解析清除当前轨迹指令
int Parser_Clear_Current_Trajectory_E(char *msg);
//解析清除控制器所有轨迹指令
int Parser_Clear_All_Trajectory_E(char *msg);
//解析添加路径点
int Parser_Add_Waypoint_E(char *msg);
//解析设置手爪松开
int Parser_Set_Gripper_E(char *msg);
// 解析轨迹等待
int Paser_Timer_E(char *msg);
//解析关节示教返回值
int Parser_Joint_Teach_E(char *msg);
//解析位置示教返回值
int Parser_Pos_Teach_E(char *msg);
//解析姿态示教返回值
int Parser_Ort_Teach_E(char *msg);
//解析停止示教返回值
int Parser_Teach_Stop_E(char *msg);
//解析控制器状态
int Parser_Controller_State_E(char *msg, float *voltage, float *current, float *temperature, uint16_t *sys_err);
//解析设置机械臂电源
int Parser_Set_Arm_Power_E(char *msg);
//解析读取机械臂电源状态
int Parser_Get_Arm_Power_State_E(int* power,char* msg);
//解析清除错误代码
int Parser_Clear_System_Err_E(char *msg);
//解析机械臂硬件版本
int Parser_Get_Arm_Hardware_Version_E(char* version,char* msg);
//解析机械臂软件版本
int Parser_Get_Arm_Software_Version_E(int *plan_version, int *ctrl_version, int *kernal1, int *kernal2, char* msg);
//解析读取控制器的累计运行时间
int Parser_Get_System_Runtime_E(char* state,int* day,int* hour,int* min,int* sec,char *msg);
//解析关节的累计转动角度
int Parser_Get_Joint_Odom_E(char* state,float* odom,char *msg);
//解析设置高速网口
int Parser_Set_High_Speed_Eth_E(char *msg);
//解析弹窗继续
int Parser_Set_Popup_E(char *msg);
//解析数字IO输出设置返回值
int Parser_Set_DO_State_E(char *msg);
//解析模拟IO输出设置返回值
int Parser_Set_AO_State_E(char *msg);
//解析指定通道数字IO输出状态
int Parser_Get_DO_State_E(char *msg, byte *state);
//解析指定通道数字IO输入状态
int Parser_Get_DI_State_E(char *msg, byte *state);
//解析指定通道模拟IO输出状态
int Parser_Get_AO_State_E(char *msg, byte *voltage);
//解析指定通道模拟IO输入状态
int Parser_Get_AI_State_E(char *msg, byte *voltage);
//解析获取新控制器IO状态
int Parser_Get_IO_State_NC_E(char *msg, int *io_mode, int *io_state);
//解析所有IO输入通道的状态
int Parser_IO_Input_E(char *msg, byte *DI_state, float *AI_voltage);
//解析所有IO输出通道的状态
int Parser_IO_Output_E(char *msg, byte *DO_state, float *AO_voltage);
//解析工具端数字IO输出设置返回值
int Parser_Set_Tool_DO_State_E(char *msg);
//解析工具端数字IO输出设置返回值
int Parser_Set_Tool_DO_State_2_E(char *msg);
//解析查询末端工具电源输出
int Parser_Get_Tool_IO_State_E(float* IO_Mode, float* IO_State,char *msg);
//设置工具端电源输出返回值解析
int Parser_Set_Tool_Voltage_E(char *msg);
//解析查询末端工具电源输出
int Parser_Get_Tool_Voltage_E(char *msg, byte *voltage);
//解析设置手爪行程
int Parser_Set_Gripper_Route_E(char *msg);
//解析拖动示教指令
int Parser_Drag_Teach_E(char *msg);
//解析开始轨迹复现
int Parser_Run_Drag_Trajectory_E(char *msg);
//解析暂停轨迹复现
int Parser_Pause_Drag_Trajectory_E(char *msg);
//解析继续轨迹复现
int Parser_Continue_Drag_Trajectory_E(char *msg);
//解析停止轨迹复现
int Parser_Stop_Drag_Trajectory_E(char *msg);
//运动到轨迹复现起点
int Parser_Drag_Trajectory_Origin_E(char *msg);
//获取六维力数据
int Parser_Get_Force_Data_E(char *msg, float *Force);
//解析清空六维力数据
int Parser_Clear_Force_Data_E(char* msg);
//解析设置六维力重心
int Parser_Set_Force_Sensor_E(char* msg);
//解析停止六维力重心
int Parser_Stop_Force_Sensor_E(char* msg);
//解析查询末端一维力数据
int Parser_Get_Fz_E(char* msg,float *Fz);
//解析读线圈
int Parser_Get_read_coils_E(char* msg, int *colis_data);
//解析写单圈数据
int Parser_Write_single_coil_E(char* msg);
//解析获取升降机构状态
int Parser_Get_Lift_Speed_E(int* height,int* current,int* err_flag,char *msg);
//解析站点导航控制
int Parser_Set_Car_Station_E(char *msg);
//解析获取升降机构状态
int Parser_Get_Car_State_E(int* volume,int* voltage,int* current,int* car_err,int* car_state,int* station,char *msg);
//停止力位混合透传
//int Parser_Force_Position_Move(char* msg);
//解析文件预下发返回
int Parser_RunProject_E(char* msg);
//解析文件下发返回
int Parser_SendFile_E(char* msg);
//控制器准备升级接口
int Parser_Run_UpdateSoftware_E(char* msg);
// 解析示教运动切换坐标系
int Parser_Set_TEACH_FRAME_E(char * msg);
// 解析设置机械臂运行模式
int Parser_Set_Arm_Run_Mode_NC_E(char * msg);
// 解析获取机械臂运行模式
int Parser_Get_Arm_Run_Mode_NC_E(char * msg, int *mode);
//解析数字IO输出设置返回值
int Parser_Set_Net_State_E(char *msg);

//解析安装方式
int Parser_Get_Install_Pose_E(char *msg, float * x, float * y, float * z);
// 解析升级进度
int Parser_Get_Update_scale_E(char *msg, int *scale);
// 解析弹窗信息
int Parser_Get_Popup_E(char * msg, int * pop_num);
// 解析设置升降机版本
int Parser_Set_Lift_Version_E(char * msg);
// 解析设置销售版本
int Parser_Set_Sale_Version_E(char * msg);
#ifdef __cplusplus
}
#endif
#endif // RM_PARSER_DATA_H
