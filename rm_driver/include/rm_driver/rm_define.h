#ifndef RM_DEFINE_H
#define RM_DEFINE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

#define ARM_DOF                     7
#define M_PI		                3.14159265358979323846

#define RM_MOVE_NBLOCK  0                                          ///<机械臂运动设置，非阻塞模式
#define RM_MOVE_MULTI_BLOCK  1                                 ///<机械臂运动设置，多线程阻塞模式
static inline int RM_MOVE_SINGLE_BLOCK(int timeout){return timeout;}    ///<机械臂运动设置，单线程阻塞模式超时时间

/**
 * @brief 线程模式
 * @ingroup Init_Class
 */
typedef enum {
    RM_SINGLE_MODE_E,       ///< 单线程模式，单线程非阻塞等待数据返回
    RM_DUAL_MODE_E,         ///< 双线程模式，增加接收线程监测队列中的数据
    RM_TRIPLE_MODE_E,       ///< 三线程模式，在双线程模式基础上增加线程监测UDP接口数据
}rm_thread_mode_e;

/**
 * @brief 机械臂型号
 * @ingroup Algo
 */
typedef enum{
    RM_MODEL_RM_65_E,       ///< RM_65
    RM_MODEL_RM_75_E,       ///< RM_75
    RM_MODEL_RM_63_I_E,     ///< RML_63I(已弃用)
    RM_MODEL_RM_63_II_E,        ///< RML_63II
    RM_MODEL_RM_63_III_E,       ///< RML_63III
    RM_MODEL_ECO_65_E,      ///< ECO_65
    RM_MODEL_ECO_62_E,      ///< ECO_62
    RM_MODEL_GEN_72_E,       ///< GEN_72
    RM_MODEL_ECO_63_E,       ///< ECO63
    RM_MODEL_UNIVERSAL_E
}rm_robot_arm_model_e;

/**
 * @brief 机械臂末端力传感器版本
 * @ingroup Algo
 */
typedef enum{
    RM_MODEL_RM_B_E,    ///< 标准版
    RM_MODEL_RM_ZF_E,   ///< 一维力版
    RM_MODEL_RM_SF_E,   ///< 六维力版
    RM_MODEL_RM_ISF_E,   ///< 一体化六维力版
}rm_force_type_e;

/**
 * @brief 事件类型
 * @ingroup Init_Class
 */
typedef enum 
{
    RM_NONE_EVENT_E,                    ///< 无事件
    RM_CURRENT_TRAJECTORY_STATE_E,      ///< 当前轨迹到位
    RM_PROGRAM_RUN_FINISH_E,            ///< 在线编程运行结束
} rm_event_type_e;

/**
 * @brief 事件信息
 * @ingroup Init_Class
 */
typedef struct 
{
    int handle_id;       ///< 返回消息的机械臂id
    rm_event_type_e event_type;     ///< 事件类型，包含无事件、当前轨迹到位、在线编程运行结束
    bool trajectory_state;      ///< 当前轨迹到位状态
    int device;     ///< 到位设备，0：关节 1：夹爪 2：灵巧手 3：升降机构 4：扩展关节 其他：保留
    int trajectory_connect;     ///< 是否连接下一条轨迹，0：全部到位，1：连接下一条轨迹
    int program_id;     ///< 运行结束的在线编程程序id
}rm_event_push_data_t;

/**
 * @brief 机械臂当前规划类型
 * 
 */
typedef enum 
{
    RM_NO_PLANNING_E,                   ///< 无规划
    RM_JOINT_SPACE_PLANNING_E,          ///< 关节空间规划
    RM_CARTESIAN_LINEAR_PLANNING_E,     ///< 笛卡尔空间直线规划
    RM_CARTESIAN_ARC_PLANNING_E,        ///< 笛卡尔空间圆弧规划
    RM_SPLINE_CURVE_MOTION_PLANNING_E,  ///< 样条曲线运动规划
    RM_TRAJECTORY_REPLAY_PLANNING_E,    ///< 示教轨迹复现规划
}rm_arm_current_trajectory_e;

typedef struct
{
    int joint_speed;   ///< 关节速度。
    int lift_state;    ///< 升降关节信息。1：上报；0：关闭上报；-1：不设置，保持之前的状态
    int expand_state;  ///< 扩展关节信息（升降关节和扩展关节为二选一，优先显示升降关节）1：上报；0：关闭上报；-1：不设置，保持之前的状态
    int hand_state;          ///< 灵巧手状态。1：上报；0：关闭上报；-1：不设置，保持之前的状态(1.7.0版本无这个)
    int arm_current_status;     ///< 机械臂当前状态。1：上报；0：关闭上报；-1：不设置，保持之前的状态
    int aloha_state;     ///< aloha主臂状态是否上报。1：上报；0：关闭上报；-1：不设置，保持之前的状态
    int plus_base;   ///< 末端设备基础信息。1：上报；0：关闭上报；-1：不设置，保持之前的状态
    int plus_state;  ///< 末端设备实时信息。1：上报；0：关闭上报；-1：不设置，保持之前的状态
}rm_udp_custom_config_t;

/**
 * @brief 机械臂主动上报接口配置
 * @ingroup UdpConfig
 */
typedef struct {
    int cycle;      ///< 广播周期，5ms的倍数
    bool enable;     ///< 使能，是否主动上报
    int port;       ///< 广播的端口号
    int force_coordinate;       ///< 系统外受力数据的坐标系，-1不支持力传感器 0为传感器坐标系 1为当前工作坐标系 2为当前工具坐标系
    char ip[28];       ///< 自定义的上报目标IP地址
    rm_udp_custom_config_t custom_config;     ///< 自定义项内容
} rm_realtime_push_config_t;

/**
 * @brief 四元数
 * 
 */
typedef struct
{
	float w;
	float x;
	float y;
	float z;
} rm_quat_t;

/**
 * @brief 位置坐标
 * 
 */
typedef struct
{
	float x; //* unit: m
	float y;
	float z;
} rm_position_t;

/**
 * @brief 欧拉角
 * 
 */
typedef struct
{
	float rx; //* unit: rad
	float ry;
	float rz;
} rm_euler_t;

/**
 * @brief 机械臂位置姿态结构体
 * @ingroup Algo
 */
typedef struct 
{
    rm_position_t position;     ///< 位置，单位：m
    rm_quat_t quaternion;       ///< 四元数
    rm_euler_t euler;           ///< 欧拉角，单位：rad
}rm_pose_t;

/**
 * @brief 坐标系名称
 * 不超过10个字符
 * @ingroup ToolCoordinateConfig
 * @ingroup WorkCoordinateConfig
 */
typedef struct 
{
    char name[12];
}rm_frame_name_t;


/**
 * @brief 坐标系
 * @ingroup Algo
 * @ingroup ToolCoordinateConfig
 * @ingroup WorkCoordinateConfig
 */
typedef struct 
{
    char frame_name[12];    ///< 坐标系名称
    rm_pose_t pose;         ///< 坐标系位姿
    float payload;      ///< 坐标系末端负载重量，单位：kg
    float x;    ///< 坐标系末端负载质心位置，单位：m
    float y;    ///< 坐标系末端负载质心位置，单位：m
    float z;    ///< 坐标系末端负载质心位置，单位：m
}rm_frame_t;

typedef struct{
    char build_time[20];    ///< 编译时间
    char version[20];       ///< 版本号
}rm_ctrl_version_t;

typedef struct{
    char model_version[5];      ///< 动力学模型版本号
}rm_dynamic_version_t;

typedef struct{
    char build_time[20];    ///<编译时间
    char version[20];       ///< 版本号
}rm_planinfo_t;

typedef struct {
    char version[20];   ///< 算法库版本号
}rm_algorithm_version_t;

typedef struct {
    char build_time[20];    ///<编译时间
    char version[20];       ///< 版本号
}rm_software_build_info_t;
/**
 * @brief  机械臂软件信息
 * 
 */
typedef struct
{
    char product_version[20];           ///< 机械臂型号
    char robot_controller_version[10];  ///< 机械臂控制器版本，若为四代控制器，则该字段为"4.0"
    rm_algorithm_version_t algorithm_info;      ///< 算法库信息
    rm_software_build_info_t ctrl_info;        ///< ctrl 层软件信息
    rm_dynamic_version_t dynamic_info;      ///< 动力学版本（三代）
    rm_software_build_info_t plan_info;        ///< plan 层软件信息（三代）
    rm_software_build_info_t com_info;        ///< communication 模块软件信息（四代）
    rm_software_build_info_t program_info;        ///< 流程图编程模块软件信息（四代）
}rm_arm_software_version_t;

/**
 * @brief  错误代码结构体
 * 
*/
typedef struct 
{
    uint8_t err_len;   ///< 错误代码个数
    int err[24];       ///< 错误代码数组
}rm_err_t;

/**
 * @brief  机械臂当前状态
 * 
*/
typedef struct 
{
    rm_pose_t pose;         ///< 机械臂当前位姿
    float joint[ARM_DOF];   ///< 机械臂当前关节角度
    rm_err_t err;
}rm_current_arm_state_t;

/**
 * @brief  机械臂关节状态参数
 * 
*/
typedef struct
{
    float joint_current[ARM_DOF];               ///< 关节电流，单位mA，精度：0.001mA
    bool joint_en_flag[ARM_DOF];                ///< 当前关节使能状态 ，1为上使能，0为掉使能
    uint16_t joint_err_code[ARM_DOF];           ///< 当前关节错误码
    float joint_position[ARM_DOF];              ///< 关节角度，单位°，精度：0.001°
    float joint_temperature[ARM_DOF];           ///< 当前关节温度，精度0.001℃
    float joint_voltage[ARM_DOF];               ///< 当前关节电压，精度0.001V
    float joint_speed[ARM_DOF];                 ///< 当前关节速度，精度0.01RPM。
}rm_joint_status_t;

/**
 * @brief 位置示教方向
 * 
 */
typedef enum
{
    RM_X_DIR_E,        ///< 位置示教，x轴方向
    RM_Y_DIR_E,        ///< 位置示教，y轴方向
    RM_Z_DIR_E,        ///< 位置示教，z轴方向
}rm_pos_teach_type_e;

/**
 * @brief 姿态示教方向
 * 
 */
typedef enum
{
    RM_RX_ROTATE_E,        ///< 姿态示教，绕x轴旋转
    RM_RY_ROTATE_E,        ///< 姿态示教，绕y轴旋转
    RM_RZ_ROTATE_E,        ///< 姿态示教，绕z轴旋转
}rm_ort_teach_type_e;

/**
 * @brief 数字IO配置结构体
 *      io_mode:模式，0-通用输入模式、1-通用输出模式、2-输入开始功能复用模式、3-输入暂停功能复用模式、
 *                   4-输入继续功能复用模式、5-输入急停功能复用模式、6-输入进入电流环拖动复用模式
 *                   7-输入进入力只动位置拖动模式（六维力版本可配置）、8-输入进入力只动姿态拖动模式（六维力版本可配置）
 *                   9-输入进入力位姿结合拖动复用模式（六维力版本可配置）、10-输入外部轴最大软限位复用模式（外部轴模式可配置）
 *                   11-输入外部轴最小软限位复用模式（外部轴模式可配置）、12-输入初始位姿功能复用模式
 *                   13-输出碰撞功能复用模式、14-实时调速功能复用模式
 *      io_state:数字io状态（0低 1高）（该成员在set时无效）
 *      io_real_time_config_t:实时调速功能，io配置
 *          speed:速度取值范围0-100     (当io_mode不为14时，默认值为-1)
 *          mode :模式取值范围1或2      (当io_mode不为14时，默认值为-1)
 *                  1表示单次触发模式，单次触发模式下当IO拉低速度设置为speed参数值，IO恢复高电平速度设置为初始值
 *                  2表示连续触发模式，连续触发模式下IO拉低速度设置为speed参数值，IO恢复高电平速度维持当前值
 */
typedef struct
{
    int io_mode;    // io_mode:模式0~14
    struct
    {
        int speed;  // speed:速度取值范围0-100
        int mode;   // mode :模式取值范围1或2
    }io_real_time_config_t;
}rm_io_config_t;

/**
 * @brief 数字IO状态获取结构体
 *      io_state:数字io状态（0低 1高）（该成员在set时无效）
 *      io_config:io配置结构体
 */
typedef struct
{
    int io_state;               // io_state:数字io状态（0低 1高）
    rm_io_config_t io_config;   // io_config:数字io配置结构体
}rm_io_get_t;

/**
 * @brief 复合模式拖动示教参数
 * 
 */
typedef struct{
    int free_axes[6];       ///< 自由驱动方向[x,y,z,rx,ry,rz]，0-在参考坐标系对应方向轴上不可拖动，1-在参考坐标系对应方向轴上可拖动
    int frame;              ///< 参考坐标系，0-工作坐标系 1-工具坐标系。
    int singular_wall;      ///< 仅在六维力模式拖动示教中生效，用于指定是否开启拖动奇异墙，0表示关闭拖动奇异墙，1表示开启拖动奇异墙，若无配置参数，默认启动拖动奇异墙
}rm_multi_drag_teach_t;

/**
 * @brief 力位混合控制传感器枚举
 * 
 */
typedef enum{
    RM_FP_OF_SENSOR_E = 0,     ///<一维力
    RM_FP_SF_SENSOR_E,         ///<六维力
}rm_force_position_sensor_e;

/**
 * @brief 力位混合控制模式枚举
 * 
 */
typedef enum{
    RM_FP_BASE_COORDINATE_E = 0,   ///<基坐标系力控
    RM_FP_TOOL_COORDINATE_E,       ///<工具坐标系力控
}rm_force_position_mode_e;

/**
 * @brief 力位混合控制模式（单方向）力控方向枚举
 * 
 */
typedef enum{
    RM_FP_X_E = 0,      ///<沿X轴
    RM_FP_Y_E,          ///<沿Y轴
    RM_FP_Z_E,          ///<沿Z轴
    RM_FP_RX_E,         ///<沿RX姿态方向
    RM_FP_RY_E,         ///<沿RY姿态方向
    RM_FP_RZ_E,         ///<沿RZ姿态方向
}rm_force_position_dir_e;

/**
 * @brief 力位混合控制参数
 * 
 */
typedef struct 
{
    int sensor;            ///< 传感器，0-一维力；1-六维力
    int mode;              ///< 0-基坐标系力控；1-工具坐标系力控；
    int control_mode[6];       ///<  6个力控方向（Fx Fy Fz Mx My Mz）的模式 0-固定模式 1-浮动模式 2-弹簧模式 3-运动模 4-力跟踪模式 8-力跟踪+姿态自适应模式
    float desired_force[6];     ///< 力控轴维持的期望力/力矩，力控轴的力控模式为力跟踪模式时，期望力/力矩设置才会生效 ，精度0.1N。
    float limit_vel[6];     ///< 力控轴的最大线速度和最大角速度限制，只对开启力控方向生效。
}rm_force_position_t;

/**
 * @brief 透传力位混合补偿参数
 * 建议初始化方式，避免一些未知错误
 * rm_force_position_move_t my_fp_move = (rm_force_position_move_t){ 0 };
 */
typedef struct 
{
    int flag;                   ///< 0-下发目标角度，1-下发目标位姿
    rm_pose_t pose;             ///< 当前坐标系下的目标位姿，支持四元数/欧拉角表示姿态。位置精度：0.001mm，欧拉角表示姿态，姿态精度：0.001rad，四元数方式表示姿态，姿态精度：0.000001
    float joint[ARM_DOF];       ///< 目标关节角度，单位：°，精度：0.001°
    int sensor;                 ///< 传感器，0-一维力；1-六维力
    int mode;                   ///< 0-基坐标系力控；1-工具坐标系力控；
    bool follow;                ///< 表示驱动器的运动跟随效果，true 为高跟随，false 为低跟随。
    int control_mode[6];        ///< 6个力控方向的模式 0-固定模式 1-浮动模式 2-弹簧模式 3-运动模式 4-力跟踪模式 5-浮动+运动模式 6-弹簧+运动模式 7-力跟踪+运动模式 8-姿态自适应模式
    float desired_force[6];     ///< 力控轴维持的期望力/力矩，力控轴的力控模式为力跟踪模式时，期望力/力矩设置才会生效 ，精度0.1N。
    float limit_vel[6];         ///< 力控轴的最大线速度和最大角速度限制，只对开启力控方向生效。
    int trajectory_mode;        ///< 高跟随模式下，0-完全透传模式、1-曲线拟合模式、2-滤波模式
    int radio;                  ///< 曲线拟合模式0-100和滤波模式下的平滑系数（数值越大效果越好），滤波模式下取值范围0~1000，曲线拟合模式下取值范围0~100
}rm_force_position_move_t;

/**
 * @brief 角度透传模式结构体
 * 建议初始化方式，避免一些未知错误
 * rm_movej_canfd_mode_t my_j_canfd = (rm_movej_canfd_mode_t){ 0 };
 */
typedef struct
{
    float* joint;           // 关节角度（若为六轴机械臂，那么最后一个元素无效），单位°
    float expand;           // 扩展关节角度（若没有扩展关节，那么此成员值无效）
    bool follow;            // 跟随模式，0-低跟随，1-高跟随,若使用高跟随，透传周期要求不超过 10ms
    int trajectory_mode;    // 高跟随模式下，0-完全透传模式、1-曲线拟合模式、2-滤波模式
    int radio;              // 曲线拟合模式和滤波模式下的平滑系数（数值越大效果越好），滤波模式下取值范围0~1000，曲线拟合模式下取值范围0~100
}rm_movej_canfd_mode_t;

/**
 * @brief 姿态透传模式结构体
 * 建议初始化方式，避免一些未知错误
 * rm_movep_canfd_mode_t my_p_canfd = (rm_movep_canfd_mode_t){ 0 };
 */
typedef struct
{
    rm_pose_t pose;         // 位姿 (优先采用四元数表达)
    bool follow;            // 跟随模式，0-低跟随，1-高跟随,若使用高跟随，透传周期要求不超过 10ms
    int trajectory_mode;    // 高跟随模式下，0-完全透传模式、1-曲线拟合模式、2-滤波模式
    int radio;              // 曲线拟合模式和滤波模式下的平滑系数（数值越大效果越好），滤波模式下取值范围0~1000，曲线拟合模式下取值范围0~100
}rm_movep_canfd_mode_t;

/**
 * @brief 无线网络信息结构体
 * 
 */
typedef struct{
    int channel;               ///< 如果是 AP 模式，则存在此字段，标识 wifi 热点的物理信道号
    char ip[16];               ///< IP 地址
    char mac[18];              ///< MAC 地址
    char mask[16];             ///< 子网掩码
    char mode[5];              ///< ap 代表热点模式，sta 代表联网模式，off 代表未开启无线模式
    char password[16];         ///< 密码
    char ssid[32];             ///< 网络名称 (SSID)
}rm_wifi_net_t;

/**
 * @brief  机械臂所有状态参数
 * 
*/
typedef struct
{
    float joint_current[ARM_DOF];           ///< 关节电流，单位mA
    int joint_en_flag[ARM_DOF];             ///< 关节使能状态
    float joint_temperature[ARM_DOF];       ///< 关节温度,单位℃
    float joint_voltage[ARM_DOF];           ///< 关节电压，单位V
    int joint_err_code[ARM_DOF];            ///< 关节错误码
    rm_err_t err;                           ///< 错误代码
}rm_arm_all_state_t;

/**
 * @brief 夹爪状态
 * 
 */
typedef struct
{
    int enable_state;      ///< 夹爪使能标志，0 表示未使能，1 表示使能
    int status;            ///< 夹爪在线状态，0 表示离线， 1表示在线
    int error;              ///< 夹爪错误信息，低8位表示夹爪内部的错误信息bit5-7 保留bit4 内部通bit3 驱动器bit2 过流 bit1 过温bit0 堵转
    int mode;               ///< 当前工作状态：1 夹爪张开到最大且空闲，2 夹爪闭合到最小且空闲，3 夹爪停止且空闲，4 夹爪正在闭合，5 夹爪正在张开，6 夹爪闭合过程中遇到力控停止
    int current_force;      ///< 夹爪当前的压力，单位g
    int temperature;        ///< 当前温度，单位℃
    int actpos;             ///< 夹爪开口度
}rm_gripper_state_t;

/**
 * @brief  六维力传感器数据结构体
 * 
*/
typedef struct {
    float force_data[6];         ///< 当前力传感器原始数据，力的单位为N；力矩单位为Nm。
    float zero_force_data[6];        ///< 当前力传感器系统外受力数据，力的单位为N；力矩单位为Nm。
    float work_zero_force_data[6];     ///< 当前工作坐标系下系统外受力原始数据，力的单位为N；力矩单位为Nm。
    float tool_zero_force_data[6];     ///< 当前工具坐标系下系统外受力原始数据，力的单位为N；力矩单位为Nm。
} rm_force_data_t;

/**
 * @brief  一维力传感器数据结构体
 * 
*/
typedef struct {
    float Fz;         ///< 原始数据
    float zero_Fz;        ///< 传感器坐标系下系统外受力数据，力的单位为N；力矩单位为Nm。
    float work_zero_Fz;     ///< 当前工作坐标系下系统外受力原始数据，力的单位为N；力矩单位为Nm。
    float tool_zero_Fz;     ///< 当前工具坐标系下系统外受力原始数据，力的单位为N；力矩单位为Nm。
} rm_fz_data_t;

/**
 * @brief  外设数据读写参数结构体
 * 
*/
typedef struct {
    int port;       ///< 通讯端口，0-控制器RS485端口，1-末端接口板RS485接口，3-控制器ModbusTCP设备
    int address;    ///< 数据起始地址
    int device;     ///< 外设设备地址
    int num;        ///< 要读的数据的数量
} rm_peripheral_read_write_params_t;

/**
 * @brief  升降机构、扩展关节状态结构体
 * 
*/
typedef struct {
    int pos;        ///< 扩展关节角度，单位度，精度 0.001°(若为升降机构高度，则单位：mm，精度：1mm，范围：0 ~2300)
    int current;        ///< 驱动电流，单位：mA，精度：1mA
    int err_flag;       ///< 驱动错误代码，错误代码类型参考关节错误代码
    int mode;       ///< 当前状态，0-空闲，1-正方向速度运动，2-正方向位置运动，3-负方向速度运动，4-负方向位置运动
} rm_expand_state_t;

/**
 * @brief 文件下发
 * @ingroup OnlineProgramming
 */
typedef struct {
    char project_path[300];      ///< 下发文件路径文件名
    int project_path_len;   ///< 名称长度
    int plan_speed;     ///< 规划速度比例系数
    int only_save;      ///< 0-保存并运行文件，1-仅保存文件，不运行
    int save_id;        ///< 保存到控制器中的编号
    int step_flag;      ///< 设置单步运行方式模式，1-设置单步模式 0-设置正常运动模式
    int auto_start;     ///< 设置默认在线编程文件，1-设置默认  0-设置非默认
    int project_type;   ///< 下发文件类型。0-在线编程文件，1-拖动示教轨迹文件
    // int err_line;       ///< 若运行失败，该参数返回有问题的工程行数，err_line 为 0，则代表校验数据长度不对
} rm_send_project_t;

/**
 * @brief 在线编程存储信息
 * @ingroup OnlineProgramming
 */
typedef struct  {
    int id;     ///< 在线编程文件id
    int size;   ///< 文件大小
    int speed;  ///< 默认运行速度
    char trajectory_name[32];   ///< 文件名称
}rm_trajectory_data_t;
/**
 * @brief 查询在线编程列表
 * @ingroup OnlineProgramming
 */
typedef struct
{
    int page_num;       // 页码
    int page_size;       // 每页大小
    int list_size;   //返回总数量
    char vague_search[32];  // 模糊搜索
    rm_trajectory_data_t trajectory_list[100];   // 符合的在线编程列表
}rm_program_trajectorys_t;

/**
 * @brief 在线编程运行状态
 * @ingroup OnlineProgramming
 */
typedef struct
{
    int run_state;  ///< 运行状态 0 未开始 1运行中 2暂停中
    int id;         ///< 运行轨迹编号
    int edit_id;    ///< 上次编辑的在线编程编号 id
    int plan_num;   ///< 运行行数
    int total_loop;     ///< 循环指令数量
    int step_mode;      ///< 单步模式，1 为单步模式，0 为非单步模式
    int plan_speed;     ///< 全局规划速度比例 1-100
    int loop_num[100];        ///< 循环行数
    int loop_cont[100];       ///< 对应循环次数
}rm_program_run_state_t;

/**
 * @brief 流程图程序运行状态
 */
typedef struct
{
    int run_state;  ///< 运行状态 0 未开始 1运行中 2暂停中
    int id;         ///< 当前使能的文件id。
    char name[32];  ///< 当前使能的文件名称。
    int plan_speed;     ///< 当前使能的文件全局规划速度比例 1-100。
    int step_mode;    ///< 单步模式，0为空，1为正常, 2为单步。
    char modal_id[50];   ///< 运行到的流程图块的id。未运行则不返回
}rm_flowchart_run_state_t;

/**
 * @brief 全局路点存储信息
 * @ingroup OnlineProgramming
 */
typedef struct
{
    char point_name[20];    ///< 路点名称
    float joint[ARM_DOF];   ///< 关节角度
    rm_pose_t pose;     ///< 位姿信息
    char work_frame[12];    ///< 工作坐标系名称
    char tool_frame[12];    ///< 工具坐标系名称
    char time[50];      ///<  路点新增或修改时间
}rm_waypoint_t;
/**
 * @brief 全局路点列表
 * @ingroup OnlineProgramming
 */
typedef struct{
    int page_num;       ///< 页码
    int page_size;      ///< 每页大小
    int total_size;     ///< 列表长度
    char vague_search[32];  ///< 模糊搜索 
    int list_len;       ///<返回符合的全局路点列表长度
    rm_waypoint_t points_list[100];   ///< 返回符合的全局路点列表
}rm_waypoint_list_t;

/**
 * @brief 几何模型长方体参数
 * @ingroup Electronic_Fence
 */
typedef struct{
    float x_min_limit;      ///< 长方体基于世界坐标系 X 方向最小位置，单位 m
    float x_max_limit;      ///< 长方体基于世界坐标系 X 方向最大位置，单位 m
    float y_min_limit;      ///< 长方体基于世界坐标系 Y 方向最小位置，单位 m
    float y_max_limit;      ///< 长方体基于世界坐标系 Y 方向最大位置，单位 m
    float z_min_limit;      ///< 长方体基于世界坐标系 Z 方向最小位置，单位 m
    float z_max_limit;      ///< 长方体基于世界坐标系 Z 方向最大位置，单位 m
}rm_fence_config_cube_t;
/**
 * @brief 几何模型点面矢量平面参数
 * @ingroup Electronic_Fence
 */
typedef struct{
    float x1, y1, z1;       ///< 点面矢量平面三点法中的第一个点坐标，单位 m
    float x2, y2, z2;       ///< 点面矢量平面三点法中的第二个点坐标，单位 m
    float x3, y3, z3;       ///< 点面矢量平面三点法中的第三个点坐标，单位 m
}rm_fence_config_plane_t;
/**
 * @brief 几何模型球体参数
 * @ingroup Electronic_Fence
 */
typedef struct{
    float x;        ///< 表示球心在世界坐标系 X 轴的坐标，单位 m
    float y;        ///< 表示球心在世界坐标系 Y 轴的坐标，单位 m
    float z;        ///< 表示球心在世界坐标系 Z 轴的坐标，单位 m
    float radius;       ///< 表示半径，单位 m
}rm_fence_config_sphere_t;
/**
 * @brief 几何模型参数
 * @ingroup Electronic_Fence
 */
typedef struct{
    int form;       ///< 形状，1 表示长方体，2 表示点面矢量平面，3 表示球体
    char name[12];  ///< 电子围栏名称，不超过 10 个字节，支持字母、数字、下划线
    rm_fence_config_cube_t cube;    ///< 长方体参数
    rm_fence_config_plane_t plan;    ///< 点面矢量平面参数
    rm_fence_config_sphere_t sphere;    ///< 球体参数
}rm_fence_config_t;

/**
 * @brief 几何模型名称结构体
 * @ingroup Electronic_Fence
 */
typedef struct
{
    char name[12];    ///< 几何模型名称,不超过10个字符
}rm_fence_names_t;

/**
 * @brief 几何模型参数列表
 * @ingroup Electronic_Fence
 */
typedef struct
{
    rm_fence_config_t config[10];    
}rm_fence_config_list_t;
/**
 * @brief 包络球参数
 * 
 */
typedef struct{
    char name[12];            ///< 工具包络球体的名称，1-10 个字节，支持字母数字下划线
	float radius;           ///< 工具包络球体的半径，单位 m
    float x;        ///< 工具包络球体球心基于末端法兰坐标系的 X 轴坐标，单位 m
    float y;        ///< 工具包络球体球心基于末端法兰坐标系的 Y 轴坐标，单位 m
    float z;        ///< 工具包络球体球心基于末端法兰坐标系的 Z 轴坐标，单位 m
}rm_envelopes_ball_t;
/**
 * @brief 包络球参数集合
 * 
 */
typedef struct{
	rm_envelopes_ball_t balls[5];///< 包络参数列表，每个工具最多支持 5 个包络球，可以没有包络
    int size;   ///< 包络球数量
    char tool_name[12];///< 控制器中已存在的工具坐标系名称，如果不存在该字段，则为临时设置当前包络参数
}rm_envelope_balls_list_t;

/**
 * @brief 电子围栏/虚拟墙使能状态参数
 * 
 */
typedef struct
{
    bool enable_state;  ///< 电子围栏/虚拟墙使能状态，true 代表使能，false 代表禁使能
    int in_out_side;    ///< 0-机器人在电子围栏/虚拟墙内部，1-机器人在电子围栏外部
    int effective_region;   ///< 0-电子围栏针对整臂区域生效，1-虚拟墙针对末端生效
}rm_electronic_fence_enable_t;


/**
 * @brief  （UDP主动上报机械臂信息）力传感器数据结构体
 * 
*/
typedef struct {
    float force[6];         ///< 当前力传感器原始数据，0.001N或0.001Nm
    float zero_force[6];        ///< 当前力传感器系统外受力数据，0.001N或0.001Nm
    int coordinate;         ///< 系统外受力数据的坐标系，0为传感器坐标系 1为当前工作坐标系 2为当前工具坐标系
} rm_force_sensor_t;

/***
 * 扩展关节数据
 *
 */
typedef struct {
    float pos;            ///< 当前角度  精度 0.001°，单位：°
    int current;        ///< 当前驱动电流，单位：mA，精度：1mA
    int err_flag;       ///< 驱动错误代码，错误代码类型参考关节错误代码
    int en_flag;        ///< 当前关节使能状态 ，1 为上使能，0 为掉使能
    int joint_id;       ///< 关节id号
    int mode;           ///< 当前升降状态，0-空闲，1-正方向速度运动，2-正方向位置运动，3-负方向速度运动，4-负方向位置运动
} rm_udp_expand_state_t;

/***
 * 升降机构状态
 *
 */
typedef struct {
    int height;         ///< 当前升降机构高度，单位：mm，精度：1mm
    float pos;            ///< 当前角度  精度 0.001°，单位：°
    int current;        ///< 当前驱动电流，单位：mA，精度：1mA
    int err_flag;       ///< 驱动错误代码，错误代码类型参考关节错误代码
    int en_flag;        ///< 当前关节使能状态 ，1 为上使能，0 为掉使能
} rm_udp_lift_state_t;
/***
 * 灵巧手状态
 *
 */
typedef struct {
    int hand_pos[6];         ///< 表示灵巧手位置
    int hand_angle[6];         ///< 表示灵巧手角度
    int hand_force[6];            ///< 表示灵巧手自由度力，单位mN
    int hand_state[6];        ///< 表示灵巧手自由度状态，由灵巧手厂商定义状态含义
    int hand_err;       ///< 表示灵巧手系统错误，由灵巧手厂商定义错误含义，例如因时状态码如下：1表示有错误，0表示无错误
} rm_udp_hand_state_t;

/***
 * 
 * 轨迹连接配置
 */
typedef enum{
    RM_TRAJECTORY_DISCONNECT_E = 0,   ///<立即规划并执行轨迹，不连接后续轨迹
    RM_TRAJECTORY_CONNECT_E           ///<将当前轨迹与下一条轨迹一起规划
}rm_trajectory_connect_config_e;

/**
 * @brief 机械臂当前状态
 * 
 */
typedef enum {
    RM_IDLE_E,                     // 使能但空闲状态
    RM_MOVE_L_E,                   // move L运动中状态
    RM_MOVE_J_E,                   // move J运动中状态
    RM_MOVE_C_E,                   // move C运动中状态
    RM_MOVE_S_E,                   // move S运动中状态
    RM_MOVE_THROUGH_JOINT_E,       // 角度透传状态
    RM_MOVE_THROUGH_POSE_E,        // 位姿透传状态
    RM_MOVE_THROUGH_FORCE_POSE_E,  // 力控透传状态
    RM_MOVE_THROUGH_CURRENT_E,     // 电流环透传状态
    RM_STOP_E,                     // 急停状态
    RM_SLOW_STOP_E,                // 缓停状态
    RM_PAUSE_E,                    // 暂停状态
    RM_CURRENT_DRAG_E,             // 电流环拖动状态
    RM_SENSOR_DRAG_E,              // 六维力拖动状态
    RM_TECH_DEMONSTRATION_E        // 示教状态
} rm_udp_arm_current_status_e;

/***
 * aloha主臂状态
 *
 */
typedef struct {
    int io1_state;         ///<  IO1状态（手柄光电检测），0为按键未触发，1为按键触发。
    int io2_state;        ///<  IO2状态（手柄光电检测），0为按键未触发，1为按键触发。
} rm_udp_aloha_state_t;

/**
 * 末端设备基础信息(末端生态协议支持)
*/
typedef struct{
    char manu[10];              // 设备厂家
    int type;               // 设备类型 1：两指夹爪 2：五指灵巧手 3：三指夹爪
    char hv[10];   // 硬件版本
    char sv[10];   // 软件版本
    char bv[10];       // boot版本
    int id;                 // 设备ID
    int dof;                // 自由度
    int check;              // 自检开关
    int bee;                // 蜂鸣器开关
    bool force;             // 力控支持
    bool touch;             // 触觉支持
    int touch_num;          // 触觉个数
    int touch_sw;       // 触觉开关
    int hand;               // 手方向 1 ：左手 2： 右手
    int pos_up[12];         // 位置上限,单位：无量纲
    int pos_low[12];        // 位置下限,单位：无量纲
    int angle_up[12];        // 角度上限,单位：0.01度
    int angle_low[12];       // 角度下限,单位：0.01度
    int speed_up[12];        // 速度上限,单位：无量纲
    int speed_low[12];       // 速度下限,单位：无量纲
    int force_up[12];        // 力上限,单位：0.001N 
    int force_low[12];       // 力下限,单位：0.001N 
} rm_plus_base_info_t;
// 单位：无量纲
/**
 * 末端设备实时信息(末端生态协议支持)
*/
typedef struct{
    int sys_state;      // 系统状态:0正常1设备故障
    int dof_state[12];   // 各自由度当前状态:0正在松开1正在闭合2位置到位停止3力控到位停止4触觉到位停止5电流保护停止6发生故障
    int dof_err[12];     // 各自由度错误信息
    int pos[12];       // 各自由度当前位置,单位：无量纲
    int speed[12];  //各自由度当前速度,闭合正，松开负，单位：无量纲
    int angle[12];     // 各自由度当前角度，单位：0.01度
    int current[12];   // 各自由度当前电流，单位：mA
    int normal_force[18];         // 自由度触觉三维力的法向力,1-6自由度触觉三维力的法向力*3
    int tangential_force[18];     // 自由度触觉三维力的切向力
    int tangential_force_dir[18]; // 自由度触觉三维力的切向力方向
    uint32_t tsa[12];         // 自由度触觉自接近
    uint32_t tma[12];         // 自由度触觉互接近
    int touch_data[18];    // 触觉传感器原始数据(示例中有，但未显示数据的JSON情况)
    int force[12]; //自由度力矩,闭合正，松开负，单位0.001N
} rm_plus_state_info_t;


/**
 * @brief  udp主动上报机械臂信息
 * 
*/
typedef struct 
{
    int errCode;                        ///< 数据解析错误码，-3为数据解析错误，代表推送的数据不完整或格式不正确
    char arm_ip[16];                    ///< 推送数据的机械臂的IP地址
    rm_joint_status_t joint_status;     ///< 关节状态
    rm_force_sensor_t force_sensor;     ///< 力数据（六维力或一维力版本支持）
    rm_err_t err;                       ///< 错误码
    rm_pose_t waypoint;                 ///< 当前路点信息
    rm_udp_lift_state_t liftState;      ///< 升降关节数据
    rm_udp_expand_state_t expandState;  ///< 扩展关节数据
    rm_udp_hand_state_t handState;      ///< 灵巧手数据
    rm_udp_arm_current_status_e arm_current_status;     ///< 机械臂状态
    rm_udp_aloha_state_t aloha_state;   ///< aloha主臂状态
    int rm_plus_state;                  ///< 末端设备状态，0-设备在线，1-表示协议未开启，2-表示协议开启但是设备不在线
    rm_plus_base_info_t plus_base_info;   ///< 末端设备基础信息
    rm_plus_state_info_t plus_state_info; ///< 末端设备实时信息
}rm_realtime_arm_joint_state_t; 

/**
 * @brief 逆解参数
 * @ingroup Algo
 */
typedef struct {
    float q_in[ARM_DOF];    ///< 上一时刻关节角度，单位°
    rm_pose_t q_pose;      ///< 目标位姿
    uint8_t flag;           ///< 姿态参数类别：0-四元数；1-欧拉角
} rm_inverse_kinematics_params_t;

typedef struct {
    int result;  // 0：成功，1：逆解失败，-1：上一时刻关节角度输入为空或超关节限位，-2：目标位姿四元数不合法， -3：当前机器人非六自由度，当前仅支持六自由度机器人
    int  num;                        // number of solutions
    float q_ref[8];          // 参考关节角度，通常是当前关节角度, 单位 °
    float q_solve[8][8];     // 关节角全解, 单位 °
} rm_inverse_kinematics_all_solve_t;

/**
 * @brief 包络球描述数据结构
*/
typedef struct
{
    float radius;     // 球体半径（单位：m）
    float centrePoint[3]; // 球体中心位置（单位：m，以法兰坐标系为参考坐标系）
} rm_tool_sphere_t;     // 工具包络球参数


/**
 * @brief 旋转矩阵
 * @ingroup Algo
 */
typedef struct
{
    short irow;
    short iline;
    float data[4][4];
} rm_matrix_t;
/**
 * @brief 机械臂事件回调函数
 * @ingroup Init_Class
 */
typedef void (*rm_event_callback_ptr)(rm_event_push_data_t data);
/**
 * @brief UDP机械臂状态主动上报回调函数
 * @ingroup Init_Class
 */
typedef void (*rm_realtime_arm_state_callback_ptr)(rm_realtime_arm_joint_state_t data);

/**
 * @brief 机械臂基本信息
 * @ingroup Init_Class
 */
typedef struct
{
    uint8_t arm_dof;    ///< 机械臂自由度
    rm_robot_arm_model_e arm_model;              ///< 机械臂型号
    rm_force_type_e force_type;                  ///< 末端力传感器版本
    uint8_t robot_controller_version;                      ///< 机械臂控制器版本，4：四代控制器，3：三代控制器。
}rm_robot_info_t;

/**
 * @brief 机械臂控制句柄
 * @ingroup Init_Class
 */
typedef struct {
    int id;         ///< 句柄id，连接成功id大于0，连接失败返回-1
}rm_robot_handle;


typedef struct
{
    float d[8];     //* unit: m
    float a[8];     //* unit: m
    float alpha[8]; //* unit: °
    float offset[8];    //* unit: °
} rm_dh_t;

/**
 * @brief 版本号结构体
 * 不超过10个字符
 * @ingroup ToolCoordinateConfig
 * @ingroup WorkCoordinateConfig
 */
typedef struct {
    char version[10];
} rm_version_t;

/**
 * @brief 轨迹信息结构体
 */
typedef struct {
    int point_num;           ///< 轨迹点数量
    char name[20];      ///< 轨迹名称	
    char create_time[20];    ///< 创建时间
}rm_trajectory_info_t;
/**
 * @brief 轨迹列表结构体
 * @ingroup OnlineProgramming
 */
typedef struct{
    int page_num;       ///< 页码
    int page_size;      ///< 每页大小
    int total_size;     ///< 列表长度
    char vague_search[32];  ///< 模糊搜索 
    int list_len;       ///<返回符合的轨迹列表长度
    rm_trajectory_info_t tra_list[100];   ///< 返回符合的轨迹列表
}rm_trajectory_list_t;
/**
 * @brief Modbus TCP主站信息结构体
 */
typedef struct {
    char master_name[20]; // Modbus 主站名称，最大长度15个字符，不超过15个字符
    char ip[16];          // TCP主站 IP 地址
    int port;             // TCP主站端口号	
}rm_modbus_tcp_master_info_t;
/**
 * @brief Modbus TCP主站列表结构体
 */
typedef struct{
    int page_num;       ///< 页码
    int page_size;      ///< 每页大小
    int total_size;     ///< 列表长度
    char vague_search[32];  ///< 模糊搜索	
    int list_len;       ///<返回符合的TCP主站列表长度
    rm_modbus_tcp_master_info_t master_list[100];   ///< 返回符合的TCP主站列表
}rm_modbus_tcp_master_list_t;

/**
 * @brief Modbus RTU读数据参数结构体
 */
typedef struct {
    int address;    ///< 数据起始地址
    int device;     ///< 外设设备地址	
    int type;       ///< 0-控制器端modbus主机；1-工具端modbus主机。
    int num;        ///< 要读的数据的数量，数据长度不超过109
}rm_modbus_rtu_read_params_t;
/**
 * @brief Modbus RTU写数据结构体
 */
typedef struct {
    int address;    ///< 数据起始地址
    int device;     ///< 外设设备地址
    int type;       ///< 0-控制器端modbus主机；1-工具端modbus主机。
    int num;        ///< 要写的数据的数量，最大不超过100
    int data[120];  ///< 要写的数据，数据长度不超过100
}rm_modbus_rtu_write_params_t;

/**
 * @brief Modbus TCP读数据参数结构体
 */
typedef struct {
    int address;          // 数据起始地址
    char master_name[20]; // Modbus 主站名称，最大长度15个字符，不超过15个字符（master_name与IP二选一，若有IP和port优先使用IP和port）
    char ip[16];          // 主机连接的 IP 地址（master_name与IP二选一，若有IP和port优先使用IP和port）
    int port;             // 主机连接的端口号
    int num;              // 读取数据数量，最大不超过100
}rm_modbus_tcp_read_params_t;
/**
 * @brief Modbus TCP写数据结构体
 */
typedef struct {
    int address;          // 数据起始地址
    char master_name[20]; // Modbus 主站名称，最大长度15个字符，不超过15个字符（master_name与IP二选一，若有IP和port优先使用IP和port）
    char ip[16];          // 主机连接的 IP 地址（master_name与IP二选一，若有IP和port优先使用IP和port）
    int port;             // 主机连接的端口号
    int num;              // 写入数据数量，最大不超过100
    int data[120];        // 写入的数据，数据长度不超过100
}rm_modbus_tcp_write_params_t;

#ifdef __cplusplus
}
#endif

#endif