#ifndef RM_SERVICE_H
#define RM_SERVICE_H
  
#ifdef __cplusplus  
extern "C" {
#endif  
  
#include "rm_interface.h"  
#include "rm_interface_global.h"
  
#ifdef __cplusplus  
}  
  
class RM_Service {  
public:  
/**  
 * @defgroup Init_Class 初始化及连接
 * 
 * 此模块为API及机械臂初始化相关接口，包含API版本号查询、API初始化、连接/断开机械臂、日志设置、
 * 机械臂仿真/真实模式设置、机械臂信息获取、运动到位信息及机械臂实时状态信息回调函数注册等
 * @{  
 */
/**
 * @brief 查询sdk版本号
 * 
 * @code
 * char *version = rm_api_version();
 * printf("api version: %s\n", version);
 * @endcode
 * @return char* 返回版本号
 */
RM_INTERFACE_EXPORT char* rm_api_version(void);
/**
 * @brief 初始化线程模式
 * 
 * @param mode RM_SINGLE_MODE_E：单线程模式，单线程非阻塞等待数据返回；  
 *             RM_DUAL_MODE_E：双线程模式，增加接收线程监测队列中的数据；  
 *             RM_TRIPLE_MODE_E：三线程模式，在双线程模式基础上增加线程监测UDP接口数据；
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - -1: 创建线程失败。查看日志以获取具体错误
 * @see rm_create_robot_arm
 */
RM_INTERFACE_EXPORT int rm_init(rm_thread_mode_e mode);

/**
 * @brief 关闭所有连接，销毁所有线程
 * 
 * @return int 函数执行的状态码。  
            - 0: 成功。  
 */
RM_INTERFACE_EXPORT int rm_destory(void);

/**
 * @brief 日志打印配置
 * 
 * @param LogCallback 日志打印回调函数
 * @param level 日志打印等级，0：debug级别，1：info级别，2：warn：级别，3：error级别  
 */
 RM_INTERFACE_EXPORT void rm_set_log_call_back(void (*LogCallback)(const char* message, va_list args),int level);

/**
 * @brief 保存日志到文件
 * 
 * @param path 日志保存文件路径
 */
RM_INTERFACE_EXPORT void rm_set_log_save(const char* path);

/**
 * @brief 设置全局超时时间
 * 
 * @param timeout 接收控制器返回指令超时时间，多数接口默认超时时间为500ms，单位ms
 */
RM_INTERFACE_EXPORT void rm_set_timeout(int timeout);

/**
 * @brief 创建一个机械臂，用于实现对该机械臂的控制
 * 
 * @param ip 机械臂的ip地址
 * @param port 机械臂的端口号
 * @return rm_robot_handle* 创建成功后，返回机械臂控制句柄，达到最大连接数5或者连接失败返回空
 * @see rm_init
 */
RM_INTERFACE_EXPORT rm_robot_handle *rm_create_robot_arm(const char *ip,int port);

/**
 * @brief 手动设置机械臂自由度
 * 
 * @param handle 机械臂控制句柄
 * @param dof 机械臂自由度
 * @return int 函数执行的状态码。
            - 0: 成功。
            - -1: 未找到对应句柄,句柄为空或已被删除。
            - -2: 设置失败，自由度设置不合理（负数或者大于10）。
 */
RM_INTERFACE_EXPORT int rm_set_robot_dof(rm_robot_handle *handle, int dof);
/**
 * @brief 根据句柄删除机械臂
 * 
 * @param handle 需要删除的机械臂句柄
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - -1: 未找到对应句柄,句柄为空或已被删除。
 */
RM_INTERFACE_EXPORT int rm_delete_robot_arm(rm_robot_handle *handle);
/**
 * @brief 机械臂仿真/真实模式设置
 * 
 * @param handle 机械臂控制句柄
 * @param mode 模式 0:仿真 1:真实
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_arm_run_mode(rm_robot_handle *handle, int mode);
/**
 * @brief 机械臂仿真/真实模式获取
 * 
 * @param handle 机械臂控制句柄
 * @param mode 模式 0:仿真 1:真实
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_arm_run_mode(rm_robot_handle *handle,int *mode);
/**
 * @brief 获取机械臂基本信息
 * 
 * @param handle 机械臂控制句柄
 * @param robot_info 机械臂基本信息结构体
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - -1: 未找到对应句柄,句柄为空或已被删除。
            - -2: 获取到的机械臂基本信息非法，检查句柄是否已被删除。  
 */
RM_INTERFACE_EXPORT int rm_get_robot_info(rm_robot_handle *handle, rm_robot_info_t *robot_info);
/**
 * @brief 机械臂事件回调注册
 * 
 * @param handle 机械臂控制句柄
 * @param event_callback 机械臂事件回调函数
 */
RM_INTERFACE_EXPORT void rm_get_arm_event_call_back(rm_event_callback_ptr event_callback);
/**
 * @brief UDP机械臂状态主动上报信息回调注册
 * 
 * @param handle 机械臂控制句柄
 * @param realtime_callback 机械臂状态信息回调函数
 */
RM_INTERFACE_EXPORT void rm_realtime_arm_state_call_back(rm_realtime_arm_state_callback_ptr realtime_callback);
/** @} */ // 结束初始化组的定义
/**  
 * @defgroup Joint_Config 关节配置
 * 
 * 对机械臂的关节参数进行设置，如果关节发生错误，则无法修改关节参数，必须先清除关节错误代码。另外设置关节之前，
 * 必须先将关节掉使能，否则会设置不成功。  
 * 关节所有参数在修改完成后，会自动保存到关节 Flash，立即生效，之后关节处于掉使能状态，修改完参数后必须
 * 发送指令控制关节上使能。  
 * @attention 睿尔曼机械臂在出厂前所有参数都已经配置到最佳状态，一般不建议用户修改关节的底层参数。若用户确需修改，首先
 * 应使机械臂处于非使能状态，然后再发送修改参数指令，参数设置成功后，发送关节恢复使能指令。需要注意的是，关节恢复
 * 使能时，用户需要保证关节处于静止状态，以免上使能过程中关节发生报错。关节正常上使能后，用户方可控制关节运动。
 * @{  
 */
/**
 * @brief 设置关节最大速度
 * 
 * @param handle 机械臂句柄
 * @param joint_num 关节序号
 * @param max_speed 关节最大速度，单位：°/s
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。  
 */
RM_INTERFACE_EXPORT int rm_set_joint_max_speed(rm_robot_handle *handle,int joint_num,float max_speed);
/**
 * @brief 设置关节最大加速度
 * 
 * @param handle 机械臂句柄
 * @param joint_num 关节序号
 * @param max_acc 关节最大加速度，单位：°/s²
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。  
 */
RM_INTERFACE_EXPORT int rm_set_joint_max_acc(rm_robot_handle *handle,int joint_num,float max_acc);
/**
 * @brief 设置关节最小限位
 * 
 * @param handle 机械臂句柄
 * @param joint_num 关节序号
 * @param min_pos 关节最小位置，单位：°
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。  
 */
RM_INTERFACE_EXPORT int rm_set_joint_min_pos(rm_robot_handle *handle,int joint_num,float min_pos);
/**
 * @brief 设置关节最大限位
 * 
 * @param handle 机械臂句柄
 * @param joint_num 关节序号
 * @param max_pos 关节最大位置，单位：°
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。  
 */
RM_INTERFACE_EXPORT int rm_set_joint_max_pos(rm_robot_handle *handle,int joint_num,float max_pos);
/**
 * @brief 设置关节最大速度(驱动器)
 * 
 * @param handle 机械臂句柄
 * @param joint_num 关节序号
 * @param max_speed 关节最大速度，单位：°/s
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。  
 */
RM_INTERFACE_EXPORT int rm_set_joint_drive_max_speed(rm_robot_handle *handle,int joint_num,float max_speed);
/**
 * @brief 设置关节最大加速度(驱动器)
 * 
 * @param handle 机械臂句柄
 * @param joint_num 关节序号
 * @param max_acc 关节最大加速度，单位：°/s² 
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。  
 */
RM_INTERFACE_EXPORT int rm_set_joint_drive_max_acc(rm_robot_handle *handle,int joint_num,float max_acc);
/**
 * @brief 设置关节最小限位(驱动器)
 * 
 * @param handle 机械臂句柄
 * @param joint_num 关节序号
 * @param min_pos 关节最小位置
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。  
 */
RM_INTERFACE_EXPORT int rm_set_joint_drive_min_pos(rm_robot_handle *handle,int joint_num,float min_pos);
/**
 * @brief 设置关节最大限位(驱动器)
 * 
 * @param handle 机械臂句柄
 * @param joint_num 关节序号
 * @param max_pos 关节最大位置
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。  
 */
RM_INTERFACE_EXPORT int rm_set_joint_drive_max_pos(rm_robot_handle *handle,int joint_num,float max_pos);
/**
 * @brief 设置关节使能状态
 * 
 * @param handle 机械臂句柄
 * @param joint_num 关节序号
 * @param en_state 1：上使能 0：掉使能
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。   
 */
RM_INTERFACE_EXPORT int rm_set_joint_en_state(rm_robot_handle *handle,int joint_num,int en_state);
/**
 * @brief 设置关节零位
 * 
 * @param handle 机械臂句柄
 * @param joint_num 关节序号
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。  
 */
RM_INTERFACE_EXPORT int rm_set_joint_zero_pos(rm_robot_handle *handle,int joint_num);
/**
 * @brief 清除关节错误代码
 * 
 * @param handle 机械臂句柄
 * @param joint_num 关节序号
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。  
 */
RM_INTERFACE_EXPORT int rm_set_joint_clear_err(rm_robot_handle *handle,int joint_num);
/**
 * @brief 一键设置关节限位
 * 
 * @param handle 机械臂句柄
 * @param limit_mode 1-正式模式，各关节限位为规格参数中的软限位和硬件限位
 * @return int 函数执行的状态码。   
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。  
 */
RM_INTERFACE_EXPORT int rm_auto_set_joint_limit(rm_robot_handle *handle,int limit_mode);
/**
 * @brief 查询关节最大速度
 * 
 * @param handle 机械臂句柄
 * @param max_speed 关节1~7转速数组，单位：°/s
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_joint_max_speed(rm_robot_handle *handle,float *max_speed);
/**
 * @brief 查询关节最大加速度
 * 
 * @param handle 机械臂句柄
 * @param max_acc 关节1~7加速度数组，单位：°/s
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_joint_max_acc(rm_robot_handle *handle,float *max_acc);
/**
 * @brief 查询关节最小限位
 * 
 * @param handle 机械臂句柄
 * @param min_pos 关节1~7最小位置数组，单位：°
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_joint_min_pos(rm_robot_handle *handle,float *min_pos);
/**
 * @brief 查询关节最大限位
 * 
 * @param handle 机械臂句柄
 * @param max_pos 关节1~7最大位置数组，单位：°
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_joint_max_pos(rm_robot_handle *handle,float *max_pos);
/**
 * @brief 查询关节(驱动器)最大速度
 * 
 * @param handle 机械臂句柄
 * @param max_speed 关节1~7转速数组，单位：°/s
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_joint_drive_max_speed(rm_robot_handle *handle,float *max_speed);
/**
 * @brief 查询关节(驱动器)最大加速度
 * 
 * @param handle 机械臂句柄
 * @param max_acc 关节1~7加速度数组，单位：°/s
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_joint_drive_max_acc(rm_robot_handle *handle,float *max_acc);
/**
 * @brief 查询关节(驱动器)最小限位
 * 
 * @param handle 机械臂句柄
 * @param min_pos 关节1~7最小位置数组，单位：°
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_joint_drive_min_pos(rm_robot_handle *handle,float *min_pos);
/**
 * @brief 查询关节(驱动器)最大限位
 * 
 * @param handle 机械臂句柄
 * @param max_pos 关节1~7最大位置数组，单位：°
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_joint_drive_max_pos(rm_robot_handle *handle,float *max_pos);
/**
 * @brief 查询关节使能状态
 * 
 * @param handle 机械臂句柄
 * @param en_state 关节1~7使能状态数组，1-使能状态，0-掉使能状态
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_joint_en_state(rm_robot_handle *handle,uint8_t *en_state);
/**
 * @brief 查询关节错误代码
 * 
 * @param handle 机械臂句柄
 * @param err_flag 反馈关节错误代码，错误码请参见 \ref robotic_error
 * @param brake_state 反馈关节抱闸状态，1 代表抱闸未打开，0 代表抱闸已打开
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_joint_err_flag(rm_robot_handle *handle,uint16_t *err_flag,uint16_t *brake_state);
/** @} */ // 结束关节配置组的定义

/**  
 * @defgroup ArmTipVelocityParameters 机械臂末端参数配置
 * 
 * 机械臂末端运动参数设置及获取
 * @{  
 */
/**
 * @brief 设置机械臂末端最大线速度
 * 
 * @param handle 机械臂句柄
 * @param speed 末端最大线速度，单位m/s
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_arm_max_line_speed(rm_robot_handle *handle,float speed);
/**
 * @brief 设置机械臂末端最大线加速度
 * 
 * @param handle 机械臂句柄
 * @param acc 末端最大线加速度，单位m/s^2
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_arm_max_line_acc(rm_robot_handle *handle,float acc);
/**
 * @brief 设置机械臂末端最大角速度
 * 
 * @param handle 机械臂句柄
 * @param speed 末端最大角速度，单位rad/s
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_arm_max_angular_speed(rm_robot_handle *handle,float speed);
/**
 * @brief 设置机械臂末端最大角加速度
 * 
 * @param handle 机械臂句柄
 * @param acc 末端最大角加速度，单位rad/s^2
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_arm_max_angular_acc(rm_robot_handle *handle,float acc);
/**
 * @brief 设置机械臂末端参数为默认值
 * 
 * @param handle 机械臂句柄
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_arm_tcp_init(rm_robot_handle *handle);
/**
 * @brief 设置机械臂动力学碰撞检测等级
 * 
 * @param handle 机械臂句柄
 * @param collision_stage 等级：0~8，0-无碰撞，8-碰撞最灵敏
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_collision_state(rm_robot_handle *handle,int collision_stage);
/**
 * @brief 查询碰撞防护等级
 * 
 * @param handle 机械臂句柄
 * @param collision_stage 等级，范围：0~8
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_collision_stage(rm_robot_handle *handle,int *collision_stage);
/**
 * @brief 获取机械臂末端最大线速度
 * 
 * @param handle 机械臂句柄
 * @param speed 末端最大线速度，单位m/s
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_arm_max_line_speed(rm_robot_handle *handle, float *speed);
/**
 * @brief 获取机械臂末端最大线加速度
 * 
 * @param handle 机械臂句柄
 * @param acc 末端最大线加速度，单位m/s^2
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_arm_max_line_acc(rm_robot_handle *handle, float *acc);
/**
 * @brief 获取机械臂末端最大角速度
 * 
 * @param handle 机械臂句柄
 * @param speed 末端最大角速度，单位rad/s
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_arm_max_angular_speed(rm_robot_handle *handle, float *speed);
/**
 * @brief 获取机械臂末端最大角加速度
 * 
 * @param handle 机械臂句柄
 * @param acc 末端最大角加速度，单位rad/s^2
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_arm_max_angular_acc(rm_robot_handle *handle, float *acc);
/**
 * @brief 设置DH参数
 *
 * @param handle 机械臂控制句柄
 * @param dh DH参数
 * @return int 函数执行的状态码。
            - 0: 成功。
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
 */
RM_INTERFACE_EXPORT int rm_set_DH_data(rm_robot_handle *handle, rm_dh_t dh);
/**
 * @brief 获取DH参数
 *
 * @param handle 机械臂控制句柄
 * @param dh DH参数
 * @return int 函数执行的状态码。
            - 0: 成功。
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
 */
RM_INTERFACE_EXPORT int rm_get_DH_data(rm_robot_handle *handle, rm_dh_t *dh);
/**
 * @brief 恢复机械臂默认 DH 参数
 *
 * @param handle 机械臂控制句柄
 * @return int 函数执行的状态码。
            - 0: 成功。
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
 */
RM_INTERFACE_EXPORT int rm_set_DH_data_default(rm_robot_handle *handle);
/** @} */ // 结束组的定义
/**  
 * @defgroup ToolCoordinateConfig 工具坐标系配置
 * 
 * 工具坐标系标定、切换、删除、修改、查询及工具包络参数等管理
 * @{  
 */
/**
 * @brief 六点法自动设置工具坐标系 标记点位
 * 
 * @param handle 机械臂控制句柄 
 * @param point_num 1~6代表6个标定点
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_auto_tool_frame(rm_robot_handle *handle,int point_num);
/**
 * @brief 六点法自动设置工具坐标系 提交
 * 
 * @param handle 机械臂控制句柄 
 * @param name 工具坐标系名称，不能超过十个字节。
 * @param payload 新工具执行末端负载重量  单位kg
 * @param x 新工具执行末端负载位置 位置x 单位m
 * @param y 新工具执行末端负载位置 位置y 单位m
 * @param z 新工具执行末端负载位置 位置z 单位m
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_generate_auto_tool_frame(rm_robot_handle *handle, const char *name,float payload,float x,float y,float z);
/**
 * @brief 手动设置工具坐标系
 * 
 * @param handle 机械臂句柄
 * @param frame 新工具坐标系参数结构体
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_manual_tool_frame(rm_robot_handle *handle, rm_frame_t frame);
/**
 * @brief 切换当前工具坐标系
 * 
 * @param handle 机械臂句柄
 * @param tool_name 目标工具坐标系名称
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_change_tool_frame(rm_robot_handle *handle, const char* tool_name);
/**
 * @brief 删除指定工具坐标系
 * 
 * @param handle 机械臂句柄
 * @param tool_name 要删除的工具坐标系名称
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_delete_tool_frame(rm_robot_handle *handle, const char* tool_name);
/**
 * @brief 修改指定工具坐标系
 * 
 * @param handle 机械臂控制句柄 
 * @param frame 要修改的工具坐标系名称
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_update_tool_frame(rm_robot_handle *handle, rm_frame_t frame);
/**
 * @brief 获取所有工作坐标系名称
 * 
 * @param handle 机械臂控制句柄 
 * @param frame_names 返回的工作坐标系名称字符数组
 * @param len 返回的工作坐标系名称长度
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_total_tool_frame(rm_robot_handle *handle, rm_frame_name_t *frame_names, int *len);
/**
 * @brief  获取指定工具坐标系
 * 
 * @param handle 机械臂控制句柄 
 * @param name 指定的工具坐标系名称
 * @param frame 返回的工具参数
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_given_tool_frame(rm_robot_handle *handle, const char *name, rm_frame_t *frame);
/**
 * @brief 获取当前工具坐标系
 * 
 * @param handle 机械臂控制句柄 
 * @param tool_frame 返回的坐标系
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_current_tool_frame(rm_robot_handle *handle, rm_frame_t *tool_frame);
/**
 * @brief 设置工具坐标系的包络参数
 * 
 * @param handle 机械臂控制句柄 
 * @param envelope 包络参数列表，每个工具最多支持 5 个包络球，可以没有包络
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_tool_envelope(rm_robot_handle *handle, rm_envelope_balls_list_t envelope);
/**
 * @brief 获取工具坐标系的包络参数
 * 
 * @param handle 机械臂控制句柄 
 * @param tool_name 控制器中已存在的工具坐标系名称
 * @param envelope 包络参数列表，每个工具最多支持 5 个包络球，可以没有包络
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回获取失败，检查工具坐标系是否存在。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_tool_envelope(rm_robot_handle *handle, const char* tool_name, rm_envelope_balls_list_t *envelope);
/** @} */ // 结束组的定义

/**  
 * @defgroup WorkCoordinateConfig 工作坐标系配置
 * 
 * 工作坐标系标定、切换、删除、修改、查询等管理
 * @{  
 */
/**
 * @brief 三点法自动设置工作坐标系
 * 
 * @param handle 机械臂控制句柄 
 * @param workname 工作坐标系名称，不能超过十个字节。
 * @param point_num 1~3代表3个标定点，依次为原点、X轴一点、Y轴一点，4代表生成坐标系。
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_auto_work_frame(rm_robot_handle *handle,const char *workname, int point_num);
/**
 * @brief 手动设置工作坐标系
 * 
 * @param handle 机械臂控制句柄 
 * @param work_name 工作坐标系名称，不能超过十个字节。
 * @param pose 新工作坐标系相对于基坐标系的位姿
 * @return  int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
 */
RM_INTERFACE_EXPORT int rm_set_manual_work_frame(rm_robot_handle *handle, const char* work_name, rm_pose_t pose);
/**
 * @brief 切换当前工作坐标系
 * 
 * @param handle 机械臂控制句柄 
 * @param work_name 目标工作坐标系名称
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_change_work_frame(rm_robot_handle *handle, const char* work_name);
/**
 * @brief 删除指定工作坐标系
 * 
 * @param handle 机械臂控制句柄 
 * @param work_name 要删除的工具坐标系名称
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_delete_work_frame(rm_robot_handle *handle, const char* work_name);
/**
 * @brief 修改指定工作坐标系
 * 
 * @param handle 机械臂控制句柄 
 * @param work_name 指定工具坐标系名称
 * @param pose 更新工作坐标系相对于基坐标系的位姿
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_update_work_frame(rm_robot_handle *handle, const char* work_name, rm_pose_t pose);
/**
 * @brief 获取所有工作坐标系名称
 * 
 * @param handle 机械臂控制句柄 
 * @param frame_names 返回的工作坐标系名称字符数组
 * @param len 返回的工作坐标系名称长度
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_total_work_frame(rm_robot_handle *handle, rm_frame_name_t *frame_names, int *len);
/**
 * @brief 获取指定工作坐标系
 * 
 * @param handle 机械臂控制句柄 
 * @param name 指定的工作坐标系名称
 * @param pose 获取到的位姿
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_given_work_frame(rm_robot_handle *handle, const char *name, rm_pose_t *pose);
/**
 * @brief 获取当前工作坐标系
 * 
 * @param handle 机械臂控制句柄 
 * @param work_frame 返回的坐标系
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_current_work_frame(rm_robot_handle *handle, rm_frame_t *work_frame);
/** @} */ // 结束组的定义

/**  
 * @defgroup ArmState 机械臂状态查询
 * 
 * 机械臂当前状态、关节温度、电流、电压查询
 * @{  
 */
/**
 * @brief 获取机械臂当前状态
 * 
 * @param handle 机械臂控制句柄 
 * @param state 机械臂当前状态结构体
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_current_arm_state(rm_robot_handle *handle,rm_current_arm_state_t *state);
/**
 * @brief 获取关节当前温度
 * 
 * @param handle 机械臂控制句柄 
 * @param temperature 关节1~7温度数组
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_current_joint_temperature(rm_robot_handle *handle, float *temperature);
/**
 * @brief 获取关节当前电流
 * 
 * @param handle 机械臂控制句柄 
 * @param current 关节1~7电流数组
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_current_joint_current(rm_robot_handle *handle, float *current);
/**
 * @brief 获取关节当前电压
 * 
 * @param handle 机械臂控制句柄 
 * @param voltage 关节1~7电压数组
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_current_joint_voltage(rm_robot_handle *handle, float *voltage);
/**
 * @brief 获取当前关节角度
 * 
 * @param handle 机械臂控制句柄 
 * @param joint 当前7个关节的角度数组
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_joint_degree(rm_robot_handle *handle, float *joint);
/**
 * @brief 获取机械臂所有状态信息
 * 
 * @param handle 机械臂控制句柄 
 * @param state 存储机械臂信息的结构体
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_arm_all_state(rm_robot_handle *handle, rm_arm_all_state_t *state);
/** @} */ // 结束组的定义

/**  
 * @defgroup InitPose 初始位置设置
 * 
 * 记录机械臂初始位置
 * @{  
 */
/**
 * @brief 设置机械臂的初始位置角度
 * 
 * @param handle 机械臂控制句柄 
 * @param joint 机械臂初始位置关节角度数组
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_init_pose(rm_robot_handle *handle, float *joint);
/**
 * @brief 获取机械臂初始位置角度
 * 
 * @param handle 机械臂控制句柄 
 * @param joint 机械臂初始位置关节角度数组
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_init_pose(rm_robot_handle *handle, float *joint);
/** @} */ // 结束组的定义

/**  
 * @defgroup MovePlan 机械臂轨迹指令类
 * 
 * 关节运动、笛卡尔空间运动以及角度及位姿透传
 * @{  
 */
/**
 * @brief 关节空间运动
 * 
 * @param handle 机械臂控制句柄 
 * @param joint 目标关节1~7角度数组
 * @param v 速度比例1~100，即规划速度和加速度占关节最大线转速和加速度的百分比
 * @param r 轨迹交融半径，目前默认0。
 * @param trajectory_connect 轨迹连接标志  
 *        - 0：立即规划并执行轨迹，不与后续轨迹连接。  
 *        - 1：将当前轨迹与下一条轨迹一起规划，但不立即执行。阻塞模式下，即使发送成功也会立即返回。  
 * @param block 阻塞设置
 *        - 多线程模式：  
 *            - 0：非阻塞模式，发送指令后立即返回。  
 *            - 1：阻塞模式，等待机械臂到达目标位置或规划失败后返回。  
 *        - 单线程模式：  
 *            - 0：非阻塞模式。  
 *            - 其他值：阻塞模式并设置超时时间，单位为秒。
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
            - -4: 当前到位设备校验失败，即当前到位设备不为关节。
            - -5: 单线程模式超时未接收到返回，请确保超时时间设置合理。
 */
RM_INTERFACE_EXPORT int rm_movej(rm_robot_handle *handle, const float *joint, int v, int r,int trajectory_connect,int block);
/**
 * @brief 笛卡尔空间直线运动
 * 
 * @param handle 机械臂控制句柄 
 * @param pose 目标位姿,位置单位：米，姿态单位：弧度
 * @param v 速度比例1~100，即规划速度和加速度占机械臂末端最大线速度和线加速度的百分比
 * @param r 轨迹交融半径，目前默认0。
 * @param trajectory_connect 轨迹连接标志  
 *        - 0：立即规划并执行轨迹，不与后续轨迹连接。  
 *        - 1：将当前轨迹与下一条轨迹一起规划，但不立即执行。阻塞模式下，即使发送成功也会立即返回。  
 * @param block 阻塞设置
 *        - 多线程模式：  
 *            - 0：非阻塞模式，发送指令后立即返回。  
 *            - 1：阻塞模式，等待机械臂到达目标位置或规划失败后返回。  
 *        - 单线程模式：  
 *            - 0：非阻塞模式，发送指令后立即返回。  
 *            - 其他值：阻塞模式并设置超时时间，单位为秒。
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
            - -4: 当前到位设备校验失败，即当前到位设备不为关节。
            - -5: 单线程模式超时未接收到返回，请确保超时时间设置合理。
 */
RM_INTERFACE_EXPORT int rm_movel(rm_robot_handle *handle,rm_pose_t pose, int v, int r, int trajectory_connect, int block);
/**
 * @brief 笛卡尔空间直线偏移运动
 * @details 该函数用于机械臂末端在当前位姿的基础上沿某坐标系（工具或工作）进行位移或旋转运动。
 * @param handle 机械臂控制句柄 
 * @param offset 位置姿态偏移，位置单位：米，姿态单位：弧度
 * @param v 速度百分比系数，1~100
 * @param r 交融半径百分比系数，0~100。
 * @param trajectory_connect 轨迹连接标志  
 *        - 0：立即规划并执行轨迹，不与后续轨迹连接。  
 *        - 1：将当前轨迹与下一条轨迹一起规划，但不立即执行。阻塞模式下，即使发送成功也会立即返回。  
 * @param frame_type 参考坐标系类型:0-工作，1-工具
 * @param block 阻塞设置
 *        - 多线程模式：  
 *            - 0：非阻塞模式，发送指令后立即返回。  
 *            - 1：阻塞模式，等待机械臂到达目标位置或规划失败后才返回。  
 *        - 单线程模式：  
 *            - 0：非阻塞模式，发送指令后立即返回。  
 *            - 其他值：阻塞模式并设置超时时间，根据运动时间设置，单位为秒。
 * @attention 使用单线程阻塞模式时，请设置超时时间确保轨迹在超时时间内运行结束返回
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
            - -4: 当前到位设备校验失败，即当前到位设备不为关节。
            - -5: 单线程模式超时未接收到返回，请确保超时时间设置合理。
 */
RM_INTERFACE_EXPORT int rm_movel_offset(rm_robot_handle *handle,rm_pose_t offset, int v, int r, int trajectory_connect, int frame_type, int block);
/**
 * @brief 样条曲线运动
 * 
 * @param handle 机械臂控制句柄 
 * @param pose 目标位姿,位置单位：米，姿态单位：弧度
 * @param v 速度比例1~100，即规划速度和加速度占机械臂末端最大线速度和线加速度的百分比
 * @param r 轨迹交融半径，目前默认0。
 * @param trajectory_connect 轨迹连接标志  
 *        - 0：立即规划并执行轨迹，不与后续轨迹连接。  
 *        - 1：将当前轨迹与下一条轨迹一起规划，但不立即执行。阻塞模式下，即使发送成功也会立即返回。  
 * @note 样条曲线运动需至少连续下发三个点位（trajectory_connect设置为1），否则运动轨迹为直线
 * @param block 阻塞设置
 *        - 多线程模式：  
 *            - 0：非阻塞模式，发送指令后立即返回。  
 *            - 1：阻塞模式，等待机械臂到达目标位置或规划失败后返回。  
 *        - 单线程模式：  
 *            - 0：非阻塞模式，发送指令后立即返回。  
 *            - 其他值：阻塞模式并设置超时时间，根据运动时间设置，单位为秒。
 * @attention 使用单线程阻塞模式时，请设置超时时间确保轨迹在超时时间内运行结束返回
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
            - -4: 当前到位设备校验失败，即当前到位设备不为关节。
            - -5: 单线程模式超时未接收到返回，请确保超时时间设置合理。
 */
RM_INTERFACE_EXPORT int rm_moves(rm_robot_handle *handle,rm_pose_t pose, int v, int r, int trajectory_connect, int block);
/**
 * @brief 笛卡尔空间圆弧运动
 * 
 * @param handle 机械臂控制句柄 
 * @param pose_via 中间点位姿，位置单位：米，姿态单位：弧度
 * @param pose_to 终点位姿
 * @param v 速度比例1~100，即规划速度和加速度占机械臂末端最大角速度和角加速度的百分比
 * @param r 轨迹交融半径，目前默认0。
 * @param loop 规划圈数，目前默认0.
 * @param trajectory_connect 轨迹连接标志  
 *        - 0：立即规划并执行轨迹，不与后续轨迹连接。  
 *        - 1：将当前轨迹与下一条轨迹一起规划，但不立即执行。阻塞模式下，即使发送成功也会立即返回。  
 * @param block 阻塞设置
 *        - 多线程模式：  
 *            - 0：非阻塞模式，发送指令后立即返回。  
 *            - 1：阻塞模式，等待机械臂到达目标位置或规划失败后返回。  
 *        - 单线程模式：  
 *            - 0：非阻塞模式。  
 *            - 其他值：阻塞模式并设置超时时间，根据运动时间设置，单位为秒。
 * @attention 使用单线程阻塞模式时，请设置超时时间确保轨迹在超时时间内运行结束返回
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
            - -4: 当前到位设备校验失败，即当前到位设备不为关节。 
            - -5: 单线程模式超时未接收到返回，请确保超时时间设置合理。
 */
RM_INTERFACE_EXPORT int rm_movec(rm_robot_handle *handle,rm_pose_t pose_via, rm_pose_t pose_to, int v, int r, int loop, int trajectory_connect, int block);
/**
 * @brief 该函数用于关节空间运动到目标位姿
 * 
 * @param handle 机械臂控制句柄 
 * @param pose 目标位姿，位置单位：米，姿态单位：弧度。
 * @param v 速度比例1~100，即规划速度和加速度占机械臂末端最大线速度和线加速度的百分比
 * @param r 轨迹交融半径，目前默认0。
 * @param trajectory_connect 轨迹连接标志  
 *        - 0：立即规划并执行轨迹，不与后续轨迹连接。  
 *        - 1：将当前轨迹与下一条轨迹一起规划，但不立即执行。阻塞模式下，即使发送成功也会立即返回。  
 * @param block 阻塞设置
 *        - 多线程模式：  
 *            - 0：非阻塞模式，发送指令后立即返回。  
 *            - 1：阻塞模式，等待机械臂到达目标位置或规划失败后返回。  
 *        - 单线程模式：  
 *            - 0：非阻塞模式。  
 *            - 其他值：阻塞模式并设置超时时间，根据运动时间设置，单位为秒。
 * @attention 使用单线程阻塞模式时，请设置超时时间确保轨迹在超时时间内运行结束返回
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
            - -4: 当前到位设备校验失败，即当前到位设备不为关节。 
            - -5: 单线程模式超时未接收到返回，请确保超时时间设置合理。
 */
RM_INTERFACE_EXPORT int rm_movej_p(rm_robot_handle *handle,rm_pose_t pose, int v, int r, int trajectory_connect, int block);
/**
 * @brief 角度不经规划，直接通过CANFD透传给机械臂
 * @details 角度透传到 CANFD，若指令正确，机械臂立即执行  
 * <b>备注</b>：  
 *     透传效果受通信周期和轨迹平滑度影响，因此要求通信周期稳定，避免大幅波动。  
 *     用户在使用此功能时，建议进行良好的轨迹规划，以确保机械臂的稳定运行。  
 *     I系列有线网口周期最快可达2ms，提供了更高的实时性。
 * @param handle 机械臂控制句柄 
 * @param joint 关节1~7目标角度数组,单位：°
 * @param follow true-高跟随，false-低跟随。若使用高跟随，透传周期要求不超过 10ms。
 * @param expand 如果存在通用扩展轴，并需要进行透传，可使用该参数进行透传发送。
 * @param trajectory_mode 高跟随模式下，0-完全透传模式、1-曲线拟合模式、2-滤波模式
 * @param radio 曲线拟合模式和滤波模式下的平滑系数（数值越大效果越好），滤波模式下取值范围0~100，曲线拟合模式下取值范围0~1000
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - -1: 数据发送失败，通信过程中出现问题。
 */
RM_INTERFACE_EXPORT int rm_movej_canfd(rm_robot_handle *handle, float *joint, bool follow, int expand, int trajectory_mode=0, int radio=0);
/**
 * @brief 位姿不经规划，直接通过CANFD透传给机械臂
 * @details 当目标位姿被透传到机械臂控制器时，控制器首先尝试进行逆解计算。
 * 若逆解成功且计算出的各关节角度与当前角度差异不大，则直接下发至关节执行，跳过额外的轨迹规划步骤。
 * 这一特性适用于需要周期性调整位姿的场景，如视觉伺服等应用。  
 * <b>备注</b>：  
 *     透传效果受通信周期和轨迹平滑度影响，因此要求通信周期稳定，避免大幅波动。  
 *     用户在使用此功能时，建议进行良好的轨迹规划，以确保机械臂的稳定运行。  
 *     I系列有线网口周期最快可达2ms，提供了更高的实时性。     
 * @param handle 机械臂控制句柄 
 * @param pose 位姿 (优先采用四元数表达)
 * @param follow true-高跟随，false-低跟随。若使用高跟随，透传周期要求不超过 10ms。
 * @param trajectory_mode 高跟随模式下，0-完全透传模式、1-曲线拟合模式、2-滤波模式
 * @param radio 曲线拟合模式和滤波模式下的平滑系数（数值越大效果越好），滤波模式下取值范围0~100，曲线拟合模式下取值范围0~1000
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - -1: 数据发送失败，通信过程中出现问题。
 */
RM_INTERFACE_EXPORT int rm_movep_canfd(rm_robot_handle *handle, rm_pose_t pose, bool follow, int trajectory_mode=0, int radio=0);
/**
 * @brief 关节空间跟随运动
 * @param handle 机械臂控制句柄 
 * @param joint 关节1~7目标角度数组,单位：°
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - -1: 数据发送失败，通信过程中出现问题。
 */
RM_INTERFACE_EXPORT int rm_movej_follow(rm_robot_handle *handle,float *joint);
/**
 * @brief 笛卡尔空间跟随运动
 * @param handle 机械臂控制句柄 
 * @param pose 位姿 (优先采用四元数表达)
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - -1: 数据发送失败，通信过程中出现问题。
 */
RM_INTERFACE_EXPORT int rm_movep_follow(rm_robot_handle *handle, rm_pose_t pose);
/** @} */ // 结束组的定义

/**  
 * @defgroup ArmMotionControl 机械臂运动控制指令类
 * 
 * 控制运动的急停、缓停、暂停、继续、清除轨迹以及查询当前规划类型
 * @{  
 */
/**
 * @brief 轨迹缓停，在当前正在运行的轨迹上停止
 * 
 * @param handle 机械臂控制句柄 
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_arm_slow_stop(rm_robot_handle *handle);
/**
 * @brief 轨迹急停，关节最快速度停止，轨迹不可恢复
 * 
 * @param handle 机械臂控制句柄 
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
 */
RM_INTERFACE_EXPORT int rm_set_arm_stop(rm_robot_handle *handle);
/**
 * @brief 轨迹暂停，暂停在规划轨迹上，轨迹可恢复
 * 
 * @param handle 机械臂控制句柄 
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_arm_pause(rm_robot_handle *handle);
/**
 * @brief 轨迹暂停后，继续当前轨迹运动
 * 
 * @param handle 机械臂控制句柄 
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_arm_continue(rm_robot_handle *handle);
/**
 * @brief 清除当前轨迹
 * 
 * @param handle 机械臂控制句柄 
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
 * @attention 必须在暂停后使用，否则机械臂会发生意外！！！！ 
 */
RM_INTERFACE_EXPORT int rm_set_delete_current_trajectory(rm_robot_handle *handle);
/**
 * @brief 清除所有轨迹
 * 
 * @param handle 机械臂控制句柄 
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 * @attention 必须在暂停后使用，否则机械臂会发生意外！！！！
 */
RM_INTERFACE_EXPORT int rm_set_arm_delete_trajectory(rm_robot_handle *handle);
/**
 * @brief 获取当前正在规划的轨迹信息
 * 
 * @param handle 机械臂控制句柄 
 * @param type 返回的规划类型
 * @param data 无规划和关节空间规划为当前关节1~7角度数组；笛卡尔空间规划则为当前末端位姿
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_arm_current_trajectory(rm_robot_handle *handle,rm_arm_current_trajectory_e *type,float *data);
/** @} */ // 结束组的定义

/**  
 * @defgroup ArmTeachMove 机械臂示教指令类
 * 
 * 关节、位置、姿态的示教及步进控制
 * @{  
 */
/**
 * @brief 关节步进
 * 
 * @param handle 机械臂控制句柄 
 * @param joint_num 关节序号，1~7
 * @param step 步进的角度，
 * @param v 速度比例1~100，即规划速度和加速度占机械臂末端最大线速度和线加速度的百分比
 * @param block 阻塞设置
 *        - 多线程模式：  
 *            - 0：非阻塞模式，发送指令后立即返回。  
 *            - 1：阻塞模式，等待机械臂到达目标位置或规划失败后返回。  
 *        - 单线程模式：  
 *            - 0：非阻塞模式。  
 *            - 其他值：阻塞模式并设置超时时间，根据运动时间设置，单位为秒。
 * @attention 使用单线程阻塞模式时，请设置超时时间确保轨迹在超时时间内运行结束返回
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
            - -4: 当前到位设备校验失败，即当前到位设备不为关节。 
            - -5: 单线程模式超时未接收到返回，请确保超时时间设置合理。 
 */
RM_INTERFACE_EXPORT int rm_set_joint_step(rm_robot_handle *handle,int joint_num, float step, int v, int block);
/**
 * @brief 当前工作坐标系下，位置步进
 * 
 * @param handle 机械臂控制句柄 
 * @param type 示教类型
 * @param step 步进的距离，单位m，精确到0.001mm
 * @param v 速度比例1~100，即规划速度和加速度占机械臂末端最大线速度和线加速度的百分比
 * @param block 阻塞设置
 *        - 多线程模式：  
 *            - 0：非阻塞模式，发送指令后立即返回。  
 *            - 1：阻塞模式，等待机械臂到达目标位置或规划失败后返回。  
 *        - 单线程模式：  
 *            - 0：非阻塞模式。  
 *            - 其他值：阻塞模式并设置超时时间，根据运动时间设置，单位为秒。
 * @attention 使用单线程阻塞模式时，请设置超时时间确保轨迹在超时时间内运行结束返回
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
            - -4: 当前到位设备校验失败，即当前到位设备不为关节。 
            - -5: 单线程模式超时未接收到返回，请确保超时时间设置合理。
 * @attention 参考坐标系默认为当前工作坐标系，可调用rm_set_teach_frame修改为工具坐标系，
 */
RM_INTERFACE_EXPORT int rm_set_pos_step(rm_robot_handle *handle, rm_pos_teach_type_e type, float step, int v, int block);
/**
 * @brief 当前工作坐标系下，姿态步进
 * 
 * @param handle 机械臂控制句柄 
 * @param type 示教类型
 * @param step 步进的弧度，单位rad，精确到0.001rad
 * @param v 速度比例1~100，即规划速度和加速度占机械臂末端最大线速度和线加速度的百分比
 * @param block 阻塞设置
 *        - 多线程模式：  
 *            - 0：非阻塞模式，发送指令后立即返回。  
 *            - 1：阻塞模式，等待机械臂到达目标位置或规划失败后返回。  
 *        - 单线程模式：  
 *            - 0：非阻塞模式。  
 *            - 其他值：阻塞模式并设置超时时间，根据运动时间设置，单位为秒。
 * @attention 使用单线程阻塞模式时，请设置超时时间确保轨迹在超时时间内运行结束返回
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
            - -4: 当前到位设备校验失败，即当前到位设备不为关节。 
            - -5: 单线程模式超时未接收到返回，请确保超时时间设置合理。
 * @attention 参考坐标系默认为当前工作坐标系，可调用rm_set_teach_frame修改为工具坐标系，
 */
RM_INTERFACE_EXPORT int rm_set_ort_step(rm_robot_handle *handle, rm_ort_teach_type_e type, float step, int v, int block);
/**
 * @brief 切换示教运动坐标系
 * 
 * @param handle 机械臂控制句柄 
 * @param frame_type 0: 工作坐标系运动, 1: 工具坐标系运动
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_teach_frame(rm_robot_handle *handle, int frame_type);
/**
 * @brief 获取示教参考坐标系
 * 
 * @param handle 机械臂控制句柄 
 * @param frame_type 0: 工作坐标系运动, 1: 工具坐标系运动
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_teach_frame(rm_robot_handle *handle,int *frame_type);
/**
 * @brief 关节示教
 * 
 * @param handle 机械臂控制句柄 
 * @param joint_num 示教关节的序号，1~7
 * @param direction 示教方向，0-负方向，1-正方向
 * @param v 速度比例1~100，即规划速度和加速度占关节最大线转速和加速度的百分比
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_joint_teach(rm_robot_handle *handle,int joint_num, int direction, int v);
/**
 * @brief 当前工作坐标系下，笛卡尔空间位置示教
 * 
 * @param handle 机械臂控制句柄 
 * @param type 示教类型
 * @param direction 示教方向，0-负方向，1-正方向
 * @param v 即规划速度和加速度占机械臂末端最大线速度和线加速度的百分比
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 * @attention 参考坐标系默认为当前工作坐标系，可调用rm_set_teach_frame修改为工具坐标系，
 */
RM_INTERFACE_EXPORT int rm_set_pos_teach(rm_robot_handle *handle,rm_pos_teach_type_e type, int direction, int v);
/**
 * @brief 当前工作坐标系下，笛卡尔空间姿态示教
 * 
 * @param handle 机械臂控制句柄 
 * @param type 示教类型
 * @param direction 示教方向，0-负方向，1-正方向
 * @param v 速度比例1~100，即规划速度和加速度占机械臂末端最大角速度和角加速度的百分比
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 * @attention 参考坐标系默认为当前工作坐标系，可调用rm_set_teach_frame修改为工具坐标系，
 */
RM_INTERFACE_EXPORT int rm_set_ort_teach(rm_robot_handle *handle,rm_ort_teach_type_e type, int direction, int v);
/**
 * @brief 示教停止
 * 
 * @param handle 机械臂控制句柄 
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_stop_teach(rm_robot_handle *handle);
/** @} */ // 结束组的定义

/**  
 * @defgroup ControllerConfig 系统配置
 * 
 * 控制器状态获取、电源控制、错误清除、有线网口IP地址配置、软件信息获取
 * @{  
 */
/**
 * @brief 获取控制器状态
 * 
 * @param handle 机械臂控制句柄 
 * @param voltage 返回的电压
 * @param current 返回的电流
 * @param temperature 返回的温度
 * @param err_flag 控制器运行错误代码
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_controller_state(rm_robot_handle *handle, float *voltage, float *current, float *temperature, int *err_flag);
/**
 * @brief 设置机械臂电源
 * 
 * @param handle 机械臂控制句柄 
 * @param arm_power 1-上电状态，0 断电状态
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_arm_power(rm_robot_handle *handle, int arm_power);
/**
 * @brief 读取机械臂电源状态
 * 
 * @param handle 机械臂控制句柄 
 * @param power_state 获取到的机械臂电源状态，1-上电状态，0 断电状态
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_arm_power_state(rm_robot_handle *handle, int *power_state);
/**
 * @brief 读取控制器的累计运行时间
 * 
 * @param handle 机械臂控制句柄 
 * @param day 读取到的时间
 * @param hour 读取到的时间
 * @param min 读取到的时间
 * @param sec 读取到的时间
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_system_runtime(rm_robot_handle *handle, int *day, int *hour, int *min, int *sec);
/**
 * @brief 清零控制器的累计运行时间
 * 
 * @param handle 机械臂控制句柄 
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_clear_system_runtime(rm_robot_handle *handle);
/**
 * @brief 读取关节的累计转动角度
 * 
 * @param handle 机械臂控制句柄 
 * @param joint_odom 各关节累计的转动角度，单位°
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_joint_odom(rm_robot_handle *handle, float *joint_odom);
/**
 * @brief 清零关节累计转动的角度
 * 
 * @param handle 机械臂控制句柄 
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_clear_joint_odom(rm_robot_handle *handle);
/**
 * @brief 配置有线网口 IP 地址
 * 
 * @param handle 机械臂控制句柄 
 * @param ip 有线网口 IP 地址
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_NetIP(rm_robot_handle *handle, const char* ip);
/**
 * @brief 清除系统错误
 * 
 * @param handle 机械臂控制句柄 
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_clear_system_err(rm_robot_handle *handle);
/**
 * @brief 读取机械臂软件信息
 * 
 * @param handle 机械臂控制句柄 
 * @param software_info 机械臂软件信息结构体
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_arm_software_info(rm_robot_handle *handle,rm_arm_software_version_t *software_info);
/**
 * @brief 查询控制器RS485模式
 * 
 * @param handle 机械臂控制句柄 
 * @param mode 0-代表默认 RS485 串行通讯，1-代表 modbus-RTU 主站模式，2-代表 modbus-RTU 从站模式；
 * @param baudrate 波特率
 * @param timeout modbus 协议超时时间，单位 100ms，仅在 modbus-RTU 模式下提供此字段
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
            - -4: 四代控制器不支持此接口
 */
RM_INTERFACE_EXPORT int rm_get_controller_RS485_mode(rm_robot_handle *handle, int* mode, int* baudrate, int* timeout);
/**
 * @brief 查询工具端 RS485 模式
 * 
 * @param handle 机械臂控制句柄 
 * @param mode 0-代表默认 RS485 串行通讯 1-代表 modbus-RTU 主站模式
 * @param baudrate 波特率
 * @param timeout modbus 协议超时时间，单位 100ms，仅在 modbus-RTU 模式下提供此字段
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
            - -4: 四代控制器不支持此接口
 */
RM_INTERFACE_EXPORT int rm_get_tool_RS485_mode(rm_robot_handle *handle, int* mode, int* baudrate, int* timeout);
/**
 * @brief 查询关节软件版本号
 * 
 * @param handle 机械臂控制句柄 
 * @param version 获取到的各关节软件版本号数组，需转换为十六进制，例如获取某关节版本为54536，转换为十六进制为D508，则当前关节的版本号为 Vd5.0.8（三代控制器）
 * @param joint_v 获取到的各关节软件版本号字符串数组（四代控制器）
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_joint_software_version(rm_robot_handle *handle,int *version, rm_version_t *joint_v);
RM_INTERFACE_EXPORT int rm_get_joint_software_version(rm_robot_handle *handle,int *version);
/**
 * @brief 查询末端接口板软件版本号
 * 
 * @param handle 机械臂控制句柄 
 * @param version （三代控制器）获取到的末端接口板软件版本号,需转换为十六进制，例如获取到版本号393，转换为十六进制为189，则当前关节的版本号为 V1.8.9
 * @param end_v （四代控制器）获取到的末端接口板软件版本号字符串
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_tool_software_version(rm_robot_handle *handle, int *version, rm_version_t *end_v);
RM_INTERFACE_EXPORT int rm_get_tool_software_version(rm_robot_handle *handle, int *version);
/** @} */ // 结束组的定义

/**  
 * @defgroup CommunicationConfig 配置通讯内容
 * 
 * 机械臂控制器可通过网口、WIFI、RS232-USB 接口和 RS485 接口与用户通信，用户使用时无需切换，可使用上述任一接口，
 * 控制器收到指令后，若指令格式正确，则会通过相同的接口反馈数据。
 * @{  
 */
/**
 * @brief 配置 wifiAP 模式
 * 
 * @param handle 机械臂控制句柄 
 * @param wifi_name wifi名称
 * @param password wifi密码
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - -1: 数据发送失败，通信过程中出现问题。
 * @attention 设置成功后蜂鸣器响，手动重启控制器进入 WIFIAP 模式。
 */
RM_INTERFACE_EXPORT int rm_set_wifi_ap(rm_robot_handle *handle, const char* wifi_name, const char* password);
/**
 * @brief 配置WiFi STA模式
 * 
 * @param handle 机械臂控制句柄 
 * @param router_name 路由器名称
 * @param password 路由器Wifi密码
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - -1: 数据发送失败，通信过程中出现问题。
 * @attention 设置成功后蜂鸣器响，手动重启控制器进入 WIFISTA 模式。
 */
RM_INTERFACE_EXPORT int rm_set_wifi_sta(rm_robot_handle *handle, const char* router_name, const char* password);

/**
 * @brief 控制器RS485接口波特率设置，设置成功后蜂鸣器响
 * 
 * @param handle 机械臂控制句柄 
 * @param baudrate 波特率：9600,19200,38400,115200和460800，若用户设置其他数据，控制器会默认按照460800处理。
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 四代控制器不支持该接口
 * @attention 该指令下发后控制器会记录当前波特率，断电重启后仍会使用该波特率对外通信。
 */
RM_INTERFACE_EXPORT int rm_set_RS485(rm_robot_handle *handle, int baudrate);
/**
 * @brief 获取有线网卡信息，未连接有线网卡则会返回无效数据
 * 
 * @param handle 机械臂控制句柄 
 * @param ip 网络地址
 * @param mask 子网掩码
 * @param mac MAC地址
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_wired_net(rm_robot_handle *handle, char * ip, char * mask, char * mac);
/**
 * @brief 查询无线网卡网络信息
 * 
 * @param handle 机械臂控制句柄 
 * @param wifi_net 无线网卡网络信息结构体
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 * @attention 结构体中的channel值只有在AP模式时才可获取到，标识 wifi 热点的物理信道号
 */
RM_INTERFACE_EXPORT int rm_get_wifi_net(rm_robot_handle *handle, rm_wifi_net_t *wifi_net);
/**
 * @brief 恢复网络出厂设置
 * 
 * @param handle 机械臂控制句柄 
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_net_default(rm_robot_handle *handle);
/**
 * @brief 配置关闭 wifi 功能，需要重启后生效
 * 
 * @param handle 机械臂控制句柄 
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_wifi_close(rm_robot_handle *handle);
/** @} */ // 结束组的定义

/**  
 * @defgroup ControllerIOConfig 控制器IO配置及获取
 * 
 * 机械臂控制器提供IO端口，用于与外部设备交互。以下是具体的IO端口配置和特性：  
 *  
 * - 数字IO:  
 *   - 数量: 4路  
 *   - 功能: DO/DI复用（数字输出/数字输入）  
 *   - 电压范围: 可配置为0~24V  
 * @{  
 */
/**
 * @brief 设置数字IO模式
 * 
 * @param handle 机械臂控制句柄 
 * @param io_num IO 端口号，范围：1~4
 * @param io_mode 模式：
 *                0-通用输入模式，1-通用输出模式、2-输入开始功能复用模式、3-输入暂停功能复用模式、
 *                4-输入继续功能复用模式、5-输入急停功能复用模式、6-输入进入电流环拖动复用模式、7-输入进入力只动位置拖动模式（六维力版本可配置）、
 *                8-输入进入力只动姿态拖动模式（六维力版本可配置）、9-输入进入力位姿结合拖动复用模式（六维力版本可配置）、
 *                10-输入外部轴最大软限位复用模式（外部轴模式可配置）、11-输入外部轴最小软限位复用模式（外部轴模式可配置）、
 *                12-输入初始位姿功能复用模式、13-输出碰撞功能复用模式、14-实时调速功能复用模式
 * @param io_speed_mode 模式取值1或2(只有io_mode为14时生效)：
 *                          1表示单次触发模式，单次触发模式下当IO拉低速度设置为speed参数值，IO恢复高电平速度设置为初始值。
 *                          2表示连续触发模式，连续触发模式下IO拉低速度设置为speed参数值，IO恢复高电平速度维持当前值
 * @param io_speed 速度取值范围0-100(只有io_mode为14时生效)
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_IO_mode(rm_robot_handle *handle, int io_num, int io_mode, int io_speed_mode=0, int io_speed=0);
/**
 * @brief 设置数字IO输出
 * 
 * @param handle 机械臂控制句柄 
 * @param io_num IO 端口号，范围：1~4
 * @param state IO 状态，1-输出高，0-输出低
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_DO_state(rm_robot_handle *handle, int io_num, int state);
/**
 * @brief 
 * 
 * @param handle 机械臂控制句柄 
 * @param io_num IO 端口号，范围：1~4
 * @param state IO 状态
 * @param io_mode 模式：
 *                0-通用输入模式，1-通用输出模式、2-输入开始功能复用模式、3-输入暂停功能复用模式、
 *                4-输入继续功能复用模式、5-输入急停功能复用模式、6-输入进入电流环拖动复用模式、7-输入进入力只动位置拖动模式（六维力版本可配置）、
 *                8-输入进入力只动姿态拖动模式（六维力版本可配置）、9-输入进入力位姿结合拖动复用模式（六维力版本可配置）、
 *                10-输入外部轴最大软限位复用模式（外部轴模式可配置）、11-输入外部轴最小软限位复用模式（外部轴模式可配置）、
 *                12-输入初始位姿功能复用模式13-输出碰撞功能复用模式、14-实时调速功能复用模式
 * @param io_speed_mode 模式取值1或2(只有io_mode为14时生效)：
 *                          1表示单次触发模式，单次触发模式下当IO拉低速度设置为speed参数值，IO恢复高电平速度设置为初始值。
 *                          2表示连续触发模式，连续触发模式下IO拉低速度设置为speed参数值，IO恢复高电平速度维持当前值
 * @param io_speed 速度取值范围0-100(只有io_mode为14时生效)
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_IO_state(rm_robot_handle *handle, int io_num, int* state, int* mode, int* io_speed_mode=nullptr, int* io_speed=nullptr);
/**
 * @brief 获取所有 IO 输入状态
 * 
 * @param handle 机械臂控制句柄 
 * @param DI_state 数字输入状态，1：高，0：低，-1：该端口不是输入模式
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_IO_input(rm_robot_handle *handle, int *DI_state);
/**
 * @brief 获取所有 IO 输出状态
 * 
 * @param handle 机械臂控制句柄 
 * @param DO_state 数字输出状态，1：高，0：低，-1：该端口不是输出模式
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_IO_output(rm_robot_handle *handle, int *DO_state);
/**
 * @brief 设置控制器电源输出
 * 
 * @param handle 机械臂控制句柄 
 * @param voltage_type 电源输出类型，0：0V，2：12V，3：24V
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_voltage(rm_robot_handle *handle, int voltage_type);
/**
 * @brief 获取控制器电源输出类
 * 
 * @param handle 机械臂控制句柄 
 * @param voltage_type 电源输出类型，0：0V，2：12V，3：24V
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_voltage(rm_robot_handle *handle, int *voltage_type);
/** @} */ // 结束组的定义

/**  
 * @defgroup ToolIOConfig 末端工具 IO 控制
 * 
 * 机械臂末端工具端提供多种IO端口，用于与外部设备交互。以下是端口的具体配置和特性：
 * - 电源输出:  
 *   - 数量: 1路  
 *   - 可配置电压: 0V, 5V, 12V, 24V  
 *  
 * - 数字IO:  
 *   - 数量: 2路  
 *   - 可配置性: 输入/输出均可配置  
 *   - 输入特性:  
 *     - 参考电平: 12V～24V  
 *   - 输出特性:  
 *     - 电压范围: 5～24V（与输出电压一致）  
 *  
 * - 通讯接口:  
 *   - 数量: 1路  
 *   - 可配置协议: RS485
 * @{  
 */
/**
 * @brief 设置工具端数字 IO 输出
 * 
 * @param handle 机械臂控制句柄 
 * @param io_num IO 端口号，范围：1~2
 * @param state IO 状态，1-输出高，0-输出低
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_tool_DO_state(rm_robot_handle *handle, int io_num, int state);
/**
 * @brief 设置工具端数字 IO 模式
 * 
 * @param handle 机械臂控制句柄 
 * @param io_num IO 端口号，范围：1~2
 * @param io_mode 模式，0-输入状态，1-输出状态
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_tool_IO_mode(rm_robot_handle *handle, int io_num, int io_mode);
/**
 * @brief 获取数字 IO 状态
 * 
 * @param handle 机械臂控制句柄 
 * @param state 0-输入模式，1-输出模式
 * @param mode 0-低，1-高
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_tool_IO_state(rm_robot_handle *handle, int* state, int* mode);
/**
 * @brief 设置工具端电源输出
 * 
 * @param handle 机械臂控制句柄 
 * @param voltage_type 电源输出类型，0：0V，1：5V，2：12V，3：24V，
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 * @attention 电源输出设置为 5V 时，工具端的IO 暂不支持输入输出功能
 */
RM_INTERFACE_EXPORT int rm_set_tool_voltage(rm_robot_handle *handle, int voltage_type);
/**
 * @brief 获取工具端电源输出
 * 
 * @param handle 机械臂控制句柄 
 * @param voltage_type 电源输出类型，0：0V，1：5V，2：12V，3：24V，
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_tool_voltage(rm_robot_handle *handle, int *voltage_type);
/** @} */ // 结束组的定义

/**  
 * @defgroup GripperControl 末端工具—手爪控制
 * 
 * 睿尔曼机械臂末端配备了因时机器人公司的 EG2-4C2 手爪，为了便于用户操作手爪，机械臂控制器
 * 对用户开放了手爪的控制协议（手爪控制协议与末端modbus 功能互斥）
 * @{  
 */
/**
 * @brief 设置手爪行程，即手爪开口的最大值和最小值，设置成功后会自动保存，手爪断电不丢失 
 * 
 * @param handle 机械臂控制句柄 
 * @param min_limit 手爪开口最小值，范围：0~1000，无单位量纲
 * @param max_limit 手爪开口最大值，范围：0~1000，无单位量纲
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_gripper_route(rm_robot_handle *handle, int min_limit, int max_limit);
/**
 * @brief 松开手爪，即手爪以指定的速度运动到开口最大处
 * 
 * @param handle 机械臂控制句柄 
 * @param speed 手爪松开速度，范围 1~1000，无单位量纲
 * @param block true 表示阻塞模式，等待控制器返回夹爪到位指令；false 表示非阻塞模式，不接收夹爪到位指令；
 * @param timeout 阻塞模式：设置等待夹爪到位超时时间，单位：秒
 *              非阻塞模式：0-发送后立即返回；其他值-接收设置成功指令后返回；
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。  
            - -4:超时
 */
RM_INTERFACE_EXPORT int rm_set_gripper_release(rm_robot_handle *handle, int speed, bool block, int timeout);
/**
 * @brief 手爪力控夹取，手爪以设定的速度和力夹取，当夹持力超过设定的力阈值后，停止夹取
 * 
 * @param handle 机械臂控制句柄 
 * @param speed 手爪夹取速度，范围 1~1000，无单位量纲
 * @param force 力控阈值，范围：50~1000，无单位量纲
 * @param block true 表示阻塞模式，等待控制器返回夹爪到位指令；false 表示非阻塞模式，不接收夹爪到位指令；
 * @param timeout 阻塞模式：设置等待夹爪到位超时时间，单位：秒
 *              非阻塞模式：0-发送后立即返回；其他值-接收设置成功指令后返回；
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。  
            - -4:超时
 */
RM_INTERFACE_EXPORT int rm_set_gripper_pick(rm_robot_handle *handle, int speed, int force, bool block, int timeout);
/**
 * @brief 
 * 
 * @param handle 机械臂控制句柄 
 * @param speed 手爪夹取速度，范围 1~1000，无单位量纲
 * @param force 力控阈值，范围：50~1000，无单位量纲
 * @param block true 表示阻塞模式，等待控制器返回夹爪到位指令；false 表示非阻塞模式，不接收夹爪到位指令；
 * @param timeout 阻塞模式：设置等待夹爪到位超时时间，单位：秒
 *              非阻塞模式：0-发送后立即返回；其他值-接收设置成功指令后返回；
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。  
            - -4:超时
 */
RM_INTERFACE_EXPORT int rm_set_gripper_pick_on(rm_robot_handle *handle, int speed, int force, bool block, int timeout);
/**
 * @brief 设置手爪达到指定位置
 * @details 手爪到达指定位置，当当前开口小于指定开口时，手爪以指定速度松开到指定开口位置；当当前开口大于指定开口时，
 * 手爪以指定速度和力矩闭合往指定开口处闭合，当夹持力超过力矩阈值或者达到指定位置后，手爪停止。
 * @param handle 机械臂控制句柄 
 * @param position 手爪开口位置，范围：1~1000，无单位量纲
 * @param block true 表示阻塞模式，等待控制器返回夹爪到位指令；false 表示非阻塞模式，不接收夹爪到位指令；
 * @param timeout 阻塞模式：设置等待夹爪到位超时时间，单位：秒
 *              非阻塞模式：0-发送后立即返回；其他值-接收设置成功指令后返回；
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
            - -4:超时
 */
RM_INTERFACE_EXPORT int rm_set_gripper_position(rm_robot_handle *handle, int position, bool block, int timeout);
/**
 * @brief 查询夹爪状态
 * 
 * @param handle 机械臂控制句柄 
 * @param state 夹爪状态结构体
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 * @attention 此接口默认不更新数据，从首次控制夹爪开始后，使能更新状态，如果此时控制灵巧手或打开末端modbus功能，将不再更新数据。另外夹爪需要支持最新的固件，方可支持此功能
 */
RM_INTERFACE_EXPORT int rm_get_gripper_state(rm_robot_handle *handle, rm_gripper_state_t *state);
/** @} */ // 结束组的定义

/**  
 * @defgroup ForceSensor 末端传感器六维力
 * 
 * 睿尔曼机械臂六维力版末端配备集成式六维力传感器，无需外部走线，用户可直接通过协议对六维力进行操作，
 * 获取六维力数据。如下图所示，正上方为六维力的 Z 轴，航插反方向为六维力的 Y 轴，坐标系符合右手定则。
 * 机械臂位于零位姿态时，工具坐标系与六维力的坐标系方向一致。  
 * 另外，六维力额定力 200N，额定力矩 8Nm，过载水平 300%FS，工作温度 5~80℃，准度 0.5%FS。使用过程中
 * 注意使用要求，防止损坏六维力传感器。
 * @{  
 */
/**
 * @brief 查询当前六维力传感器得到的力和力矩信息：Fx,Fy,Fz,Mx,My,Mz
 * 
 * @param handle 机械臂控制句柄 
 * @param data 力传感器数据结构体
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_force_data(rm_robot_handle *handle, rm_force_data_t *data);
/**
 * @brief 将六维力数据清零，标定当前状态下的零位
 * 
 * @param handle 机械臂控制句柄 
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_clear_force_data(rm_robot_handle *handle);
/**
 * @brief 自动设置六维力重心参数
 * @details 设置六维力重心参数，六维力重新安装后，必须重新计算六维力所受到的初始力和重心。分别在不同姿态下，获取六维力的数据，
 * 用于计算重心位置。该指令下发后，机械臂以固定的速度运动到各标定点。  
 * 以RM65机械臂为例，四个标定点的关节角度分别为：  
 * 位置1关节角度：{0,0,-60,0,60,0}  
 * 位置2关节角度：{0,0,-60,0,-30,0}  
 * 位置3关节角度：{0,0,-60,0,-30,180}  
 * 位置4关节角度：{0,0,-60,0,-120,0}  
 * @attention 必须保证在机械臂静止状态下标定;  
 * 该过程不可中断，中断后必须重新标定。
 * @param handle 机械臂控制句柄 
 * @param block true 表示阻塞模式，等待标定完成后返回；false 表示非阻塞模式，发送后立即返回
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_force_sensor(rm_robot_handle *handle, bool block);
/**
 * @brief 手动标定六维力数据
 * @details 六维力重新安装后，必须重新计算六维力所受到的初始力和重心。该手动标定流程，适用于空间狭窄工作区域，以防自动标定过程中
 * 机械臂发生碰撞，用户可以手动选取四个位置下发，当下发完四个点后，机械臂开始自动沿用户设置的目标运动，并在此过程中计算六维力重心。
 * @attention 上述4个位置必须按照顺序依次下发，当下发完位置4后，机械臂开始自动运行计算重心。
 * @param handle 机械臂控制句柄 
 * @param count 点位；1~4
 * @param joint 关节角度，单位：°
 * @param block true 表示阻塞模式，等待标定完成后返回；false 表示非阻塞模式，发送后立即返回
 * @return int 函数执行的状态码。  
            - 0: 计算成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_manual_set_force(rm_robot_handle *handle, int count, float *joint, bool block);
/**
 * @brief 停止标定力传感器重心
 * @details 在标定力传感器过程中，如果发生意外，发送该指令，停止机械臂运动，退出标定流程
 * @param handle 机械臂控制句柄 
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_stop_set_force_sensor(rm_robot_handle *handle);
/** @} */ // 结束组的定义

/**  
 * @defgroup ForceSensor 末端传感器一维力
 * 
 * 睿尔曼机械臂末端接口板集成了一维力传感器，可获取 Z 方向的力，量程200N，准度 0.5%FS。
 * @{  
 */
/**
 * @brief 查询末端一维力数据
 * 
 * @param handle 机械臂控制句柄 
 * @param data 一维力数据结构体
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
            - -4: 非力控版本机械臂，不支持此功能。
 */
RM_INTERFACE_EXPORT int rm_get_Fz(rm_robot_handle *handle, rm_fz_data_t *data);
/**
 * @brief 清零末端一维力数据，清空一维力数据后，后续所有获取到的数据都是基于当前的偏置。
 * 
 * @param handle 机械臂控制句柄 
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_clear_Fz(rm_robot_handle *handle);
/**
 * @brief 自动标定一维力数据
 * @details 设置一维力重心参数，一维力重新安装后，必须重新计算一维力所受到的初始力和重心。
 * 分别在不同姿态下，获取一维力的数据，用于计算重心位置，该步骤对于基于一维力的力位混合控制操作具有重要意义。
 * @param handle 机械臂控制句柄 
 * @param block true 表示阻塞模式，等待标定完成后返回；false 表示非阻塞模式，发送后立即返回
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_auto_set_Fz(rm_robot_handle *handle, bool block);
/**
 * @brief 手动标定一维力数据
 * @details 设置一维力重心参数，一维力重新安装后，必须重新计算一维力所受到的初始力和重心。该手动标定流程，
 * 适用于空间狭窄工作区域，以防自动标定过程中机械臂发生碰撞，用户可以手动选取2个位置下发，当下发完后，
 * 机械臂开始自动沿用户设置的目标运动，并在此过程中计算一维力重心。
 * @param handle 机械臂控制句柄 
 * @param joint1 位置1关节角度数组，单位：度
 * @param joint2 位置2关节角度数组，单位：度
 * @param block true 表示阻塞模式，等待标定完成后返回；false 表示非阻塞模式，发送后立即返回
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_manual_set_Fz(rm_robot_handle *handle, float *joint1, float *joint2, bool block);
/** @} */ // 结束组的定义

/**  
 * @defgroup DragTeach 拖动示教
 * 
 * 睿尔曼机械臂在拖动示教过程中，可记录拖动的轨迹点，并根据用户的指令对轨迹进行复现。
 * @{  
 */
/**
 * @brief 拖动示教开始
 * 
 * @param handle 机械臂控制句柄 
 * @param trajectory_record 拖动示教时记录轨迹，0-不记录，1-记录轨迹
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_start_drag_teach(rm_robot_handle *handle, int trajectory_record);
/**
 * @brief 拖动示教结束
 * 
 * @param handle 机械臂控制句柄 
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_stop_drag_teach(rm_robot_handle *handle);
/**
 * @brief 开始复合模式拖动示教
 * @attention 仅支持三代控制器，四代控制器使用rm_start_multi_drag_teach_new
 * @param handle 机械臂控制句柄 
 * @param mode 拖动示教模式 0-电流环模式，1-使用末端六维力，只动位置，2-使用末端六维力，只动姿态，3-使用末端六维力，位置和姿态同时动
 * @param singular_wall 仅在六维力模式拖动示教中生效，用于指定是否开启拖动奇异墙，0表示关闭拖动奇异墙，1表示开启拖动奇异墙
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
            - -4: 四代控制器不支持该接口。
 * @note 失败的可能原因:  
            - 当前机械臂非六维力版本（六维力拖动示教）。  
            - 机械臂当前处于 IO 急停状态
            - 机械臂当前处于仿真模式
            - 输入参数有误
            - 使用六维力模式拖动示教时，当前已处于奇异区
 */
RM_INTERFACE_EXPORT int rm_start_multi_drag_teach(rm_robot_handle *handle, int mode, int singular_wall);
/**
 * @brief 开始复合模式拖动示教-新参数
 * 
 * @param handle 机械臂控制句柄 
 * @param teach_state 复合拖动示教参数
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 * @note 失败的可能原因:  
            - 当前机械臂非六维力版本（六维力拖动示教）。  
            - 机械臂当前处于 IO 急停状态
            - 机械臂当前处于仿真模式
            - 输入参数有误
            - 使用六维力模式拖动示教时，当前已处于奇异区
 */
RM_INTERFACE_EXPORT int rm_start_multi_drag_teach(rm_robot_handle *handle, rm_multi_drag_teach_t teach_state);
/**
 * @brief 设置电流环拖动示教灵敏度
 * 
 * @param handle 机械臂控制句柄 
 * @param grade 等级，0到100，表示0~100%，当设置为100时保持初始状态
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_drag_teach_sensitivity(rm_robot_handle *handle, int grade);
/**
 * @brief 获取电流环拖动示教灵敏度
 * 
 * @param handle 机械臂控制句柄 
 * @param grade 等级，0到100，表示0~100%，当设置为100时保持初始状态
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_drag_teach_sensitivity(rm_robot_handle *handle, int *grade);
/**
 * @brief 运动到轨迹起点
 * @details 轨迹复现前，必须控制机械臂运动到轨迹起点，如果设置正确，机械臂将以20%的速度运动到轨迹起点
 * @param handle 机械臂控制句柄 
 * @param block 阻塞设置
 *        - 多线程模式：  
 *            - 0：非阻塞模式，发送指令后立即返回。  
 *            - 1：阻塞模式，等待机械臂到达目标位置或规划失败后返回。  
 *        - 单线程模式：  
 *            - 0：非阻塞模式。  
 *            - 其他值：阻塞模式并设置超时时间，根据运动时间设置，单位为秒。
 * @attention 使用单线程阻塞模式时，请设置超时时间确保轨迹在超时时间内运行结束返回
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
            - -4: 当前到位设备校验失败，即当前到位设备不为关节。 
            - -5: 单线程模式超时未接收到返回，请确保超时时间设置合理。 
 * @see rm_run_drag_trajectory
 */
RM_INTERFACE_EXPORT int rm_drag_trajectory_origin(rm_robot_handle *handle, int block);
/**
 * @brief 轨迹复现开始
 * 
 * @param handle 机械臂控制句柄 
 * @param block 阻塞设置
 *        - 多线程模式：  
 *            - 0：非阻塞模式，发送指令后立即返回。  
 *            - 1：阻塞模式，等待机械臂到达目标位置或规划失败后返回。  
 *        - 单线程模式：  
 *            - 0：非阻塞模式。  
 *            - 其他值：阻塞模式并设置超时时间，根据运动时间设置，单位为秒。
 * @attention 使用单线程阻塞模式时，请设置超时时间确保轨迹在超时时间内运行结束返回
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误，请确保机械臂当前位置为拖动示教起点。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。  
            - -4: 当前到位设备校验失败，即当前到位设备不为关节。 
            - -5: 单线程模式超时未接收到返回，请确保超时时间设置合理。
 * @attention 必须在拖动示教结束后才能使用，同时保证机械臂位于拖动示教的起点位置，可调用rm_drag_trajectory_origin接口运动至起点位置
 * @see rm_drag_trajectory_origin
 */
RM_INTERFACE_EXPORT int rm_run_drag_trajectory(rm_robot_handle *handle, int block);
/**
 * @brief 控制机械臂在轨迹复现过程中的暂停
 * 
 * @param handle 机械臂控制句柄 
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_pause_drag_trajectory(rm_robot_handle *handle);
/**
 * @brief 控制机械臂在轨迹复现过程中暂停之后的继续，
 * 
 * @param handle 机械臂控制句柄 
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_continue_drag_trajectory(rm_robot_handle *handle);
/**
 * @brief 控制机械臂在轨迹复现过程中的停止
 * 
 * @param handle 机械臂控制句柄 
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_stop_drag_trajectory(rm_robot_handle *handle);
/**
 * @brief 保存拖动示教轨迹
 * 
 * @param handle 机械臂控制句柄 
 * @param name 轨迹要保存的文件路径及名称，长度不超过300个字符，例: c:/rm_test.txt
 * @param num 轨迹点数
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_save_trajectory(rm_robot_handle *handle, const char* name, int *num);
/**
 * @brief 力位混合控制
 * @details 在笛卡尔空间轨迹规划时，使用该功能可保证机械臂末端接触力恒定，使用时力的方向与机械臂运动方向不能在同一方向。
 * 开启力位混合控制，执行笛卡尔空间运动，接收到运动完成反馈后，需要等待2S后继续下发下一条运动指令。
 * @param handle 机械臂控制句柄 
 * @param sensor 0-一维力；1-六维力
 * @param mode 0-基坐标系力控；1-工具坐标系力控；
 * @param direction 力控方向；0-沿X轴；1-沿Y轴；2-沿Z轴；3-沿RX姿态方向；4-沿RY姿态方向；5-沿RZ姿态方向
 * @param N 力的大小，单位N
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_force_position(rm_robot_handle *handle, int sensor, int mode, int direction, float N);
/**
 * @brief 力位混合控制-新参数
 * @details 在笛卡尔空间轨迹规划时，使用该功能可保证机械臂末端接触力恒定，使用时力的方向与机械臂运动方向不能在同一方向。
 * 开启力位混合控制，执行笛卡尔空间运动，接收到运动完成反馈后，需要等待2S后继续下发下一条运动指令。
 * @param handle 机械臂控制句柄 
 * @param param 力位混合控制参数
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_force_position(rm_robot_handle *handle, rm_force_position_t param);
/**
 * @brief 结束力位混合控制
 * 
 * @param handle 机械臂控制句柄 
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_stop_force_position(rm_robot_handle *handle);
/**
 * @brief 设置六维力拖动示教模式
 * 
 * @param handle 机械臂控制句柄 
 * @param mode 0表示快速拖动模式 1表示精准拖动模式
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
            - -4: 非六维力版本机械臂，不支持此功能。
 */
RM_INTERFACE_EXPORT int rm_set_force_drag_mode(rm_robot_handle *handle, int mode);
/**
 * @brief 获取六维力拖动示教模式
 * 
 * @param handle 机械臂控制句柄 
 * @param mode 0表示快速拖动模式 1表示精准拖动模式
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。处理建议：1、联系睿尔曼公司技术支持确认控制器版本是否支持此功能；
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
            - -4: 非六维力版本机械臂，不支持此功能。
 */
RM_INTERFACE_EXPORT int rm_get_force_drag_mode(rm_robot_handle *handle, int *mode);
/** @} */ // 结束组的定义

/**  
 * @defgroup HandControl 五指灵巧手
 * 
 * 睿尔曼机械臂末端配置因时的五指灵巧手，可通过协议对灵巧手进行设置。
 * @{  
 */
/**
 * @brief 设置灵巧手目标手势序列号
 * 
 * @param handle 机械臂控制句柄 
 * @param posture_num 预先保存在灵巧手内的手势序号，范围：1~40
 * @param block true 表示阻塞模式，等待灵巧手运动结束后返回；false 表示非阻塞模式，发送后立即返回
 * @param timeout 阻塞模式下超时时间设置，单位：秒
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
            - -4: 超时未返回
 */
RM_INTERFACE_EXPORT int rm_set_hand_posture(rm_robot_handle *handle, int posture_num, bool block, int timeout);
/**
 * @brief 设置灵巧手目标手势序列号
 * 
 * @param handle 机械臂控制句柄 
 * @param seq_num 预先保存在灵巧手内的手势序号，范围：1~40
 * @param block true 表示阻塞模式，等待灵巧手运动结束后返回；false 表示非阻塞模式，发送后立即返回
 * @param timeout 阻塞模式下超时时间设置，单位：秒
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
            - -4: 超时未返回
 */
RM_INTERFACE_EXPORT int rm_set_hand_seq(rm_robot_handle *handle, int seq_num, bool block, int timeout);
/**
 * @brief 设置灵巧手各自由度角度
 * @details 设置灵巧手角度，灵巧手有6个自由度，从1~6分别为小拇指，无名指，中指，食指，大拇指弯曲，大拇指旋转
 * @param handle 机械臂控制句柄 
 * @param hand_angle 手指角度数组，范围：0~1000. 另外，-1代表该自由度不执行任何操作，保持当前状态
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_hand_angle(rm_robot_handle *handle, const int *hand_angle);
/**
 * @brief 设置灵巧手各自由度跟随角度
 * @details 设置灵巧手跟随角度，灵巧手有6个自由度，从1~6分别为小拇指，无名指，中指，食指，大拇指弯曲，大拇指旋转
 * @param handle 机械臂控制句柄 
 * @param hand_angle 手指角度数组，最大表示范围为-32768到+32767，按照灵巧手厂商定义的角度做控制，例如因时的范围为0-2000
 * @param block 设置等待机械臂返回状态超时时间，设置0时为非阻塞模式。单位为毫秒。
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_hand_follow_angle(rm_robot_handle *handle, const int *hand_angle, int block);
/**
 * @brief 灵巧手位置跟随控制
 * @details 设置灵巧手跟随位置，灵巧手有6个自由度，从1~6分别为小拇指，无名指，中指，食指，大拇指弯曲，大拇指旋转，最高50Hz的控制频率
 * @param handle 机械臂控制句柄 
 * @param hand_pos 手指位置数组，最大范围为0-65535，按照灵巧手厂商定义的角度做控制，例如因时的范围为0-1000
 * @param block 设置等待机械臂返回状态超时时间，设置0时为非阻塞模式。单位为毫秒。
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_hand_follow_pos(rm_robot_handle *handle, const int *hand_pos, int block);
/**
 * @brief 设置灵巧手速度
 * 
 * @param handle 机械臂控制句柄 
 * @param speed 手指速度，范围：1~1000
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_hand_speed(rm_robot_handle *handle, int speed);
/**
 * @brief 设置灵巧手力阈值
 * 
 * @param handle 机械臂控制句柄 
 * @param hand_force 手指力，范围：1~1000
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_hand_force(rm_robot_handle *handle, int hand_force);
/** @} */ // 结束组的定义

/**  
 * @defgroup ModbusConfig Modbus 配置
 * 
 * 睿尔曼机械臂在控制器和末端接口板上各提供一个RS485通讯接口，这些接口可通过JSON协议配置为标准的Modbus RTU模式。  
 * 在Modbus RTU模式下，用户可通过JSON协议对连接在端口上的外设进行读写操作。  
 *  
 * @attention 
 * - 控制器的RS485接口在未配置为Modbus RTU模式时，可用于直接控制机械臂。  
 * - Modbus RTU模式与机械臂控制模式不兼容。若需恢复机械臂控制模式，必须关闭该端口的Modbus RTU模式。  
 * - 关闭Modbus RTU模式后，系统将自动切换回机械臂控制模式，使用波特率460800BPS，停止位1，数据位8，无校验。  
 *  
 * 此外，I系列控制器还支持modbus-TCP主站配置，允许用户配置使用modbus-TCP主站，以连接外部设备的modbus-TCP从站。  
 * 
 * @{  
 */
/**
 * @brief 配置通讯端口ModbusRTU模式
 * @details 配置通讯端口ModbusRTU模式，机械臂启动后，要对通讯端口进行任何操作，必须先启动该指令，否则会返回报错信息。
 *          另外，机械臂会对用户的配置方式进行保存，机械臂重启后会自动恢复到用户断电之前配置的模式。
 * @param handle 机械臂控制句柄 
 * @param port 通讯端口，0-控制器RS485端口为RTU主站，1-末端接口板RS485接口为RTU主站，2-控制器RS485端口为RTU从站
 * @param baudrate 波特率，支持 9600,115200,460800 三种常见波特率
 * @param timeout 超时时间，单位百毫秒。。对Modbus设备所有的读写指令，在规定的超时时间内未返回响应数据，则返回超时报错提醒。超时时间不能为0，若设置为0，则机械臂按1进行配置。
 * @note 其他配置默认为：数据位-8，停止位-1，奇偶校验-无
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
            - -4: 四代控制器不支持该接口
 */
RM_INTERFACE_EXPORT int rm_set_modbus_mode(rm_robot_handle *handle, int port, int baudrate, int timeout);
/**
 * @brief 关闭通讯端口 Modbus RTU 模式
 * 
 * @param handle 机械臂控制句柄 
 * @param port 通讯端口，0-控制器RS485端口为RTU主站，1-末端接口板RS485接口为RTU主站，2-控制器RS485端口为RTU从站
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
            - -4: 四代控制器不支持该接口
 */
RM_INTERFACE_EXPORT int rm_close_modbus_mode(rm_robot_handle *handle, int port);
/**
 * @brief 配置连接 ModbusTCP 从站
 * 
 * @param handle 机械臂控制句柄 
 * @param ip 从机IP地址
 * @param port 端口号
 * @param timeout 超时时间，单位毫秒。
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
            - -4: 四代控制器不支持该接口
 */
RM_INTERFACE_EXPORT int rm_set_modbustcp_mode(rm_robot_handle *handle, const char *ip, int port, int timeout);
/**
 * @brief 关闭通讯端口ModbusRTU模式
 * 
 * @param handle 机械臂控制句柄 
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
            - -4: 四代控制器不支持该接口
 */
RM_INTERFACE_EXPORT int rm_close_modbustcp_mode(rm_robot_handle *handle);
/**
 * @brief 读线圈
 * 
 * @param handle 机械臂控制句柄 
 * @param params 线圈读取参数结构体，该指令最多一次性支持读 8 个线圈数据，即返回的数据不会超过一个字节
 * @param data 返回线圈状态，数据类型：int8
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 读取失败，超时时间内未获取到数据。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
            - -4: 四代控制器不支持该接口
 */
RM_INTERFACE_EXPORT int rm_read_coils(rm_robot_handle *handle, rm_peripheral_read_write_params_t params, int *data);
/**
 * @brief 读离散量输入
 * 
 * @param handle 机械臂控制句柄 
 * @param params 离散量输入读取参数结构体，该指令最多一次性支持读 8 个离散量数据，即返回的数据不会超过一个字节
 * @param data 返回离散量，数据类型：int8
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 读取失败，超时时间内未获取到数据。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
            - -4: 四代控制器不支持该接口
 */
RM_INTERFACE_EXPORT int rm_read_input_status(rm_robot_handle *handle, rm_peripheral_read_write_params_t params, int *data);
/**
 * @brief 读保持寄存器
 * 
 * @param handle 机械臂控制句柄 
 * @param params 保持寄存器数据读取参数结构体，该指令每次只能读 1 个寄存器，即 2 个字节
 * 的数据，不可一次性读取多个寄存器数据，该结构体成员num无需设置
 * @param data 返回寄存器数据，数据类型：int16
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 读取失败，超时时间内未获取到数据。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
            - -4: 四代控制器不支持该接口
 */
RM_INTERFACE_EXPORT int rm_read_holding_registers(rm_robot_handle *handle, rm_peripheral_read_write_params_t params, int *data);
/**
 * @brief 读输入寄存器
 * 
 * @param handle 机械臂控制句柄 
 * @param params 输入寄存器数据读取参数结构体，该指令每次只能读 1 个寄存器，即 2 个字节
 * 的数据，不可一次性读取多个寄存器数据，该结构体成员num无需设置
 * @param data 返回寄存器数据，数据类型：int16
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 读取失败，超时时间内未获取到数据。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
            - -4: 四代控制器不支持该接口
 */
RM_INTERFACE_EXPORT int rm_read_input_registers(rm_robot_handle *handle, rm_peripheral_read_write_params_t params, int *data);
/**
 * @brief 写单圈数据
 * 
 * @param handle 机械臂控制句柄 
 * @param params 单圈数据写入参数结构体，该结构体成员num无需设置
 * @param data 要写入线圈的数据，数据类型：int16
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 写操作失败，超时时间内未获取到数据，或者指令内容错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
            - -4: 四代控制器不支持该接口
 */
RM_INTERFACE_EXPORT int rm_write_single_coil(rm_robot_handle *handle, rm_peripheral_read_write_params_t params, int data);
/**
 * @brief 写单个寄存器
 * 
 * @param handle 机械臂控制句柄 
 * @param params 单个寄存器数据写入参数结构体，该结构体成员num无需设置
 * @param data 要写入寄存器的数据，类型：int16
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 写操作失败，超时时间内未获取到数据，或者指令内容错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
            - -4: 四代控制器不支持该接口
 */
RM_INTERFACE_EXPORT int rm_write_single_register(rm_robot_handle *handle, rm_peripheral_read_write_params_t params, int data);
/**
 * @brief 写多个寄存器
 * 
 * @param handle 机械臂控制句柄 
 * @param params 多个寄存器数据写入参数结构体。其中寄存器每次写的数量不超过10个，即该结构体成员num<=10。
 * @param data 要写入寄存器的数据数组，类型：byte。
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 写操作失败，超时时间内未获取到数据，或者指令内容错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
            - -4: 四代控制器不支持该接口
 */
RM_INTERFACE_EXPORT int rm_write_registers(rm_robot_handle *handle, rm_peripheral_read_write_params_t params, int *data);
/**
 * @brief 写多圈数据
 * 
 * @param handle 机械臂控制句柄 
 * @param params 多圈数据写入参数结构体。每次写的数量不超过 160 个，即该结构体成员num<=160。
 * @param data 要写入线圈的数据数组，类型：byte。
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 写操作失败，超时时间内未获取到数据，或者指令内容错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
            - -4: 四代控制器不支持该接口
 */
RM_INTERFACE_EXPORT int rm_write_coils(rm_robot_handle *handle, rm_peripheral_read_write_params_t params, int *data);
/**
 * @brief 读多圈数据
 * 
 * @param handle 机械臂控制句柄 
 * @param params 多圈数据读取参数结构体，要读的线圈的数量 8< num <= 120，该指令最多一次性支持读 120 个线圈数据， 即 15 个 byte
 * @param data 返回线圈状态，数据类型：int8
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 读取失败，超时时间内未获取到数据。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
            - -4: 四代控制器不支持该接口
 */
RM_INTERFACE_EXPORT int rm_read_multiple_coils(rm_robot_handle *handle, rm_peripheral_read_write_params_t params, int *data);
/**
 * @brief 读多个保存寄存器
 * 
 * @param handle 机械臂控制句柄 
 * @param params 多个保存寄存器读取参数结构体，要读的寄存器的数量 2 < num < 13，该指令最多一次性支持读 12 个寄存器数据， 即 24 个 byte
 * @param data 返回寄存器数据，数据类型：int8
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 读取失败，超时时间内未获取到数据。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
            - -4: 四代控制器不支持该接口
 */
RM_INTERFACE_EXPORT int rm_read_multiple_holding_registers(rm_robot_handle *handle, rm_peripheral_read_write_params_t params, int *data);
/**
 * @brief 读多个输入寄存器
 * 
 * @param handle 机械臂控制句柄 
 * @param params 多个输入寄存器读取参数结构体，要读的寄存器的数量 2 < num < 13，该指令最多一次性支持读 12 个寄存器数据， 即 24 个 byte
 * @param data 返回寄存器数据，数据类型：int8
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 读取失败，超时时间内未获取到数据。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
            - -4: 四代控制器不支持该接口
 */
RM_INTERFACE_EXPORT int rm_read_multiple_input_registers(rm_robot_handle *handle, rm_peripheral_read_write_params_t params, int *data);
/** @} */ // 结束组的定义

/**  
 * @defgroup InstallPos 系统安装方式
 * 
 * 睿尔曼机械臂可支持不同形式的安装方式，但是安装方式不同，机器人的动力学模型参数和坐标系的方向也有所差别。
 * @{  
 */
/**
 * @brief 设置安装方式参数
 * 
 * @param handle 机械臂控制句柄 
 * @param x 旋转角，单位 °
 * @param y 俯仰角，单位 °
 * @param z 方位角，单位 °
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_install_pose(rm_robot_handle *handle, float x, float y, float z);
/**
 * @brief 获取安装方式参数
 * 
 * @param handle 机械臂控制句柄 
 * @param x 旋转角(out)，单位 °
 * @param y 俯仰角(out)，单位 °
 * @param z 方位角(out)，单位 °
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_install_pose(rm_robot_handle *handle, float *x, float *y, float *z);
/** @} */ // 结束组的定义

/**  
 * @defgroup ForcePositionControl 透传力位混合控制补偿
 * 
 * 针对睿尔曼带一维力和六维力版本的机械臂，用户除了可直接使用示教器调用底层的力位混合控制模块外，还可以将
 * 自定义的轨迹以周期性透传的形式结合底层的力位混合控制算法进行补偿。
 * 
 * @attention 该功能只适用于一维力传感器和六维力传感器机械臂版本  
 * 
 * 透传效果和周期、轨迹是否平滑有关，周期要求稳定，防止出现较大波动，用户使用该指令时请做好轨迹规划，轨迹规划的
 * 平滑程度决定了机械臂的运行状态。基础系列 WIFI 和网口模式透传周期最快 20ms，USB 和 RS485 模式透传周期最快 10ms。
 * 高速网口的透传周期最快也可到 10ms，不过在使用该高速网口前，需要使用指令打开配置。另外 I 系列有线网口周期最快可达 2ms。
 * @{  
 */
/**
 * @brief 开启透传力位混合控制补偿模式
 * 
 * @param handle 机械臂控制句柄 
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_start_force_position_move(rm_robot_handle *handle);
/**
 * @brief 停止透传力位混合控制补偿模式
 * 
 * @param handle 机械臂控制句柄 
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_stop_force_position_move(rm_robot_handle *handle);
/**
 * @brief 透传力位混合补偿-角度方式
 * 
 * @param handle 机械臂控制句柄 
 * @param joint 目标关节角度
 * @param sensor 所使用传感器类型，0-一维力，1-六维力
 * @param mode 模式，0-沿基坐标系，1-沿工具端坐标系
 * @param dir 力控方向，0~5分别代表X/Y/Z/Rx/Ry/Rz，其中一维力类型时默认方向为Z方向
 * @param force 力的大小 单位N
 * @param follow 是否高跟随
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_force_position_move_joint(rm_robot_handle *handle,const float *joint,int sensor,int mode,int dir,float force, bool follow);
/**
 * @brief 透传力位混合补偿-位姿方式
 * 
 * @param handle 机械臂控制句柄 
 * @param pose 当前坐标系下目标位姿
 * @param sensor 所使用传感器类型，0-一维力，1-六维力
 * @param mode 模式，0-沿基坐标系，1-沿工具端坐标系
 * @param dir 力控方向，0~5分别代表X/Y/Z/Rx/Ry/Rz，其中一维力类型时默认方向为Z方向
 * @param force 力的大小 单位N
 * @param follow 是否高跟随
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_force_position_move_pose(rm_robot_handle *handle, rm_pose_t pose,int sensor,int mode,int dir,float force, bool follow);
/**
 * @brief 透传力位混合补偿-新参数
 * 
 * @param handle 机械臂控制句柄 
 * @param param 透传力位混合补偿参数
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_force_position_move(rm_robot_handle *handle, rm_force_position_move_t param);
/** @} */ // 结束组的定义

/**  
 * @defgroup LiftControl 升降机构
 * 
 * 升降机构速度开环控制、位置闭环控制及状态获取
 * @{  
 */
/**
 * @brief 升降机构速度开环控制
 * 
 * @param handle 机械臂控制句柄 
 * @param speed 速度百分比，-100~100。
        - speed<0：升降机构向下运动
        - speed>0：升降机构向上运动
        - speed=0：升降机构停止运动
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_lift_speed(rm_robot_handle *handle, int speed);
/**
 * @brief 升降机构位置闭环控制
 * 
 * @param handle 机械臂控制句柄 
 * @param speed 速度百分比，1~100
 * @param height 目标高度，单位 mm，范围：0~2600
 * @param block 阻塞设置
 *        - 多线程模式：  
 *            - 0：非阻塞模式，发送指令后立即返回。  
 *            - 1：阻塞模式，等待升降机到达目标位置或规划失败后返回。  
 *        - 单线程模式：  
 *            - 0：非阻塞模式，发送指令后立即返回。  
 *            - 其他值：阻塞模式并设置超时时间，单位为秒。
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
            - -4: 当前到位设备校验失败，即当前到位设备不为升降机构。
            - -5: 单线程模式超时未接收到返回，请确保超时时间设置合理。
 */
RM_INTERFACE_EXPORT int rm_set_lift_height(rm_robot_handle *handle, int speed, int height, int block);
/**
 * @brief 获取升降机构状态
 * 
 * @param handle 机械臂控制句柄 
 * @param state 当前升降机构状态
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_lift_state(rm_robot_handle *handle, rm_expand_state_t *state);
/** @} */ // 结束组的定义

/**  
 * @defgroup ExpandControl 通用扩展关节
 * 
 * 扩展关节速度环控制、位置环控制及状态获取
 * @{  
 */
/**
 * @brief 扩展关节状态获取
 * 
 * @param handle 机械臂控制句柄 
 * @param state 扩展关节状态结构体
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_expand_state(rm_robot_handle *handle, rm_expand_state_t *state);
/**
 * @brief 扩展关节速度环控制
 * 
 * @param handle 机械臂控制句柄 
 * @param speed 速度百分比，-100~100。
        - speed<0：扩展关节反方向运动
        - speed>0：扩展关节正方向运动
        - speed=0：扩展关节停止运动
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_expand_speed(rm_robot_handle *handle, int speed);
/**
 * @brief 扩展关节位置环控制
 * 
 * @param handle 机械臂控制句柄 
 * @param speed 速度百分比，1~100
 * @param pos 扩展关节角度，单位度
 * @param block 阻塞设置
 *        - 多线程模式：  
 *            - 0：非阻塞模式，发送指令后立即返回。  
 *            - 1：阻塞模式，等待升降机到达目标位置或规划失败后返回。  
 *        - 单线程模式：  
 *            - 0：非阻塞模式，发送指令后立即返回。  
 *            - 其他值：阻塞模式并设置超时时间，单位为秒。
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
            - -4: 当前到位设备校验失败，即当前到位设备不为扩展关节。
            - -5: 单线程模式超时未接收到返回，请确保超时时间设置合理。
 */
RM_INTERFACE_EXPORT int rm_set_expand_pos(rm_robot_handle *handle, int speed, int pos, int block);
/** @} */ // 结束组的定义

/**  
 * @defgroup OnlineProgramming 在线编程
 * 
 * 包含在线编程文件下发、在线编程文件管理、全局路点管理等相关功能接口。
 * @{  
 */  
/**
 * @brief 文件下发
 * 
 * @param handle 机械臂控制句柄 
 * @param project 文件下发参数配置结构体
 * @param errline 若运行失败，该参数返回有问题的工程行数，err_line 为 0，则代表校验数据长度不对
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
            - -4: 文件名称校验失败
            - -5: 文件读取失败
            - -6: 四代控制器不支持该接口
 */
RM_INTERFACE_EXPORT int rm_send_project(rm_robot_handle *handle, rm_send_project_t project, int *errline);
/**
 * @brief 轨迹规划中改变速度比例系数
 * 
 * @param handle 机械臂控制句柄
 * @param speed 当前进度条的速度数据
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
 */
RM_INTERFACE_EXPORT int rm_set_plan_speed(rm_robot_handle *handle, int speed);

/**
 * @brief 获取在线编程列表
 * 
 * @param handle 机械臂控制句柄 
 * @param page_num 页码
 * @param page_size 每页大小
 * @param vague_search 模糊搜索
 * @param trajectorys 在线编程程序列表
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_program_trajectory_list(rm_robot_handle *handle, int page_num, int page_size, const char *vague_search,rm_program_trajectorys_t *trajectorys);

/**
 * @brief 开始运行指定编号轨迹
 * 
 * @param handle 机械臂控制句柄 
 * @param id 运行指定的ID，1-100，存在轨迹可运行
 * @param speed 1-100，需要运行轨迹的速度，若设置为0，则按照存储的速度运行
 * @param block 阻塞设置
 *        - 多线程模式：  
 *            - 0：非阻塞模式，发送指令后立即返回。  
 *            - 1：阻塞模式，等待机械臂到达目标位置或规划失败后返回。  
 *        - 单线程模式：  
 *            - 0：非阻塞模式。  
 *            - 其他值：阻塞模式并设置超时时间，单位为秒。
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
            - -4: 运行状态已停止但未接收到运行成功，是否在外部停止了轨迹。
 */ 
RM_INTERFACE_EXPORT int rm_set_program_id_run(rm_robot_handle *handle, int id, int speed, int block);
/**
 * @brief 查询在线编程运行状态
 * 
 * @param handle 机械臂控制句柄 
 * @param run_state 在线编程运行状态结构体
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
            - -4: 四代控制器不支持该接口
 */
RM_INTERFACE_EXPORT int rm_get_program_run_state(rm_robot_handle *handle, rm_program_run_state_t *run_state);
/**
 * @brief 查询流程图运行状态
 * 
 * @param handle 机械臂控制句柄
 * @param run_state 流程图运行状态结构体
 * @return int 函数执行的状态码。
            - 0: 成功。
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
            - -4: 三代控制器不支持该接口
 */
RM_INTERFACE_EXPORT int rm_get_flowchart_program_run_state(rm_robot_handle *handle, rm_flowchart_run_state_t *run_state);
/**
 * @brief 删除指定编号编程文件
 * 
 * @param handle 机械臂控制句柄 
 * @param id 指定编程轨迹的编号
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_delete_program_trajectory(rm_robot_handle *handle, int id);
/**
 * @brief 修改指定编号的编程文件
 * 
 * @param handle 机械臂控制句柄 
 * @param id 指定在线编程轨迹编号
 * @param speed 更新后的规划速度比例 1-100
 * @param name 更新后的文件名称
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_update_program_trajectory(rm_robot_handle *handle, int id, int speed, const char* name);
/**
 * @brief 设置 IO 默认运行编号
 * 
 * @param handle 机械臂控制句柄 
 * @param id 设置 IO 默认运行的在线编程文件编号，支持 0-100，0 代表取消设置
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_default_run_program(rm_robot_handle *handle, int id);
/**
 * @brief 获取 IO 默认运行编号
 * 
 * @param handle 机械臂控制句柄 
 * @param id IO 默认运行的在线编程文件编号，支持 0-100，0 代表无默认
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_default_run_program(rm_robot_handle *handle, int *id);
/**
 * @brief 新增全局路点
 * 
 * @param handle 机械臂控制句柄 
 * @param waypoint 新增全局路点参数（无需输入新增全局路点时间）
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_add_global_waypoint(rm_robot_handle *handle, rm_waypoint_t waypoint);
/**
 * @brief 更新全局路点
 * 
 * @param handle 机械臂控制句柄 
 * @param waypoint 更新全局路点参数（无需输入更新全局路点时间）
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_update_global_waypoint(rm_robot_handle *handle, rm_waypoint_t waypoint);
/**
 * @brief 删除全局路点
 * 
 * @param handle 机械臂控制句柄 
 * @param point_name 全局路点名称
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_delete_global_waypoint(rm_robot_handle *handle, const char* point_name);
/**
 * @brief 查询指定全局路点
 * 
 * @param handle 机械臂控制句柄 
 * @param name 指定全局路点名称
 * @param point 返回指定的全局路点参数
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_given_global_waypoint(rm_robot_handle *handle, const char* name, rm_waypoint_t *point);
/**
 * @brief 查询多个全局路点
 * 
 * @param handle 机械臂控制句柄 
 * @param page_num 页码
 * @param page_size 每页大小
 * @param vague_search 模糊搜索
 * @param point_list 返回的全局路点列表
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_global_waypoints_list(rm_robot_handle *handle, int page_num, int page_size, const char *vague_search,rm_waypoint_list_t *point_list);
/** @} */ // 结束在线编程组的定义

/**  
 * @defgroup UdpConfig UDP主动上报
 *  
 * 睿尔曼机械臂提供 UDP 机械臂状态主动上报接口，使用时，需要和机械臂处于同一局域网络下，通过设置主动上报配置接口的目标 IP或和机械臂建立 TCP 连接，
 * 机械臂即会主动周期性上报机械臂状态数据，数据周期可配置，默认 5ms。配置正确并开启三线程模式后，通过注册回调函数可接收并处理主动上报数据。
 * @{  
 */  
/**
 * @brief 设置 UDP 机械臂状态主动上报配置
 * 
 * @param handle 机械臂控制句柄 
 * @param config UDP配置结构体
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_realtime_push(rm_robot_handle *handle, rm_realtime_push_config_t config);
/**
 * @brief 查询 UDP 机械臂状态主动上报配置
 * 
 * @param handle 机械臂控制句柄 
 * @param config 获取到的UDP机械臂状态主动上报配置
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_realtime_push(rm_robot_handle *handle, rm_realtime_push_config_t *config);
/** @} */ // 结束组的定义

/**  
 * @defgroup Electronic_Fence 电子围栏和虚拟墙
 *  
 * I 系列机械臂具备电子围栏与虚拟墙功能，并提供了针对控制器所保存的电子围栏或虚拟墙几何模型参数的操作接口。
 * 用户可以通过这些接口，实现对电子围栏或虚拟墙的新增、查询、更新和删除操作，在使用中，可以灵活地使用保存在
 * 控制器中的参数配置，需要注意的是，目前控制器支持保存的参数要求不超过10 个。  
 * 
 * <b> 电子围栏</b>  
 * 电子围栏功能通过精确设置参数，确保机械臂的轨迹规划、示教等运动均在设定的电子围栏范围内进行。当机械臂的运动
 * 轨迹可能超出电子围栏的界限时，系统会立即返回相应的错误码，并自动中止运动，从而有效保障机械臂的安全运行。
 * @attention 电子围栏目前仅支持长方体和点面矢量平面这两种形状，并且其仅在仿真模式下生效，为用户提供一个预演轨迹与进行轨迹优化的安全环境。
 * 
 * <b> 虚拟墙</b>  
 * 虚拟墙功能支持在电流环拖动示教与力控拖动示教两种模式下，对拖动范围进行精确限制。在这两种特定的示教模式下，用户可以借助虚拟墙功能，确保
 * 机械臂的拖动操作不会超出预设的范围。
 * @attention 虚拟墙功能目前支持长方体和球体两种形状，并仅在上述两种示教模式下有效。在其他操作模式下，此功能将自动失效。因此，请确保在正确的操作模式
 * 下使用虚拟墙功能，以充分发挥其限制拖动范围的作用。
 * 
 * @{  
 */  
/**
 * @brief 新增几何模型参数
 * 
 * @param handle 机械臂控制句柄 
 * @param config 几何模型参数结构体
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 -4:form设置有误
 */
RM_INTERFACE_EXPORT int rm_add_electronic_fence_config(rm_robot_handle *handle, rm_fence_config_t config);
/**
 * @brief 更新几何模型参数
 * 
 * @param handle 机械臂控制句柄 
 * @param config 几何模型参数结构体
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_update_electronic_fence_config(rm_robot_handle *handle, rm_fence_config_t config);
/**
 * @brief 删除指定几何模型
 * 
 * @param handle 机械臂控制句柄 
 * @param form_name 几何模型名称，不超过 10 个字节，支持字母、数字、下划线
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_delete_electronic_fence_config(rm_robot_handle *handle, const char* form_name);
/**
 * @brief 查询所有几何模型名称
 * 
 * @param handle 机械臂控制句柄 
 * @param names 几何模型名称列表，长度为实际存在几何模型数量
 * @param len 几何模型名称列表长度
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_electronic_fence_list_names(rm_robot_handle *handle, rm_fence_names_t *names, int *len);
/**
 * @brief 查询指定几何模型参数
 * 
 * @param handle 机械臂控制句柄 
 * @param name 指定几何模型名称
 * @param config 返回几何模型参数结构体
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_given_electronic_fence_config(rm_robot_handle *handle, const char* name, rm_fence_config_t *config);
/**
 * @brief 查询所有几何模型参数
 * 
 * @param handle 机械臂控制句柄 
 * @param config_list 几何模型信息列表，长度为实际存在几何模型数量
 * @param len 几何模型信息列表长度
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_electronic_fence_list_infos(rm_robot_handle *handle, rm_fence_config_list_t *config_list, int *len);
/**
 * @brief 设置电子围栏使能状态
 * 
 * @param handle 机械臂控制句柄 
 * @param state 电子围栏使能状态
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 * @note 电子围栏功能通过精确设置参数，确保机械臂的轨迹规划、示教等运动均在设定的电子围栏范围内进行。当机械臂的运动轨迹可能超出电子围栏的界限时，
 * 系统会立即返回相应的错误码，并自动中止运动，从而有效保障机械臂的安全运行。需要注意的是，电子围栏目前仅支持长方体和点面矢量平面这两种形状，并
 * 且其仅在仿真模式下生效，为用户提供一个预演轨迹与进行轨迹优化的安全环境
 */
RM_INTERFACE_EXPORT int rm_set_electronic_fence_enable(rm_robot_handle *handle, rm_electronic_fence_enable_t state);
/**
 * @brief 获取电子围栏使能状态
 * 
 * @param handle 机械臂控制句柄 
 * @param state 电子围栏使能状态
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_electronic_fence_enable(rm_robot_handle *handle,rm_electronic_fence_enable_t *state);
/**
 * @brief 设置当前电子围栏参数配置
 * 
 * @param handle 机械臂控制句柄 
 * @param config 当前电子围栏参数结构体（无需设置电子围栏名称）
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_electronic_fence_config(rm_robot_handle *handle, rm_fence_config_t config);
/**
 * @brief 获取当前电子围栏参数
 * 
 * @param handle 机械臂控制句柄 
 * @param config 返回当前电子围栏参数结构体（返回参数中不包含电子围栏名称）
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_electronic_fence_config(rm_robot_handle *handle, rm_fence_config_t *config);
/**
 * @brief 设置虚拟墙使能状态
 * 
 * @param handle 机械臂控制句柄 
 * @param state 虚拟墙状态结构体
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_virtual_wall_enable(rm_robot_handle *handle, rm_electronic_fence_enable_t state);
/**
 * @brief 获取虚拟墙使能状态
 * 
 * @param handle 机械臂控制句柄 
 * @param state 虚拟墙状态结构体
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_virtual_wall_enable(rm_robot_handle *handle,rm_electronic_fence_enable_t *state);
/**
 * @brief 设置当前虚拟墙参数
 * 
 * @param handle 机械臂控制句柄 
 * @param config 当前虚拟墙参数（无需设置虚拟墙名称）
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_virtual_wall_config(rm_robot_handle *handle, rm_fence_config_t config);
/**
 * @brief 获取当前虚拟墙参数
 * 
 * @param handle 机械臂控制句柄 
 * @param config 当前虚拟墙参数（返回参数中不包含虚拟墙名称）
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_virtual_wall_config(rm_robot_handle *handle, rm_fence_config_t *config);
/** @} */ // 结束电子围栏组的定义

/**  
 * @defgroup SelfCollision 自碰撞安全检测
 *  
 * 睿尔曼机械臂支持自碰撞安全检测，自碰撞安全检测使能状态下，可确保在轨迹规划、示教等运动过程中机械臂的各个部分不会相互碰撞。 
 * @attention 以上自碰撞安全检测功能目前只在仿真模式下生效，用于进行预演轨迹与轨迹优化。 
 * @{  
 */  
/**
 * @brief 设置自碰撞安全检测使能状态
 * 
 * @param handle 机械臂控制句柄 
 * @param state true代表使能，false代表禁使能
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_set_self_collision_enable(rm_robot_handle *handle, bool state);
/**
 * @brief 获取自碰撞安全检测使能状态
 * 
 * @param handle 机械臂控制句柄 
 * @param state true代表使能，false代表禁使能
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 */
RM_INTERFACE_EXPORT int rm_get_self_collision_enable(rm_robot_handle *handle,bool *state);
/** @} */ // 结束组的定义

/**
 * @brief 设置末端生态协议模式
 * @param handle 机械臂控制句柄
 * @param mode 末端生态协议模式
 *            0：禁用协议 
 *            9600：开启协议（波特率9600）
 *            115200：开启协议（波特率115200）
 *            256000：开启协议（波特率256000）
 *            460800：开启协议（波特率460800）
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
*/
RM_INTERFACE_EXPORT int rm_set_rm_plus_mode(rm_robot_handle *handle, int mode);

/**
 * @brief 查询末端生态协议模式
 * @param handle 机械臂控制句柄
 * @param mode 末端生态协议模式
 *            0：禁用协议 
 *            9600：开启协议（波特率9600）
 *            115200：开启协议（波特率115200）
 *            256000：开启协议（波特率256000）
 *            460800：开启协议（波特率460800）
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
 *            
*/
RM_INTERFACE_EXPORT int rm_get_rm_plus_mode(rm_robot_handle *handle, int *mode);

/**
 * @brief 设置触觉传感器模式(末端生态协议支持)
 * @param handle 机械臂控制句柄
 * @param mode 触觉传感器开关状态 0：关闭触觉传感器 1：打开触觉传感器（返回处理后数据） 2：打开触觉传感器（返回原始数据）
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
*/
RM_INTERFACE_EXPORT int rm_set_rm_plus_touch(rm_robot_handle *handle, int mode);

/**
 * @brief 查询触觉传感器模式(末端生态协议支持)
 * @param handle 机械臂控制句柄
 * @param mode 触觉传感器开关状态 0：关闭触觉传感器 1：打开触觉传感器（返回处理后数据） 2：打开触觉传感器（返回原始数据）
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
*/
RM_INTERFACE_EXPORT int rm_get_rm_plus_touch(rm_robot_handle *handle, int *mode);

/**
 * @brief 读取末端设备基础信息(末端生态协议支持)
 * @param handle 机械臂控制句柄
 * @param info 末端设备基础信息
 * @return int 函数执行的状态码。  
            - 0: 成功。  
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。  
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。  
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。 
*/
RM_INTERFACE_EXPORT int rm_get_rm_plus_base_info(rm_robot_handle *handle, rm_plus_base_info_t *info);

/**
 * @brief 读取末端设备实时信息(末端生态协议支持)
 * @param handle 机械臂控制句柄
 * @param info 末端设备实时信息
 * @return int 函数执行的状态码。
            - 0: 成功。
            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。
            - -1: 数据发送失败，通信过程中出现问题。
            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。
            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
*/
RM_INTERFACE_EXPORT int rm_get_rm_plus_state_info(rm_robot_handle *handle, rm_plus_state_info_t *info);



/******************************************算法接口*******************************************************/
/**  
 * @defgroup Algo 算法接口
 *  
 * 针对睿尔曼机械臂，提供正逆解、各种位姿参数转换等工具接口。
 * @{  
 */  
/**
 * @brief 查询算法库版本号
 * @return char* 返回版本号
 */
RM_INTERFACE_EXPORT char* rm_algo_version(void);
/**
 * @brief 初始化算法依赖数据(不连接机械臂时调用)
 * 
 * @param Mode 机械臂型号
 * @param Type 传感器型号
 */
RM_INTERFACE_EXPORT void rm_algo_init_sys_data(rm_robot_arm_model_e Mode, rm_force_type_e Type);
/**
 * @brief 设置安装角度
 * 
 * @param x X轴安装角度 单位°
 * @param y Y轴安装角度 单位°
 * @param z z轴安装角度 单位°
 */
RM_INTERFACE_EXPORT void rm_algo_set_angle(float x, float y, float z);
/**
 * @brief 获取安装角度
 * 
 * @param x X轴安装角度 单位°
 * @param y Y轴安装角度 单位°
 * @param z z轴安装角度 单位°
 */
RM_INTERFACE_EXPORT void rm_algo_get_angle(float* x, float* y, float* z);
/**
 * @brief 设置工作坐标系
 * 
 * @param coord_work 坐标系数据
 */
RM_INTERFACE_EXPORT void rm_algo_set_workframe(const rm_frame_t* const coord_work);
/**
 * @brief 获取当前工作坐标系
 * 
 * @param coord_work 当前工作坐标系
 */
RM_INTERFACE_EXPORT void rm_algo_get_curr_workframe(rm_frame_t* coord_work);
/**
 * @brief 设置工具坐标系
 * 
 * @param coord_tool 坐标系数据
 */
RM_INTERFACE_EXPORT void rm_algo_set_toolframe(const rm_frame_t* const coord_tool);
/**
 * @brief 获取当前工具坐标系
 * 
 * @param coord_tool 当前工具坐标系
 */
RM_INTERFACE_EXPORT void rm_algo_get_curr_toolframe(rm_frame_t* coord_tool);
/**
 * @brief 设置关节最大限位
 * 
 * @param joint_limit 单位°
 */
RM_INTERFACE_EXPORT void rm_algo_set_joint_max_limit(const float* const joint_limit);
/**
 * @brief 获取关节最大限位
 * 
 * @param joint_limit 返回关节最大限位
 */
RM_INTERFACE_EXPORT void rm_algo_get_joint_max_limit(float* joint_limit);
/**
 * @brief 设置关节最小限位
 * 
 * @param joint_limit 单位°
 */
RM_INTERFACE_EXPORT void rm_algo_set_joint_min_limit(const float* const joint_limit);
/**
 * @brief 获取关节最小限位
 * 
 * @param joint_limit 返回关节最小限位
 */
RM_INTERFACE_EXPORT void rm_algo_get_joint_min_limit(float* joint_limit);
/**
 * @brief 设置关节最大速度
 * 
 * @param joint_slim_max RPM
 */
RM_INTERFACE_EXPORT void rm_algo_set_joint_max_speed(const float* const joint_slim_max);
/**
 * @brief 获取关节最大速度
 * 
 * @param joint_slim_max 返回关节最大速度
 */
RM_INTERFACE_EXPORT void rm_algo_get_joint_max_speed(float* joint_slim_max);
/**
 * @brief 设置关节最大加速度
 * 
 * @param joint_alim_max RPM/s
 */
RM_INTERFACE_EXPORT void rm_algo_set_joint_max_acc(const float* const joint_alim_max);
/**
 * @brief 获取关节最大加速度
 * 
 * @param joint_alim_max 返回关节最大加速度
 */
RM_INTERFACE_EXPORT void rm_algo_get_joint_max_acc(float* joint_alim_max);
/**
 * @brief 设置逆解求解模式
 * 
 * @param mode true：遍历模式，冗余参数遍历的求解策略。适于当前位姿跟要求解的位姿差别特别大的应用场景，如MOVJ_P、位姿编辑等，耗时较长
              false：单步模式，自动调整冗余参数的求解策略。适于当前位姿跟要求解的位姿差别特别小、连续周期控制的场景，如笛卡尔空间规划的位姿求解等，耗时短
 */
RM_INTERFACE_EXPORT void rm_algo_set_redundant_parameter_traversal_mode(bool mode);
/**
 * @brief 逆解函数，默认遍历模式，可使用Algo_Set_Redundant_Parameter_Traversal_Mode接口设置逆解求解模式
 * 
 * @param handle 机械臂控制句柄 
 * @param params 逆解输入参数结构体
 * @param q_out 输出的关节角度 单位°
 * @return int 逆解结果
 *            - 0: 逆解成功
 *            - 1: 逆解失败
 *            - -1: 上一时刻关节角度输入为空
 *            - -2: 目标位姿四元数不合法
 * @attention 机械臂已连接时，可直接调用该接口进行计算，计算使用的参数均为机械臂当前的参数；  
 * 未连接机械臂时，需首先调用初始化算法依赖数据接口，并按照实际需求设置使用的坐标系及关节速度位置等限制
 *（不设置则按照出厂默认的参数进行计算），此时机械臂控制句柄设置为NULL即可
 */
RM_INTERFACE_EXPORT int rm_algo_inverse_kinematics(rm_robot_handle *handle, rm_inverse_kinematics_params_t params, float *q_out);

/**
 * @brief 计算逆运动学全解(当前仅支持六自由度机器人)
 * @param handle 机械臂控制句柄，连接机械臂时传入机械臂控制句柄，不连接时传入NULL 
 * @param params 逆解输入参数结构体
 * @return rm_inverse_kinematics_all_solve_t 逆解的全解结构体
*/
RM_INTERFACE_EXPORT rm_inverse_kinematics_all_solve_t rm_algo_inverse_kinematics_all(rm_robot_handle *handle, rm_inverse_kinematics_params_t params);

/**
 * @brief 从多解中选取最优解(当前仅支持六自由度机器人)
 * @param weight 权重,建议默认值为{1,1,1,1,1,1}
 * @param params 待选解的全解结构体
 * @return int 最优解索引，选解结果为ik_solve.q_solve[i],如果没有合适的解返回-1（比如求出8组解，但是8组都有关节角度超限位，那么就返回-1）
*/
RM_INTERFACE_EXPORT int rm_algo_ikine_select_ik_solve(float *weight, rm_inverse_kinematics_all_solve_t params);


/**
 * @brief 检查逆解结果是否超出关节限位(当前仅支持六自由度机器人)
 * @param q_solve 选解，单位：°
 * @return int 0:未超限位,1~dof: 第i个关节超限位,-1：当前机器人非六自由度，当前仅支持六自由度机器人
*/
RM_INTERFACE_EXPORT int rm_algo_ikine_check_joint_position_limit(const float* const q_solve);

/**
 * @brief 检查逆解结果是否超速(当前仅支持六自由度机器人)
 * @param dt 两帧数据之间的时间间隔，即控制周期，单位sec
 * @param q_ref 参考关节角度或者第一帧数据角度，单位：°
 * @param q_solve 求解结果，即下一帧要发送的角度
 * @return int 0:表示未超限 i:表示关节i超限，优先报序号小的关节 -1：当前机器人非六自由度，当前仅支持六自由度机器人
*/
RM_INTERFACE_EXPORT int rm_algo_ikine_check_joint_velocity_limit(float dt, const float* const q_ref, const float* const q_solve);

/**
 * @brief 根据参考位形计算臂角大小（仅支持RM75）
 * @param q_ref 当前参考位形的关节角度，单位°
 * @param arm_angle 计算结果，当前参考位形对应的臂角大小，单位°
 * @return int 
 *       0: 求解成功
 *      -1: 求解失败，或机型非RM75
 *      -2: q_ref 输入参数非法
 */
RM_INTERFACE_EXPORT int rm_algo_calculate_arm_angle_from_config_rm75(float *q_ref, float *arm_angle);

/**
 * @brief 臂角法求解RM75逆运动学
 * @param params rm_inverse_kinematics_params_t，逆解参数结构体
 * @param arm_angle 指定轴角大小，单位:°
 * @param q_solve   求解结果，单位:°
 * @return int 
 *               0: 求解成功
 *              -1: 求解失败
 *              -2: 求解结果超出限位
 *              -3: 机型非RM75
 */
RM_INTERFACE_EXPORT int rm_algo_inverse_kinematics_rm75_for_arm_angle(rm_inverse_kinematics_params_t params, float arm_angle, float *q_solve);



/**
 * @brief 通过分析雅可比矩阵最小奇异值, 判断机器人是否处于奇异状态
 * @param q 要判断的关节角度（机械零位描述），单位：°
 * @param 最小奇异值阈值，若传NULL，则使用内部默认值，默认值为0.01（该值在0-1之间）
 * @return    0:在当前阈值条件下正常
 *            -1:表示在当前阈值条件下判断为奇异区
 *            -2:表示计算失败
*/
RM_INTERFACE_EXPORT int rm_algo_universal_singularity_analyse(const float* const q, float singluar_value_limit);
/**
 * @brief 恢复初始阈值(仅适用于解析法分析机器人奇异状态)，阈值初始化为：limit_qe=10deg,limit_qw=10deg,limit_d = 0.05m      
*/
RM_INTERFACE_EXPORT void rm_algo_kin_singularity_thresholds_init();

/**
 * @brief 设置自定义阈值(仅适用于解析法分析机器人奇异状态)
 * @param limit_qe  肘部奇异区域范围设置(即J3接近0的范围),unit：°,default: about 10deg
 * @param limit_qw  腕部奇异区域范围设置(即J5接近0的范围),unit：°,default: about 10deg
 * @param limit_d 肩部奇异区域范围设置(即腕部中心点距离奇异平面的距离), unit: m, default: 0.05
*/
RM_INTERFACE_EXPORT void rm_algo_kin_set_singularity_thresholds(float limit_qe_algo, float limit_qw_algo, float limit_d_algo);

/**
 * @brief 获取自定义阈值(仅适用于解析法分析机器人奇异状态)
 * 
 * @param limit_qe  肘部奇异区域范围获取(即J3接近0的范围), unit: °, default: about 10deg
 * @param limit_qw  腕部奇异区域范围获取(即J5接近0的范围), unit: °, default: about 10deg
 * @param limit_d   肩部奇异区域范围获取(即腕部中心点距离奇异平面的距离), unit: m, default: 0.05
 */
RM_INTERFACE_EXPORT void rm_algo_kin_get_singularity_thresholds(float* limit_qe_algo, float* limit_qw_algo, float* limit_d_algo);

/**
 * @brief 解析法判断机器人是否处于奇异位形（仅支持六自由度）
 * @param q 要判断的关节角度,单位：°
 * @param distance 输出参数，返回腕部中心点到肩部奇异平面的距离，该值越接近0说明越接近肩部奇异,单位m,不需要时可传NULL
 * @return 0表正常，-1表肩部奇异，-2表肘部奇异，-3表腕部奇异，-4仅支持6自由度机械臂
*/
RM_INTERFACE_EXPORT int rm_algo_kin_robot_singularity_analyse(const float* const q, float *distance);
/**
 * @brief 设置工具包络球参数
 * @param toolSphere_i 工具包络球编号 (0~4)
 * @param data 工具包络球参数,注意其参数在末端法兰坐标系下描述
 */
RM_INTERFACE_EXPORT void rm_algo_set_tool_envelope(const int toolSphere_i, rm_tool_sphere_t data);

/**
 * @brief 获取工具包络球参数
 * @param toolSphere_i 工具rm_get_tool_voltage包络球编号 (0~4)
 * @param data 工具包络球参数,注意其参数在末端法兰坐标系下描述
 */
RM_INTERFACE_EXPORT void rm_algo_get_tool_envelope(const int toolSphere_i, rm_tool_sphere_t *data);
/**
 * @brief 自碰撞检测
 * @param joint 要判断的关节角度，单位：°
 * @return int 
 * 		0: 无碰撞
 * 		1: 发生碰撞,超出关节限位将被认为发生碰撞
 */
RM_INTERFACE_EXPORT int rm_algo_safety_robot_self_collision_detection(float *joint);
/**
 * @brief 机械臂正解函数
 *
 * @param handle 机械臂控制句柄 
 * @param joint 
 * 
 * @return rm_pose_t 目标位姿 
 * @attention 机械臂已连接时，可直接调用该接口进行计算，计算使用的参数均为机械臂当前的参数；  
 * 未连接机械臂时，需首先调用初始化算法依赖数据接口，并按照实际需求设置使用的坐标系及关节速度位置等限制
 *（不设置则按照出厂默认的参数进行计算），此时机械臂控制句柄设置为NULL即可
 */
RM_INTERFACE_EXPORT rm_pose_t rm_algo_forward_kinematics(rm_robot_handle *handle,const float* const joint);
/**
 * @brief 欧拉角转四元数
 * 
 * @param eu 欧拉角 
 * @return rm_quat_t 四元数
 */
RM_INTERFACE_EXPORT rm_quat_t rm_algo_euler2quaternion(rm_euler_t eu);
/**
 * @brief 四元数转欧拉角
 * 
 * @param qua 四元数
 * @return rm_euler_t 欧拉角
 */
RM_INTERFACE_EXPORT rm_euler_t rm_algo_quaternion2euler(rm_quat_t qua);
/**
 * @brief 欧拉角转旋转矩阵
 * 
 * @param state 欧拉角
 * @return rm_matrix_t 旋转矩阵
 */
RM_INTERFACE_EXPORT rm_matrix_t rm_algo_euler2matrix(rm_euler_t state);
/**
 * @brief 位姿转旋转矩阵
 * 
 * @param state 位姿
 * @return rm_matrix_t 旋转矩阵
 */
RM_INTERFACE_EXPORT rm_matrix_t rm_algo_pos2matrix(rm_pose_t state);
/**
 * @brief 旋转矩阵转位姿
 * 
 * @param matrix 旋转矩阵
 * @return rm_pose_t 位姿
 */
RM_INTERFACE_EXPORT rm_pose_t rm_algo_matrix2pos(rm_matrix_t matrix);
/**
 * @brief 基坐标系转工作坐标系
 * 
 * @param matrix 工作坐标系在基坐标系下的矩阵
 * @param state 工具端坐标在基坐标系下位姿
 * @return rm_pose_t 基坐标系在工作坐标系下的位姿
 */
RM_INTERFACE_EXPORT rm_pose_t rm_algo_base2workframe(rm_matrix_t matrix, rm_pose_t state);
/**
 * @brief 工作坐标系转基坐标系
 * 
 * @param matrix 工作坐标系在基坐标系下的矩阵
 * @param state 工具端坐标在工作坐标系下位姿
 * @return rm_pose_t 工作坐标系在基坐标系下的位姿
 */
RM_INTERFACE_EXPORT rm_pose_t rm_algo_workframe2base(rm_matrix_t matrix, rm_pose_t state);
/**
 * @brief 计算环绕运动位姿
 * 
 * @param handle 机械臂控制句柄 
 * @param curr_joint 当前关节角度 单位°
 * @param rotate_axis 旋转轴: 1:x轴, 2:y轴, 3:z轴
 * @param rotate_angle 旋转角度: 旋转角度, 单位(度)
 * @param choose_axis 指定计算时使用的坐标系
 * @return rm_pose_t 计算位姿结果
 */
RM_INTERFACE_EXPORT rm_pose_t rm_algo_rotate_move(rm_robot_handle *handle,const float* const curr_joint, int rotate_axis, float rotate_angle, rm_pose_t choose_axis);
/**
 * @brief 计算沿工具坐标系运动位姿
 * 
 * @param handle 机械臂控制句柄
 * @param curr_joint 当前关节角度，单位：度
 * @param move_lengthx 沿X轴移动长度，单位：米
 * @param move_lengthy 沿Y轴移动长度，单位：米
 * @param move_lengthz 沿Z轴移动长度，单位：米
 * @return rm_pose_t 工作坐标系下的位姿
 */
RM_INTERFACE_EXPORT rm_pose_t rm_algo_cartesian_tool(rm_robot_handle *handle,const float* const curr_joint, float move_lengthx,
                         float move_lengthy, float move_lengthz);
                         /**
 * @brief 计算Pos和Rot沿某坐标系有一定的位移和旋转角度后，所得到的位姿数据
 * 
 * @param handle 机械臂控制句柄，连接机械臂时传入机械臂控制句柄，不连接时传入NULL
 * @param poseCurrent 当前时刻位姿（欧拉角形式）
 * @param deltaPosAndRot 移动及旋转数组，位置移动（单位：m），旋转（单位：度）
 * @param frameMode 坐标系模式选择 0:Work（work即可任意设置坐标系），1:Tool
 * @return rm_pose_t 平移旋转后的位姿
 */
RM_INTERFACE_EXPORT rm_pose_t rm_algo_pose_move(rm_robot_handle *handle,rm_pose_t poseCurrent, const float *deltaPosAndRot, int frameMode);
/**
 * @brief 末端位姿转成工具位姿
 * 
 * @param handle 机械臂控制句柄 
 * @param eu_end 基于世界坐标系和默认工具坐标系的末端位姿
 * @return rm_pose_t 基于工作坐标系和工具坐标系的末端位姿
 */
RM_INTERFACE_EXPORT rm_pose_t rm_algo_end2tool(rm_robot_handle *handle,rm_pose_t eu_end);
/**
 * @brief 工具位姿转末端位姿
 * 
 * @param handle 机械臂控制句柄 
 * @param eu_tool 基于工作坐标系和工具坐标系的末端位姿
 * @return rm_pose_t 基于世界坐标系和默认工具坐标系的末端位姿
 */
RM_INTERFACE_EXPORT rm_pose_t rm_algo_tool2end(rm_robot_handle *handle,rm_pose_t eu_tool);
/**
 * @brief 获取DH参数
 * 
 * @return rm_DH_t DH参数
 */
RM_INTERFACE_EXPORT rm_dh_t rm_algo_get_dh();
/**
 * @brief 设置DH参数
 *
 * @param dh DH参数
 */
RM_INTERFACE_EXPORT void rm_algo_set_dh(rm_dh_t dh);
/** @} */ // 结束算法组的定义

/*********************************************四代控制器新增接口*******************************************************/

/**
 * @brief 查询轨迹列表
 * @param handle 机械臂控制句柄
 * @param page_num 页码
 * @param page_size 每页大小
 * @param vague_search 模糊搜索
 * @param list 轨迹列表
 * @return int 函数执行的状态码。
 *            - 0: 成功。
 *            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。
 *            - -1: 数据发送失败，通信过程中出现问题。
 *            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。
 *            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
 *            - -4: 三代控制器不支持该接口
 */
RM_INTERFACE_EXPORT int rm_get_trajectory_file_list(rm_robot_handle *handle, int page_num, int page_size, const char *vague_search,rm_trajectory_list_t *trajectory_list);
/**
 * @brief 开始运行指定轨迹
 * @param handle 机械臂控制句柄
 * @param trajectory_name 轨迹名称
 * @return int 函数执行的状态码。
 *            - 0: 成功。
 *            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。
 *            - -1: 数据发送失败，通信过程中出现问题。
 *            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。
 *            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
 *            - -4: 三代控制器不支持该接口
 */
RM_INTERFACE_EXPORT int rm_set_run_trajectory(rm_robot_handle *handle, const char *trajectory_name);
/**
 * @brief 删除指定轨迹
 * @param handle 机械臂控制句柄
 * @param trajectory_name 轨迹名称
 * @return int 函数执行的状态码。
 *            - 0: 成功。
 *            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。
 *            - -1: 数据发送失败，通信过程中出现问题。
 *            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。
 *            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
 *            - -4: 三代控制器不支持该接口
 */
RM_INTERFACE_EXPORT int rm_delete_trajectory_file(rm_robot_handle *handle, const char *trajectory_name);
/**
 * @brief 保存轨迹到控制机器
 * @param handle 机械臂控制句柄
 * @param trajectory_name 轨迹名称
 * @return int 函数执行的状态码。
 *            - 0: 成功。
 *            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。
 *            - -1: 数据发送失败，通信过程中出现问题。
 *            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。
 *            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
 *            - -4: 三代控制器不支持该接口
 */
RM_INTERFACE_EXPORT int rm_save_trajectory_file(rm_robot_handle *handle, const char *trajectory_name);

/**
 * @brief 设置机械臂急停状态
 * @param handle 机械臂控制句柄
 * @param state 急停状态，true：急停，false：恢复
 * @return int 函数执行的状态码。
 *            - 0: 成功。
 *            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。
 *            - -1: 数据发送失败，通信过程中出现问题。
 *            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。
 *            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
 *            - -4: 三代控制器不支持该接口
 */
RM_INTERFACE_EXPORT int rm_set_arm_emergency_stop(rm_robot_handle *handle, bool state);
/**
 * @brief 新增Modbus TCP主站
 * @param handle 机械臂控制句柄
 * @param master Modbus TCP主站信息
 * @return int 函数执行的状态码。
 *            - 0: 成功。
 *            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。
 *            - -1: 数据发送失败，通信过程中出现问题。
 *            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。
 *            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
 *            - -4: 三代控制器不支持该接口
 */
RM_INTERFACE_EXPORT int rm_add_modbus_tcp_master(rm_robot_handle *handle, rm_modbus_tcp_master_info_t master);
/**
 * @brief 修改Modbus TCP主站
 * @param handle 机械臂控制句柄
 * @param master_name Modbus TCP主站名称
 * @param master 要修改的Modbus TCP主站信息
 * @return int 函数执行的状态码。
 *            - 0: 成功。
 *            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。
 *            - -1: 数据发送失败，通信过程中出现问题。
 *            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。
 *            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
 *            - -4: 三代控制器不支持该接口
 */
RM_INTERFACE_EXPORT int rm_update_modbus_tcp_master(rm_robot_handle *handle, const char *master_name, rm_modbus_tcp_master_info_t master);
/**
 * @brief 删除Modbus TCP主站
 * @param handle 机械臂控制句柄
 * @param master_name Modbus TCP主站名称
 * @return int 函数执行的状态码。
 *            - 0: 成功。
 *            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。
 *            - -1: 数据发送失败，通信过程中出现问题。
 *            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。
 *            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
 *            - -4: 三代控制器不支持该接口
 */
RM_INTERFACE_EXPORT int rm_delete_modbus_tcp_master(rm_robot_handle *handle, const char *master_name);
/**
 * @brief 查询Modbus TCP主站
 * @param handle 机械臂控制句柄
 * @param master_name Modbus TCP主站名称
 * @param master Modbus TCP主站信息
 * @return int 函数执行的状态码。
 *            - 0: 成功。
 *            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。
 *            - -1: 数据发送失败，通信过程中出现问题。
 *            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。
 *            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
 *            - -4: 三代控制器不支持该接口
 */
RM_INTERFACE_EXPORT int rm_get_modbus_tcp_master(rm_robot_handle *handle, const char *master_name, rm_modbus_tcp_master_info_t *master);
/**
 * @brief 查询TCP主站列表
 * @param handle 机械臂控制句柄
 * @param page_num 页码
 * @param page_size 每页大小
 * @param vague_search 模糊搜索
 * @param list TCP主站列表
 * @return int 函数执行的状态码。
 *            - 0: 成功。
 *            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。
 *            - -1: 数据发送失败，通信过程中出现问题。
 *            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。
 *            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
 *            - -4: 三代控制器不支持该接口
 */
RM_INTERFACE_EXPORT int rm_get_modbus_tcp_master_list(rm_robot_handle *handle, int page_num, int page_size, const char *vague_search,rm_modbus_tcp_master_list_t *list);
/**
 * @brief 设置控制器RS485模式(四代控制器支持)
 * @param handle 机械臂控制句柄
 * @param controller_rs485_mode 0代表默认RS485串行通讯，1代表modbus-RTU主站模式，2-代表modbus-RTU从站模式。
 * @param baudrate 波特率(当前支持9600 19200 38400 57600 115200 230400 460800)
 * @return int 函数执行的状态码。
 *            - 0: 成功。
 *            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。
 *            - -1: 数据发送失败，通信过程中出现问题。
 *            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。
 *            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
 *            - -4: 三代控制器不支持该接口
 */
RM_INTERFACE_EXPORT int rm_set_controller_rs485_mode(rm_robot_handle *handle, int controller_rs485_mode, int baudrate);
/**
 * @brief 查询控制器RS485模式(四代控制器支持)
 * @param handle 机械臂控制句柄
 * @param controller_rs485_mode 0代表默认RS485串行通讯，1代表modbus-RTU主站模式，2-代表modbus-RTU从站模式。
 * @param baudrate 波特率(当前支持9600 19200 38400 57600 115200 230400 460800)
 * @return int 函数执行的状态码。
 *            - 0: 成功。
 *            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。
 *            - -1: 数据发送失败，通信过程中出现问题。
 *            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。
 *            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
 *            - -4: 三代控制器不支持该接口
 */
RM_INTERFACE_EXPORT int rm_get_controller_rs485_mode_v4(rm_robot_handle *handle, int *controller_rs485_mode, int *baudrate);
/**
 * @brief 设置工具端RS485模式(四代控制器支持)
 * @param handle 机械臂控制句柄
 * @param mode 通讯端口，0-设置工具端RS485端口为RTU主站，1-设置工具端RS485端口为灵巧手模式，2-设置工具端RS485端口为夹爪模式。
 * @param baudrate 波特率(当前支持9600,115200,460800)
 * @return int 函数执行的状态码。
 *            - 0: 成功。
 *            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。
 *            - -1: 数据发送失败，通信过程中出现问题。
 *            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。
 *            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
 *            - -4: 三代控制器不支持该接口
 */
RM_INTERFACE_EXPORT int rm_set_tool_rs485_mode(rm_robot_handle *handle, int mode, int baudrate);
/**
 * @brief 查询工具端RS485模式(四代控制器支持)
 * @param handle 机械臂控制句柄
 * @param tool_rs485_mode 0-代表modbus-RTU主站模式，1-代表灵巧手模式，2-代表夹爪模式。
 * @param baudrate 波特率(当前支持9600,115200,460800)
 * @return int 函数执行的状态码。
 *            - 0: 成功。
 *            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。
 *            - -1: 数据发送失败，通信过程中出现问题。
 *            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。
 *            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
 *            - -4: 三代控制器不支持该接口
 */
RM_INTERFACE_EXPORT int rm_get_tool_rs485_mode_v4(rm_robot_handle *handle, int *tool_rs485_mode, int *baudrate);
/**
 * @brief Modbus RTU协议读线圈
 * @param handle 机械臂控制句柄
 * @param param 读线圈参数
 * @param data 读线圈数据，数组大小为param.num
 * @return int 函数执行的状态码。
 *            - 0: 成功。
 *            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。
 *            - -1: 数据发送失败，通信过程中出现问题。
 *            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。
 *            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
 *            - -4: 三代控制器不支持该接口
 */
RM_INTERFACE_EXPORT int rm_read_modbus_rtu_coils(rm_robot_handle *handle, rm_modbus_rtu_read_params_t param, int *data);
/**
 * @brief Modbus RTU协议写线圈
 * @param handle 机械臂控制句柄
 * @param param 写线圈参数
 * @return int 函数执行的状态码。
 *            - 0: 成功。
 *            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。
 *            - -1: 数据发送失败，通信过程中出现问题。  
 *            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。
 *            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
 *            - -4: 三代控制器不支持该接口
 */
RM_INTERFACE_EXPORT int rm_write_modbus_rtu_coils(rm_robot_handle *handle, rm_modbus_rtu_write_params_t param);
/**
 * @brief Modbus RTU协议读离散量输入
 * @param handle 机械臂控制句柄
 * @param param 读离散输入参数
 * @param data 读离散输入数据，数组大小为param.num
 * @return int 函数执行的状态码。
 *            - 0: 成功。
 *            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。
 *            - -1: 数据发送失败，通信过程中出现问题。
 *            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。
 *            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
 *            - -4: 三代控制器不支持该接口
 */
RM_INTERFACE_EXPORT int rm_read_modbus_rtu_input_status(rm_robot_handle *handle, rm_modbus_rtu_read_params_t param, int *data);
/**
 * @brief Modbus RTU协议读保持寄存器
 * @param handle 机械臂控制句柄
 * @param param 读保持寄存器参数
 * @param data 读保持寄存器数据，数组大小为param.num
 * @return int 函数执行的状态码。
 *            - 0: 成功。
 *            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。
 *            - -1: 数据发送失败，通信过程中出现问题。
 *            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。
 *            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
 *            - -4: 三代控制器不支持该接口
 */
RM_INTERFACE_EXPORT int rm_read_modbus_rtu_holding_registers(rm_robot_handle *handle, rm_modbus_rtu_read_params_t param, int *data);
/**
 * @brief Modbus RTU协议写保持寄存器
 * @param handle 机械臂控制句柄
 * @param param 写保持寄存器参数
 * @return int 函数执行的状态码。
 *            - 0: 成功。
 *            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。
 *            - -1: 数据发送失败，通信过程中出现问题。
 *            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。
 *            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
 *            - -4: 三代控制器不支持该接口
 */
RM_INTERFACE_EXPORT int rm_write_modbus_rtu_registers(rm_robot_handle *handle, rm_modbus_rtu_write_params_t param);
/**
 * @brief Modbus RTU协议读输入寄存器
 * @param handle 机械臂控制句柄
 * @param param 读输入寄存器参数
 * @param data 读输入寄存器数据，数组大小为param.num
 * @return int 函数执行的状态码。
 *            - 0: 成功。
 *            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。
 *            - -1: 数据发送失败，通信过程中出现问题。
 *            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。
 *            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
 *            - -4: 三代控制器不支持该接口
 */
RM_INTERFACE_EXPORT int rm_read_modbus_rtu_input_registers(rm_robot_handle *handle, rm_modbus_rtu_read_params_t param, int *data);
/**
 * @brief Modbus TCP协议读线圈
 * @param handle 机械臂控制句柄
 * @param param 读线圈参数
 * @param data 读线圈数据，数组大小为param.num
 * @return int 函数执行的状态码。
 *            - 0: 成功。
 *            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。
 *            - -1: 数据发送失败，通信过程中出现问题。
 *            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。
 *            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
 *            - -4: 三代控制器不支持该接口。
 */
RM_INTERFACE_EXPORT int rm_read_modbus_tcp_coils(rm_robot_handle *handle, rm_modbus_tcp_read_params_t param, int *data);
/**
 * @brief Modbus TCP协议写线圈
 * @param handle 机械臂控制句柄
 * @param param 写线圈参数
 * @return int 函数执行的状态码。
 *            - 0: 成功。
 *            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。
 *            - -1: 数据发送失败，通信过程中出现问题。
 *            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。
 *            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
 *            - -4: 三代控制器不支持该接口。
 */
RM_INTERFACE_EXPORT int rm_write_modbus_tcp_coils(rm_robot_handle *handle, rm_modbus_tcp_write_params_t param);
/**
 * @brief Modbus TCP协议读离散量输入
 * @param handle 机械臂控制句柄
 * @param param 读离散输入参数
 * @param data 读离散输入数据，数组大小为param.num
 * @return int 函数执行的状态码。
 *            - 0: 成功。
 *            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。
 *            - -1: 数据发送失败，通信过程中出现问题。
 *            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。
 *            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
 *            - -4: 三代控制器不支持该接口。
 */
RM_INTERFACE_EXPORT int rm_read_modbus_tcp_input_status(rm_robot_handle *handle, rm_modbus_tcp_read_params_t param, int *data);
/**
 * @brief Modbus TCP协议读保持寄存器
 * @param handle 机械臂控制句柄
 * @param param 读保持寄存器参数
 * @param data 读保持寄存器数据，数组大小为param.num
 * @return int 函数执行的状态码。
 *            - 0: 成功。
 *            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。
 *            - -1: 数据发送失败，通信过程中出现问题。
 *            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。
 *            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
 *            - -4: 三代控制器不支持该接口。
 */
RM_INTERFACE_EXPORT int rm_read_modbus_tcp_holding_registers(rm_robot_handle *handle, rm_modbus_tcp_read_params_t param, int *data);
/**
 * @brief Modbus TCP协议写保持寄存器
 * @param handle 机械臂控制句柄
 * @param param 写保持寄存器参数
 * @return int 函数执行的状态码。
 *            - 0: 成功。
 *            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。
 *            - -1: 数据发送失败，通信过程中出现问题。
 *            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。
 *            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
 *            - -4: 三代控制器不支持该接口。
 */
RM_INTERFACE_EXPORT int rm_write_modbus_tcp_registers(rm_robot_handle *handle, rm_modbus_tcp_write_params_t param);
/**
 * @brief Modbus TCP协议读输入寄存器
 * @param handle 机械臂控制句柄
 * @param param 读输入寄存器参数
 * @param data 读输入寄存器数据，数组大小为param.num
 * @return int 函数执行的状态码。
 *            - 0: 成功。
 *            - 1: 控制器返回false，传递参数错误或机械臂状态发生错误。   
 *            - -1: 数据发送失败，通信过程中出现问题。
 *            - -2: 数据接收失败，通信过程中出现问题或者控制器超时没有返回。
 *            - -3: 返回值解析失败，接收到的数据格式不正确或不完整。
 *            - -4: 三代控制器不支持该接口。
 */
RM_INTERFACE_EXPORT int rm_read_modbus_tcp_input_registers(rm_robot_handle *handle, rm_modbus_tcp_read_params_t param, int *data);


};  
  
#endif // __cplusplus  
  
#endif // MY_CPP_WRAPPER_H