/**
 * @file:      robot_define.h
 * 
 * @author:    leon  
 *
 * @date:      2023-03-30 13:07:10 
 *
 * @modified:  leon 
 *
 * @update:	   2023-03-30 13:07:10 
 *
 * @copyright: Copyright (c) 2018-2023 RealMan Co., Ltd.. All rights reserved. 
 */
#ifndef __ROBOT_DEFINE_H__
#define __ROBOT_DEFINE_H__

#include "constant_define.h"
#include "rman_int.h"

typedef struct
{
    short irow;
    short iline;
    float data[4][4];
} Matrix;

typedef struct
{
	float w;
	float x; //* unit: rad
	float y;
	float z;
} Quat;

typedef struct
{
	float x; //* unit: m
	float y;
	float z;
} Pos;

typedef struct
{
	float rx; //* unit: rad
	float ry;
	float rz;
} Euler;

typedef struct
{
	Pos position;
	Quat quaternion;
	Euler euler;
} Pose;

typedef struct
{
    float x;  //* unit: m
    float y;
    float z;
    float rx; //* unit: rad
    float ry;
    float rz;
} Coord;

typedef struct
{
    short irow;
    short iline;
    float data[7][7];
} Matrix_more;

typedef struct
{
    float d[ROBOT_DOF];     //* unit: m
    float a[ROBOT_DOF];     //* unit: m
    float alpha[ROBOT_DOF]; //* unit: rad

    /* Deviation of the mechanical zero from the modeling zero.
       i.e. offset = modeling zero - mechanical zero
    */
    float offset[ROBOT_DOF];    //* unit: rad

    float err_d[ROBOT_DOF];     //* unit: m
    float err_a[ROBOT_DOF];     //* unit: m
    float err_alpha[ROBOT_DOF]; //* unit: rad
    float err_theta[ROBOT_DOF]; //* unit: rad
} DH;

typedef enum
{
    B,
    ZF,
    SF
} SensorType;

typedef enum
{
    RM65,
    RM75,
    RML63I,
    RML63II,
    RML63III,
    NANO,
    ECO65
} RobotType;

typedef struct
{
    float mass;       //* unit:kg
    float com[3];     //* center of mass [x y z], unit:m
    float inertia[6]; //* [Ixx Iyy Izz Ixy Ixz Iyz], unit: kg/(m^2)

} PayloadCfg;

typedef struct
{
#define STRING_LEN 10
    char servo_version[STRING_LEN];   //* Servo control software version
    char pcb_version[STRING_LEN];     //* Electronic PCB hardware version
    char machine_version[STRING_LEN]; //* Mechanical hardware version
    char joint_type[STRING_LEN];      //* Joint type

    float pos_max_limit; //* The maximum position reachable by the joint, unit: rad
    float pos_min_limit; //* The minimum position reachable by the joint, unit: rad
    float vel_limit;     //* Joint max velocity, unit:rad/s
    float acc_limit;     //* Joint max acceleration, unit:rad/(s^2)
    float gear_ratio;    //* Reduction gear ratio of harmonic reducer
    float rated_current; //* Rated current of motor, unit: A
    float rated_torque;  //* Rated torque of motor,  unit: Nm
} JointCfg;

typedef struct
{
    float Fx; //* unit: N
    float Fy;
    float Fz;
    float Tx; //* unit: Nm
    float Ty;
    float Tz;
} ForceSensor;

typedef struct
{
    float ref_position[ROBOT_DOF];   //* unit:rad
    float fb_position[ROBOT_DOF];    //* unit:rad
    float fb_velocity[ROBOT_DOF];    //* unit:rad/s
    float fb_temperature[ROBOT_DOF]; //* unit:degree centigrade
    float fb_current[ROBOT_DOF];     //* unit:A
    float fb_voltage[ROBOT_DOF];     //* unit:V
    int encoder_motor[ROBOT_DOF];    //* Feedback value from the encoder of the motor
    int encoder_link[ROBOT_DOF];     //* Feedback value from the encoder of the link
    short joint_enable_state;        //* Enable:1, Disable:0
    ForceSensor fb_force;            //* Feedback value from the force sensor, unit: N or Nm
} RobotState;

typedef struct
{
    float linear_vel_limit;  //* unit: m/s
    float linear_acc_limit;  //* unit: m/(s^2)
    float angular_vel_limit; //* unit: rad/s
    float angular_acc_limit; //* unit: rad/(s^2)
} TcpCfg;

typedef struct
{
    Euler install_angle; //* robot install angle, unit:rad.
    Coord tool;          //* the tool coordinate system
    Coord work;          //* the work coordinate system, based on fixed coordinate system description
    Matrix Ttool;        //* from link-6 coordinate system to tool coordinate system
    Matrix Twork2base;   //* from work coordinate system to world coordinate system
    Matrix Tbase;        //* from world coordinate system to base coordinate system
} RobotFrame;

typedef struct
{
    float k_g[ROBOT_DOF];                            //* parameters of gravity-regulation-factor of drag function， unit: 1
    float k_f[ROBOT_DOF];                            //* parameters of friction-regulation-factor of drag function, unit: 1
    float k_l[ROBOT_DOF];                            //* parameters of joint payload-friction-constant, unit: 1
    float k_i[ROBOT_DOF];                            //* parameters of joint torque constant, unit: Nm/A
    float joint_param[ROBOT_DOF][FRIC_PARMS_NUM];    //* parameters of joints friction dynamics
} DynParam;

typedef struct
{
    DH dh;
    RobotType robot_type;
    SensorType sensor_type;
    PayloadCfg payload;
    TcpCfg tcp_limit;
    JointCfg joint[ROBOT_DOF];
    RobotState fb_data;
    RobotFrame frame;
    DynParam  dyn_params;

    float collision_threshold[ROBOT_DOF][3];

    float joint_cmd[ROBOT_DOF];		 //* Joint position cmd, unit: rad
    float joint_cur[ROBOT_DOF];      //* Joint current position, unit: rad, without offset.

    short rbt_dof;                       //* Robot dof
    float cycle_period;                  //* Control cycle period of the controller, unit:sec, default:CYCLE_PERIOD
    float gravity[3];                    //* Acceleration of gravity, unit: m/(s^2), default:[0, 0, -9.81];
    short safety_level;                  //* Safety level of collision detection   
} Robot;

//六维力拖动示教用
typedef enum
{
	None_Teach = 0,
	Current_Teach = 1,
	Pos_Teach = 2,
	Ort_Teach = 3,
	Multi_Teach = 4
}TEACH_MODES;



//基于传感器的力控用
//力传感器标定结果，成功或失败
typedef enum 
{
    FAILED_CALIBRATION = 0,
    SUCCESSFUL_CALIBRATION = 1
} CALIBRATION_RESULT;

//施加力模式
typedef enum 
{
    BASE_MODE = 0,
    TOOL_MODE = 1
} COORDINATE_SYSTERM_MODE;

//坐标系方向
typedef enum 
{
    X_DIRECTION = 0,
    Y_DIRECTION = 1,
    Z_DIRECTION = 2,
    RX_DIRECTION = 3,
    RY_DIRECTION = 4,
    RZ_DIRECTION = 5
} COORDINATE_SYSTERM_DIRECTION;


//质心相关结构体
typedef struct
{
	float rx;//质心x坐标，单位：米
	float ry;//质心y坐标，单位：米
	float rz;//质心z坐标，单位：米
	float f0[6];//零位数据，单位：N、N、N、NM、NM、NM
	Matrix G_b;//重力在基坐标系下的向量，单位N
}Algo_Center_G;

//多组力的数据
typedef struct
{
	float f[6];//单位：N、N、N、NM、NM、NM
}multi_f;
//多组关节角度的数据
typedef struct
{
	float joint[7];//单位：弧度
}multi_joint;
//计算质心用到的结构体
typedef struct
{
	multi_f f_6[4];
    multi_joint joint_angle[4];
    Algo_Center_G Center;
}sensor_calibration;


#endif
