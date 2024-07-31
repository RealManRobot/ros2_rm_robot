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

typedef float MAT3X2[3][2];
typedef float MAT3X3[3][3];
typedef float MAT3X6[3][6];
typedef float MAT4X4[4][4];
typedef float MAT8X3[8][3];
typedef float MAT16X3[16][3];
typedef float MAT20X3[20][3];
typedef float MAT6X6[6][6];
typedef float MAT6X7[6][7];
typedef float MAT7X6[7][6];
typedef float VEC3[3];
typedef float VEC4[4];
typedef float VEC6[6];
typedef float VEC7[7];
typedef float VEC8[8];
typedef float VEC16[16];
typedef float VEC20[20];
typedef int VEC8_int[8];
typedef int VEC16_int[16];
typedef int VEC20_int[20];

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
	float x;
	float y;
	float z;
	float distance;
}dAndP;//点到直线的距离和交点(即垂足)
typedef struct
{
	int isCself;        //机械臂自身是否发生碰撞
	int isCcube;        //机械臂是否与正方体发生碰撞
}Collis_self_cube;      //碰撞检测结果
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
    Pose pose;
    float joint[7];
} Position_data;        //机械臂位姿数据(Pose形式和矩阵形式)

typedef struct
{
    Pose pose;
    Matrix T_BT;
} Pose_data;        //机械臂位姿数据(Pose形式和矩阵形式)
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
    _not_avoid_singularity = 0,         //不规避奇异点（movej_p用）
    _avoid_singularity = 1             //规避奇异点
} Singular_flag;

typedef enum
{
    _selfcollision = 0,         //发生自碰撞
    _no_selfcollision = 1     //未发生自碰撞
} Is_self_collision;             //是否发生自碰撞

typedef enum
{
    _virtual_wallcollision = 0,    //撞墙
    _no_virtual_wallcollision = 1  //不撞墙
} Is_wall_collision;                 //是否撞墙标志      

typedef enum
{
    CUBOID_WALL = 0,        //长方体or正方体墙（长宽高）
    SPHERE_WALL = 1,        //球体墙（球心和半径）
    CYLINDER_WALL = 2,      //圆柱墙（圆柱轴线，母线长，半径）
    CONE_WALL = 3           //圆锥墙（底面半径，高or母线长）
} VirtualWallType;

typedef struct ToolSpherePara
{
    float radius;     // 球体半径（单位：m）
    VEC3 centrePoint; // 球体中心位置（单位：m，以法兰坐标系为参考坐标系）
} ToolSpherePara;     // 工具包络球参数

typedef struct ToolEnvelopeData
{
    ToolSpherePara toolEnvelope[5]; // 支持使用最多5个球体包络
} ToolEnvelopeData;                 // 工具包络数据

typedef struct
{
    int tLinkNum;       //工具包络球连杆数
    float linkRadius[5];  //工具包络球连杆半径
}ToolLinkPara;      //工具连杆数据

typedef enum
{
    _fence_collision = 0,    //撞墙
    _no_fence_collision = 1  //不撞墙
} Is_fence_collision;                 //是否撞墙标志

typedef enum
{
    CUBOID_FENCE = 0,        //长方体or正方体围栏
    SINGLE_FENCE = 1,        //单面围栏
} SafeFenceType;

typedef enum
{
    _inside_fence = 0,    //在围栏内
    _outside_fence = 1    //在围栏外
} IsInsideFence;          //是否在围栏内

typedef enum
{
    _whole_robot = 0,    //整臂
    _only_tcp = 1        //仅末端
} IsRobotOrTCP;                 //整臂or末端

typedef enum
{
    INSIDE_WALL = 0, // 墙内
    OUTSIDE_WALL = 1 // 墙外
} VirtualWallDragScope; // 虚拟墙拖动区域

typedef enum
{
    WHOLE_ROBOT_MODE = 0, // 机械臂后四个关节+末端+工具
    FLANGE_TOOL_MODE = 1  // 末端+工具
} VirtualWallCrashObject;// 虚拟墙作用部位

typedef enum
{
    fence_succeed = 0,     //电子围栏设置成功
    fence_failed = 1       //电子围栏设置失败
} IsFenceSucceed;
typedef struct 
{
    MAT3X2 cuboid_data;      //data[0][0],data[0][1]:-x,+x;data[1][0],data[1][1]:-y,+y;data[2][0],data[2][1]:-z,+z
    Pos plane_point[3];
}FenceData;     


typedef struct 
{
    SafeFenceType _fence_type;      // 围栏类型  
    MAT3X2 cuboid_data;              // 立方体数据cuboid_data[0][0-1]:xmin xmax,cuboid_data[1][0-1]:ymin ymax,cuboid_data[2][0-1]:zmin zmax
    Pos single_Pdata[3];            // 单面围栏数据 [0][1][2]为平面上任意3点坐标
    IsInsideFence _is_insidefence;  // 机械臂是否在围栏内（0：在围栏内，1：在围栏外）
    int _is_robot_or_tcp;           // 整臂or末端
    float current_joint_angle[ROBOT_DOF]; //当前关节角度
}SafeFencePara;

typedef enum
{
    RM65,
    RM75,
    RML63I,
    RML63II,
    RML63III,
    ECO65,
    ECO62,
    GEN72,
    UNIVERSAL
} RobotType;


typedef enum
{
    _B,
    _ZF,
    _SF
} SensorType;

typedef struct
{
    float mass;       //* unit:kg
    float com[3];     //* center of mass [x y z], unit:m
    float inertia[6]; //* [Ixx Iyy Izz Ixy Ixz Iyz], unit: kg/(m^2)
    float radius;     //* radius of the tool
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
    float link_dyn_param[ROBOT_DOF*10];              //* parameters of link dynamics, unit: kg*m^2, kg*m, kg
                                                     //* Ixx Ixy Ixz Iyy Iyz Izz mx my mz m
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

    float joint_direction[ROBOT_DOF];
    float collision_threshold[ROBOT_DOF][3];

    float joint_cmd[ROBOT_DOF];		 //* Joint position cmd, unit: rad
    float joint_cur[ROBOT_DOF];      //* Joint current position, unit: rad, without offset.

    short rbt_dof;                       //* Robot dof
    float cycle_period;                  //* Control cycle period of the controller, unit:sec, default:CYCLE_PERIOD
    float gravity[3];                    //* Acceleration of gravity, unit: m/(s^2), default:[0, 0, -9.81];
    short safety_level;                  //* Safety level of collision detection   

    int all_joint_num;                   //* All DOF of robot and external axis, all_joint_num = rbt_dof + external_axis_dof
    int is_avoid_singularity;            //* 是否规避奇异点标志位，1:规避奇异点,0:不规避奇异点

    int dynamics_model_type;             //* Dynamics model type, 1:old model, 2: new model
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
    WORK_MODE = 0,
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
	Matrix G_b;//重力在世界坐标系下的向量，单位N
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
