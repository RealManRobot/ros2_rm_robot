/**
 * @file:      contant_define.h
 * 
 * @author:    leon  
 *
 * @date:      2023-03-30 17:31:30 
 *
 * @modified:  leon 
 *
 * @update:	   2023-03-30 17:31:30 
 *
 * @copyright: Copyright (c) 2018-2023 RealMan Co., Ltd.. All rights reserved. 
 */
#ifndef __CONSTANT_DEFINE_H__
#define __CONSTANT_DEFINE_H__

#ifndef ROBOT_DOF
#define ROBOT_DOF 7
#endif

#ifndef CYCLE_PERIOD
#define CYCLE_PERIOD (0.004f)
#endif

#ifndef FRIC_PARMS_NUM
#define FRIC_PARMS_NUM 5
#endif

#ifndef TEMP_PARAMS_NUM
#define TEMP_PARAMS_NUM 5
#endif

#ifndef SAFETY_LEVEL_MAX
#define SAFETY_LEVEL_MAX 8
#endif

#ifndef ALGO_PI
#define ALGO_PI (3.14159265358979f)
#endif

#ifndef PI_RAD
#define PI_RAD (0.0174532925199433f)
#endif

#ifndef PI_ANG
#define PI_ANG (57.2957795130823f)
#endif

#ifndef DEG2RAD
#define DEG2RAD (0.0174532925199433f)
#endif

#ifndef RAD2DEG
#define RAD2DEG (57.2957795130823f)
#endif

#ifndef PIX2
#define PIX2 (6.28318530717959f)
#endif

#ifndef PI_2
#define PI_2 (1.5707963267949f)
#endif

//* (RM75)outside the reachable zone
#ifndef OUTSIDE_REACHABLE_ZONE
#define OUTSIDE_REACHABLE_ZONE (-1)       
#endif

//* (RM6)fit = 1000;(RM75)fit0 = .....=fit8 = 0
#ifndef INVERSE_FAIL
#define INVERSE_FAIL (-5)                 
#endif

//* overspeed
#ifndef OVERSPEED
#define OVERSPEED (-3)                    
#endif

#ifndef UNKNOWN_TYPE
#define UNKNOWN_TYPE (-2)
#endif

#ifndef SUCCESS
#define SUCCESS 0
#endif

#ifndef GRAVITY_ACC_CONSTANT
#define GRAVITY_ACC_CONSTANT (-9.81f)
#endif

#endif