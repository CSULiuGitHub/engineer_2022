#ifndef _HOLDER_CONFIG_H
#define _HOLDER_CONFIG_H


#include <stdint.h>
#include "config.h"

/******************************************************************************
  * @brief  云台控制相关参数 
  * @attention 双闭环控制 
	* 
  ****************************************************************************/

/* 进行调试前必须定义下面参数，其中波形参数只能定义一个 */
/* #define HOLDER_DEBUG */
/* #define HOLDER_PITCH_WAVE */
/* #define HOLDER_YAW_WAVE */
/* #define HOLDER_VISION_WAVE */

//#define HOLDER_VISION_WAVE
////    #define HOLDER_DEBUG
////    #define HOLDER_PITCH_WAVE
//#ifdef DEBUG
////    #define HOLDER_DEBUG
////    #define HOLDER_YAW_WAVE
////    #define HOLDER_PITCH_WAVE
//    //#define HOLDER_VISION_WAVE
//#endif


/*******************************************************/
float Holder_liftx_Position_Kp = 0.5;
float Holder_liftx_Speed_Kp = 2;

float Holder_liftx_Position_Ki = 0;
float Holder_liftx_Speed_Ki = 0.;

float Holder_liftx_Position_Kd = 0;
float Holder_liftx_Speed_Kd = 0;

float Holder_lift2_0_Position_Kp = 0.3;
float Holder_lift2_0_Speed_Kp = 0.34;

float Holder_lift2_0_Position_Ki = 0.002;
float Holder_lift2_0_Speed_Ki = 0;

float Holder_lift2_0_Position_Kd = 0;
float Holder_lift2_0_Speed_Kd = 0;

float Holder_lift2_1_Position_Kp = 0.5;
float Holder_lift2_1_Speed_Kp = 1;

float Holder_lift2_1_Position_Ki = 0;
float Holder_lift2_1_Speed_Ki = 0;

float Holder_lift2_1_Position_Kd = 0;
float Holder_lift2_1_Speed_Kd = 0;

PID_param arm0_position = {0.5, 0, 0};//给i会出现积分饱和，以后有时间再搞
PID_param arm0_speed = {3.00, 0, 0};

PID_param arm1_position = {0.5, 0, 0};
PID_param arm1_speed = {3.00, 0, 0};

PID_param arm_yaw_position = {0.35, 0, 0};
PID_param arm_yaw_speed = {8, 0.3, 0.01};
/****************************************************************/
float Holder_liftx_Position_Plim = 16000;
float Holder_liftx_Position_Ilim = 16000;

float Holder_liftx_Speed_Inclim = 16000;

float Holder_lift2_Position_Plim = 16000; 
float Holder_lift2_Position_Ilim = 10000;  // change

float Holder_lift2_Speed_Inclim = 16000;

float Holder_Out_Limit = 16000;


#endif
