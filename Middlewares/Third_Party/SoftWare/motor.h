/**
  ******************************************************************************
  * @file    
  * @author  sy
  * @brief   
  * @date     
  ******************************************************************************
  * @attention
  *
  * Copyright (c) CSU_RM_FYT
  * All rights reserved.
  *
  * This software component is licensed by SY under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  * opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MOTOR_H
#define _MOTOR_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "pid.h"
/* typedef -------------------------------------------------------------------*/
typedef struct _Mcircle_t
{
    int32_t Circle;
    int16_t Angle;
}Mcircle_t;

typedef struct _MotorData_t
{
    uint16_t zero_drift;
    int32_t Angle; //�޸� 32
    int16_t Speed;
    int16_t Current;
		int16_t TarSpeed;
    int16_t TarCurrent;
    int16_t Output;
}MotorData_t;

typedef struct _M2006_t
{
    Mcircle_t Circle;
    MotorData_t Rx;
    MotorData_t LPf;
    int16_t Output;
}M2006_t;

typedef struct _M3508_t
{
		Mcircle_t Mc;
		int32_t TarPosition; //�޸� 32
    int16_t TarSpeed;
    int16_t TarCurrent;
    MotorData_t Rx;
    MotorData_t LPf;
		PID_AbsoluteType PidPosition;
    PID_IncrementType PidSpeed;
    PID_IncrementType PidCurrent;
    int16_t Output;
}M3508_t;

typedef struct _GM6020_t
{
    int16_t TarPosition;
    int16_t TarSpeed;
		PID_AbsoluteType PidPosition;
    PID_IncrementType PidSpeed;
    Mcircle_t Mc;
    MotorData_t Rx;
}GM6020_t;

/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
/* function ------------------------------------------------------------------*/
void Circle_Continue(Mcircle_t *Mc, uint16_t angle);
#ifdef __cplusplus
}
#endif

#endif /* */
  
/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
