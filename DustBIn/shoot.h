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
#ifndef _SHOOT_H
#define _SHOOT_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* includes ------------------------------------------------------------------*/
#include "motor.h"
#include "message.h"
#include "pid.h"
#include "FreeRTOS.h"
#include "task.h"
/* typedef -------------------------------------------------------------------*/
typedef struct _Shoot_t
{
    Mcircle_t Position;
	  int16_t Speed;
	  int16_t Current;
	  int16_t LpfPosition;
	  int16_t LpfSpeed;
	  int16_t LpfCurrent;
	  PID_IncrementType PidSpeed;
	  PID_IncrementType PidPosition;
	  PID_AbsoluteType  PidSpeedstop;
	  int16_t TarPosition;
	  int16_t TarSpeed;
	  int16_t TarCurrent;
	  int16_t OutputLpf;
	  int16_t Output;
	  uint8_t CanData[8];
}Shoot_t;

/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
extern Shoot_t M2006;
static int16_t curerror = 0;
static float Position=0;
/* function ------------------------------------------------------------------*/
void  Shoot_PidInit(void);
void Shoot_Process(RemoteData_t RDMsg);
portTickType REVOL_uiGetRevolTime(void);
void Shoot_Speed_Control(RemoteData_t RDMsg);
void Prepare(RemoteData_t RDMsg);
void Open(void);
void Close(void);
void Shoot_CanTransmit(void);
void Shoot_Freq_Control(void); /*< 连发速率控制*/
#ifdef __cplusplus
}
#endif

#endif /* define*/
