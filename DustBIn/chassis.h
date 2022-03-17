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
#ifndef _CHASSIS_H
#define _CHASSIS_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* includes ------------------------------------------------------------------*/
#include "message.h"
#include "motor.h"
#include "pid.h"
/* typedef -------------------------------------------------------------------*/
typedef struct _MoveData_t
{
    int16_t Right;    
    int16_t Front;
    int16_t ClockWise;
}MoveData_t;

typedef struct _SuperCap_t
{
    uint16_t InVol;
    uint16_t CapVol;
    uint16_t InCur;
    uint16_t Power;
    uint16_t TargetPower;
    uint8_t  CanData[8];
}SuperCap_t;

typedef struct _Chassis_t
{
    M3508_t M3508[4];
    uint8_t CanData[8];
    SuperCap_t SuperCap;
    MoveData_t MoveData;
}Chassis_t;

/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
extern Chassis_t Chassis;
/* function ------------------------------------------------------------------*/
void Chassis_Init(void);
void Chassis_Process(RemoteData_t RDMsg);
void Chassis_Protect(RemoteData_t RDMsg);
void Turn_Round(RemoteData_t RDMsg);
void Fast_Fix_Spin(RemoteData_t RDMsg);
void Chassis_Speed_Reset(void);
void Dynamic_Spin(void);
void Chassis_Power_Limit(void);
void Chassis_CanTransmit(void);/*< ·ÅÖÐ¶Ï */

#ifdef __cplusplus
}
#endif

#endif /* */
  
/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
