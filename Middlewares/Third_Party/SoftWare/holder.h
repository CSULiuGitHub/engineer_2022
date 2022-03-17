/**
  ******************************************************************************
  * @file    holder.h
  * @author  
  * @brief   ��̨����
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
#ifndef _HOLDER_H
#define _HOLDER_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* includes ------------------------------------------------------------------*/
#include "motor.h"
#include "message.h"

/* typedef -------------------------------------------------------------------*/

typedef struct _Holder_t
{
		M3508_t lift[2];
		M3508_t Lift_x;
		M3508_t arm[2];
    M3508_t arm_yaw;
    uint8_t CanData[8];
		uint8_t CanData_0xff[8];
}Holder_t;

/* define --------------------------------------------------------------------*/

#define	LIMIT(data, max, min)\
	do{\
		if((data) > (max)){\
			(data) = (max);\
		}else if((data) < (min)){\
			(data) = (min);\
		}\
	}while(0)

/* variables -----------------------------------------------------------------*/
extern Holder_t Holder;
/* function ------------------------------------------------------------------*/
static void Holder_GetMoveData(RemoteData_t RDMsg);

void Holder_Pid_Init(void);
void Holder_Process(HolderData_t HDMsg,RemoteData_t RDMsg);

/******************************************************************/
void Holder_Angle_Reset(void);
void Holder_CanTransmit(void);
void Holder_Debug(void); /*< 云台调参用*/
//void Send_Vision(void);
#ifdef __cplusplus
}
#endif

#endif /* */
  
/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
