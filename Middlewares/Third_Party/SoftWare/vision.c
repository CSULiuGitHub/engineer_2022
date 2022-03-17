/**
  ******************************************************************************
  * @file    
  * @author  sy
  * @brief
  * @date     
  ******************************************************************************
  * @attention
  *
  * Copyright (c) CSU_RM_FYT.
  * All rights reserved.
  *
  * This software component is licensed by SY under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  * opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* includes ------------------------------------------------------------------*/
#include "misc_func.h"
#include "vision.h"
#include "config.h"
#include "string.h"
#include "judge.h"
/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
vData_t vData = {0};
uint8_t global_vision_state = VISION_MANUAL;
/* function ------------------------------------------------------------------*/

int16_t vData_x;
int16_t vData_y;
int16_t vData_z;
uint8_t vData_buf[16];


void Vision_RecvData(uint8_t byte)
{
    static uint8_t count = 0;
    
    vData_buf[count] = byte;
    if (vData_buf[count] == 0x7b || count>0)
    {
        count++;
    }
    else
    {
        count = 0;
    }
    
    if (count == 8)
    {
        count = 0; /*< 重新开始接收 */
        if (vData_buf[7] == 0x7d)
        {
            vData_x = ((vData_buf[1]<<8) + vData_buf[2]);
            vData_y = ((vData_buf[3]<<8) + vData_buf[4]);  
            vData_z = ((vData_buf[5]<<8) + vData_buf[6]);  
        }
    }
}

void Vision_SendData(RemoteData_t RDMsg)
{
	//以前步兵用，为工程以后预留
	
}
/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
