/**
  ******************************************************************************
  * @file    holder.c
  * @author  
  * @brief   云台任务
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
  * 云台注意事项：
  ******************************************************************************
  */

/*includes ------------------------------------------------------------------*/
#include "holder_config.h"
#include <string.h>
#include "misc_func.h"
#include "vision.h"
#include "holder.h"
#include "motor.h"
#include "usart.h"
#include "judge.h"
#include "can.h"
#include "tim.h"
#include "fast_tri_func.h"

/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/
#define VISION (1) /*< 视觉模式 */
#define MANUAL (0) /*< 手动模式 */

//#define HOLDER_DEBUG
//#define HOLDER_WAVE
//#define HOLDER_ARM_YAW_WAVE
//#define HOLDER_ARM_PITCH_WAVE

#define SUP   1
#define SMID  3
#define SDOWN 2

#define LIFT_LIMIT 200000
#define LIFT_X_LIMIT 400000
#define ARM_PITCH_LIMIT 30000
#define ARM_YAW_LIMIT 80000

#define RPC_ZERO(IN,RANGE)	((IN < RANGE && IN > -RANGE) ? 0 : IN) 
/* variables -----------------------------------------------------------------*/
Holder_t Holder = {0};

int32_t liftx_Tarposition;
int32_t lift2_Tarposition;
/* function ------------------------------------------------------------------*/
void Grab_Status_Set(void);
void Grab_Status_Reset(void);
void Grab_Mode_handle(void);
void Grab_Mode_Auto(void);
/**
  * @brief  云台电机PID初始化
  * @param  void
  * @retval void
  * @attention 抬升两个M3508,前移一个M3508，爪子翻转之类所有电机都放一个
	*            任务，
  */
void Holder_Pid_Init(void)
{
		pid_init_absolute(&Holder.Lift_x.PidPosition, Holder_liftx_Position_Kp, Holder_liftx_Position_Ki, 
										Holder_liftx_Position_Kd, Holder_liftx_Position_Ilim, Holder_liftx_Position_Plim);
		pid_init_increment(&Holder.Lift_x.PidSpeed, Holder_liftx_Speed_Kp, Holder_liftx_Speed_Ki, Holder_liftx_Speed_Kd, 
										Holder_liftx_Speed_Inclim);
		
		pid_init_absolute(&Holder.lift[0].PidPosition, Holder_lift2_0_Position_Kp, Holder_lift2_0_Position_Ki, Holder_lift2_0_Position_Kd, 
										Holder_lift2_Position_Ilim, Holder_lift2_Position_Plim );
		pid_init_increment(&Holder.lift[0].PidSpeed, Holder_lift2_0_Speed_Kp, Holder_lift2_0_Speed_Ki, Holder_lift2_0_Speed_Kd, 
										Holder_lift2_Speed_Inclim);
		
		pid_init_absolute(&Holder.lift[1].PidPosition, Holder_lift2_1_Position_Kp, Holder_lift2_1_Position_Ki, Holder_lift2_1_Position_Kd, 
										Holder_lift2_Position_Ilim, Holder_lift2_Position_Plim );
		pid_init_increment(&Holder.lift[1].PidSpeed, Holder_lift2_1_Speed_Kp, Holder_lift2_1_Speed_Ki, Holder_lift2_1_Speed_Kd, 
										Holder_lift2_Speed_Inclim);
	
		pid_init_absolute(&Holder.arm[0].PidPosition, arm0_position.Kp, arm0_position.Ki, arm0_position.Kd, 2000, 0);
		pid_init_increment(&Holder.arm[0].PidSpeed, arm0_speed.Kp, arm0_speed.Ki, arm0_speed.Kd, 16000);
		
		pid_init_absolute(&Holder.arm[1].PidPosition, arm1_position.Kp, arm1_position.Ki, arm1_position.Kd, 0, 0);
		pid_init_increment(&Holder.arm[1].PidSpeed, arm1_speed.Kp, arm1_speed.Ki, arm1_speed.Kd, 16000);
	
		pid_init_absolute(&Holder.arm_yaw.PidPosition, arm_yaw_position.Kp, arm_yaw_position.Ki, arm_yaw_position.Kd, 10000.0f, 90000.0f);
		pid_init_increment(&Holder.arm_yaw.PidSpeed, arm_yaw_speed.Kp, arm_yaw_speed.Ki, arm_yaw_speed.Kd, 16000);
}

/**
  * @brief  获取云台控制数据
  * @param  控制消息结构体
  * @retval void
  * @attention 
  */
static void Holder_GetMoveData(RemoteData_t RDMsg)
{
	static int32_t arm_tmp = 0;
	static int32_t arm_yaw_tmp = 0;//临时
	static uint8_t delay_tick = 0;
	if((RDMsg.S1 == SMID) && (RDMsg.S2 == SDOWN))
	{
			liftx_Tarposition -= RPC_ZERO(RDMsg.Ch2,10);
			lift2_Tarposition += RPC_ZERO(RDMsg.Ch3,10);
			arm_tmp += RPC_ZERO(RDMsg.Ch0,10);
			arm_yaw_tmp += RPC_ZERO(RDMsg.Ch1,10);//零漂处理，理应当在
	}
	if(RDMsg.S1 == SUP)
	{   
			if(delay_tick < 100)delay_tick++;
			if(RDMsg.KeyBoard.q && delay_tick == 100)
			{
					Grab_Mode_Auto();
					delay_tick = 0;
			}
			if(RDMsg.KeyBoard.w && delay_tick == 100)
			{
					Grab_Mode_handle();
					delay_tick = 0;
			}
			if(RDMsg.KeyBoard.a && delay_tick == 100)
			{
					Grab_Status_Set();
					delay_tick = 0;
			}
//			if(RDMsg.KeyBoard.s && delay_tick == 100)
//			{
//					Grab_Status_Reset();
//					delay_tick = 0;
//			}
			
	}
	
	LIMIT(liftx_Tarposition, 0, -LIFT_X_LIMIT);
	LIMIT(lift2_Tarposition, LIFT_LIMIT, 0);
	LIMIT(arm_tmp, ARM_PITCH_LIMIT, 3000);
	LIMIT(arm_yaw_tmp, 0, -ARM_YAW_LIMIT);
	
	Holder.Lift_x.TarPosition = Misc_RAMP_Int32(liftx_Tarposition, Holder.Lift_x.TarPosition, 800);
	Holder.lift[1].TarPosition = Misc_RAMP_Int32(lift2_Tarposition, Holder.lift[1].TarPosition, 800);
	Holder.lift[0].TarPosition = - Holder.lift[1].TarPosition;
	Holder.arm[0].TarPosition = Misc_RAMP_Int32(arm_tmp, Holder.arm[0].TarPosition, 800);
	Holder.arm[1].TarPosition = -Holder.arm[0].TarPosition;
	Holder.arm_yaw.TarPosition = Misc_RAMP_Int32(arm_yaw_tmp, Holder.arm_yaw.TarPosition, 500);

}
/*************************************************
@Function: Holder_Arm_PidRun
@Description: 云台爪子yaw轴pid计算
@Called By: Holder_PidRun
@Input: void
@Others: 对电机返回速度低通滤波
*************************************************/
static void Holder_Arm_Yaw_PidRun(void)
{
		Holder.arm_yaw.TarSpeed = pid_absolute_update(Holder.arm_yaw.TarPosition, Holder.arm_yaw.Rx.Angle, &Holder.arm_yaw.PidPosition);
		Holder.arm_yaw.LPf.Speed = 0.8 * Holder.arm_yaw.Rx.Speed + 0.2 * Holder.arm_yaw.LPf.Speed;
		Holder.arm_yaw.Output = pid_increment_update(Holder.arm_yaw.TarSpeed,Holder.arm_yaw.LPf.Speed, &Holder.arm_yaw.PidSpeed);
}
/*************************************************
@Function: Holder_Arm_Pitch_PidRun
@Description: 云台爪子pitch轴pid计算
@Called By: Holder_PidRun
@Input: void
@Others: 
*************************************************/
static void Holder_Arm_Pitch_PidRun(void)
{
		Holder.arm[0].TarSpeed = pid_absolute_update(Holder.arm[0].TarPosition, Holder.arm[0].Rx.Angle, &Holder.arm[0].PidPosition);
		Holder.arm[0].Output = pid_increment_update(Holder.arm[0].TarSpeed, Holder.arm[0].Rx.Speed, &Holder.arm[0].PidSpeed);
		Holder.arm[1].Output = -Holder.arm[0].Output;
}
/**
  * @brief  pid计算
  * @param  
  * @retval void
  * @attention 
  */
static void Holder_PidRun(void)
{
		#ifdef HOLDER_WAVE
		static int32_t tmp[5];
		static uint16_t test = 0;
				#ifdef HOLDER_ARM_YAW_WAVE

				tmp[0] += 5;
				tmp[1] = (int32_t)Holder.arm_yaw.Rx.Angle;
				tmp[2] = (int32_t)Holder.arm_yaw.TarSpeed;
				tmp[3] = (int32_t)Holder.arm_yaw.Rx.Speed;
				tmp[4] = (int32_t)Holder.arm_yaw.Output;
				#endif
				#ifdef HOLDER_ARM_PITCH_WAVE
	
				tmp[0] = (int32_t)Holder.arm[0].TarPosition;
				tmp[1] = (int32_t)Holder.arm[0].Rx.Angle;
				tmp[2] = (int32_t)Holder.arm[0].TarSpeed;
				tmp[3] = (int32_t)Holder.arm[0].Rx.Speed;
				tmp[4] = (int32_t)Holder.arm[0].Output;
				#endif
				test++;
				if(test%4 == 0)
				{
						UART2_SendWave(5, 4, &tmp[0], &tmp[1],
																 &tmp[2], &tmp[3],
																 &tmp[4]
													);
						test = 0;
				}

				
    #endif

    #ifdef HOLDER_DEBUG
        Holder_Pid_Init();
    #endif
	
		Holder.Lift_x.LPf.Angle = 0.8 * Holder.Lift_x.Rx.Angle + 0.2 * Holder.Lift_x.LPf.Angle;
		Holder.Lift_x.TarSpeed = pid_absolute_update(Holder.Lift_x.TarPosition, Holder.Lift_x.LPf.Angle, &Holder.Lift_x.PidPosition);
	
		Holder.Lift_x.LPf.TarSpeed = 0.8 * Holder.Lift_x.TarSpeed + 0.2 * Holder.Lift_x.LPf.TarSpeed;
		Holder.Lift_x.LPf.Speed = 0.8 * Holder.Lift_x.Rx.Speed + 0.2 * Holder.Lift_x.LPf.Speed;
		Holder.Lift_x.Output = pid_increment_update(Holder.Lift_x.LPf.TarSpeed, Holder.Lift_x.LPf.Speed, &Holder.Lift_x.PidSpeed);
	
		Holder.Lift_x.LPf.Output = 0.8 * Holder.Lift_x.Output + 0.2 * Holder.Lift_x.LPf.Output;
		
	
		Holder.lift[0].LPf.Angle = 0.8 * Holder.lift[0].Rx.Angle + 0.2 * Holder.lift[0].LPf.Angle;
		Holder.lift[0].TarSpeed = pid_absolute_update(Holder.lift[0].TarPosition, Holder.lift[0].LPf.Angle, &Holder.lift[0].PidPosition);
	
		Holder.lift[0].LPf.TarSpeed = 0.8 * Holder.lift[0].TarSpeed + 0.2 * Holder.lift[0].LPf.TarSpeed;
		Holder.lift[0].LPf.Speed = 0.8 * Holder.lift[0].Rx.Speed + 0.2 * Holder.lift[0].LPf.Speed;
		Holder.lift[0].Output = pid_increment_update(Holder.lift[0].LPf.TarSpeed, Holder.lift[0].LPf.Speed, &Holder.lift[0].PidSpeed);
	
		Holder.lift[0].LPf.Output = 0.8 * Holder.lift[0].Output + 0.2 * Holder.lift[0].LPf.Output;
		
		Holder.lift[1].LPf.Angle = 0.8 * Holder.lift[1].Rx.Angle + 0.2 * Holder.lift[1].LPf.Angle;
		Holder.lift[1].TarSpeed = pid_absolute_update(Holder.lift[1].TarPosition, Holder.lift[1].LPf.Angle, &Holder.lift[1].PidPosition);
	
		Holder.lift[1].LPf.TarSpeed = 0.8 * Holder.lift[1].TarSpeed + 0.2 * Holder.lift[1].LPf.TarSpeed;
		Holder.lift[1].LPf.Speed = 0.8 * Holder.lift[1].Rx.Speed + 0.2 * Holder.lift[1].LPf.Speed;
		Holder.lift[1].Output = pid_increment_update(Holder.lift[1].LPf.TarSpeed, Holder.lift[1].LPf.Speed, &Holder.lift[1].PidSpeed);
	
		Holder.lift[1].LPf.Output = 0.8 * Holder.lift[1].Output + 0.2 * Holder.lift[1].LPf.Output;
		
		Holder_Arm_Pitch_PidRun();
		Holder_Arm_Yaw_PidRun();
}

/**
  * @brief  云台电机can控制
  * @param  void
  * @retval void
  * @attention
  */
void Holder_CanTransmit(void)
{   
    if(Observer.Tx.DR16_Rate>15) /*< 遥控器保护，数据量16时才开启控制 */
    {
				Holder.Lift_x.LPf.Output =  constrain_int16_t(Holder.Lift_x.LPf.Output, -Holder_Out_Limit, Holder_Out_Limit);
				Holder.lift[0].LPf.Output = constrain_int16_t(Holder.lift[0].LPf.Output, -Holder_Out_Limit, Holder_Out_Limit);
				Holder.lift[1].LPf.Output = constrain_int16_t(Holder.lift[1].LPf.Output, -Holder_Out_Limit, Holder_Out_Limit);
				
				Holder.CanData[0] = (uint8_t)(Holder.arm[0].Output>>8);
				Holder.CanData[1] = (uint8_t)(Holder.arm[0].Output);
			
				Holder.CanData[2] = (uint8_t)(Holder.arm[1].Output>>8);
				Holder.CanData[3] = (uint8_t)(Holder.arm[1].Output);
			
				Holder.CanData[4] = (uint8_t)(Holder.Lift_x.LPf.Output>>8);
				Holder.CanData[5] =	(uint8_t)(Holder.Lift_x.LPf.Output);

				Holder.CanData[6] = (uint8_t)(Holder.lift[0].LPf.Output>>8);
				Holder.CanData[7] = (uint8_t)(Holder.lift[0].LPf.Output);
			
				Holder.CanData_0xff[0] =  (uint8_t)(Holder.lift[1].LPf.Output>>8);
				Holder.CanData_0xff[1] =  (uint8_t)(Holder.lift[1].LPf.Output);
				
				Holder.CanData_0xff[2] = (uint8_t)(Holder.arm_yaw.Output>>8);
				Holder.CanData_0xff[3] = (uint8_t)(Holder.arm_yaw.Output);
    }
    else
    {
        Holder_Angle_Reset(); /*< 关闭遥控器后，云台目标角度一直保持当前状态 */
    }

		CAN1_Transmit(0x200, Holder.CanData);
		CAN1_Transmit(0x1FF,Holder.CanData_0xff);
}
/*************************************************
@Function: Grab_Mode_Auto
@Description: 切换自动空接
@Called By: 
@Input: 
@Others: 爪子控制到时候整理一下放到一起（测试先）
*************************************************/
void Grab_Mode_Auto(void)
{
		HAL_GPIO_WritePin(GRAB_STATUS_GPIO_Port, GRAB_STATUS_Pin,  GPIO_PIN_RESET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(GRAB_MODE_GPIO_Port, GRAB_MODE_Pin,  GPIO_PIN_SET);
}
/*************************************************
@Function: 
@Description: 切换手动（也可当松开爪子用）
@Called By: 
@Input: 
@Others: 
*************************************************/
void Grab_Mode_handle(void)
{
		HAL_GPIO_WritePin(GRAB_MODE_GPIO_Port, GRAB_MODE_Pin,  GPIO_PIN_RESET);
}
/*************************************************
@Function:
@Description: 手动夹紧（确保是手动模式）
@Called By: 
@Input: 
@Others: 
*************************************************/
void Grab_Status_Set(void)
{
		if(HAL_GPIO_ReadPin(GRAB_MODE_GPIO_Port, GRAB_MODE_Pin) == GPIO_PIN_RESET)
		{
				HAL_GPIO_TogglePin(GRAB_STATUS_GPIO_Port, GRAB_STATUS_Pin);
		}
}
/*************************************************
@Function:
@Description: 手动松开
@Called By: 
@Input: 
@Others: 
*************************************************/
void Grab_Status_Reset(void)
{
		HAL_GPIO_WritePin(GRAB_STATUS_GPIO_Port, GRAB_STATUS_Pin,  GPIO_PIN_RESET);
}
/**
  * @brief  云台进程
  * @param  云台状态 云台控制
  * @retval void
  * @attention 目前视觉识别颜色需要改程序，后期改进
  */
void Holder_Process(HolderData_t HDMsg,RemoteData_t RDMsg)
{
		Holder_GetMoveData(RDMsg);
		Holder_PidRun();
		Holder_CanTransmit();
}

/************************************云台辅助函数*********************************/
/**
  * @brief  云台PID输出清零，角度保持（重置为当前角度）
  * @param  void
  * @retval void
  * @attention
  */
void Holder_Angle_Reset(void)
{
//		Holder.Lift_x.TarPosition = Holder.Lift_x.LPf.Angle;
//		lift2_Tarposition = Holder.lift[0].LPf.Angle;
//		Holder.lift[0].TarPosition = Holder.lift[0].LPf.Angle;
//		Holder.lift[1].TarPosition = Holder.lift[1].LPf.Angle;
		
    Holder.Lift_x.PidPosition.ctrOut = 0;
		Holder.Lift_x.PidPosition.errD = 0;
		Holder.Lift_x.PidPosition.errI = 0;
		Holder.Lift_x.PidPosition.errP = 0;
		Holder.Lift_x.PidPosition.errNow = 0;
		Holder.Lift_x.PidPosition.errOld = 0;
		
		Holder.Lift_x.PidSpeed.ctrOut = 0;
		Holder.Lift_x.PidSpeed.dCtrOut = 0;
		Holder.Lift_x.PidSpeed.errNow = 0;
		Holder.Lift_x.PidSpeed.errOld1 = 0;
		Holder.Lift_x.PidSpeed.errOld2 = 0;
	
	
		Holder.lift[0].PidPosition.ctrOut = 0;
		Holder.lift[0].PidPosition.errD = 0;
		Holder.lift[0].PidPosition.errI = 0;
		Holder.lift[0].PidPosition.errP = 0;
		Holder.lift[0].PidPosition.errNow = 0;
		Holder.lift[0].PidPosition.errOld = 0;
		
		Holder.lift[0].PidSpeed.ctrOut = 0;
		Holder.lift[0].PidSpeed.dCtrOut = 0;
		Holder.lift[0].PidSpeed.errNow = 0;
		Holder.lift[0].PidSpeed.errOld1 = 0;
		Holder.lift[0].PidSpeed.errOld2 = 0;
		
		Holder.lift[1].PidPosition.ctrOut = 0;
		Holder.lift[1].PidPosition.errD = 0;
		Holder.lift[1].PidPosition.errI = 0;
		Holder.lift[1].PidPosition.errP = 0;
		Holder.lift[1].PidPosition.errNow = 0;
		Holder.lift[1].PidPosition.errOld = 0;
		
		Holder.lift[1].PidSpeed.ctrOut = 0;
		Holder.lift[1].PidSpeed.dCtrOut = 0;
		Holder.lift[1].PidSpeed.errNow = 0;
		Holder.lift[1].PidSpeed.errOld1 = 0;
		Holder.lift[1].PidSpeed.errOld2 = 0;

    memset(Holder.CanData,0,sizeof(Holder.CanData)); /*< ????CAN */
		memset(Holder.CanData_0xff,0,sizeof(Holder.CanData));
}


/**
  * @brief  云台电机堵转保护
  * @param  void
  * @retval void
  * @attention 要想一个好的解决方案
  */
void Holder_Protect(void)
{ 

}


void Holder_Debug()
{
    #ifdef HOLDER_VISION_WAVE
//				vData_x  	= -1;
        UART2_SendWave(3, 2, &vData_x, &vData_y, &vData_z);
    #endif
}

/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
