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
#include "chassis_config.h"
#include "chassis.h"
#include <string.h>
#include "holder.h"
#include "usart.h"
#include "tim.h"
#include "can.h"
#include "fast_tri_func.h"
#include "misc_func.h"
#include "judge.h"
#include "judge_tx.h"
#include "ui.h"
#include "FreeRTOS.h"
#include "task.h"
/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/
#define CHASSIS_WAVE
//#define CHASSIS_DEBUG
#define GAIN_I 0.1
#define GAIN_J 0.1
#define GAIN_K 0.2
#define FOLLOW 1

#define SUP   1
#define SMID  3
#define SDOWN 2

#define UP_PITCH_ANGLE_IMU (3400)
#define DOWN_PITCH_ANGLE_IMU (4715)
#define UP_PITCH_ANGLE_MOTOR (4706)
#define DOWN_PITCH_ANGLE_MOTOR (3416)

#define RPC_ZERO(IN,RANGE)	((IN < RANGE && IN > -RANGE) ? 0 : IN) //感觉写反了 如果IN在 (-range, range)范围内，取0

/* variables -----------------------------------------------------------------*/
Chassis_t Chassis = {0};
int16_t front_temp,right_temp,roll_temp;

float fmax_speed_spin = 0; /*< 最大旋转速度 */
float fmax_wheel_speed = 0;

uint8_t turn_round_state = 0;


int16_t s16_max_front_speed = 320; /* 默认最大前后移动速度 */
int16_t s16_max_right_speed = 320; /* 默认最大左右移动速度 */


float chassis_pitch_angle = 0.0f;
//chassis_angle = (|imu_angle-4704|)/987 - (|motor_angle-3416|)/1269

/* function ------------------------------------------------------------------*/


/**
  * @brief  底盘PID初始化
  * @param  void
  * @retval void
  * @attention 
  */
void Chassis_Init(void)
{
    uint8_t i;
	
    for(i=0;i<4;i++)
    {
        pid_init_increment(&Chassis.M3508[i].PidSpeed,Chassis_Speed_Kp,
                           Chassis_Speed_Ki,Chassis_Speed_Kd,Chassis_Speed_Inc_Limit);
    }
    for(i=0;i<4;i++)
    {
        pid_init_increment(&Chassis.M3508[i].PidCurrent,Chassis_Current_Kp,
                           Chassis_Current_Ki,Chassis_Current_Kd,Chassis_Current_Inc_Limit);
    }
		pid_init_absolute(&Chassis.arm[0].PidPosition, chassis_arm_position.Kp, chassis_arm_position.Ki, chassis_arm_position.Kd, 0, 0);
		pid_init_increment(&Chassis.arm[0].PidSpeed, chassis_arm_position.Kp, chassis_arm_position.Ki, chassis_arm_position.Kd, 16000);
		pid_init_absolute(&Chassis.arm[1].PidPosition, chassis_arm1_position.Kp, chassis_arm1_position.Ki, chassis_arm1_position.Kd, 0, 0);
		pid_init_increment(&Chassis.arm[1].PidSpeed, chassis_arm1_position.Kp, chassis_arm1_position.Ki, chassis_arm1_position.Kd, 16000);
}
/**
  * @brief  获取底盘控制数据
  * @param  遥控器消息结构体
  * @retval void
  * @attention 
  */
#define FRONT      (0)
#define RIGHT      (1)
#define CLOCK_WISE (2)

int16_t chassis_ramp_speed = 10;
void Chassis_GetMoveData(RemoteData_t RDMsg)
{
		static int32_t arm_tmp = 0;
		static uint8_t delay_tick = 0;
		if((RDMsg.S1 == SMID) && (RDMsg.S2 == SUP))
		{
				Remote_Control_GetMoveData(RDMsg);
				arm_tmp += RDMsg.Ch1;
		}
		//中期检查测试用
		if(RDMsg.S1 == SUP)
		{
				if(delay_tick < 100)delay_tick++;
				if((RDMsg.KeyBoard.z == 1) && (delay_tick == 100))
				{
						HAL_GPIO_TogglePin(GRAB_FLOOR_GPIO_Port, GRAB_FLOOR_Pin);
						delay_tick = 0;
				}
				if((RDMsg.KeyBoard.x == 1) && (delay_tick == 100))
				{
						HAL_GPIO_TogglePin(CARD_GPIO_Port, CARD_Pin);
						delay_tick = 0;
				}
				if((RDMsg.KeyBoard.c == 1) && (delay_tick == 100))
				{
						HAL_GPIO_TogglePin(RESCUE_GPIO_Port,RESCUE_Pin);
						delay_tick = 0;
				}
		}
    /* 目标速度斜坡加速 稳定功率 */
		
    Chassis.MoveData.Right = Misc_RAMP_Int16(front_temp, Chassis.MoveData.Right, chassis_ramp_speed);
    Chassis.MoveData.Front = Misc_RAMP_Int16(right_temp, Chassis.MoveData.Front, chassis_ramp_speed);
		Chassis.MoveData.ClockWise = Misc_RAMP_Int16(roll_temp, Chassis.MoveData.ClockWise, chassis_ramp_speed);
		
		Chassis.arm[0].TarPosition = Misc_RAMP_Int32(arm_tmp, Chassis.arm[0].TarPosition, 500);
		Chassis.arm[1].TarPosition = -Chassis.arm[0].TarPosition;
}

/**
  * @brief  电机速度控制
  * @param  void
  * @retval void
  * @attention 
  */
static void Chassis_Speed_Control(void)
{
    uint8_t i;
    float tmp_speed[4];
    float cmp_index = 1;

    
		tmp_speed[0] = (+Chassis.MoveData.Right / GAIN_I 
							+Chassis.MoveData.Front / GAIN_J 
							+Chassis.MoveData.ClockWise / GAIN_K);

		tmp_speed[1] = (-Chassis.MoveData.Right / GAIN_I 
									+Chassis.MoveData.Front / GAIN_J 
									+Chassis.MoveData.ClockWise / GAIN_K);
		
		tmp_speed[2] = (-Chassis.MoveData.Right / GAIN_I 
									-Chassis.MoveData.Front / GAIN_J 
									+Chassis.MoveData.ClockWise / GAIN_K);
		
		tmp_speed[3] = (+Chassis.MoveData.Right / GAIN_I 
									-Chassis.MoveData.Front / GAIN_J  
									+Chassis.MoveData.ClockWise / GAIN_K);



    for(i = 0; i<4; i++)
    {
        Chassis.M3508[i].TarSpeed = (int16_t)(tmp_speed[i]*cmp_index);
    }
		//为速度分级预留
}
/*************************************************
@Function: Chassis_Arm_PidRun
@Description: 抓地面矿石的爪子
@Called By: Chassis_PidRun
@Input: void
@Others: 
*************************************************/
static void Chassis_Arm_PidRun(void)
{
		Chassis.arm[0].TarSpeed = pid_absolute_update(Chassis.arm[0].TarPosition, Chassis.arm[0].Rx.Angle, &Chassis.arm[0].PidPosition);
		Chassis.arm[0].Output = pid_increment_update(Chassis.arm[0].TarSpeed, Chassis.arm[0].Rx.Speed, &Chassis.arm[0].PidSpeed);
		Chassis.arm[1].TarSpeed = pid_absolute_update(Chassis.arm[1].TarPosition, Chassis.arm[1].Rx.Angle, &Chassis.arm[1].PidPosition);
		Chassis.arm[1].Output = pid_increment_update(Chassis.arm[1].TarSpeed, Chassis.arm[1].Rx.Speed, &Chassis.arm[1].PidSpeed);
}
/**
  * @brief  底盘PID输出
  * @param  void
  * @retval void
  * @attention （通过速度环（外环）所得值再计算电流环，通过电流坏（内环）直接控制输出）（rx,lpf???）
  */
void Chassis_PidRun(void)
{
    uint8_t i;

    #ifdef CHASSIS_WAVE
        UART2_SendWave(5, 2, &Chassis.M3508[0].TarSpeed, &Chassis.M3508[0].Rx.Speed,
                             &Chassis.M3508[0].TarCurrent, &Chassis.M3508[0].Rx.Current,
                             &Chassis.M3508[0].LPf.Output);
    #endif

    #ifdef CHASSIS_DEBUG
        Chassis_Init();
    #endif

    for (i = 0; i < 4; i++)     //rx speed lpf
    {
        Chassis.M3508[i].LPf.Speed = 0.8 * Chassis.M3508[i].Rx.Speed + 0.2 * Chassis.M3508[i].LPf.Speed;
    }   
    for (i = 0; i < 4; i++)     //speed loop
    {
        Chassis.M3508[i].TarCurrent = pid_increment_update(Chassis.M3508[i].TarSpeed, Chassis.M3508[i].LPf.Speed, &Chassis.M3508[i].PidSpeed);
    }
    for (i = 0; i < 4; i++)     //tar current lpf 
    {
        Chassis.M3508[i].LPf.TarCurrent = 0.8 * Chassis.M3508[i].TarCurrent + 0.2 * Chassis.M3508[i].LPf.TarCurrent;
    }
    
    for (i = 0; i < 4; i++)     //rx current lpf 
    {
        Chassis.M3508[i].LPf.Current = 0.8 * Chassis.M3508[i].Rx.Current + 0.2 * Chassis.M3508[i].LPf.Current;
    }
    
    for (i = 0; i < 4; i++)     //current loop
    {
        Chassis.M3508[i].Output = pid_increment_update(Chassis.M3508[i].LPf.TarCurrent, Chassis.M3508[i].LPf.Current, &Chassis.M3508[i].PidCurrent);
    }
    
    for (i = 0; i < 4; i++)     //out lpf
    {
        Chassis.M3508[i].LPf.Output = 0.8 * Chassis.M3508[i].Output + 0.2 * Chassis.M3508[i].LPf.Output;
    }
		
		Chassis_Arm_PidRun();
}


/**
  * @brief  底盘电机CAN
  * @param  void
  * @retval void
  * @attention 放中断里面
  */
void Chassis_CanTransmit(void)
{
    uint8_t i;
    if(Observer.Tx.DR16_Rate>15) /*< 遥控器保护，数据量16时才开启控制 */
    {
        for(i=0;i<4;i++)
        {
            /* 输出限幅 */
            Chassis.M3508[i].LPf.Output = constrain_int16_t(Chassis.M3508[i].LPf.Output,
                                                    -Chassis_Out_Limit, Chassis_Out_Limit);
            
            /* CAN 赋值 */
            Chassis.CanData[2*i]=(uint8_t)(Chassis.M3508[i].LPf.Output>>8);
            Chassis.CanData[2*i+1]=(uint8_t)(Chassis.M3508[i].LPf.Output);
        }
				for(i = 0; i < 2; i++)
				{
						Chassis.CanData_0xff[2*i] = (uint8_t)(Chassis.arm[i].Output>>8);
						Chassis.CanData_0xff[2*i + 1] = (uint8_t)(Chassis.arm[i].Output);
				}
    }
    else
    {
        Chassis_Speed_Reset(); /*< 关闭遥控器后，底盘目标速度一直保持当前状态（0） */
    }
    CAN2_Transmit(0x200,Chassis.CanData);
    CAN2_Transmit(0x1FF,Chassis.CanData_0xff);
}
/**
  * @brief  底盘进程
  * @param  控制指令结构体
  * @retval void
* @attention 保护写在CAN发送前，（V）一键复位，PID误差和输出清零
  */
void Chassis_Process(RemoteData_t RDMsg)
{
    Chassis_GetMoveData(RDMsg);
//    Chassis_Protect(RDMsg);
//    Turn_Round(RDMsg);    /*< 一键掉头*/
    Chassis_Speed_Control();
    Chassis_PidRun();
    Chassis_CanTransmit();
}

/*********************************底盘辅助函数*********************************/


/**
  * @brief  底盘速度初始化，PID输出清零
  * @param  void
  * @retval void
  * @attention 
  */
void Chassis_Speed_Reset(void)
{
    uint8_t i;
    
    Chassis.MoveData.Front = 0;
    Chassis.MoveData.ClockWise = 0;
    Chassis.MoveData.Right = 0;
    for(i = 0; i < 4; i++)
    {
        Chassis.M3508[i].TarSpeed            = 0;
        Chassis.M3508[i].PidSpeed.errNow     = 0;
        Chassis.M3508[i].PidSpeed.errOld1    = 0;
        Chassis.M3508[i].PidSpeed.errOld2    = 0;
        Chassis.M3508[i].PidSpeed.ctrOut     = 0;
        Chassis.M3508[i].PidSpeed.dCtrOut    = 0;
        Chassis.M3508[i].PidCurrent.errNow   = 0;
        Chassis.M3508[i].PidCurrent.errOld1  = 0;
        Chassis.M3508[i].PidCurrent.errOld2  = 0;
        Chassis.M3508[i].PidCurrent.dCtrOut  = 0;
        Chassis.M3508[i].PidCurrent.ctrOut   = 0;
    }
    memset(Chassis.CanData,0,sizeof(Chassis.CanData));
		memset(Holder.CanData_0xff,0,sizeof(Holder.CanData));
}


/**
  * @brief  底盘速度初始化，PID输出清零
  * @param  控制指令结构体
  * @retval void
  * @attention 
  */
void Chassis_Protect(RemoteData_t RDMsg)
{
    if (RDMsg.KeyBoard.v == 1)
    {

    }
}

/**
  * @brief  (Q键)一键掉头，云台偏航+180度，底盘与云台差角+180度	(先为工程预留)
  * @param  控制指令结构体
  * @retval void
  * @attention 
  */
void Turn_Round(RemoteData_t RDMsg)
{

}


/**
  * @brief  键盘控制速度解算
  * @param  遥控器消息结构体
  * @retval 
  * @attention 
  */
void Keyboard_Control_GetMoveData(RemoteData_t RDMsg)
{
    if(RDMsg.KeyBoard.w)
        front_temp = s16_max_front_speed;
    else if((!RDMsg.KeyBoard.w)&&(RDMsg.KeyBoard.s))
        front_temp =-s16_max_front_speed;
    else
        front_temp = 0;
    
    if(RDMsg.KeyBoard.d)
        right_temp = s16_max_right_speed;
    else if((!RDMsg.KeyBoard.d)&&(RDMsg.KeyBoard.a))
        right_temp =-s16_max_right_speed;
    else
        right_temp = 0;
    
    if(RDMsg.KeyBoard.ctrl)
    {
        front_temp = (front_temp>0? s16_max_front_speed:-s16_max_front_speed)*1.5;
        right_temp = (right_temp>0? s16_max_right_speed:-s16_max_right_speed)*1.5;
    }else if((!RDMsg.KeyBoard.ctrl)&&(RDMsg.KeyBoard.shift))
    {
        front_temp = (front_temp>0? s16_max_front_speed:-s16_max_front_speed)*0.7;;
        right_temp = (right_temp>0? s16_max_right_speed:-s16_max_right_speed)*0.7;
    }
}

/**
  * @brief  遥控器控制速度解算
  * @param  遥控器消息结构体
  * @retval 
  * @attention 
  */
void Remote_Control_GetMoveData(RemoteData_t RDMsg)
{
    right_temp = RPC_ZERO(RDMsg.Ch2, 10);
    front_temp = RPC_ZERO(RDMsg.Ch3, 10);
		roll_temp = RPC_ZERO(RDMsg.Ch0, 10);
    
}

/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
