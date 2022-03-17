/**
  ******************************************************************************
  * @file    holder.c
  * @author  sy xl qj
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
  * 1.先开环测试是否存在机械问题
  * 2.计算函数尽量自定义，不要用math.h库
  ******************************************************************************
  */

/* includes ------------------------------------------------------------------*/
#include "config.h"
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

#define MAX_PITCH_ANGLE (-3370)  /*< 电机模式600  陀螺仪模式450 */
#define MIN_PITCH_ANGLE (-4775) /*< 电机模式-530 陀螺仪模式330 */
/* variables -----------------------------------------------------------------*/
Holder_t Holder;
static uint8_t holder_ctrl_state; /*< 云台控制模式 手动或视觉*/
/* function ------------------------------------------------------------------*/

/**
  * @brief  云台PID控制，手动控制
  * @param  void
  * @retval void
  * @attention 电机失控：输出低通滤波调低，效果明显
  */
static void Holder_Pid_Manual(void)
{
    static uint8_t init_state = 0;

    #ifdef HOLDER_DEBUG
    /* Pitch 电机反馈*/
        pid_init_absolute(&Holder.Pitch.PidAngle, Pitch_Angle_kp, Pitch_Angle_ki, Pitch_Angle_kd, 500, 9999); //0.21, 0, 0.3, 30, 9999
        pid_init_predifferential(&Holder.Pitch.PidSpeed_pre, Pitch_Speed_kp, Pitch_Speed_ki, Pitch_Speed_kd, 0.8, 9999); //228, 10, 30, 200, 9999
    
    /* Yaw 陀螺仪反馈*/
        pid_init_absolute(&Holder.Yaw.PidAngle, Yaw_Angle_kp, Yaw_Angle_ki, Yaw_Angle_kd, 999999, 999999);
        pid_init_absolute(&Holder.Yaw.PidSpeed, Yaw_Speed_kp, Yaw_Speed_ki, Yaw_Speed_kd, 999999, 999999);
    #endif

    if (init_state == 0) /*< 未初始化 */
    {
        /* Yaw 陀螺仪反馈*/
        pid_init_absolute(&Holder.Yaw.PidAngle, Yaw_Angle_kp, Yaw_Angle_ki, Yaw_Angle_kd, 999999, 999999);
        pid_init_absolute(&Holder.Yaw.PidSpeed, Yaw_Speed_kp, Yaw_Speed_ki, Yaw_Speed_kd, 999999, 999999);
        
        /* Pitch 电机反馈*/
        pid_init_absolute(&Holder.Pitch.PidAngle, Pitch_Angle_kp, Pitch_Angle_ki, Pitch_Angle_kd, 500, 9999); //0.21, 0, 0.3, 30, 9999
        pid_init_predifferential(&Holder.Pitch.PidSpeed_pre, Pitch_Speed_kp, Pitch_Speed_ki, Pitch_Speed_kd, 0.8, 28000); //228, 10, 30, 200, 9999
        
        init_state = 1;
    }
    else if (init_state == 1)/*< 已初始化 */
    {
        Holder_Pi_Tunning_Absolute(Yaw_Angle_Tunning_Kp, Yaw_Angle_Min_Kp, Yaw_Angle_ki, 0,Yaw_Angle_kd, &Holder.Yaw.PidAngle);
        Holder.Yaw.TarSpeedLpf = pid_absolute_update(Holder.Yaw.TarAngle,Holder.Yaw.Angle,&Holder.Yaw.PidAngle);
        Holder.Yaw.TarSpeed = 0.8 * Holder.Yaw.TarSpeedLpf + 0.2 * Holder.Yaw.TarSpeed;
        Holder.Yaw.OutputLpf = pid_absolute_update(Holder.Yaw.TarSpeed,Holder.Yaw.Speed,&Holder.Yaw.PidSpeed);
        Holder.Yaw.Output = 0.8 * Holder.Yaw.OutputLpf + 0.2 * Holder.Yaw.Output;
        Holder.Yaw.Output = Constrain_Int32_t(Holder.Yaw.Output, -28000, 28000);


        Holder_Pi_Tunning_Absolute(Pitch_Angle_Tunnng_Kp, Pitch_Angle_Min_Kp, Pitch_Angle_ki, 0,Pitch_Angle_kd, &Holder.Pitch.PidAngle);
        Holder.Pitch.TarSpeedLpf = pid_absolute_update(Holder.Pitch.TarAngle,Holder.Pitch.Angle,&Holder.Pitch.PidAngle);
        Holder.Pitch.TarSpeed = 0.8 * Holder.Pitch.TarSpeedLpf + 0.2 * Holder.Pitch.TarSpeed;
        Holder.Pitch.OutputLpf = pid_predifferential_update(Holder.Pitch.TarSpeed,Holder.Pitch.Speed,&Holder.Pitch.PidSpeed_pre);
        Holder.Pitch.Output = 0.8 * Holder.Pitch.OutputLpf + 0.2 * Holder.Pitch.Output;
        Holder.Pitch.Output = Constrain_Int32_t(Holder.Pitch.Output, -28000, 28000);
    }
}


/**
  * @brief  云台PID控制，视觉控制
  * @param  void
  * @retval void
  * @attention 变结构P减少超调，效果明显
  */
static void Holder_Pid_Vision(void)
{
    static uint8_t init_state = 0;

    if (init_state == 0) /*< 未初始化 */
    {
        /* Yaw 陀螺仪反馈 */
        pid_init_absolute(&Holder.Yaw.PidAngle, Yaw_Angle_kp, Yaw_Angle_ki, Yaw_Angle_kd, 9999, 9999);
        pid_init_absolute(&Holder.Yaw.PidSpeed, Yaw_Speed_kp, Yaw_Speed_ki, Yaw_Speed_kd, 9999, 9999);

        /* Pitch 电机反馈 */
        pid_init_absolute(&Holder.Pitch.PidAngle, Pitch_Angle_kp, Pitch_Angle_ki, Pitch_Angle_kd, 500, 120); //0.21, 0, 0.3, 30, 9999
        pid_init_predifferential(&Holder.Pitch.PidSpeed_pre, Pitch_Speed_kp, Pitch_Speed_ki, Pitch_Speed_kd, 0.8, 28000); //228, 10, 30, 200, 9999

        init_state = 1;
    }
    else if (init_state == 1)/*< 已初始化 */
    {
        Holder.Yaw.TarAngle = Holder.Yaw.Angle+vData.Pos.x*180;
        
        float y_ = Holder.Yaw.TarAngle;
//        UART2_SendWave(2,4,&y_, &vData.Pos.x);
//        float pos_ = vData.Pos.x*8192*100/2160;
//        UART2_SendWave(1,4,&pos_);
        Holder_Pi_Tunning_Absolute(Yaw_Angle_Tunning_Kp, Yaw_Angle_Min_Kp, Yaw_Angle_ki, 0,Yaw_Angle_kd, &Holder.Yaw.PidAngle);
        Holder.Yaw.TarSpeedLpf = pid_absolute_update(Holder.Yaw.TarAngle,Holder.Yaw.Angle,&Holder.Yaw.PidAngle);
        Holder.Yaw.TarSpeed = 0.8 * Holder.Yaw.TarSpeedLpf + 0.2 * Holder.Yaw.TarSpeed;
        Holder.Yaw.OutputLpf = pid_absolute_update(Holder.Yaw.TarSpeed,Holder.Yaw.Speed,&Holder.Yaw.PidSpeed);
        Holder.Yaw.Output = 0.8 * Holder.Yaw.OutputLpf + 0.2 * Holder.Yaw.Output;
        Holder.Yaw.Output = Constrain_Int32_t(Holder.Yaw.Output, -28000, 28000);

        Holder_Pi_Tunning_Absolute(Pitch_Angle_Tunnng_Kp, Pitch_Angle_Min_Kp, Pitch_Angle_ki, 0,Pitch_Angle_kd, &Holder.Pitch.PidAngle);
        Holder.Pitch.TarSpeedLpf = pid_absolute_update(Holder.Pitch.TarAngle,Holder.Pitch.Angle,&Holder.Pitch.PidAngle);
        Holder.Pitch.TarSpeed = 0.8 * Holder.Pitch.TarSpeedLpf + 0.2 * Holder.Pitch.TarSpeed;
        Holder.Pitch.OutputLpf = pid_predifferential_update(Holder.Pitch.TarSpeed,Holder.Pitch.Speed,&Holder.Pitch.PidSpeed_pre);
        Holder.Pitch.Output = 0.8 * Holder.Pitch.OutputLpf + 0.2 * Holder.Pitch.Output;
        Holder.Pitch.Output = Constrain_Int32_t(Holder.Pitch.Output, -28000, 28000);
    }
}


/**
  * @brief  云台PID控制
  * @param  void
  * @retval void
  * @attention 多种控制器混控，效果不是特别好
  *            vision数据是目标与图像中心的偏差值
  */
static void Holder_PidRun(void)
{
	/* 自瞄 */
	if (holder_ctrl_state == VISION)
	{
        Holder_Pid_Vision();
	}
	/* 手动 */
	else if (holder_ctrl_state == MANUAL)
	{
        Holder_Pid_Manual();
	}
}


/**
  * @brief  获取云台控制数据
  * @param  控制消息结构体
  * @retval void
  * @attention 暂时只有键盘控制，后期加上
  */

static void Holder_GetMoveData(RemoteData_t RDMsg)
{
    static uint8_t vision_init = 0;
    /* 遥控器控制 */
    if (RDMsg.S1 == 1)
    {
        /* 遥控器控制 */
        Holder.Pitch.TarAngle += 0.05f * RDMsg.Ch1;
        Holder.Yaw.TarAngle   -= 0.8f * RDMsg.Ch0;
        
//        int16_t r = RDMsg.Wheel;
//        UART2_SendWave(1,2,&r);
    }
    else
    {
        int32_t x = 0; /*< 鼠标移动速度，注意是速度，积分后才是位移*/
        int32_t y = 0;

        /* 按住鼠标右键开启视觉自瞄 */
        if (RDMsg.MouseClick_right == 1 && vData.Pos.z != -1) /*<  && vData.Pos.z != -1 */
        {
            
            if (vision_init == 0)
            {
                Holder_Angle_Reset();
                vision_init = 1;
            }
            
            holder_ctrl_state = VISION; /*< 开启自瞄状态*/
            Holder.Yaw.TarAngle -= 0;
            
            y= -RDMsg.Mouse_y; /*< 俯仰微动 */

            if (y > 0)
            {
                Holder.Pitch.TarAngle   +=  1.7f*Quake_Sqrt(y);
            }
            else if (y < 0)
            {
                Holder.Pitch.TarAngle   -=  1.7f*Quake_Sqrt(-y);
            }
        }
        else
        {   
            if (vision_init == 1)
            {
                Holder_Angle_Reset();
                vision_init = 0;
            }
            
            holder_ctrl_state = MANUAL; /*< 开启手动状态*/
            
            x=  RDMsg.Mouse_x;

            y= -RDMsg.Mouse_y;
  
            x = Constrain_Int32_t(x, -420, 420);
            Holder.Yaw.TarAngle   -=  x;
            
            if (y > 0)
            {
                Holder.Pitch.TarAngle   +=  1.7f*Quake_Sqrt(y);
            }
            else if (y < 0)
            {
                Holder.Pitch.TarAngle   -=  1.7f*Quake_Sqrt(-y);
            }
            
        }
    }
	
	/* 角度保护 */
	if(Holder.Pitch.TarAngle > MAX_PITCH_ANGLE)
	{
		Holder.Pitch.TarAngle = MAX_PITCH_ANGLE;
	}

	if(Holder.Pitch.TarAngle < MIN_PITCH_ANGLE)
	{
		Holder.Pitch.TarAngle = MIN_PITCH_ANGLE;
	}
}


/**
  * @brief  获取云台状态数据
  * @param  云台状态结构体
  * @retval void
  * @attention 俯仰Pitch角度范围 (-540, 540)
  */
static void Holder_MsgIn(HolderData_t HDMsg)
{   
    /* 陀螺仪反馈 */
    Holder.Yaw.Angle = HDMsg.Angle[2]*100.0f; //HDMsg.Angle[2];乘以100扩大精度
    Holder.Yaw.Speed = HDMsg.Gyro[2];         //HDMsg.Gyro[2]
    
    /* 电机反馈 */
    int32_t tmp_pitch_angle = Holder.Pitch._0x209.Rx.Angle;
    
    /* 俯仰抬头到底最大540，低头减小到0后，突变为8192，再减小到7630 */
    if (tmp_pitch_angle > 1000) /*< 大于540，小于7630就行，保险设中间值1000*/
    {
        Holder.Pitch.Angle = tmp_pitch_angle - 8191;
    }
    else
    {
        Holder.Pitch.Angle = tmp_pitch_angle;
    }

    Holder.Pitch.Speed = Holder.Pitch._0x209.Rx.Speed;
}


/**
  * @brief  云台参数初始化
  * @param  云台状态结构体
  * @retval void
  * @attention
  */
void Holder_Init(HolderData_t HDMsg)
{
    Holder_MsgIn(HDMsg);
	Holder_Angle_Reset();
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
        Holder.CanData[0] = Holder.Pitch.Output>>8;
        Holder.CanData[1] = Holder.Pitch.Output;        
        Holder.CanData[2] = Holder.Yaw.Output>>8;
        Holder.CanData[3] = Holder.Yaw.Output;
    }   
    else
    {
        Holder_Angle_Reset(); /*< 关闭遥控器后，云台目标角度一直保持当前状态 */
        memset(Holder.CanData,0,sizeof(Holder.CanData));
    }
    CAN1_Transmit(0x2FF,Holder.CanData);
}


/**
  * @brief  向妙算发送数据
  * @param  void
  * @retval void
  * @attention 放在led任务里面没问题，后面要加osDelay
  */
//void Send_Vision(void)
//{
//    
//    osDelay(50); 
//}


/**
  * @brief  云台进程
  * @param  云台状态 云台控制
  * @retval void
  * @attention 目前视觉识别颜色需要改程序，后期改进
  */
void Holder_Process(HolderData_t HDMsg,RemoteData_t RDMsg)
{
	Holder_MsgIn(HDMsg);
    Holder_GetMoveData(RDMsg);
    Holder_Angle_Protect(RDMsg);    
    Holder_PidRun();
    UART2_SendWave(1,2,&RDMsg.Ch0);
//	UART2_SendWave(5, 4, &Holder.Pitch.TarAngle, &Holder.Pitch.Angle, 
//                        &Holder.Pitch.TarSpeed, &Holder.Pitch.Speed, &Holder.Pitch.Output);
//	UART2_SendWave(5, 4, &Holder.Yaw.TarAngle, &Holder.Yaw.Angle, 
//                         &Holder.Yaw.TarSpeed, &Holder.Yaw.Speed, &Holder.Yaw.Output);
 
//    float a = Holder.Yaw.Angle;
//    float b = Holder.Yaw.Output;
//    float dangle = Get_fDirAngel();
//    UART2_SendWave(3, 4, &a, &b, &dangle);

//    Get_VisionData(); /*< 解析视觉数据 */
//    UART2_SendWave(3, 4, &vData.Pos.x, &vData.Pos.y, &vData.Pos.z);
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
    Get_Mc_Imu_Pitch()->Circle = 0; /*< 连续化圈数清零，防止数据超限*/
    Holder.Yaw.TarAngle   = Holder.Yaw.Angle;
	Holder.Pitch.TarAngle = Holder.Pitch.Angle;
    Holder.Pitch.PidAngle.ctrOut = 0;
    Holder.Pitch.PidAngle.errD   = 0;
    Holder.Pitch.PidAngle.errI   = 0;
    Holder.Pitch.PidSpeed.ctrOut = 0;
    Holder.Pitch.PidAngle.errD   = 0;
    Holder.Pitch.PidAngle.errI   = 0;
}


/**
  * @brief  云台角度保护，防止疯转
  * @param  void
  * @retval void
  * @attention
  */
void Holder_Angle_Protect(RemoteData_t RDMsg)
{ 
    /*（V）一键手动复位，底盘也复位（误差和输出清零） */
    if (RDMsg.KeyBoard.v == 1)
    {
        Holder_Angle_Reset();
    }
}


/**
  * @brief  向外提供云台结构体
  * @param  void
  * @retval 云台结构体
  * @attention 
  */
Holder_t *Get_Holder_t(void)
{
	return &Holder;
}
/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
