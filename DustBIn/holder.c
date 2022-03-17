/**
  ******************************************************************************
  * @file    holder.c
  * @author  sy xl qj
  * @brief   ��̨����
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
  * ��̨ע�����
  * 1.�ȿ��������Ƿ���ڻ�е����
  * 2.���㺯�������Զ��壬��Ҫ��math.h��
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
#define VISION (1) /*< �Ӿ�ģʽ */
#define MANUAL (0) /*< �ֶ�ģʽ */

#define MAX_PITCH_ANGLE (-3370)  /*< ���ģʽ600  ������ģʽ450 */
#define MIN_PITCH_ANGLE (-4775) /*< ���ģʽ-530 ������ģʽ330 */
/* variables -----------------------------------------------------------------*/
Holder_t Holder;
static uint8_t holder_ctrl_state; /*< ��̨����ģʽ �ֶ����Ӿ�*/
/* function ------------------------------------------------------------------*/

/**
  * @brief  ��̨PID���ƣ��ֶ�����
  * @param  void
  * @retval void
  * @attention ���ʧ�أ������ͨ�˲����ͣ�Ч������
  */
static void Holder_Pid_Manual(void)
{
    static uint8_t init_state = 0;

    #ifdef HOLDER_DEBUG
    /* Pitch �������*/
        pid_init_absolute(&Holder.Pitch.PidAngle, Pitch_Angle_kp, Pitch_Angle_ki, Pitch_Angle_kd, 500, 9999); //0.21, 0, 0.3, 30, 9999
        pid_init_predifferential(&Holder.Pitch.PidSpeed_pre, Pitch_Speed_kp, Pitch_Speed_ki, Pitch_Speed_kd, 0.8, 9999); //228, 10, 30, 200, 9999
    
    /* Yaw �����Ƿ���*/
        pid_init_absolute(&Holder.Yaw.PidAngle, Yaw_Angle_kp, Yaw_Angle_ki, Yaw_Angle_kd, 999999, 999999);
        pid_init_absolute(&Holder.Yaw.PidSpeed, Yaw_Speed_kp, Yaw_Speed_ki, Yaw_Speed_kd, 999999, 999999);
    #endif

    if (init_state == 0) /*< δ��ʼ�� */
    {
        /* Yaw �����Ƿ���*/
        pid_init_absolute(&Holder.Yaw.PidAngle, Yaw_Angle_kp, Yaw_Angle_ki, Yaw_Angle_kd, 999999, 999999);
        pid_init_absolute(&Holder.Yaw.PidSpeed, Yaw_Speed_kp, Yaw_Speed_ki, Yaw_Speed_kd, 999999, 999999);
        
        /* Pitch �������*/
        pid_init_absolute(&Holder.Pitch.PidAngle, Pitch_Angle_kp, Pitch_Angle_ki, Pitch_Angle_kd, 500, 9999); //0.21, 0, 0.3, 30, 9999
        pid_init_predifferential(&Holder.Pitch.PidSpeed_pre, Pitch_Speed_kp, Pitch_Speed_ki, Pitch_Speed_kd, 0.8, 28000); //228, 10, 30, 200, 9999
        
        init_state = 1;
    }
    else if (init_state == 1)/*< �ѳ�ʼ�� */
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
  * @brief  ��̨PID���ƣ��Ӿ�����
  * @param  void
  * @retval void
  * @attention ��ṹP���ٳ�����Ч������
  */
static void Holder_Pid_Vision(void)
{
    static uint8_t init_state = 0;

    if (init_state == 0) /*< δ��ʼ�� */
    {
        /* Yaw �����Ƿ��� */
        pid_init_absolute(&Holder.Yaw.PidAngle, Yaw_Angle_kp, Yaw_Angle_ki, Yaw_Angle_kd, 9999, 9999);
        pid_init_absolute(&Holder.Yaw.PidSpeed, Yaw_Speed_kp, Yaw_Speed_ki, Yaw_Speed_kd, 9999, 9999);

        /* Pitch ������� */
        pid_init_absolute(&Holder.Pitch.PidAngle, Pitch_Angle_kp, Pitch_Angle_ki, Pitch_Angle_kd, 500, 120); //0.21, 0, 0.3, 30, 9999
        pid_init_predifferential(&Holder.Pitch.PidSpeed_pre, Pitch_Speed_kp, Pitch_Speed_ki, Pitch_Speed_kd, 0.8, 28000); //228, 10, 30, 200, 9999

        init_state = 1;
    }
    else if (init_state == 1)/*< �ѳ�ʼ�� */
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
  * @brief  ��̨PID����
  * @param  void
  * @retval void
  * @attention ���ֿ�������أ�Ч�������ر��
  *            vision������Ŀ����ͼ�����ĵ�ƫ��ֵ
  */
static void Holder_PidRun(void)
{
	/* ���� */
	if (holder_ctrl_state == VISION)
	{
        Holder_Pid_Vision();
	}
	/* �ֶ� */
	else if (holder_ctrl_state == MANUAL)
	{
        Holder_Pid_Manual();
	}
}


/**
  * @brief  ��ȡ��̨��������
  * @param  ������Ϣ�ṹ��
  * @retval void
  * @attention ��ʱֻ�м��̿��ƣ����ڼ���
  */

static void Holder_GetMoveData(RemoteData_t RDMsg)
{
    static uint8_t vision_init = 0;
    /* ң�������� */
    if (RDMsg.S1 == 1)
    {
        /* ң�������� */
        Holder.Pitch.TarAngle += 0.05f * RDMsg.Ch1;
        Holder.Yaw.TarAngle   -= 0.8f * RDMsg.Ch0;
        
//        int16_t r = RDMsg.Wheel;
//        UART2_SendWave(1,2,&r);
    }
    else
    {
        int32_t x = 0; /*< ����ƶ��ٶȣ�ע�����ٶȣ����ֺ����λ��*/
        int32_t y = 0;

        /* ��ס����Ҽ������Ӿ����� */
        if (RDMsg.MouseClick_right == 1 && vData.Pos.z != -1) /*<  && vData.Pos.z != -1 */
        {
            
            if (vision_init == 0)
            {
                Holder_Angle_Reset();
                vision_init = 1;
            }
            
            holder_ctrl_state = VISION; /*< ��������״̬*/
            Holder.Yaw.TarAngle -= 0;
            
            y= -RDMsg.Mouse_y; /*< ����΢�� */

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
            
            holder_ctrl_state = MANUAL; /*< �����ֶ�״̬*/
            
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
	
	/* �Ƕȱ��� */
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
  * @brief  ��ȡ��̨״̬����
  * @param  ��̨״̬�ṹ��
  * @retval void
  * @attention ����Pitch�Ƕȷ�Χ (-540, 540)
  */
static void Holder_MsgIn(HolderData_t HDMsg)
{   
    /* �����Ƿ��� */
    Holder.Yaw.Angle = HDMsg.Angle[2]*100.0f; //HDMsg.Angle[2];����100���󾫶�
    Holder.Yaw.Speed = HDMsg.Gyro[2];         //HDMsg.Gyro[2]
    
    /* ������� */
    int32_t tmp_pitch_angle = Holder.Pitch._0x209.Rx.Angle;
    
    /* ����̧ͷ�������540����ͷ��С��0��ͻ��Ϊ8192���ټ�С��7630 */
    if (tmp_pitch_angle > 1000) /*< ����540��С��7630���У��������м�ֵ1000*/
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
  * @brief  ��̨������ʼ��
  * @param  ��̨״̬�ṹ��
  * @retval void
  * @attention
  */
void Holder_Init(HolderData_t HDMsg)
{
    Holder_MsgIn(HDMsg);
	Holder_Angle_Reset();
}


/**
  * @brief  ��̨���can����
  * @param  void
  * @retval void
  * @attention
  */
void Holder_CanTransmit(void)
{   
    if(Observer.Tx.DR16_Rate>15) /*< ң����������������16ʱ�ſ������� */
    {
        Holder.CanData[0] = Holder.Pitch.Output>>8;
        Holder.CanData[1] = Holder.Pitch.Output;        
        Holder.CanData[2] = Holder.Yaw.Output>>8;
        Holder.CanData[3] = Holder.Yaw.Output;
    }   
    else
    {
        Holder_Angle_Reset(); /*< �ر�ң��������̨Ŀ��Ƕ�һֱ���ֵ�ǰ״̬ */
        memset(Holder.CanData,0,sizeof(Holder.CanData));
    }
    CAN1_Transmit(0x2FF,Holder.CanData);
}


/**
  * @brief  �����㷢������
  * @param  void
  * @retval void
  * @attention ����led��������û���⣬����Ҫ��osDelay
  */
//void Send_Vision(void)
//{
//    
//    osDelay(50); 
//}


/**
  * @brief  ��̨����
  * @param  ��̨״̬ ��̨����
  * @retval void
  * @attention Ŀǰ�Ӿ�ʶ����ɫ��Ҫ�ĳ��򣬺��ڸĽ�
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

//    Get_VisionData(); /*< �����Ӿ����� */
//    UART2_SendWave(3, 4, &vData.Pos.x, &vData.Pos.y, &vData.Pos.z);
}

/************************************��̨��������*********************************/
/**
  * @brief  ��̨PID������㣬�Ƕȱ��֣�����Ϊ��ǰ�Ƕȣ�
  * @param  void
  * @retval void
  * @attention
  */
void Holder_Angle_Reset(void)
{
    Get_Mc_Imu_Pitch()->Circle = 0; /*< ������Ȧ�����㣬��ֹ���ݳ���*/
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
  * @brief  ��̨�Ƕȱ�������ֹ��ת
  * @param  void
  * @retval void
  * @attention
  */
void Holder_Angle_Protect(RemoteData_t RDMsg)
{ 
    /*��V��һ���ֶ���λ������Ҳ��λ������������㣩 */
    if (RDMsg.KeyBoard.v == 1)
    {
        Holder_Angle_Reset();
    }
}


/**
  * @brief  �����ṩ��̨�ṹ��
  * @param  void
  * @retval ��̨�ṹ��
  * @attention 
  */
Holder_t *Get_Holder_t(void)
{
	return &Holder;
}
/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
