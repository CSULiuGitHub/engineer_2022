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
#include "shoot.h"
#include "motor.h"
#include "tim.h"
#include <string.h>
#include "usart.h"
#include "can.h"
#include "judge.h"
#include "robot_config.h"
#include "ui.h"
/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
portTickType posishoot_time;//�����ʱ����
Shoot_t M2006 = {0};

static int8_t lau = 0; 
static int8_t lauerror = 0; 
static int8_t pre = 0; 
static int8_t preerror = 0; 
static int8_t min = 0 ;
static int8_t prenum = 1 ;
typedef enum
{
	start  = 0,     
	wait   = 1,
	review = 2
}launch;

static launch state=start;
//static uint8_t count_judgesendfric=200;
/* function ------------------------------------------------------------------*/

/**
  * @brief  ����״̬��ͨ�˲���4�ף�
  * @param  void
  * @retval void
  * @attention 
  */
void Shoot_LPfIn(void)
{
    uint8_t i;
    for (i = 0; i < 4; i++)
    {
        M2006.LpfCurrent = 0.8 * M2006.Current + 0.2 * M2006.LpfCurrent;
    }
    for (i = 0; i < 4; i++)
    {
        M2006.LpfSpeed = 0.8 * M2006.Speed + 0.2 * M2006.LpfSpeed;
    }    
	for (i = 0; i < 4; i++)
    {
        M2006.LpfPosition = 0.8*M2006.Position.Angle + 0.2*M2006.LpfPosition;
    }    
}


/**
  * @brief  ����PID��ʼ��
  * @param  void
  * @retval void
  * @attention 
  */
void  Shoot_PidInit(void)
{
    pid_init_increment(&M2006.PidSpeed,3.1,0.01,0.00,9999);//4.2/0.01
    pid_init_increment(&M2006.PidPosition,0.01,0.00005,0.000,9999);//0.01,0.00005,0.000,9999;
}


/**
  * @brief  ����״̬��
  * @param  ����ָ��ṹ��
  * @retval ��ʾ����״̬��16λ����
  * @attention 
  */
int16_t Shoot(RemoteData_t RDMsg)
{
    uint8_t wheel;
    if (RDMsg.Wheel>300)
    {
        wheel = 1;
    }
    else
    {
        wheel = 0;
    }
	lauerror = lau - (RDMsg.MouseClick_left || wheel);

	switch(state)
	{
		case start:
			if(lauerror<0)
			{
				min=1;
				lauerror=0;
				state=review;
			}
		break;

		case wait:
			if(lauerror>=0)
			{
				min=0;//stop launch
				state=wait;
			}
			else
			{
				min=1;//launch one time
				state=review;
				posishoot_time = xTaskGetTickCount();//����ָ���´�ʱ��ϵͳʱ��,���ڷ�����ʱ����
			}
		break;
			
		case review:
			if(lauerror>0)
			{
				min=0;//stop launch
				state=wait;
			}
			else if((lauerror == 0)&(RDMsg.MouseClick_left == 1 || wheel == 1))
			{
				min=2;//launch one more time
				state=review;
			}
		break;
	}

	lau=RDMsg.MouseClick_left || wheel;
	if((M2006.TarPosition-Position*1000)>200)
	{
		min=1;
	}
	return min;
}




/**
  * @brief  ����PID���
  * @param  ����ָ��ṹ��
  * @retval void
  * @attention 
  */
void Shoot_PidRun(RemoteData_t RDMsg)
{
	M2006.TarSpeed    = pid_increment_update( M2006.TarPosition,M2006.LpfPosition,&M2006.PidPosition);
	M2006.Output      = pid_increment_update( M2006.TarSpeed,M2006.LpfSpeed,&M2006.PidSpeed); 
}
/**
  * @brief  ����Lpf���
  * @param  void
  * @retval void
  * @attention 
  */
void Shoot_LPfOut(void)
{
    M2006.OutputLpf = 0.8 * M2006.Output + 0.2 * M2006.OutputLpf;
}
/**
  * @brief  ����CAN����
  * @param  void
  * @retval void
  * @attention 
  */
void Shoot_CanTransmit(void)
{
    if(Observer.Tx.DR16_Rate>15)
    {
        M2006.CanData[6]=(uint8_t)(M2006.OutputLpf>>8);
        M2006.CanData[7]=(uint8_t)(M2006.OutputLpf);
//        TIM1_SetPWMPluse(235);  
//        judge_send_UI(3, 0x0103);//client������
    }   
    else
    {
        memset(M2006.CanData,0,sizeof(M2006.CanData));
		TIM1_SetPWMPluse(200);
    }
    CAN1_Transmit(0x200,M2006.CanData);
}


/**
  * @brief  17mm���������������
  * @param  uint8_t
  * @retval void
  * @attention 
  */
//static uint8_t Heat17_Control(void)
//{
//    uint16_t heat17 = JUDGE_u16GetRemoteHeat17();
//	uint16_t heat17_limit = JUDGE_u16GetHeatLimit();
//}

/**
  * @brief  ѡ�񲦵���ʽ
  * @param  ��ʾ����״̬��16λ����
  * @retval void
  * @attention 
  */
uint16_t heat17_limit;
uint16_t u16shoot_freq = 2000;
void Choose(int16_t a)
{
	static int16_t count = 0;
    
    /* �������� */
    uint16_t heat17 = JUDGE_u16GetRemoteHeat17();
	heat17_limit = JUDGE_u16GetHeatLimit();
//    UART2_SendWave(1, 2, &heat17_limit);
    
    if (heat17_limit < 50)
    {
        heat17_limit = 50;
    }
    
    if (heat17 >= heat17_limit-15)
    {
        a = 0;
    }
    
    
	switch(a)
	{
		case 0: /*< ֹͣ */
			M2006.TarSpeed = 0;
			pid_init_absolute(&M2006.PidSpeedstop,4.2,0.0,0.00,9999, 9999);
			M2006.Output   = pid_absolute_update( M2006.TarSpeed,M2006.LpfSpeed,&M2006.PidSpeedstop);
			count=0;
		break;

		case 1: /*< ���� */
			Position=M2006.Position.Circle+(float)(M2006.Position.Angle/8192);
			if(count==0)
			{
				if(M2006.Position.Circle>15)
				{
					M2006.Position.Circle=1;
					Position=M2006.Position.Circle+(float)(M2006.Position.Angle/8192);
				}
				M2006.TarPosition = 1000*(3.65f+Position);
				count=1;
			}
			pid_init_increment(&M2006.PidPosition,0.70,0.0001,0.0,9999);//0.35,0.0001,0.0,9999
			M2006.TarSpeed = pid_increment_update( M2006.TarPosition,Position*1000,&M2006.PidPosition);
			M2006.Output   = pid_increment_update( M2006.TarSpeed,M2006.LpfSpeed,&M2006.PidSpeed);
		break;

		case 2: /*< ���� */
			M2006.TarSpeed = u16shoot_freq;
			M2006.Output   = pid_increment_update( M2006.TarSpeed,M2006.LpfSpeed,&M2006.PidSpeed); 
			count=0;
		break;
        case 3: /*< ������ת */
            Position=M2006.Position.Circle+(float)(M2006.Position.Angle/8192);

            if(M2006.Position.Circle>15)
            {
                M2006.Position.Circle=1;
                Position=M2006.Position.Circle+(float)(M2006.Position.Angle/8192);
            }
            M2006.TarPosition = 1000*(-3.65f+Position);
            count=1;
            
			pid_init_increment(&M2006.PidPosition,0.70,0.0001,0.0,9999);//0.35,0.0001,0.0,9999
			M2006.TarSpeed = pid_increment_update( M2006.TarPosition,Position*1000,&M2006.PidPosition);
			M2006.Output   = pid_increment_update( M2006.TarSpeed,M2006.LpfSpeed,&M2006.PidSpeed);
            count=0;
        break;
	}
}
/**
  * @brief  ���̽���
  * @param  ����ָ��ṹ��
  * @retval void
  * @attention 
  */
void Shoot_Process(RemoteData_t RDMsg)
{    
    Shoot_LPfIn();
    
    Shoot_Speed_Control(RDMsg); /*< Ħ�����ٶȿ��� */
	
	if(Observer.Tx.DR16_Rate>15)/*< ң����������������16ʱ�ſ������� */
	{
  		Choose(Shoot(RDMsg));
	}
	
	Prepare(RDMsg);
    Shoot_LPfOut();
    Shoot_Freq_Control();
    Shoot_CanTransmit();
}

/*********************************���丨������*********************************/
/**
  * @brief  Ħ�����ٶȿ���
  * @param  ����ָ��ṹ��
  * @retval void
  * @attention  ���ٿ���
  */
void Shoot_Speed_Control(RemoteData_t RDMsg)
{
    static uint16_t shoot_speed = 240;
    static uint8_t tick = 0;
    uint16_t max_speed = 240;
    uint16_t speed_limit = JUDGE_u16GetSpeedHeat17Limit();
    
//    UART2_SendWave(1, 2, &speed_limit);
    float speed_heat17 = JUDGE_fGetSpeedHeat17();
//    UART2_SendWave(2, 4, &speed_heat17, &speed_limit);

    tick++;
    if (tick == 10)
    {
        if (speed_limit <= 15)
        {
            max_speed = 240;
        }
        else if (speed_limit <=18)
        {
            max_speed = 250;
        }
        else if (speed_limit <= 22)
        {
            max_speed = 300;
        }
        else if (speed_limit <= 30)
        {
            max_speed = 300;
        }
        else
        {
            max_speed = 240;
        }
        
        
        if (JUDGE_fGetSpeedHeat17() > speed_limit)
        {
            shoot_speed -= 5;
        }
        
        /* ���� */
        if (RDMsg.KeyBoard.f == 1)
        {
            shoot_speed -= 5;
            if (shoot_speed < 235)
            {
                shoot_speed = 200;
            }
        }
        
        /* ���� */
        if (RDMsg.KeyBoard.g == 1)
        {
            shoot_speed += 5;
            
            if (shoot_speed < 235)
            {
                shoot_speed = 235;
            }
            
            if (shoot_speed > max_speed)
            {
                shoot_speed = max_speed;
            }
        }
        if (shoot_speed > max_speed)
        {
            shoot_speed = max_speed;
        }

        TIM1_SetPWMPluse(shoot_speed);
        tick = 0;
    }

    
//        TIM1_SetPWMPluse(speed);
    switch(shoot_speed)
    {
        case 200:
            statefric='0';break;
        case 235:
        case 240:
            statefric='1';break;
        case 245:
        case 250:
            statefric='2';break;
        case 255:
        case 256:
            statefric='3';break;
    }
}

/**
  * @brief  ����ʱ���ȡ
  * @param  void
  * @retval λ�û�ʵʱָ��ʱ��
  * @attention  ���ڷ�����ʱ����
  */
portTickType REVOL_uiGetRevolTime(void)
{
	return posishoot_time;
}


/**
  * @brief ׼������״̬����Ԥ�ȣ�����z����
  * @param  ����ָ��ṹ��
  * @retval void
  * @attention  
  */
void Prepare(RemoteData_t RDMsg)
{
	preerror = pre - RDMsg.KeyBoard.z;
	if(preerror==-1)
	{
		prenum=-prenum;
	}
	if(prenum<0)
	{
		Open();
	}
	else
	{
		Close();
	}
	pre=RDMsg.KeyBoard.z;
	
}
/**
  * @brief  �򿪵��ָ�
  * @param  void
  * @retval void
  * @attention  
  */
void Open(void)
{
    __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,250);
	  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2,250);
	  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,250);
  	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,250);
}
/**
  * @brief  �رյ��ָ�
  * @param  void
  * @retval void
  * @attention  
  */
void Close(void)
{
    __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,115);
	  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2,115);
	  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,115);
  	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,115);//
}


void Shoot_Freq_Control(void)
{
    float cur = (float)JUDGE_u16GetRemoteHeat17();
    float lim = (float)JUDGE_u16GetHeatLimit();
    float rate = (float)JUDGE_u16GetHeatRate();
    float buf = lim - cur;
    u16shoot_freq = (buf/lim)*rate * 200;
    if (u16shoot_freq > 4000)
    {
        u16shoot_freq = 4000;
    }

    if (buf < 45)
    {
        u16shoot_freq = 1000;
    }
}

/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/


