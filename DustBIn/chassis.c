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
#include "chassis.h"
#include <string.h>
#include "holder.h"
#include "usart.h"
#include "tim.h"
#include "can.h"
#include "fast_tri_func.h"
#include "misc_func.h"
#include "judge.h"
#include "robot_config.h"
#include "ui.h"
#include "FreeRTOS.h"
#include "task.h"
/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/
#define GAIN_I 0.1
#define GAIN_J 0.1
#define GAIN_K 0.2
/* variables -----------------------------------------------------------------*/
Chassis_t Chassis = {0};
int16_t sign;
int16_t front_temp,right_temp;
static uint16_t numremain[8] = {0};//记键盘的前一个状态
static int16_t revlove = 0;
static int16_t count   = 0;
PID_AntiIntegralType PidClockWise;
float RoundAngle = 0;
float fmax_speed_spin = 0; /*< 最大旋转速度 */
uint8_t turn_round_state = 0;


static int16_t s16_max_front_speed = 350; /* 默认最大前后移动速度 */
static int16_t s16_max_right_speed = 300; /* 默认最大左右移动速度 */

/*速度功率控制（需实测）    34      39      46      54      61       70       80       93 */
static float spin_speed[8]  = {700.0f, 750.0f, 850.0f, 900.0f, 1000.0f, 1100.0f, 1200.0f, 1300.0f};
static float wheel_speed[8] = {5000.0f, 5500.0f, 5700.0f, 5900.0f, 6000.0f, 6300.0f, 6600.0f, 7000.0f};

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
        pid_init_increment(&Chassis.M3508[i].PidSpeed,9.8,0.83,0.8,9999);
    }
    for(i=0;i<4;i++)
    {
        pid_init_increment(&Chassis.M3508[i].PidCurrent,1.2,0.48,0.05,9999);
    }
}


/**
  * @brief  获取底盘控制数据
  * @param  遥控器消息结构体
  * @retval void
  * @attention 
  */
void Chassis_GetMoveData(RemoteData_t RDMsg)
{
    int16_t max_front_speed = s16_max_front_speed;
    int16_t max_right_speed = s16_max_right_speed;
    float DirAngle = Get_fDirAngle() + RoundAngle;
    if (DirAngle > 180)
    {
        DirAngle -= 360;
    }

    if (DirAngle < -180)
    {
        DirAngle += 360;
    }
    
    static int16_t speedcount = 0;
    
    if(RDMsg.S1 == 1)/*< 遥控器控制*/
    {
        right_temp = RDMsg.Ch2;
        front_temp = RDMsg.Ch3;

        Chassis.MoveData.Front =  (int)(((float)front_temp) * (fast_cos((int16_t)DirAngle)))
                                 +(int)(((float)right_temp) * (fast_sin((int16_t)DirAngle)));
        
        Chassis.MoveData.Right = -(int)(((float)front_temp) * (fast_sin((int16_t)DirAngle)))
                                 +(int)(((float)right_temp) * (fast_cos((int16_t)DirAngle)));
    }
	else /*< 键盘控制 */
	{
		switch(speedcount)
		{
			case 0:
                Chassis.MoveData.Front      = max_front_speed * RDMsg.KeyBoard.w - max_front_speed * RDMsg.KeyBoard.s;
                Chassis.MoveData.Right      = max_right_speed * RDMsg.KeyBoard.d - max_right_speed * RDMsg.KeyBoard.a;
                numremain[0] = RDMsg.KeyBoard.w;
                numremain[1] = RDMsg.KeyBoard.s;
                numremain[2] = RDMsg.KeyBoard.d;
                numremain[3] = RDMsg.KeyBoard.a;
                numremain[4] = RDMsg.KeyBoard.r;
                numremain[5] = RDMsg.KeyBoard.e;
                speedcount++;
            break;
            
			case 1:
                if ((numremain[0] == RDMsg.KeyBoard.w) && (numremain[1] == RDMsg.KeyBoard.s) 
                    && (numremain[2] == RDMsg.KeyBoard.d) && (numremain[3] == RDMsg.KeyBoard.a) 
                    && (numremain[4] == RDMsg.KeyBoard.r) && (numremain[5] == RDMsg.KeyBoard.e))
                {
                    speedcount++;
                }
                else
                {
                    Chassis.MoveData.Front      = max_front_speed * RDMsg.KeyBoard.w - max_front_speed * RDMsg.KeyBoard.s;
                    Chassis.MoveData.Right      = max_right_speed * RDMsg.KeyBoard.d - max_right_speed * RDMsg.KeyBoard.a;
                    speedcount = 0;
					break;
                }
            
			case 2:{
				if((RDMsg.KeyBoard.w==1)||(RDMsg.KeyBoard.s==1)||(RDMsg.KeyBoard.a==1)
                    ||(RDMsg.KeyBoard.d==1)||(RDMsg.KeyBoard.r==1)||(RDMsg.KeyBoard.e==1))
                {
                    if((RDMsg.KeyBoard.w==0)&&(RDMsg.KeyBoard.s==1))
                    {
                        front_temp = Chassis.MoveData.Front;
                        Chassis.MoveData.Front = front_temp + 1*(RDMsg.KeyBoard.shift - RDMsg.KeyBoard.ctrl);
                        if(Chassis.MoveData.Front > 0)
                        {
                            Chassis.MoveData.Front = 0;	
                        }
                    }
                    else if(RDMsg.KeyBoard.w==1)
                    {
                        front_temp = Chassis.MoveData.Front;
                        Chassis.MoveData.Front = front_temp - 1*(RDMsg.KeyBoard.shift - RDMsg.KeyBoard.ctrl);
                        if(Chassis.MoveData.Front < 0)
                        {
                            Chassis.MoveData.Front = 0;
                        }     
                    }
                    if((RDMsg.KeyBoard.d==0)&&(RDMsg.KeyBoard.a==1))
                    {
                        right_temp = Chassis.MoveData.Right;
                        Chassis.MoveData.Right = right_temp + 1*(RDMsg.KeyBoard.shift - RDMsg.KeyBoard.ctrl);
                        if(Chassis.MoveData.Right > 0)
                        {
                            Chassis.MoveData.Right = 0;
                        } 
                    }
                    else if(RDMsg.KeyBoard.d==1)
                    {
                        right_temp = Chassis.MoveData.Right;
                        Chassis.MoveData.Right = right_temp - 1*(RDMsg.KeyBoard.shift - RDMsg.KeyBoard.ctrl);
                        if(Chassis.MoveData.Right < 0)
                        {
                            Chassis.MoveData.Right = 0;                            
                        }
                    }
                    if((RDMsg.KeyBoard.r==0)&&(RDMsg.KeyBoard.e==1))
                    {
                        Chassis.MoveData.ClockWise -= 1*(RDMsg.KeyBoard.shift - RDMsg.KeyBoard.ctrl);
                        if(Chassis.MoveData.ClockWise > 0)
                        {
                             Chassis.MoveData.ClockWise = 0;                           
                        }
                    }
                    else if(RDMsg.KeyBoard.r==1)
                    {
                        Chassis.MoveData.ClockWise -= 1*(RDMsg.KeyBoard.shift - RDMsg.KeyBoard.ctrl);
                        if(Chassis.MoveData.ClockWise < 0)
                        {
                            Chassis.MoveData.ClockWise = 0;   
                        }
                    }
                    speedcount = 1;
				}
				else
                {
                    speedcount = 0;
									  break;	
                }}
		}
		
		if(Chassis.MoveData.Front > 660)
			Chassis.MoveData.Front = 660;
		else if(Chassis.MoveData.Front < -660)
			Chassis.MoveData.Front =-660;
		if(Chassis.MoveData.Right > 660)
			Chassis.MoveData.Right = 660;
		else if(Chassis.MoveData.Right < -660)
			Chassis.MoveData.Right =-660;
	}
}


/**
  * @brief  电机速度控制
  * @param  void
  * @retval void
  * @attention 
  */
float fmax_wheel_speed = 0;
static void Chassis_Speed_Control(void)
{
    uint8_t i;
    float limit_speed = fmax_wheel_speed;
    float tmp_max_speed = 1;
    float tmp_min_speed = 1;
    float tmp_speed[4];
    float cmp_index = 1;
    
    float DirAngle = Get_fDirAngle() + RoundAngle;
    
    if (DirAngle > 180)
    {
        DirAngle -= 360;
    }

    if (DirAngle < -180)
    {
        DirAngle += 360;
    }
//    
    if(!RoundAngle){
    tmp_speed[0] = ( Chassis.MoveData.Right / GAIN_I 
                  +Chassis.MoveData.Front / GAIN_J 
                  +Chassis.MoveData.ClockWise / GAIN_K);

    tmp_speed[1] = ( Chassis.MoveData.Right / GAIN_I 
                  -Chassis.MoveData.Front / GAIN_J 
                  +Chassis.MoveData.ClockWise / GAIN_K);
    
    tmp_speed[2] = (-Chassis.MoveData.Right / GAIN_I 
                  +Chassis.MoveData.Front / GAIN_J 
                  +Chassis.MoveData.ClockWise / GAIN_K);
    
    tmp_speed[3] = (-Chassis.MoveData.Right / GAIN_I 
                  -Chassis.MoveData.Front / GAIN_J  
                  +Chassis.MoveData.ClockWise / GAIN_K);
		}else{
			tmp_speed[0] = (-Chassis.MoveData.Right / GAIN_I 
                  -Chassis.MoveData.Front / GAIN_J 
                  +Chassis.MoveData.ClockWise / GAIN_K);

            tmp_speed[1] = (-Chassis.MoveData.Right / GAIN_I 
                          +Chassis.MoveData.Front / GAIN_J 
                          +Chassis.MoveData.ClockWise / GAIN_K);
            
            tmp_speed[2] = ( Chassis.MoveData.Right / GAIN_I 
                          -Chassis.MoveData.Front / GAIN_J 
                          +Chassis.MoveData.ClockWise / GAIN_K);
            
            tmp_speed[3] = ( Chassis.MoveData.Right / GAIN_I 
                          +Chassis.MoveData.Front / GAIN_J  
                          +Chassis.MoveData.ClockWise / GAIN_K);
}

    for(i = 0; i<4; i++)
    {
        /* 找最大值 */
        if (tmp_speed[i] > tmp_max_speed)
        {
            tmp_max_speed = tmp_speed[i];
        }
        
        /* 找最小值 */
        if (tmp_speed[i] < tmp_min_speed)
        {
            tmp_min_speed = tmp_speed[i];
        }
    }

    tmp_max_speed = Misc_Fabsf(tmp_max_speed);
    tmp_min_speed = Misc_Fabsf(tmp_min_speed);

    if (tmp_max_speed > limit_speed || tmp_min_speed > limit_speed)
    {
        if (tmp_max_speed < tmp_min_speed)
        {
            cmp_index = limit_speed/tmp_min_speed;
        }
        else if (tmp_max_speed > tmp_min_speed)
        {
            cmp_index = limit_speed/tmp_max_speed;
        }
        else
        {
            cmp_index = limit_speed/tmp_min_speed;
        }
    }
    else
    {
        cmp_index = 1;
    }

    for(i = 0; i<4; i++)
    {
        Chassis.M3508[i].TarSpeed = (int16_t)(tmp_speed[i]*cmp_index);
    }
}

/**
  * @brief  底盘模式选择（r键选择？）
  * @param  控制指令结构体
  * @retval void
  * @attention 
  */
void Chassis_ChooseMode(RemoteData_t RDMsg)
{
    int32_t temp;
    
    
    float DirAngle = Get_fDirAngle() + RoundAngle;
    
    if (DirAngle > 180)
    {
        DirAngle -= 360;
    }

    if (DirAngle < -180)
    {
        DirAngle += 360;
    }
    
		
    if (RDMsg.S1 == 1)
    {
			if(DirAngle > 0)
            sign = -1;
        else
            sign = 1;
        temp = sign * DirAngle * DirAngle;
        temp = temp>800? 800:temp;
        temp = temp<-800? -800:temp;
        Chassis.MoveData.ClockWise = temp;
    }
    else if (RDMsg.S1 == 2)
    {
        Chassis.MoveData.Front =  (int)(((float)front_temp) * (fast_cos((int16_t)DirAngle))) + (int)(((float)right_temp) * (fast_sin((int16_t)DirAngle)));
        Chassis.MoveData.Right = -(int)(((float)front_temp) * (fast_sin((int16_t)DirAngle))) + (int)(((float)right_temp) * (fast_cos((int16_t)DirAngle)));

        Dynamic_Spin();
    }
    else
    {
        if(RDMsg.KeyBoard.r>revlove)
         {
             count++;
             if(count >= 2)
                 count=0;
         }
         revlove = RDMsg.KeyBoard.r;
         
         if(count== 0)
         {
            if(DirAngle > 0)
                sign = -1;
            else
                sign = 1;
            
            
            temp = sign * DirAngle * DirAngle;
            temp = temp>fmax_speed_spin? fmax_speed_spin:temp;
            temp = temp<-fmax_speed_spin? -fmax_speed_spin:temp;
            Chassis.MoveData.ClockWise = temp;
         }
        if(count== 1)
        {
            statemode='l';
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
            
            if(RDMsg.KeyBoard.shift)
            {
                front_temp = front_temp*450/300;
                right_temp = right_temp*450/300;
            }else if((!RDMsg.KeyBoard.shift)&&(RDMsg.KeyBoard.ctrl))
            {
                front_temp = front_temp*250/300;
                right_temp = right_temp*250/300;
            }else
            {
                front_temp = front_temp;
                right_temp = right_temp;
            }
            Chassis.MoveData.Front =  (int)(((float)front_temp) * (fast_cos((int16_t)DirAngle))) + (int)(((float)right_temp) * (fast_sin((int16_t)DirAngle)));
            Chassis.MoveData.Right = -(int)(((float)front_temp) * (fast_sin((int16_t)DirAngle))) + (int)(((float)right_temp) * (fast_cos((int16_t)DirAngle)));
            Dynamic_Spin();
        }else
				statemode='f';
    }
	
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
            Chassis.CanData[2*i]=(uint8_t)(Chassis.M3508[i].LPf.Output>>8);
            Chassis.CanData[2*i+1]=(uint8_t)(Chassis.M3508[i].LPf.Output);
        }
    }   
    else
    {
        Chassis_Speed_Reset(); /*< 关闭遥控器后，底盘目标速度一直保持当前状态 */
        memset(Chassis.CanData,0,sizeof(Chassis.CanData));
    }
    CAN2_Transmit(0x200,Chassis.CanData);
}


/**
  * @brief  超级功率CAN发送
  * @param  void
  * @retval void
  * @attention 
  */
void SuperCap_CanTransmit(void)
{
    Chassis.SuperCap.CanData[0] = Chassis.SuperCap.TargetPower >> 8;
    Chassis.SuperCap.CanData[1] = Chassis.SuperCap.TargetPower;
    CAN2_Transmit(0x210,Chassis.SuperCap.CanData);
}


/**
  * @brief  功率控制，向超级电容发送最大功率
  * @param  void
  * @retval void
  * @attention 超级电容根据最大功率控制电流输出，防止底盘超功率
  */
uint16_t power_lim;
static void Chassis_Power_limit_F(void)
{
    static uint8_t tick = 0;

    power_lim = JUDGE_u16GetChassisPowerLimit();
    
    
    if (Chassis.SuperCap.CapVol < 1600)
    {
        s16_max_front_speed = 150;
        s16_max_right_speed = 150;
        fmax_speed_spin = 100;
        fmax_wheel_speed = 100;
    }
    else
    {
        s16_max_front_speed = 350;
        s16_max_right_speed = 300;
    }
    
    tick++;/*< 延时*/
    if (tick == 20)
    {
        if (power_lim < 50)
        {
            power_lim = 50;
        }
//        power_lim = 70;
        Chassis.SuperCap.TargetPower = power_lim * 100;/*<获得裁判系统的功率限定*/
        
        
        SuperCap_CanTransmit();
        tick = 0;
    }
    
	if (power_lim <= 40)
    {
        fmax_wheel_speed = wheel_speed[0];
        fmax_speed_spin  = spin_speed[0];
    }
    else if (power_lim <= 45)
    {
        fmax_wheel_speed = wheel_speed[1];
        fmax_speed_spin  = spin_speed[1];
    }
    else if (power_lim <= 50)
    {
        fmax_wheel_speed = wheel_speed[2];
        fmax_speed_spin  = spin_speed[2];
    }
    else if (power_lim <= 60)
    {
        fmax_wheel_speed = wheel_speed[3];
        fmax_speed_spin  = spin_speed[3];
    }
    else if (power_lim <= 70)
    {
        fmax_wheel_speed = wheel_speed[4];
        fmax_speed_spin  = spin_speed[4];
    }
    else if (power_lim <= 80)
    {
        fmax_wheel_speed = wheel_speed[5];
        fmax_speed_spin  = spin_speed[5];
    }
    else if (power_lim <= 90)
    {
        fmax_wheel_speed = wheel_speed[6];
        fmax_speed_spin  = spin_speed[6];
    }
    else if (power_lim <= 120)
    {
        fmax_wheel_speed = wheel_speed[7];
        fmax_speed_spin  = spin_speed[7];
    }
    else
    {
        fmax_wheel_speed = wheel_speed[7];
        fmax_speed_spin  = spin_speed[7];
    }
}


/**
  * @brief  底盘进程
  * @param  控制指令结构体
  * @retval void
* @attention 保护写在CAN发送前，（V）一键复位，PID误差和输出清零
  */
void Chassis_Process(RemoteData_t RDMsg)
{
    Chassis_Power_limit_F();
    Chassis_GetMoveData(RDMsg);
	Chassis_ChooseMode(RDMsg);
    Chassis_Protect(RDMsg);
    Turn_Round(RDMsg);    /*< 一键掉头*/
    Fast_Fix_Spin(RDMsg); /*< （E）键快速定点小陀螺 */
    Chassis_Speed_Control();
    Chassis_PidRun();
    Chassis_CanTransmit();/*< 放中断里面 */
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
        Chassis_Speed_Reset();
    }
}


/**
  * @brief  快速定点小陀螺
  * @param  控制指令结构体
  * @retval void
  * @attention 
  */
void Fast_Fix_Spin(RemoteData_t RDMsg)
{
    if (RDMsg.KeyBoard.e == 1)
    {
        Chassis.MoveData.Front = 0;
        Chassis.MoveData.Right = 0;
        Chassis.MoveData.ClockWise = fmax_speed_spin + 300;
    }
}


/**
  * @brief  动态小陀螺
  * @param  控制指令结构体
  * @retval void
  * @attention front 或 right速度 与 clockwise速度 成反比关系
  */
void Dynamic_Spin(void)
{
    static int32_t tick = 0;
    float tmp_clkws = 0;
    int16_t tmp_f = Chassis.MoveData.Front > 0? Chassis.MoveData.Front : -Chassis.MoveData.Front;
    int16_t tmp_r = Chassis.MoveData.Right > 0? Chassis.MoveData.Right : -Chassis.MoveData.Right;
    tick++;
    if (tick == 1080)
    {
        tick = 0;
    }
    tmp_clkws = fast_sin(tick/3);
    
    if (tmp_clkws < 0.5f && tmp_clkws > 0)
    {
        tmp_clkws = 0.5f;
    }
    
    
    if (tmp_clkws > -0.5f && tmp_clkws < 0)
    {
        tmp_clkws = -0.5;
    }
    if (tmp_clkws < 0)
    {
        tmp_clkws = -tmp_clkws;
    }
    
    Chassis.MoveData.ClockWise = tmp_clkws*fmax_speed_spin / (tmp_f*0.003f + tmp_r*0.003f + 1);
    
    Chassis.MoveData.ClockWise = constrain_int16_t(Chassis.MoveData.ClockWise, -1310, 1310);
}


/**
  * @brief  (Q键)一键掉头，云台偏航+180度，底盘与云台差角+180度
  * @param  控制指令结构体
  * @retval void
  * @attention 
  */
void Turn_Round(RemoteData_t RDMsg)
{
    static uint8_t delay_tick = 0;
    static uint8_t pro_tick = 0;
    static uint8_t state = 0;
   
    
    if (pro_tick < 25)
    {
        pro_tick++;
        Chassis_Speed_Reset();
    }
    
    if (delay_tick < 100)
    {
        delay_tick++; /*< 延时，防止连续按两次*/
        
    }
    
    if (RDMsg.KeyBoard.q == 1 && delay_tick == 100 && state == 0)
    {
        Holder.Yaw.TarAngle +=38000;
        RoundAngle = 180;
        delay_tick = 0;
        pro_tick = 0;
        state = 1;
    }
    else if (RDMsg.KeyBoard.q == 1 && delay_tick == 100 && state == 1)
    {
        Holder.Yaw.TarAngle +=38000;
        RoundAngle = 0;
        delay_tick = 0;
        pro_tick = 0;
        state = 0;
    }
  
//    if (state == 0)
//    {
//        Chassis.MoveData.Front = -Chassis.MoveData.Front;
//        Chassis.MoveData.Right = -Chassis.MoveData.Right;
//    }
}

/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
