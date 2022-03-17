#ifndef _CHASSIS_CONFIG_H
#define _CHASSIS_CONFIG_H


#include <stdint.h>
#include "config.h"

/******************************************************************************
  * @brief  ���̿�����ز���
  * @attention ˫�ջ����� 
  *     �⻷���ٶȻ�����������ʽPID������
  *     �ڻ�������������������ʽPID������
  *     ���Է�����
  *         1.���� #define CHASSIS_DEBUG����ͨ��debug�����޸Ĳ���
  *         2.���� #define CHASSIS_WAVE�������������ز���
  *         ����ʵ�ַ�����chassis.c�е�void Chassis_Init(void)������
  *
  *         ÿ��ֻ�ܶ���һ�����β��������������ע�͵�debug��ض���
  ****************************************************************************/

/* ���е���ǰ���붨��������������в��β���ֻ�ܶ���һ�� */
/* #define CHASSIS_DEBUG */
/* #define CHASSIS_WAVE */

//#define CHASSIS_DEBUG
//#define CHASSIS_WAVE

/* �ٶȻ� */
float Chassis_Speed_Kp   = 11.3f; /*< �ٶȻ�P���� */
float Chassis_Speed_Ki   = 0.53f; /*< �ٶȻ�I���� */
float Chassis_Speed_Kd   = 0.0f; /*< �ٶȻ�D���� */ //0.03
float Chassis_Speed_Inc_Limit = 16000.0f; /*< �ٶȻ������޷� */

/* ������ */
float Chassis_Current_Kp = 1.70f; /*< ������P���� */
float Chassis_Current_Ki = 0.34f; /*< �ٶȻ�I���� */
float Chassis_Current_Kd = 0.0f; /*< �ٶȻ�D���� */  //0.03
float Chassis_Current_Inc_Limit = 16000.0f; /*< �ٶȻ������޷� */

float Chassis_Out_Limit = 16000.0f; /*< ����޷���3508������16384*/

PID_param chassis_arm_position = {1,0,0};
PID_param chassis_arm_speed = {3.5,0,0};

PID_param chassis_arm1_position = {0.8,0,0};
PID_param chassis_arm1_speed = {1,0,0};
/* ��̬С���ݣ��ƶ�Խ�죬��תԽ�� */
float Move_Spin_Bias = 0.0001f; /*< ���㹫ʽ����ת�ٶ�=��ת�ٶ�/������*bias+1�� */
float Move_Spin_Limit = 0.1f;  /*< ��ת�ٶȱ仯��Χ�����ҷ�ֵ�������㹫ʽ�������ת*limit */
  
#endif
