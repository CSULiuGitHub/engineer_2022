#ifndef _SHOOT_CONFIG_H
#define _SHOOT_CONFIG_H
#include <stdint.h>
#include "config.h"

/******************************************************************************
  * @brief  �������������ز���
  * @attention ÿ��ֻ�ܶ���һ�����β��������������ע�͵�debug��ض���
  *     1.�������2006��������ʽPID����
  *     �ٶȻ������þ���ʽPID������
  *     ���Է�����
  *         1.���� #define SHOOT_FREQ_DEBUG����Ƶ���ƣ���ͨ��debug�����޸Ĳ���
  *         2.���� #define SHOOT_FREQ_WAVE����Ƶ���ƣ�����������������
  *         3.���� #define SHOOT_SPEED_DEBUG, ���ٿ���
  *         4.���� #define SHOOT_SPEED_WAVE, ���ٲ���
  *         ����ʵ�ַ�����shoot.c�е�static void Shoot_PidRun(void)������
  *
  *     2.Ħ���ֵ��snail��������
  ****************************************************************************/

/* ���е���ǰ���붨��������������в��β���ֻ�ܶ���һ�� */
/* #define SHOOT_FREQ_DEBUG */
/* #define SHOOT_FREQ_WAVE  */

/* ��Ƶ���� */
//#define SHOOT_FREQ_DEBUG
//#define SHOOT_FREQ_WAVE 

/* ���ٿ��� */
//#define SHOOT_SPEED_DEBUG
//#define SHOOT_SPEED_WAVE 

/*****************************���л���ͨ�ò���**********************************/
int32_t Shoot_Freq_Out_Limit = 10000; /*< 2006���-10000 ~ 10000 */

uint16_t Freq_Max = 3000; /*< �����Ƶ */
uint16_t Freq_Min = 1000; /*< ��С��Ƶ */
float Buf_Safety  = 35;   /*< ��ȫ�������� */

/* Ħ���ֵ�������ٿ��� */
/* �����������޷ּ� */
float Tar_Speed_Lim15 = 13.5; /*< ��������Ϊ15 ��λ(m/s)*/
float Tar_Speed_Lim18 = 16.5;
float Tar_Speed_Lim22 = 20.5;
float Tar_Speed_Lim30 = 28.5;


/*****************************��ͬ������ͬ����**********************************/
#ifdef INFANTRY_3
/* �ٶȻ� */
float Shoot_Freq_Speed_Kp = 8.5;
float Shoot_Freq_Speed_Ki = 0.8;
float Shoot_Freq_Speed_Kd = 8;
float Shoot_Freq_Speed_Kp_Limit = 9999;
float Shoot_Freq_Speed_Ki_Limit = 9999;

/* Ħ���ֵ������ռ�ձȣ����Է�ֹ���� ������500���������ֳ��ٵ����ռ�ձȣ�*/
uint16_t Pusel_Max_Lim15 = 241;
uint16_t Pusel_Max_Lim18 = 251;
uint16_t Pusel_Max_Lim22 = 305;
uint16_t Pusel_Max_Lim30 = 355;
uint16_t Pulse_Min = 235;
uint16_t Pulse_Out = 240; /*< ��ʼ���� */
#endif /*< ifdef INFANTRY_3 */

#ifdef INFANTRY_4
/* 2006����ٶȻ� */
float Shoot_Freq_Speed_Kp = 8.5;
float Shoot_Freq_Speed_Ki = 0.8;
float Shoot_Freq_Speed_Kd = 8;
float Shoot_Freq_Speed_Kp_Limit = 9999;
float Shoot_Freq_Speed_Ki_Limit = 9999;

/* Ħ���ֵ������ռ�ձȣ����Է�ֹ���� */
uint16_t Pusel_Max_Lim15 = 242;
uint16_t Pusel_Max_Lim18 = 254;
uint16_t Pusel_Max_Lim22 = 335;
uint16_t Pusel_Max_Lim30 = 355;
uint16_t Pulse_Min = 235;
uint16_t Pulse_Out = 240; /*< ��ʼ���� */
#endif /*< ifdef INFANTRY_4 */

#endif
