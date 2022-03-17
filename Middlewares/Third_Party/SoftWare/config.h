#ifndef _CONFIG_H
#define _CONFIG_H

/* 机器人编号 只能定义一个*/
#define INFANTRY_3
//#define INFANTRY_4
//#define DEBUG
typedef struct _PID_param
{
	float Kp;
	float Ki;
	float Kd;
}PID_param;

#endif
