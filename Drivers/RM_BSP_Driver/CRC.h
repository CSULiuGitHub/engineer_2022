/**
  ******************************************************************************
  * @file  CRC.h
  * @author duan
  * @brief 协议通讯头文件
  * @date     
  ******************************************************************************
  * @attention
  *
  * Copyright (c) CSU_RM_FYT.
  * All rights reserved.
  *
  ******************************************************************************
  */
#ifndef _CRC_H
#define _CRC_H

/* includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "stddef.h"
/* typedef ------------------------------------------------------------------*/
#define CRC_START_8 0x00

#define	CRC_START_16	 0xFFFF
#define	CRC_START_MODBUS 0xFFFF
#define	CRC_POLY_16		 0xA001
/* function ------------------------------------------------------------------*/
void init_crc16_tab(void);

uint16_t crc_16(const uint8_t *input_str,uint16_t num_bytes );
uint8_t crc_8( const uint8_t *input_str,uint16_t num_bytes);
#endif
