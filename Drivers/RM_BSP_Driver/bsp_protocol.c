/*
  @SEASKY---2020/09/05
  串口通信协议
*/
#include "bsp_protocol.h"
#include <stdarg.h>
#include "CRC.h"
#include "usart.h"
/*获取CRC8校验码*/
uint8_t Get_CRC8_Check(uint8_t *pchMessage,uint16_t dwLength)
{
    return crc_8(pchMessage,dwLength);
}
/*检验CRC8数据段*/
uint8_t CRC8_Check_Sum(uint8_t *pchMessage,uint16_t dwLength)
{
    uint8_t ucExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2)) return 0;
    ucExpected = Get_CRC8_Check(pchMessage, dwLength-1);
    return ( ucExpected == pchMessage[dwLength-1] );
}
/*获取CRC16校验码*/
uint16_t Get_CRC16_Check(uint8_t *pchMessage,uint32_t dwLength)
{
    return crc_16(pchMessage,dwLength);
}
/*检验CRC16数据段*/
uint16_t CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t  wExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
        {
            return 0;
        }
    wExpected = Get_CRC16_Check ( pchMessage, dwLength - 2);
    return (((wExpected & 0xff) == pchMessage[dwLength - 2] )&& (((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]));
}
/*检验数据帧头*/
uint8_t protocol_heade_Check(
    protocol *pro,
    uint8_t  *rx_buf,
    uint16_t *rx_pos)
{
    if(rx_buf[0] == PROTOCOL_CMD_ID)
        {
            pro->header.sof = rx_buf[0];
            if(CRC8_Check_Sum(&rx_buf[0],4))
						{
								pro->header.data_length = (rx_buf[2]<<8) | rx_buf[1];
								pro->header.crc_check = rx_buf[3];
								pro->cmd_id = (rx_buf[5]<<8) | rx_buf[4];
								return  1;
						}
        }
    else
        {
            *rx_pos = 0;
        }
    return 0;
}

float float_protocol(uint8_t *dat_t)
{
    uint8_t f_data[4];
    f_data[0] = *(dat_t+0);
    f_data[1] = *(dat_t+1);
    f_data[2] = *(dat_t+2);
    f_data[3] = *(dat_t+3);
    return *(float*)f_data;
}
/*
  此函数根据待发送的数据更新数据帧格式以及内容，实现数据的打包操作
  后续调用通信接口的发送函数发送tx_buf中的对应数据
*/
void get_protocol_send_data
(uint16_t send_id,              //信号id
uint16_t flags_register, //16位寄存器
float    *tx_data,              //待发送的float数据
uint8_t  float_length,   //float的数据长度
uint8_t  *tx_buf,               //待发送的数据帧
uint16_t *tx_buf_len)   //待发送的数据帧长度
{
    uint16_t crc16;
    uint16_t data_len;

    data_len = float_length*4+2;
    /*帧头部分*/
    tx_buf[0] = PROTOCOL_CMD_ID;
    tx_buf[1] = data_len & 0xff;                    //低位在前
    tx_buf[2] = (data_len >> 8) & 0xff;       //低位在前
    tx_buf[3] = Get_CRC8_Check(&tx_buf[0],3); //获取CRC8校验位

    /*数据的信号id*/
    tx_buf[4] = send_id & 0xff;
    tx_buf[5] = (send_id>>8) & 0xff;

    /*建立16位寄存器*/
    tx_buf[6] = flags_register & 0xff;
    tx_buf[7] = (flags_register>>8) & 0xff;

    /*float数据段*/
    for(int i=0; i<4*float_length; i++)
        {
            tx_buf[i+8]  = ((uint8_t*)(&tx_data[i/4]))[i%4];
        }

    /*整包校验*/
    crc16 = Get_CRC16_Check(&tx_buf[0],data_len+6);
    tx_buf[data_len+6] = crc16 & 0xff;
    tx_buf[data_len+7] = (crc16 >>8) & 0xff;

    *tx_buf_len = data_len+8;
}

/*
  此函数用于处理接收数据，
  返回数据内容的id
*/
uint16_t get_protocol_info
(uint8_t  *rx_buf,                    //接收到的原始数据
uint16_t *rx_pos,                     //原始数据指针
uint16_t *flags_register,     //接收数据的16位寄存器地址
float    *rx_data)                    //接收的float数据存储地址
{
  static protocol pro;
  static uint16_t date_length;
    if(protocol_heade_Check(&pro,rx_buf,rx_pos))
        {
            date_length = OFFSET_BYTE + pro.header.data_length;
            while(CRC16_Check_Sum(&rx_buf[0],date_length))
                {
                    *flags_register = (rx_buf[7]<<8) | rx_buf[6];
                    for(int i=0; i<(pro.header.data_length-2)/4; i++)
                        {
                            rx_data[i] = float_protocol(&rx_buf[8+4*i]);
                        }
                    for(int i=0; i<date_length; i++)
                        {
                            rx_buf[i] = 0;
                        }
												
                    *rx_pos = 0;
                    return pro.cmd_id;
                }
        }
				return 0;
}
/*************************************************
@Function:中断处理函数
@Description: 
@Called By: 放入中断接收函数
@Input: 
@Others: 
*************************************************/
uint8_t rx_buff[256] = {0};
uint16_t rx_buf_pos = 0;
float rx_data[12] = {0}; 
protocol rx_pro = {0};

void Protocol_Rx_IRQ(uint8_t res)
{
		static bool flag_headcheck = 1;
		static uint16_t register_flag = 0x0000;
		static float tx_data = 0;
		static uint16_t data_len = 0;
		static uint8_t tx_buff[40] = {0};
		
    rx_buff[rx_buf_pos]=res;
    rx_buf_pos ++;
    if(rx_buff[0]!=PROTOCOL_CMD_ID)
		{
				rx_buf_pos = 0;
		};
		
		if(rx_buf_pos >= 9)
		{
        if(flag_headcheck)
				{
						if(protocol_heade_Check(&rx_pro,rx_buff,&rx_buf_pos))
						{
								flag_headcheck = 0;//头帧校验完毕
						}
						else
						{
								rx_buf_pos = 0;
						}
				}
				else if(rx_buf_pos >= (rx_pro.header.data_length + OFFSET_BYTE))
				{
						flag_headcheck = 1;
						switch(get_protocol_info(rx_buff, &rx_buf_pos, &register_flag, rx_data))
						{
							case 0x0002:
								 Return_PID(tx_buff, &data_len);
								 HAL_UART_Transmit(&huart2,tx_buff, data_len, 100);
							break;
							case 0x0003:
								 Change_PID(rx_data);
							default:
								break;
						}
						get_protocol_send_data(CONFIRM_ID,CONFIRM_REG,&tx_data,0,tx_buff, &data_len);
				    HAL_UART_Transmit(&huart2,tx_buff,data_len, 100);
				}
		}
}

/*************************************************
@Function: Set_Addr
@Description: 设置PID参数地址
@Called By: 
@Input: 
@Others: 
*************************************************/
float *PID_ptr[6] = {0};
void Set_Addr(float *P1, float *I1, float *D1, float *P2, float *I2, float *D2)
{
		PID_ptr[0] = P1;
		PID_ptr[1] = I1;
		PID_ptr[2] = D1;
		PID_ptr[3] = P2;
		PID_ptr[4] = I2;
		PID_ptr[5] = D2;
}

/*************************************************
@Function:
@Description:PID参数改变 
@Called By: 
@Input: 协议接收到的float数据
@Others: 
*************************************************/
void Change_PID(float *rx_data)
{
		for (uint8_t i=0; i < 6; i++)
		{
				*PID_ptr[i] = *(rx_data + i);
		}
		
}
/*************************************************
@Function:
@Description: 返回上位机PID参数
@Called By: 
@Input: 
@Others: 
*************************************************/
void Return_PID(uint8_t *tx_buff, uint16_t *data_len)
{
		static float tx_data[6] = {0};
		
		for(int i = 0; i < 6;i++)
		{
				tx_data[i] = *PID_ptr[i];
		}
		get_protocol_send_data(EXCHANGE_ID,EXCHANGE_REG, tx_data, 6, tx_buff, data_len);
}
/*************************************************
@Function:Protocol_Wave_Send
@Description: 串口波形发送函数
@Called By: 
@Input: 
@Others: 
*************************************************/
void Protocol_Wave_Send(uint8_t num, ...)//先默认是float
{
		static uint16_t data_len = 0;
		static uint8_t tx_buff[40] = {0};
		static float tx_data[8] = {0};
		va_list ap;
		va_start(ap, num);
		for (int i = 0;i < num;i++)
		{
				tx_data[i] = va_arg(ap, float);
		}
		
		get_protocol_send_data(WAVE_ID,INITAL_REG, tx_data, num, tx_buff, &data_len);
		
		HAL_UART_Transmit(&huart2,tx_buff, data_len, 100);
	  va_end(ap);
}

