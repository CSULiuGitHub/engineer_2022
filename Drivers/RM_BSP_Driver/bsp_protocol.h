#ifndef _BSP_PROTOCOL_H
#define _BSP_PROTOCOL_H

#include "stdint.h"


#define PROTOCOL_CMD_ID 0XA5

#define OFFSET_BYTE 8 //�����ݶ��⣬����������ռ�ֽ���
#define WAVE_ID 0x0001 
#define CONFIRM_ID 0x0002//������Ϣȷ�Ͻ���ID
#define EXCHANGE_ID 0x0003//��Ϣ����ID
#define INITAL_REG 0x0000
#define CONFIRM_REG 0x0001 //������Ϣȷ�Ͻ��ռĴ���
#define EXCHANGE_REG 0x0002


typedef struct
{
    __packed struct
    {
        uint8_t  sof;
        uint16_t data_length;
        uint8_t  crc_check; //֡ͷCRCУ��
    } header;                                 //����֡ͷ
    uint16_t cmd_id;                  //����ID
    uint16_t frame_tail;              //֡βCRCУ��
} protocol;

/*���·�������֡�������㷢������֡����*/
void get_protocol_send_data
(uint16_t send_id,              //�ź�id
uint16_t flags_register, //16λ�Ĵ���
float    *tx_data,              //�����͵�float����
uint8_t  float_length,//float�����ݳ���
uint8_t  *tx_buf,               //�����͵�����֡
uint16_t *tx_buf_len);          //�����͵�����֡����

/*�������ݴ���*/
uint16_t get_protocol_info
(uint8_t  *rx_buf,                    //���յ���ԭʼ����
uint16_t *rx_pos,                     //ԭʼ����ָ��
uint16_t *flags_register,     //�������ݵ�16λ�Ĵ�����ַ
float    *rx_data);           //���յ�float���ݴ洢��ַ

uint8_t protocol_heade_Check(
    protocol *pro,
    uint8_t  *rx_buf,
    uint16_t *rx_pos);//ͷ֡�ж�

/*�жϺ�����ֵ����*/
void Protocol_Rx_IRQ(uint8_t res);

void Protocol_Wave_Send(uint8_t num, ...);//��Ĭ����float

void Return_PID(uint8_t *tx_buff, uint16_t *data_len);
void Change_PID(float *rx_data);
void Set_Addr(float *P1, float *I1, float *D1, float *P2, float *I2, float *D2);
#endif
