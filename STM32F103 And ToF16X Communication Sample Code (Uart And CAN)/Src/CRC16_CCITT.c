/*
 * CRC16_CCITT.c
 *
 *  Created on: 2016-10-18
 *      Author: Hypersen02
 */
#include "CRC16_CCITT.h"




/******************************************************************************
**							����˵��
**
**				���룺
**					CRC_acc:��һ�ε�CRCֵ��
**						����ǵ�һ�����㣬����
**						Ϊ0xffff
**					CRC_input:Ҫ���������
**				����ֵ��
**						�����crc�����Ϊ16λ�ֳ�
**
**
*****************************************************************************/
uint16_t UpdateCRC (uint16_t CRC_acc, uint8_t CRC_input)
{
	uint8_t i; // ѭ��������
	CRC_acc = CRC_acc ^ (CRC_input << 8);
	for (i = 0; i < 8; i++)
	{
		if ((CRC_acc & 0x8000) == 0x8000)//�����ǰcrcֵ���λΪ1
		{
			CRC_acc = CRC_acc << 1;
			CRC_acc ^= POLY;
		}
		else
		{
			CRC_acc = CRC_acc << 1;
		}
	}
	return CRC_acc;
}
/*****************************************************
 * 				����˵��
 * 			����һ�����ݵ�CRC16 CCITT��ֵ
 * 			������
 * 				���룺
 *
 * 					input_data:�ֽ����飬ΪҪ�����һ�����ݵ�����ָ��
 * 					start_pos:��ʼ����������
 * 					cal_len:Ҫ��������ݳ��ȣ�����ָinput_data�ĳ���
 *
 ****************************************************/
uint16_t CRC16_Cal(uint16_t crc_base, uint8_t *input_data, uint8_t start_pos, uint8_t cal_len)
{
	uint16_t res = UpdateCRC(crc_base, input_data[start_pos]);
	cal_len += start_pos;
	start_pos++;
	while(start_pos < cal_len)
	{
		res = UpdateCRC(res, input_data[start_pos++]);
	}
	return res;
}





