/*
 * CRC16_CCITT.c
 *
 *  Created on: 2016-10-18
 *      Author: Hypersen02
 */
#include "CRC16_CCITT.h"




/******************************************************************************
**							函数说明
**
**				输入：
**					CRC_acc:上一次的CRC值，
**						如果是第一次运算，则设
**						为0xffff
**					CRC_input:要计算的数据
**				返回值：
**						计算的crc结果，为16位字长
**
**
*****************************************************************************/
uint16_t UpdateCRC (uint16_t CRC_acc, uint8_t CRC_input)
{
	uint8_t i; // 循环计数器
	CRC_acc = CRC_acc ^ (CRC_input << 8);
	for (i = 0; i < 8; i++)
	{
		if ((CRC_acc & 0x8000) == 0x8000)//如果当前crc值最高位为1
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
 * 				函数说明
 * 			计算一组数据的CRC16 CCITT的值
 * 			参数：
 * 				输入：
 *
 * 					input_data:字节数组，为要计算的一组数据的数组指针
 * 					start_pos:起始的数组索引
 * 					cal_len:要计算的数据长度，不是指input_data的长度
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





