/*
 * util.c
 *
 *  Created on: 2016-10-9
 *      Author: Hypersen02
 */
#include "util.h"
//��4���ֽڵ�byte����ת��Ϊ32λuint32_t
uint32_t bytes2uint(uint8_t *bytes)
{
	return (uint32_t)((bytes[0] << 24) | (bytes[1] << 16) | (bytes[2] << 8) | (bytes[3]));
}
void uint2bytes(const uint32_t *s, uint8_t *bytesBuf)
{
	bytesBuf[0] = (*s)>>24;
	bytesBuf[1] = (*s)>>16;
	bytesBuf[2] = (*s)>>8;
	bytesBuf[3] = (*s);
}
void ushort2bytes(const uint16_t *s, uint8_t *bytesBuf)
{
	bytesBuf[0] = (*s)>>8;
	bytesBuf[1] = (*s);
}
void int2string(int32_t *src, uint8_t *stringBuff)
{
	 int32_t sign;
	 uint8_t i=0,j=0;
	 uint8_t temp[10];
	sign=(*src);
	do{
		temp[i]=sign%10+'0';
		sign/=10;
		i++;
	}while(sign>0);
	while(i>0){
		stringBuff[j]=temp[i-1];
		j++;
		i--;
	}
	stringBuff[j]='\0';
}
/***********
 *
 * num:Ҫת������
 * buf�����ת�����, �뱣֤buf�ռ��㹻�󣬺��������Խ��м��
 * width:С�����ֵ�λ��
 *
 */
void ftostr(float num, uint8_t *buf, uint8_t width)
{
	 uint16_t intege = num;
	 uint8_t i = 0, k, temp;
	if(intege == 0)
	{
		buf[i ++] = '0';
	}
	while(intege > 0)
	{
		buf[i ++] = intege % 10 + '0';
		intege /= 10;
	}
	for(k=0;k<i/2;k++)
	{
		temp = buf[k];
		buf[k] = buf[i-1-k];
		buf[i-1-k] = temp;
	}
	buf[i++] = '.';
	num -= (uint16_t)num;
	while(width > 0)
	{
		num = num * 10;
		buf[i++] = num + '0';
		num -= (uint16_t)num;
		width --;
	}
	while(i < 9)
	{
		buf[i++] = ' ';//�������µ�λ�ã�����������������
	}
	buf[i] = '\0';

}
