/*
 * CRC16_CCITT.h
 *
 *  Created on: 2016-10-18
 *      Author: Hypersen02
 */

#ifndef CRC16_CCITT_H_
#define CRC16_CCITT_H_
#include <stdint.h>

#define POLY 				0x1021   //生成多项式
#define	INIT_VALUE			0xffff	//初始值


uint16_t CRC16_Cal(uint16_t crc_base, uint8_t *input_data, uint8_t start_pos, uint8_t len);

#endif /* CRC16_CCITT_H_ */
