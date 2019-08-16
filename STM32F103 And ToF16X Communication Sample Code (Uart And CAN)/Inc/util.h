/*
 * util.h
 *
 *  Created on: 2016-10-9
 *      Author: Hypersen02
 */

#ifndef UTIL_H_
#define UTIL_H_
#include <stdint.h>

uint32_t bytes2uint(uint8_t *bytes);
void uint2bytes(const uint32_t *s, uint8_t *bytesBuf);
void ushort2bytes(const uint16_t *s, uint8_t *bytesBuf);
void int2string(int32_t *src, uint8_t *stringBuff);
void ftostr(float num, uint8_t *buf, uint8_t width);

#endif /* UTIL_H_ */
