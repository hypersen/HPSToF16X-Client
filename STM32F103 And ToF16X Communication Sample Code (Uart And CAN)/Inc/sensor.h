/*
 * sensor.h
 *
 *  Created on: 2016-10-18
 *      Author: Hypersen02
 */

#ifndef SENSOR_H_
#define SENSOR_H_

#include <stdint.h>

#define	SENSOR_PACKET_START 		0x0a
#define	SENSOR_DATA_LEN 	15
// error codes;
#define	CSL501_SUCCEED				1
#define	CSL501_ERROR				2
#define CSL501_INVALID_PARAM		3
#define	CSL501_INCORRECT_MODE		4
#define	CSL501_BUFFER_FULL			5
#define CSL501_OPERATION_TIMEOUT	6
#define CSL501_DATA_NOT_READY		7
#define CSL501_INVALID_COMM			8
#define CSL501_DATA_OVERFLOW		9

#define PREHEAT_START				0x1e
#define	PREHEAT_STOP				0x1f

#define USE_UART1		0xa0
#define	USE_UART2		0xa1


typedef struct RO_DATA
{
	uint16_t	distance;
	uint16_t	precision;
	uint16_t	magnitude;
	uint16_t	phase;
	uint16_t	i_raw;
	uint16_t	q_raw;
	uint16_t	gain;
	uint16_t i_adc;
	uint16_t q_adc;
	uint8_t	magnitude_exp;
	uint8_t	i_raw_exp;
	uint8_t	q_raw_exp;
	uint8_t	voltage_before;
	uint8_t	voltage_after;
	uint8_t	afe_temp;
	uint8_t	ambient_adc;
	uint8_t irq_flag;
	uint8_t irq_mode;
	uint8_t zone_mode;
	uint8_t	vga1;
	uint8_t	vga2;

}sensorData;



uint8_t packet_parser(uint8_t *packet, uint8_t data_buff[]);
void dataPackaging(uint8_t *data_buffer, uint8_t data_len);

uint8_t waitForStartUp(uint8_t);
void continuous_Stop(uint8_t);
uint8_t singleshot_Capture(sensorData *dbuf, uint8_t);
uint8_t sensorSaveFactorySettings(uint8_t);
uint8_t sensorSetTempOffset(int16_t offset, uint8_t);
//uint8_t sensorSetMagnitudeThre(void);
uint8_t sensor_LockDevice(uint8_t);
uint8_t sensor_clearProfile(uint8_t);
uint8_t sensor_set_TempP1(int32_t p1, uint8_t);
uint8_t sensor_set_TempP2(int32_t p2, uint8_t);
uint8_t sensor_get_DeviceInfo(uint8_t *data_buff, uint8_t);
uint8_t sensor_get_Temperature(uint16_t *afe_temp, uint8_t);
uint8_t sensor_set_MagThre(uint8_t msb, uint8_t lsb, uint8_t);
uint8_t sensor_set_OverRange(uint8_t msb, uint8_t lsb, uint8_t);
void sensor_continuous_start(uint8_t);
uint8_t sensor_set_DistOffset(uint8_t msb, uint8_t lsb, uint8_t);
uint8_t sensor_load_Profile(uint8_t, uint8_t);
uint8_t sensor_set_Filter(uint8_t msb, uint8_t lsb, uint8_t);

#endif /* SENSOR_H_ */
