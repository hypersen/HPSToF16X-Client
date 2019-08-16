/*
 * sensor.c
 *
 *  Created on: 2016-10-18
 *      Author: Hypersen02
 */

#include "sensor.h"
#include "CRC16_CCITT.h"
#include <string.h>
#include "usart.h"
#include <stdbool.h>



/**
 * U8 *data_buffer:????????,????????????, ??????????data_len+4??(?)
 *	U8 data_len:??????
 *
 *
 *
 */

void dataPackaging(uint8_t *data_buffer, uint8_t data_len)
{
	uint8_t i=data_len;
	uint16_t CRC16_value = 0x0000;
	CRC16_value = CRC16_Cal(0xffff, data_buffer, 0, data_len);
	for(; i>0; i--)//??????????
	{
		data_buffer[i+1] = data_buffer[i-1];
	}
	data_buffer[0]=SENSOR_PACKET_START;
	data_buffer[1]=data_len+2;//??????,data+crc

	data_buffer[data_len+2] = CRC16_value >> 8; //CRC high byte
	data_buffer[data_len+3] = CRC16_value & 0x00ff; //CRC low byte

	//*((U16 *)(data_buffer+data_len+2)) = CRC16_value;
}



void sensor_continuous_start(uint8_t uart_sel)
{
	
	uint8_t comm[] = {0x0A, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x72};
	if(uart_sel == USE_UART1)
		uart1_write(comm, 10);
	else
	  uart2_write(comm, 10);
}




uint8_t packet_parser(uint8_t *packet, uint8_t data_buff[])
{
	uint16_t crc16_cal = 0x0000;
	uint16_t crc16_packet = 0x0000;
	uint8_t len = packet[1]+2;
	//uint8_t i= 0;
	if(packet[0] != SENSOR_PACKET_START)
	{
		return 0;
	}
	crc16_packet = packet[len-2]<<8;//取出数据包crc16
	crc16_packet |= packet[len-1];
	crc16_cal = CRC16_Cal(0xffff, packet, 2, len-4);
	if(crc16_cal == crc16_packet)
	{
		/*for(i=0; i<len-4; i++)
		{
			data_buff[i] = packet[2+i];
		}*/
		memcpy(data_buff, packet+2, len-4);
		return 1;
	}
	return 0;
}
void continuous_Stop(uint8_t uart_sel)
{
	uint8_t comm[] = {0x0A, 0x30, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBC, 0x6F};
	uint8_t read_buff[5] = {0};
	if(uart_sel == USE_UART1)
	{
		uart1_write(comm, 10);
		uart1_read(read_buff, 5, 100);
	}
	else
	{
		uart2_write(comm, 10);
		uart2_read(read_buff, 5, 100);
	}
		
}
uint8_t singleshot_Capture(sensorData *dbuf, uint8_t uart_sel)
{
	uint8_t comm[16] = {0x0A, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAE, 0x57};
	uint8_t read_buff[SENSOR_DATA_LEN]={0};
	read_buff[0]=0x00;
	if(uart_sel == USE_UART1)
	{
		uart1_write(comm, 10);
		uart1_read(read_buff, SENSOR_DATA_LEN, 500);
	}
	else
	{
		uart2_write(comm, 10);
		uart2_read(read_buff, SENSOR_DATA_LEN, 500);
	}
	memset(dbuf, 0, sizeof(sensorData));
	//memset(comm, 0, 10);

	if(packet_parser(read_buff, comm) == 1)
	{

		memcpy(comm, read_buff+2, read_buff[1]-2);


		dbuf->irq_flag = comm[0];
		dbuf->irq_mode = comm[1];
		dbuf->zone_mode = comm[2];
		dbuf->distance = (comm[3]<<8)|comm[4];
		dbuf->magnitude = (comm[5]<<8)|comm[6];
		dbuf->magnitude_exp = comm[7];
		dbuf->ambient_adc = comm[8];
		dbuf->precision = comm[9];
		dbuf->precision = comm[10];
		
		return 1;
	}
	return 0;
}
uint8_t waitForStartUp(uint8_t uart_sel)
{
	/*uint8_t S = 5;//最多等待5秒
	IRQ_Pin = 1;//先设置成高电平，否则将一直是低电平
	while(!IRQ_Pin && S)
	{
		waitS(1);//延时1s
		S--;
	}
	if(IRQ_Pin == 1)
	{
		return 1;
	}*/
	
	uint8_t read_buff[9]={0};
	if(uart_sel == USE_UART1)
		uart1_read(read_buff, 8, 200);
	else
		uart2_read(read_buff, 8, 200);

	if(0 == strcmp((const char *)read_buff, "Hypersen"))//匹配启动信息
	{
		return 1;
	}
	return 0;
}
#if(TOF_VERSION_SUPPORT == 166)
/****************************************************
 * 			函数名称：void sensorEnterBootloader(void)
 * 			功能：使tof进入bootloader
 *
 ***************************************************/
uint8_t sensorEnterBootloader(void)
{
	uint8_t comm[10]={0x0a, 0x2f, 0x22, 0x33, 0x44, 0x55, 0x66, 0x00, 0xa8, 0xf6};
	uint8_t test_byte = 0xff;
	uint8_t r_buff[10] = {0};
	uint8_t s_return;
	uart1_write(&test_byte, 1);
	uart1_write(&test_byte, 1);
	waitMs(1);
	s_return = SBUF0;
	s_return = 0;
	
	uart1_write(comm, 10);
	uart1_read(r_buff, 5, 1);
	packet_parser(r_buff, &s_return);
	if(s_return != 1)//没有返回数据说明没有固件，重新上电让TOF进入BootLoader模式
	{
		sensorPowerDown();
		waitUs(100);
		sensorPowerUp();
	}
	waitMs(100);
	return s_return;
}
#elif(TOF_VERSION_SUPPORT == 167)
uint8_t sensorEnterBootloader(void)
{
	uint8_t comm[10]={0x0a, 0x2f, 0x22, 0x33, 0x44, 0x55, 0x66, 0x00, 0xa8, 0xf6};
	uint8_t r_buff[10] = {0};
	uint8_t s_return;
	
	uart1_write(comm, 10);
	uart1_read(r_buff, 5, 1);
	packet_parser(r_buff, &s_return);
	ToF_BOOT0 = 1;
	sensorPowerDown();
	waitMs(50);
	sensorPowerUp();
	waitMs(50);
	return s_return;
}
#endif

uint8_t sensorSaveFactorySettings(uint8_t uart_sel)
{
	uint8_t comm[10]={0x0A, 0x33, 0xBB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x43};
	uint8_t read_buff[5] = {0};
	uint8_t s_return = 0;
	if(uart_sel == USE_UART1)
	{
		uart1_write(comm, 10);
		uart1_read(read_buff, 5, 200);
	}
	else
	{
		uart2_write(comm, 10);
		uart2_read(read_buff, 5, 200);
	}
	packet_parser(read_buff, &s_return);
	return s_return;
}

/*********************************************
 *					说明
 *					设置Sensor的温度偏移量
 * 			offset:温度的偏移量，有符号，
 * 			offset = 实际偏移量*100
 *
 * 			返回值：设置结果
 *
 */
uint8_t sensorSetTempOffset(int16_t offset, uint8_t uart_sel)
{
					  //报头       命令	   key	v_MSB v_LSB  unuse unuse unuse crc_MSB crc_LSB
	uint8_t comm[10]={0x0A, 0x37, 0xBF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t read_buff[5] = {0};
	uint8_t s_return = 0;
	uint16_t crc = 0x0000;
	comm[3] = offset >> 8;
	comm[4] = offset;
	crc = CRC16_Cal(0xffff, comm, 0, 8);
	comm[8] = crc >> 8;
	comm[9] = crc;
	if(uart_sel == USE_UART1)
	{
		uart1_write(comm, 10);
		uart1_read(read_buff, 5, 1);
	}
	else 
	{
		uart2_write(comm, 10);
		uart2_read(read_buff, 5, 1);
	}
	packet_parser(read_buff, &s_return);
	return s_return;
}
/***************************************
 * 			说明
 * 			设置光强阈值
 *
 * 			返回值：成功：1， 失败0
 */
/*uint8_t sensorSetMagnitudeThre(void)
{
	uint8_t comm[10]={0x0A, 0x42, 0x1b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t read_buff[5] = {0};
	uint8_t s_return = 0;
	uint16_t crc = 0x0000;
	uint8_t i=0;
	float mag = 0;
	uint8_t CNT = 128;
	sensorData dbuf = {0};
	//获取ToF数据
	for(i=0; i < CNT; i++)
	{
		singleshot_Capture(&dbuf);
		mag += (((U32)dbuf.magnitude)<<(dbuf.magnitude_exp))/10000.0f;
	}
	mag = mag/CNT;
	comm[3] = ((uint16_t)(mag*100)) >> 8;
	comm[4] = ((uint16_t)(mag*100));
	crc = CRC16_Cal(0xffff, comm, 0, 8);
	comm[8] = crc >> 8;
	comm[9] = crc;
	uart1_write(comm, 10);
	uart1_read(read_buff, 5, 1);
	packet_parser(read_buff, &s_return);
	return s_return;
}*/

uint8_t sensor_LockDevice(uint8_t uart_sel)
{
	uint8_t comm[10]={0x0A, 0x40, 0xfb, 0xff, 0x00, 0x00, 0x00, 0x00, 0x75, 0x1f};
	uint8_t read_buff[5] = {0};
	uint8_t s_return = 0;
	if(uart_sel == USE_UART1)
	{
		uart1_write(comm, 10);
		uart1_read(read_buff, 5, 1);
	}
	else
	{
		uart2_write(comm, 10);
		uart2_read(read_buff, 5, 1);
	}
	packet_parser(read_buff, &s_return);
	return s_return;
}
uint8_t sensor_clearProfile(uint8_t uart_sel)
{
	uint8_t comm[10]={0x0A, 0x41, 0x01, 0xa1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5a};
	uint8_t read_buff[5] = {0};
	uint8_t s_return = 0;
	if(uart_sel == USE_UART1)
	{
		uart1_write(comm, 10);
		uart1_read(read_buff, 5, 1);
	}
	else
	{
		uart2_write(comm, 10);
		uart2_read(read_buff, 5, 1);
	}
	
	packet_parser(read_buff, &s_return);
	return s_return;
}
uint8_t sensor_set_TempP1(int32_t p1, uint8_t uart_sel)
{
	uint8_t comm[10]={0x0A, 0x34, 0xBE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t read_buff[5] = {0};
	uint8_t s_return = 0;
	uint16_t crc = 0x0000;
	comm[3] = p1 >> 24;
	comm[4] = p1 >> 16;
	comm[5] = p1 >> 8;
	comm[6] = p1;
	crc = CRC16_Cal(0xffff, comm, 0, 8);
	comm[8] = crc >> 8;
	comm[9] = crc;
	if(uart_sel == USE_UART1)
	{
		uart1_write(comm, 10);
		uart1_read(read_buff, 5, 1);
	}
	else
	{
		uart2_write(comm, 10);
		uart2_read(read_buff, 5, 1);
	}
	
	packet_parser(read_buff, &s_return);
	return s_return;
}
uint8_t sensor_set_TempP2(int32_t p2, uint8_t uart_sel)
{
	uint8_t comm[10]={0x0A, 0x35, 0xBE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t read_buff[5] = {0};
	uint8_t s_return = 0;
	uint16_t crc = 0x0000;
	comm[3] = p2 >> 24;
	comm[4] = p2 >> 16;
	comm[5] = p2 >> 8;
	comm[6] = p2;
	crc = CRC16_Cal(0xffff, comm, 0, 8);
	comm[8] = crc >> 8;
	comm[9] = crc;
	if(uart_sel == USE_UART1)
	{
		uart1_write(comm, 10);
		uart1_read(read_buff, 5, 1);
	}
	else
	{
		uart2_write(comm, 10);
		uart2_read(read_buff, 5, 1);
	}
	
	packet_parser(read_buff, &s_return);
	return s_return;
}

uint8_t sensor_get_DeviceInfo(uint8_t *data_buff, uint8_t uart_sel)
{
	uint8_t comm[10]={0x0A, 0x2E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x3C};
	uint8_t read_buff[26] = {0};
	uint8_t s_return = 0;
	if(uart_sel == USE_UART1)
	{
		uart1_write(comm, 10);
		uart1_read(read_buff, 26, 200);
	}
	else
	{
		uart2_write(comm, 10);
		uart2_read(read_buff, 26, 200);
	}
	
	s_return = packet_parser(read_buff, data_buff);
	return s_return;
}
uint8_t sensor_get_Temperature(uint16_t *afe_temp, uint8_t uart_sel)
{
	uint8_t comm[10]={0x0A, 0x3F, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x36, 0x86};
	uint8_t read_buff[8] = {0};
	uint8_t data_buff[8] = {0};
	if(uart_sel == USE_UART1)
	{
		uart1_write(comm, 10);
		uart1_read(read_buff, 6, 500);
	}
	else
	{
		uart2_write(comm, 10);
		uart2_read(read_buff, 6, 500);
	}
	
	if(1 == packet_parser(read_buff, data_buff))
	{
		*afe_temp = (data_buff[0] << 8)|data_buff[1];
		return 1;
	}
	return 0;
}

uint8_t sensor_set_MagThre(uint8_t msb, uint8_t lsb, uint8_t uart_sel)
{
	uint8_t comm[10]={0x0A, 0x42, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t read_buff[5] = {0};
	uint8_t s_return = 0;
	uint16_t crc = 0x0000;
	comm[3] = msb;
	comm[4] = lsb;
	crc = CRC16_Cal(0xffff, comm, 0, 8);
	comm[8] = crc >> 8;
	comm[9] = crc;
	if(uart_sel == USE_UART1)
	{
		uart1_write(comm, 10);
		uart1_read(read_buff, 5, 200);
	}
	else
	{
		uart2_write(comm, 10);
		uart2_read(read_buff, 5, 200);
	}
	
	packet_parser(read_buff, &s_return);
	return s_return;
}
uint8_t sensor_set_OverRange(uint8_t msb, uint8_t lsb, uint8_t uart_sel)
{
	uint8_t comm[10]={0x0A, 0x43, 0x1d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t read_buff[5] = {0};
	uint8_t s_return = 0;
	uint16_t crc = 0x0000;
	comm[3] = msb;
	comm[4] = lsb;
	crc = CRC16_Cal(0xffff, comm, 0, 8);
	comm[8] = crc >> 8;
	comm[9] = crc;
	if(uart_sel == USE_UART1)
	{
		uart1_write(comm, 10);
		uart1_read(read_buff, 5, 200);
	}
	else
	{
		uart2_write(comm, 10);
		uart2_read(read_buff, 5, 200);
	}	
	packet_parser(read_buff, &s_return);
	return s_return;
}

uint8_t sensor_set_DistOffset(uint8_t msb, uint8_t lsb, uint8_t uart_sel)
{
	uint8_t comm[10]={0x0A, 0x38, 0x1a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t read_buff[5] = {0};
	uint8_t s_return = 0;
	uint16_t crc = 0x0000;
	comm[3] = msb;
	comm[4] = lsb;
	crc = CRC16_Cal(0xffff, comm, 0, 8);
	comm[8] = crc >> 8;
	comm[9] = crc;
	if(uart_sel == USE_UART1)
	{
		uart1_write(comm, 10);
		uart1_read(read_buff, 5, 200);
	}
	else
	{
		uart2_write(comm, 10);
		uart2_read(read_buff, 5, 200);
	}
	
	packet_parser(read_buff, &s_return);
	return s_return;
}

// sel : 0x00---->user
//			 0xff---->factory
uint8_t sensor_load_Profile(uint8_t sel, uint8_t uart_sel)
{
	uint8_t comm[10]={0x0A, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t read_buff[5] = {0};
	uint8_t s_return = 0;
	uint16_t crc = 0x0000;
	comm[2] = sel;
	crc = CRC16_Cal(0xffff, comm, 0, 8);
	comm[8] = crc >> 8;
	comm[9] = crc;
	if(uart_sel == USE_UART1)
	{
		uart1_write(comm, 10);
		uart1_read(read_buff, 5, 200);
	}
	else
	{
		uart2_write(comm, 10);
		uart2_read(read_buff, 5, 200);
	}
	
	packet_parser(read_buff, &s_return);
	return s_return;
}

uint8_t sensor_set_Filter(uint8_t msb, uint8_t lsb, uint8_t uart_sel)
{
	uint8_t comm[10]={0x0A, 0x3d, 0xaa, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t read_buff[5] = {0};
	uint8_t s_return = 0;
	uint16_t crc = 0x0000;
	comm[3] = msb;
	comm[4] = lsb;
	crc = CRC16_Cal(0xffff, comm, 0, 8);
	comm[8] = crc >> 8;
	comm[9] = crc;
	if(uart_sel == USE_UART1)
	{
		uart1_write(comm, 10);
		uart1_read(read_buff, 5, 200);
	}
	else
	{
		uart2_write(comm, 10);
		uart2_read(read_buff, 5, 200);
	}
	
	packet_parser(read_buff, &s_return);
	return s_return;
}


