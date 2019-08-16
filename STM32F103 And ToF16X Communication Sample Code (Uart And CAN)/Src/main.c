/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "can.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "oled.h"
#include "sensor.h"
#include "util.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
  uint8_t sensorID_CAN = 0x00;
	CanTxMsgTypeDef        TxMessage;
	CanRxMsgTypeDef        RxMessage;
	
	volatile uint16_t dist = 0x0000;
	volatile uint32_t mag = 0x00000000;
	volatile uint8_t ambient = 0x00;
	volatile bool uart2_rx_done = false;
	volatile uint8_t buff[32] ={0};
	volatile char dev[6] = {0}; //当前的设备信息
	
const uint32_t MY_EXTID = (0x10310000);
const uint16_t MY_STDID = (0x731);
	
	#define		UART2_FOR_CAN_OUT		0		//将CAN的数据通过uart2输出
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == (&huart2))
	{
		uart2_rx_done = true;
	}
	//HAL_GPIO_WritePin(TEST_GPIO_Port, TEST_Pin, GPIO_PIN_SET);
	
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart == (&huart2))
	{
		//如果UART2发生了错误中断,中断接收会被打断，所以要让主循环的等待退出
		uart2_rx_done = true;
	}
}


/**
  * @brief  Error CAN callback.
  * @param  hcan: pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	hcan->Instance->TSR = (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2);//????????
	hcan->Instance->IER = 0;        //清除全部中断
	hcan->Instance->ESR = 0;        //清除错误计数
	HAL_CAN_MspDeInit(hcan);
	HAL_CAN_MspInit(hcan);
	//准备接收下一帧
	if(HAL_OK != HAL_CAN_Receive_IT(hcan, CAN_FIFO0))
	{
		//uart2_write("init can filter failed\n", 23);
	}
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint8_t buff[64] = {0};
	uint8_t data_buff[32] = {0};
	uint16_t crc = 0x0000;
	uint16_t crc_p = 0x00000;
	uint16_t mag_i = 0x0000;
	uint8_t mag_exp = 0x00;
	if(GPIO_Pin == GPIO_EXTI_Pin)
	{
			//HAL_UART_Receive_IT(&huart2 ,buff, 15);
			uart1_read(buff, 2, 2);//读取报头信息
			if(buff[0] == 0x0a && buff[1] == 0x0d)
			{
				uart1_read(buff+2, buff[1], 2);
				if(packet_parser(buff, data_buff) == 1)
				{	
					 dist = (data_buff[3]<<8)+data_buff[4];
					 mag_i = (data_buff[5] <<8) + data_buff[6];
					 mag_exp = data_buff[7];
					 mag = ((uint32_t)mag_i)<<(mag_exp);
					 ambient = data_buff[8];
					
					 dev[0] = 'U';
					 dev[1] = 'A';
					 dev[2] = 'R';
					 dev[3] = 'T';
					 dev[4] = '1';
					 dev[5] = 0;
					
				}
				
			}
			else
			{
				HAL_UART_Abort(&huart1);
				uart1_read(buff, 15, 5);
			}
	}
}

/**
  * @brief  Period elapsed callback in non blocking mode 
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	char disp_buff[32] = {0};
	sprintf(disp_buff, "%s", dev);
	OLED_ShowString(86,0,(uint8_t *)disp_buff,0);
	sprintf(disp_buff, "%.3f", dist/1000.0f);
	OLED_ShowString(40,1, (uint8_t *)disp_buff,0);
	sprintf(disp_buff, "%.3f", mag/10000.0f);
	OLED_ShowString(40,2,(uint8_t *)disp_buff,0);
	/*sprintf(disp_buff, "%.1f  %s", (float)ambient, dev);
	OLED_ShowString(48,3,(uint8_t *)disp_buff,0);*/
}


void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
	uint16_t mag_i = 0x0000;
	uint8_t mag_exp = 0x00;
	uint8_t buff[32] = {0};
	char str_buff[32] = {0};
	for(int i=0; i<hcan->pRxMsg->DLC; i++)
	{
		buff[i] = hcan->pRxMsg->Data[i];
	}
	
	
	
	if((buff[0]&0xf0) == (sensorID_CAN&0xf0))//通用版CAN协议
	{
		if(buff[1] == 0x24 && (buff[0] & 0xf) == 0)//连续测量数据包并且是第一个数据包
		{
			//接收第2个数据包
				if(HAL_OK == HAL_CAN_Receive(hcan, CAN_FIFO0, 100))
				{
						for(int i=8; i<8+hcan->pRxMsg->DLC; i++)
						{
							buff[i] = hcan->pRxMsg->Data[i-8];
						}
						
						if(buff[9] == 0x24 && (buff[8] & 0xf) == 1)//连续测量数据包并且是第2个数据包
						{
							 dist = (buff[3]<<8)+buff[4];
							 mag_i = (buff[5] <<8) + buff[6];
							 mag_exp = buff[7];
							 mag = ((uint32_t)mag_i)<<(mag_exp);
							 ambient = buff[11];
							
							 dev[0] = 'G';
							 dev[1] = 'C';
							 dev[2] = 0;
							 dev[3] = 0;
							 dev[4] = 0;
						}
				}
		}
		else if(buff[1] == 0xfe)//can version
		{
			sprintf(str_buff, "V%d.%d%d", buff[3], buff[4], buff[5]);
			OLED_ShowString(40,0, (uint8_t *)str_buff,0);
		}
		else if(buff[1] == 0x2e)//ToF version
		{
				if(((hcan->pRxMsg->Data[0] & 0x0f) == 3) && (hcan->pRxMsg->Data[1] == 0x2e))
				{
					buff[0] = hcan->pRxMsg->Data[5];
					buff[1] = hcan->pRxMsg->Data[6];
					buff[2] = hcan->pRxMsg->Data[7];
				}
			
				if(((hcan->pRxMsg->Data[0] & 0x0f) == 4) && (hcan->pRxMsg->Data[1] == 0x2e))
				{
					buff[3] = hcan->pRxMsg->Data[3];
					buff[4] = hcan->pRxMsg->Data[4];
					sprintf(str_buff, "20%d-%d-%d V%d.%d", buff[0], buff[1], buff[2], buff[3], buff[4]);
					OLED_ShowString(0,3, (uint8_t *)str_buff,0);
				}
			
		}	
	}
	
	//准备接收下一帧
	if(HAL_OK != HAL_CAN_Receive_IT(hcan, CAN_FIFO0))
	{
		
	}
	
	
	

	
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	CAN_FilterConfTypeDef sFilterConfig;
	uint16_t mag_i = 0;
	uint8_t mag_exp = 0;
	uint8_t data_buff[32] = {0};
	char str_buff[32] = {0};
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
	hcan.pRxMsg = &RxMessage;
	hcan.pTxMsg = &TxMessage;
			//初始化CAN Filter
	sFilterConfig.FilterIdHigh = 0;
	sFilterConfig.FilterIdLow = 0;
	sFilterConfig.FilterMaskIdHigh = 0;
	sFilterConfig.FilterMaskIdLow = 0;
	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sFilterConfig.FilterNumber = 1;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.BankNumber = 1;
	
	if(HAL_OK != HAL_CAN_ConfigFilter(&hcan, &sFilterConfig))
	{
		//uart2_write("init can filter failed\n", 23);
	}

	
	if(HAL_OK != HAL_CAN_WakeUp(&hcan))
	{
		//uart2_write("init can filter failed\n", 23);
	}
	HAL_TIM_Base_Stop_IT(&htim2);
	HAL_Delay(500);//等待传感器初始化完成
	OLED_Init();			//???OLED
	OLED_ShowString(0,0,"CVER:",0);//small front
	OLED_ShowString(0,1,"DIST:",0);
	OLED_ShowString(0,2,"MAGN:",0);
	//OLED_ShowString(0,3,"TVER:",0);
	
	//获取CAN 传感器ID号
	hcan.pTxMsg->StdId = 0x731 & 0x7ff;//
	hcan.pTxMsg->IDE = CAN_ID_STD;//
	hcan.pTxMsg->RTR = CAN_RTR_DATA;
	hcan.pTxMsg->DLC = 8;
	hcan.pTxMsg->Data[0] = 0xf0;;
	hcan.pTxMsg->Data[1] = 0xff;
	hcan.pTxMsg->Data[2] = 0x00;
	hcan.pTxMsg->Data[3] = 0xb0;
	HAL_CAN_Transmit(&hcan, 100);
	if(HAL_OK == HAL_CAN_Receive(&hcan, CAN_FIFO0, 300))
	{
		sensorID_CAN = hcan.pRxMsg->Data[0] & 0xf0;//传感器ID号
		//启动CAN中断接收数据
		if(HAL_OK != HAL_CAN_Receive_IT(&hcan, CAN_FIFO0))
		{
			//uart2_write("init can filter failed\n", 23);
		}
		//CAN固件版本
		hcan.pTxMsg->StdId = 0x731 & 0x7ff;//
		hcan.pTxMsg->IDE = CAN_ID_STD;//
		hcan.pTxMsg->RTR = CAN_RTR_DATA;
		hcan.pTxMsg->DLC = 8;
		hcan.pTxMsg->Data[0] = sensorID_CAN | 0x0;;
		hcan.pTxMsg->Data[1] = 0xfe;
		HAL_CAN_Transmit(&hcan, 100);
		
		//ToF固件信息
		hcan.pTxMsg->StdId = 0x731 & 0x7ff;//
		hcan.pTxMsg->IDE = CAN_ID_STD;//
		hcan.pTxMsg->RTR = CAN_RTR_DATA;
		hcan.pTxMsg->DLC = 8;
		hcan.pTxMsg->Data[0] = sensorID_CAN | 0x0;;
		hcan.pTxMsg->Data[1] = 0x2e;
		HAL_CAN_Transmit(&hcan, 100);
		
		//韦嘉CAN协议需要发送特定命令打开附件数据输出，方便调试
		hcan.pTxMsg->StdId = 0x731 & 0x7ff;//
		hcan.pTxMsg->IDE = CAN_ID_STD;//
		hcan.pTxMsg->RTR = CAN_RTR_DATA;
		hcan.pTxMsg->DLC = 8;
		hcan.pTxMsg->Data[0] = sensorID_CAN | 0x0;;
		hcan.pTxMsg->Data[1] = 0xf7;
		hcan.pTxMsg->Data[2] = 0xf5;
		hcan.pTxMsg->Data[3] = 0x1;
		HAL_CAN_Transmit(&hcan, 100);
		

		
		
		//开启连续输出模式
		hcan.pTxMsg->StdId = 0x731 & 0x7ff;//
		hcan.pTxMsg->IDE = CAN_ID_STD;//
		hcan.pTxMsg->RTR = CAN_RTR_DATA;
		hcan.pTxMsg->DLC = 8;
		hcan.pTxMsg->Data[0] = sensorID_CAN | 0x0;
		hcan.pTxMsg->Data[1] = 0x24;
		HAL_CAN_Transmit(&hcan, 100);
		
	}
	else
	{
		//启动CAN中断接收数据
		if(HAL_OK != HAL_CAN_Receive_IT(&hcan, CAN_FIFO0))
		{
			//uart2_write("init can filter failed\n", 23);
		}
	}
	

	
	//打开UART1和UART2的传感器连续输出模式
	continuous_Stop(USE_UART1);	
	uart1_read(buff, 8, 200);//可能会有启动信息(Hypersen)，要滤掉
	if(sensor_get_DeviceInfo(buff, 	USE_UART1) == 1)
	{
		sprintf(str_buff, "20%d-%d-%d V%d.%d", buff[17], buff[18], buff[19], buff[20], buff[21]);
		OLED_ShowString(0,3, (uint8_t *)str_buff,0);
	}
	HAL_Delay(100);
	sensor_continuous_start(USE_UART1);
	
#if !UART2_FOR_CAN_OUT
	continuous_Stop(USE_UART2);	
	uart2_read(buff, 8, 200);	//可能会有启动信息(Hypersen)，要滤掉
	if(sensor_get_DeviceInfo(buff, 	USE_UART2) == 1)
	{
		sprintf(str_buff, "20%d-%d-%d V%d.%d", buff[17], buff[18], buff[19], buff[20], buff[21]);
		OLED_ShowString(0,3, (uint8_t *)str_buff,0);
	}
	sensor_continuous_start(USE_UART2);
	
#endif
	HAL_TIM_Base_Start_IT(&htim2);

	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	}
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
