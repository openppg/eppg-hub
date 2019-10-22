/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "data_log.h"
#include "flash.h"
#include "controllerInterface.h"
#include "baro_LPS22HB.h"
#include "imu_LSM9DS1.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define APPLICATION_ADDRESS     (uint32_t)0x08008000
#if   (defined ( __CC_ARM ))
__IO uint32_t VectorTable[48] __attribute__((at(0x20000000)));
#elif (defined (__ICCARM__))
#pragma location = 0x20000000
__no_init __IO uint32_t VectorTable[48];
#elif defined   (  __GNUC__  )
__IO uint32_t VectorTable[48] __attribute__((section(".RAMVectorTable")));
#endif /* (defined ( __CC_ARM )) */

__attribute__((__section__(".crc"))) const uint32_t crc = 0xF0F0F0F0;

__attribute__((__section__(".user_data"))) const STR_CONFIG paramSpace;
STR_CONFIG parameters;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t armedState = 0;
uint8_t baroState = 0;
uint8_t imuState = 0;
uint8_t magState = 0;
int32_t uartTimeout = 0;
uint32_t debugTime[20];
char resetString[20];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	/* Copy the vector table from the Flash (mapped at the base of the application
	     load address 0x08003000) to the base address of the SRAM at 0x20000000. */
	  for (uint32_t i = 0; i < 48; i++)
	  {
	    VectorTable[i] = *(__IO uint32_t*)(APPLICATION_ADDRESS + (i << 2));
	  }
	  //volatile uint32_t *VectorTable = (volatile uint32_t *)0x20000000;
	  //for(uint32_t ui32_VectorIndex = 0; ui32_VectorIndex < 48; ui32_VectorIndex++)
	  //{
	  //  VectorTable[ui32_VectorIndex] = *(__IO uint32_t*)((uint32_t)APPLICATION_ADDRESS + (ui32_VectorIndex << 2));
	 // }
	  //  Enable SYSCFG peripheral clock
	  __HAL_RCC_SYSCFG_CLK_ENABLE();
	   // Remap RAM into 0x0000 0000
	  __HAL_SYSCFG_REMAPMEMORY_SRAM();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  __HAL_RCC_DBGMCU_CLK_ENABLE();
  __HAL_DBGMCU_FREEZE_IWDG();
  flashInit();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART4_UART_Init();
  MX_USART5_UART_Init();
  MX_USART6_UART_Init();
  MX_FATFS_Init();
  MX_SPI2_Init();
  MX_SPI1_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  //WDT set to prescaler = 32, reload value = 4095. Provides about 3200ms before reset.
  IWDG_SetPrescaler(IWDG_PRESCALER_32);
  WDT_RESET;

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  TCHAR fileName[100] = "log";
  char logData[1024];
  FRESULT logStatus = 0;
  WDT_RESET;
  logStatus = logInit(fileName);

  baroState = baroInit();
  imuState = imuInit();
  magState = magInit();

  if(logStatus == FR_OK)
  {
	  uint32_t bytesWritten = 0;
	  memset(logData, 0, sizeof(logData));
	  if(__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))
		  sprintf(resetString,"Watchdog Timer");
	  else if(__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))
		  sprintf(resetString,"Power Cycle");
	  else
		  sprintf(resetString,"Other Cause");
	  sprintf(logData, "Reset Cause: %s\n", resetString);
 	  sprintf(logData+strlen(logData), "Time(ms),Esc Num, Packet Num, ThrottleInput, RPM, Voltage (), Current (), mAh, CapTemp, FetTemp\n");
 	  logStatus = logWriteData(logData, strlen(logData), bytesWritten);
 	 __HAL_RCC_CLEAR_RESET_FLAGS();
  }

  WDT_RESET;
  uartHandlesInit();

  UART_ESC_BeginReceive(&huartM1);
  UART_ESC_BeginReceive(&huartM2);
  UART_ESC_BeginReceive(&huartM3);
  UART_ESC_BeginReceive(&huartM4);

  UART_RS485_BeginReceive(&huartController);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //WDT set to prescaler = 8, reload value = 4095. Provides about 800ms before reset.
  IWDG_SetPrescaler(IWDG_PRESCALER_8);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  WDT_RESET;
	uint32_t currentTime = HAL_GetTick();

	  static uint32_t lastTime = 0;
	  static uint32_t blinkTime = 510;
	  if(currentTime - lastTime > blinkTime)
	  {
		  STATUS_LED_TOGGLE;
		  lastTime = currentTime;
		  static uint8_t counter = 0;
		  if(uartTimeout > 0)
		  {
			  blinkTime = 50;
			  uartTimeout--;
		  }
		  else if(uartTimeout < 0)
		  {
			  blinkTime = 500;
			  uartTimeout = 0;
		  }
		  else
			  blinkTime = 500;


	  }

	  //Log data in CSV format
	  if(logStatus == FR_OK && armedState)
	  {
		  uint32_t startWriteTimeAll = HAL_GetTick();
		  static uint32_t lastPacketTime[NUM_ESCS];

		  uint32_t bytesWritten = 0;
		  memset(logData, 0, sizeof(logData));
		  for(uint8_t i = 0; i<NUM_ESCS; i++)
		  {
			  //Dont log the same data twice
			  if(escData[i].timeStamp > 0 && lastPacketTime[i] != escData[i].timeStamp)
			  {
				  sprintf(logData+strlen(logData), "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n", escData[i].timeStamp, i, escData[i].packetNum, escData[i].throttleInput, escData[i].rpm, escData[i].voltage, escData[i].current, escData[i].mah, escData[i].capTemp, escData[i].fetTemp);
				  lastPacketTime[i] = escData[i].timeStamp;
			  }
			  //logStatus = logWriteData(fileName, (uint8_t*)logData, (i+1)*256, bytesWritten);
			  //debugTime[i] = HAL_GetTick() - startWriteTime;
		  }

		  if(strlen(logData)>1)
		  {
			  logStatus = logWriteData((uint8_t*)logData, strlen(logData), bytesWritten);
			  debugTime[4] = HAL_GetTick() - startWriteTimeAll;
		  }

	  }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL5;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
