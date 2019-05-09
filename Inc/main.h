/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "stm32f0xx_ll_usart.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_pwr.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_dma.h"

#include "stm32f0xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include "escTelemetry.h"
#include "usart.h"
#include "iwdg.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

#define VERSION 0
#define CANDIDATE 0
#define BUILD 2

#pragma pack(push, 1)

typedef union
{
	struct
	{
		uint8_t version;
		uint8_t candidate;
		uint8_t build;
		//STR_TEST testData;
	};
	uint8_t raw[2048];
}STR_CONFIG;

extern STR_CONFIG parameters;
extern const STR_CONFIG paramSpace;

#pragma pack(pop)
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern uint32_t debugTime[20];
extern int32_t uartTimeout;
extern UN_ESC_PACKET motor1_data;	//usart 5
extern UN_ESC_PACKET motor2_data;	//usart 3
extern UN_ESC_PACKET motor3_data;	//usart 6
extern UN_ESC_PACKET motor4_data;	//usart 4
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IO_USART2_EN_Pin GPIO_PIN_13
#define IO_USART2_EN_GPIO_Port GPIOC
#define IO_LED_STATUS_Pin GPIO_PIN_14
#define IO_LED_STATUS_GPIO_Port GPIOC
#define IO_SD_CS_Pin GPIO_PIN_15
#define IO_SD_CS_GPIO_Port GPIOC
#define IO_M4_TELEM_Pin GPIO_PIN_0
#define IO_M4_TELEM_GPIO_Port GPIOA
#define IO_SD_WP_Pin GPIO_PIN_1
#define IO_SD_WP_GPIO_Port GPIOA
#define IO_USART2_TX_Pin GPIO_PIN_2
#define IO_USART2_TX_GPIO_Port GPIOA
#define IO_USART2_RX_Pin GPIO_PIN_3
#define IO_USART2_RX_GPIO_Port GPIOA
#define IO_M3_TELEM_Pin GPIO_PIN_4
#define IO_M3_TELEM_GPIO_Port GPIOA
#define IO_SPI1_SCK_Pin GPIO_PIN_5
#define IO_SPI1_SCK_GPIO_Port GPIOA
#define IO_M4_Pin GPIO_PIN_6
#define IO_M4_GPIO_Port GPIOA
#define IO_M3_Pin GPIO_PIN_7
#define IO_M3_GPIO_Port GPIOA
#define IO_M2_Pin GPIO_PIN_0
#define IO_M2_GPIO_Port GPIOB
#define IO_M1_Pin GPIO_PIN_1
#define IO_M1_GPIO_Port GPIOB
#define IO_SD_CD_Pin GPIO_PIN_2
#define IO_SD_CD_GPIO_Port GPIOB
#define IO_M2_TELEM_Pin GPIO_PIN_10
#define IO_M2_TELEM_GPIO_Port GPIOB
#define IO_BARO_DRDY_Pin GPIO_PIN_11
#define IO_BARO_DRDY_GPIO_Port GPIOB
#define IO_SPI2_CS_BARO_Pin GPIO_PIN_12
#define IO_SPI2_CS_BARO_GPIO_Port GPIOB
#define IO_SPI2_SCK_Pin GPIO_PIN_13
#define IO_SPI2_SCK_GPIO_Port GPIOB
#define IO_SPI2_MISO_Pin GPIO_PIN_14
#define IO_SPI2_MISO_GPIO_Port GPIOB
#define IO_SPI2_MOSI_Pin GPIO_PIN_15
#define IO_SPI2_MOSI_GPIO_Port GPIOB
#define IO_IMU_INT2_Pin GPIO_PIN_8
#define IO_IMU_INT2_GPIO_Port GPIOA
#define IO_USART1_TX_Pin GPIO_PIN_9
#define IO_USART1_TX_GPIO_Port GPIOA
#define IO_USART1_RX_Pin GPIO_PIN_10
#define IO_USART1_RX_GPIO_Port GPIOA
#define IO_I2C2_SCL_Pin GPIO_PIN_11
#define IO_I2C2_SCL_GPIO_Port GPIOA
#define IO_I2C2_SDA_Pin GPIO_PIN_12
#define IO_I2C2_SDA_GPIO_Port GPIOA
#define IO_IMU_INT1_Pin GPIO_PIN_15
#define IO_IMU_INT1_GPIO_Port GPIOA
#define IO_M1_TELEM_Pin GPIO_PIN_3
#define IO_M1_TELEM_GPIO_Port GPIOB
#define IO_USART1_EN_Pin GPIO_PIN_4
#define IO_USART1_EN_GPIO_Port GPIOB
#define IO_SPI1_MOSI_Pin GPIO_PIN_5
#define IO_SPI1_MOSI_GPIO_Port GPIOB
#define IO_MAG_INT_Pin GPIO_PIN_6
#define IO_MAG_INT_GPIO_Port GPIOB
#define IO_MAG_DRDY_Pin GPIO_PIN_7
#define IO_MAG_DRDY_GPIO_Port GPIOB
#define IO_SPI2_CS_MAG_Pin GPIO_PIN_8
#define IO_SPI2_CS_MAG_GPIO_Port GPIOB
#define IO_SPI2_CS_IMU_Pin GPIO_PIN_9
#define IO_SPI2_CS_IMU_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define STATUS_LED_ON		HAL_GPIO_WritePin(IO_LED_STATUS_GPIO_Port, IO_LED_STATUS_Pin, GPIO_PIN_SET)
#define STATUS_LED_OFF		HAL_GPIO_WritePin(IO_LED_STATUS_GPIO_Port, IO_LED_STATUS_Pin, GPIO_PIN_RESET)
#define STATUS_LED_TOGGLE	HAL_GPIO_TogglePin(IO_LED_STATUS_GPIO_Port, IO_LED_STATUS_Pin)

#define SPI_BARO_SELECT       HAL_GPIO_WritePin(IO_SPI2_CS_BARO_GPIO_Port, IO_SPI2_CS_BARO_Pin, GPIO_PIN_RESET)
#define SPI_BARO_DESELECT      HAL_GPIO_WritePin(IO_SPI2_CS_BARO_GPIO_Port, IO_SPI2_CS_BARO_Pin, GPIO_PIN_SET)

#define SPI_IMU_SELECT       HAL_GPIO_WritePin(IO_SPI2_CS_IMU_GPIO_Port, IO_SPI2_CS_IMU_Pin, GPIO_PIN_RESET)
#define SPI_IMU_DESELECT      HAL_GPIO_WritePin(IO_SPI2_CS_IMU_GPIO_Port, IO_SPI2_CS_IMU_Pin, GPIO_PIN_SET)

#define SPI_MAG_SELECT       HAL_GPIO_WritePin(IO_SPI2_CS_MAG_GPIO_Port, IO_SPI2_CS_MAG_Pin, GPIO_PIN_RESET)
#define SPI_MAG_DESELECT      HAL_GPIO_WritePin(IO_SPI2_CS_MAG_GPIO_Port, IO_SPI2_CS_MAG_Pin, GPIO_PIN_SET)

#define MOTOR1_TELEM_UART		USART5
#define MOTOR1_TELEM_DMA		DMA2
#define MOTOR1_TELEM_DMA_CH		LL_DMA_CHANNEL_2
#define MOTOR2_TELEM_UART		USART3
#define MOTOR2_TELEM_DMA		DMA1
#define MOTOR2_TELEM_DMA_CH		LL_DMA_CHANNEL_5
#define MOTOR3_TELEM_UART		USART6
#define MOTOR3_TELEM_DMA		DMA2
#define MOTOR3_TELEM_DMA_CH		LL_DMA_CHANNEL_3
#define MOTOR4_TELEM_UART		USART4
#define MOTOR4_TELEM_DMA		DMA1
#define MOTOR4_TELEM_DMA_CH		LL_DMA_CHANNEL_6

#define BMS_TX_EN				HAL_GPIO_WritePin(IO_USART1_EN_GPIO_Port, IO_USART1_EN_Pin, GPIO_PIN_SET)
#define BMS_RX_EN				HAL_GPIO_WritePin(IO_USART1_EN_GPIO_Port, IO_USART1_EN_Pin, GPIO_PIN_RESET)

#define BMS_UART				USART1
#define BMS_TX_DMA				DMA1
#define BMS_TX_DMA_CH			LL_DMA_CHANNEL_2
#define BMS_RX_DMA				DMA1
#define BMS_RX_DMA_CH			LL_DMA_CHANNEL_1

#define CONTROLLER_TX_EN		HAL_GPIO_WritePin(IO_USART2_EN_GPIO_Port, IO_USART2_EN_Pin, GPIO_PIN_SET)
#define CONTROLLER_RX_EN		HAL_GPIO_WritePin(IO_USART2_EN_GPIO_Port, IO_USART2_EN_Pin, GPIO_PIN_RESET)

#define CONTROLLER_UART			USART2
#define CONTROLLER_TX_DMA		DMA2
#define CONTROLLER_TX_DMA_CH	LL_DMA_CHANNEL_1
#define CONTROLLER_RX_DMA		DMA1
#define CONTROLLER_RX_DMA_CH	LL_DMA_CHANNEL_3

static uint16_t map(uint16_t input, uint16_t minInput, uint16_t maxInput, uint16_t minOutput, uint16_t maxOutput)
{
	return (minOutput + ((maxOutput - minOutput) * (input - minInput))/ (maxInput - minInput) );
};

#define WDT_RESET	HAL_IWDG_Refresh(&hiwdg);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
