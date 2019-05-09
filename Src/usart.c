/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
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

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "dma.h"
#include "controllerInterface.h"
#include "bmsInterface.h"
#include "tim.h"

UART_HANDLE huartM1;
UART_HANDLE huartM2;
UART_HANDLE huartM3;
UART_HANDLE huartM4;
UART_HANDLE huartController;
UART_HANDLE huartBms;

uint8_t ctrlRxBuf[100];
uint8_t bmsRxBuf[200];


void uartHandlesInit()
{
	huartM1.USARTx 			= MOTOR1_TELEM_UART;
	huartM1.DMAxTx			= NULL;
	huartM1.DMA_ChannelTx	= 0;
	huartM1.buffTx 			= NULL;
	huartM1.sizeTx 			= 0;
	huartM1.DMAxRx			= MOTOR1_TELEM_DMA;
	huartM1.DMA_ChannelRx	= MOTOR1_TELEM_DMA_CH;
	huartM1.buffRx 			= escPacket[0].raw;
	huartM1.sizeRx 			= sizeof(escPacket[0].raw);
	huartM1.cbRx 			= NULL;

	huartM2.USARTx 			= MOTOR2_TELEM_UART;
	huartM2.DMAxTx			= NULL;
	huartM2.DMA_ChannelTx	= 0;
	huartM2.buffTx 			= NULL;
	huartM2.sizeTx 			= 0;
	huartM2.DMAxRx			= MOTOR2_TELEM_DMA;
	huartM2.DMA_ChannelRx	= MOTOR2_TELEM_DMA_CH;
	huartM2.buffRx 			= escPacket[1].raw;
	huartM2.sizeRx 			= sizeof(escPacket[1].raw);
	huartM2.cbRx 			= NULL;

	huartM3.USARTx 			= MOTOR3_TELEM_UART;
	huartM3.DMAxTx			= NULL;
	huartM3.DMA_ChannelTx	= 0;
	huartM3.buffTx 			= NULL;
	huartM3.sizeTx 			= 0;
	huartM3.DMAxRx			= MOTOR3_TELEM_DMA;
	huartM3.DMA_ChannelRx	= MOTOR3_TELEM_DMA_CH;
	huartM3.buffRx 			= escPacket[2].raw;
	huartM3.sizeRx 			= sizeof(escPacket[2].raw);
	huartM3.cbRx 			= NULL;

	huartM4.USARTx 			= MOTOR4_TELEM_UART;
	huartM4.DMAxTx			= NULL;
	huartM4.DMA_ChannelTx	= 0;
	huartM4.buffTx 			= NULL;
	huartM4.sizeTx 			= 0;
	huartM4.DMAxRx			= MOTOR4_TELEM_DMA;
	huartM4.DMA_ChannelRx	= MOTOR4_TELEM_DMA_CH;
	huartM4.buffRx 			= escPacket[3].raw;
	huartM4.sizeRx 			= sizeof(escPacket[3].raw);
	huartM4.cbRx 			= NULL;

	huartController.USARTx 			= CONTROLLER_UART;
	huartController.DMAxTx			= CONTROLLER_TX_DMA;
	huartController.DMA_ChannelTx	= CONTROLLER_TX_DMA_CH;
	huartController.buffTx 			= NULL;
	huartController.sizeTx 			= 0;
	huartController.DMAxRx			= CONTROLLER_RX_DMA;
	huartController.DMA_ChannelRx	= CONTROLLER_RX_DMA_CH;
	huartController.buffRx 			= ctrlRxBuf;
	huartController.sizeRx 			= sizeof(ctrlRxBuf);
	huartController.cbRx 			= receiveControlData;

	huartBms.USARTx 			= BMS_UART;
	huartBms.DMAxTx			= BMS_TX_DMA;
	huartBms.DMA_ChannelTx	= BMS_TX_DMA_CH;
	huartBms.buffTx 			= NULL;
	huartBms.sizeTx 			= 0;
	huartBms.DMAxRx			= BMS_RX_DMA;
	huartBms.DMA_ChannelRx	= BMS_RX_DMA_CH;
	huartBms.buffRx 			= bmsRxBuf;
	huartBms.sizeRx 			= sizeof(bmsRxBuf);
	huartBms.cbRx 			= receiveBmsData;
};

/* USER CODE END 0 */

/* USART1 init function */

void MX_USART1_UART_Init(void)
{
  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration  
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 DMA Init */
  
  /* USART1_RX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMA_REQUEST_8);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_BYTE);

  /* USART1_TX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_2, LL_DMA_REQUEST_8);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, 0);
  NVIC_EnableIRQ(USART1_IRQn);

  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_DisableIT_CTS(USART1);
  LL_USART_DisableOverrunDetect(USART1);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{
  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration  
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART2 DMA Init */
  
  /* USART2_RX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_3, LL_DMA_REQUEST_9);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MDATAALIGN_BYTE);

  /* USART2_TX Init */
  LL_DMA_SetPeriphRequest(DMA2, LL_DMA_CHANNEL_1, LL_DMA_REQUEST_9);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA2, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA2, LL_DMA_CHANNEL_1, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_BYTE);

  /* USART2 interrupt Init */
  NVIC_SetPriority(USART2_IRQn, 0);
  NVIC_EnableIRQ(USART2_IRQn);

  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_DisableIT_CTS(USART2);
  LL_USART_DisableOverrunDetect(USART2);
  LL_USART_ConfigAsyncMode(USART2);
  LL_USART_Enable(USART2);

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{
  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**USART3 GPIO Configuration  
  PB10   ------> USART3_TX 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USART3 DMA Init */
  
  /* USART3_RX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_5, LL_DMA_REQUEST_10);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_5, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MDATAALIGN_BYTE);

  /* USART3 interrupt Init */
  NVIC_SetPriority(USART3_8_IRQn, 0);
  NVIC_EnableIRQ(USART3_8_IRQn);

  USART_InitStruct.BaudRate = 19200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_RX;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART3, &USART_InitStruct);
  LL_USART_ConfigHalfDuplexMode(USART3);
  LL_USART_Enable(USART3);

}
/* USART4 init function */

void MX_USART4_UART_Init(void)
{
  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART4);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART4 GPIO Configuration  
  PA0   ------> USART4_TX 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART4 DMA Init */
  
  /* USART4_RX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_6, LL_DMA_REQUEST_11);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_6, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_6, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_6, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_6, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MDATAALIGN_BYTE);

  /* USART4 interrupt Init */
  NVIC_SetPriority(USART3_8_IRQn, 0);
  NVIC_EnableIRQ(USART3_8_IRQn);

  USART_InitStruct.BaudRate = 19200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART4, &USART_InitStruct);
  LL_USART_ConfigHalfDuplexMode(USART4);
  LL_USART_Enable(USART4);

}
/* USART5 init function */

void MX_USART5_UART_Init(void)
{
  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART5);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**USART5 GPIO Configuration  
  PB3   ------> USART5_TX 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USART5 DMA Init */
  
  /* USART5_RX Init */
  LL_DMA_SetPeriphRequest(DMA2, LL_DMA_CHANNEL_2, LL_DMA_REQUEST_12);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA2, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA2, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);

  /* USART5 interrupt Init */
  NVIC_SetPriority(USART3_8_IRQn, 0);
  NVIC_EnableIRQ(USART3_8_IRQn);

  USART_InitStruct.BaudRate = 19200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_RX;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART5, &USART_InitStruct);
  LL_USART_ConfigHalfDuplexMode(USART5);
  LL_USART_Enable(USART5);

}
/* USART6 init function */

void MX_USART6_UART_Init(void)
{
  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART6);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART6 GPIO Configuration  
  PA4   ------> USART6_TX 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = GPIO_AF5_USART6;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART6 DMA Init */
  
  /* USART6_RX Init */
  LL_DMA_SetPeriphRequest(DMA2, LL_DMA_CHANNEL_3, LL_DMA_REQUEST_13);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_CHANNEL_3, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA2, LL_DMA_CHANNEL_3, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA2, LL_DMA_CHANNEL_3, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_CHANNEL_3, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_CHANNEL_3, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_CHANNEL_3, LL_DMA_MDATAALIGN_BYTE);

  /* USART6 interrupt Init */
  NVIC_SetPriority(USART3_8_IRQn, 0);
  NVIC_EnableIRQ(USART3_8_IRQn);

  USART_InitStruct.BaudRate = 19200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_RX;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART6, &USART_InitStruct);
  LL_USART_ConfigHalfDuplexMode(USART6);
  LL_USART_Enable(USART6);

}

/* USER CODE BEGIN 1 */
void UART_ESC_BeginReceive(UART_HANDLE *huart)
{
	//Enable idle interrupt, set addresses, and start DMA reception
	LL_USART_EnableIT_IDLE(huart->USARTx);
	LL_USART_EnableDMAReq_RX(huart->USARTx);
	LL_DMA_ConfigAddresses(huart->DMAxRx, huart->DMA_ChannelRx, (uint32_t)&huart->USARTx->RDR, (uint32_t)huart->buffRx, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetDataLength(huart->DMAxRx, huart->DMA_ChannelRx, huart->sizeRx);
	LL_DMA_EnableChannel(huart->DMAxRx, huart->DMA_ChannelRx);
}

void UART_ESC_ITCallback(UART_HANDLE *huart)
{
	//Dont process data if Overrun error
	if(LL_USART_IsActiveFlag_ORE(huart->USARTx))
	{
		LL_USART_ReceiveData8(huart->USARTx);
		LL_USART_ClearFlag_ORE(huart->USARTx);
		LL_USART_ClearFlag_IDLE(huart->USARTx);
	}
	else if(LL_USART_IsActiveFlag_IDLE(huart->USARTx))
	{
		uint32_t escIndex = 0;
		LL_DMA_DisableChannel(huart->DMAxRx, huart->DMA_ChannelRx);

		if(huart->USARTx == MOTOR1_TELEM_UART)
			escIndex = 0;
		else if(huart->USARTx == MOTOR2_TELEM_UART)
			escIndex = 1;
		else if(huart->USARTx == MOTOR3_TELEM_UART)
			escIndex = 2;
		else if (huart->USARTx == MOTOR4_TELEM_UART)
			escIndex = 3;

		escParseData(huart->buffRx, huart->sizeRx, escIndex);
		memset(huart->buffRx, 0, huart->sizeRx);
		LL_DMA_ConfigAddresses(huart->DMAxRx, huart->DMA_ChannelRx, (uint32_t)&huart->USARTx->RDR, (uint32_t)huart->buffRx, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
		LL_DMA_SetDataLength(huart->DMAxRx, huart->DMA_ChannelRx, huart->sizeRx);
		LL_DMA_EnableChannel(huart->DMAxRx, huart->DMA_ChannelRx);
		LL_USART_ClearFlag_IDLE(huart->USARTx);
	}

}

void UART_RS485_BeginTransmit(UART_HANDLE *huart, uint8_t *buff, uint32_t size)
{
	//Enable idle interrupt, set addresses, and start DMA reception
	//LL_USART_EnableIT_IDLE(USARTx);
	LL_USART_EnableIT_TC(huart->USARTx);
	LL_USART_EnableDMAReq_TX(huart->USARTx);
	LL_DMA_ConfigAddresses(huart->DMAxTx, huart->DMA_ChannelTx, (uint32_t)buff, (uint32_t)&huart->USARTx->TDR, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetDataLength(huart->DMAxTx, huart->DMA_ChannelTx, size);
	LL_DMA_EnableIT_TC(huart->DMAxTx, huart->DMA_ChannelTx);
	LL_DMA_EnableChannel(huart->DMAxTx, huart->DMA_ChannelTx);
}

void UART_RS485_BeginReceive(UART_HANDLE *huart)
{
	//Enable idle interrupt, set addresses, and start DMA reception
	LL_USART_EnableIT_IDLE(huart->USARTx);
	LL_USART_EnableDMAReq_RX(huart->USARTx);
	LL_DMA_ConfigAddresses(huart->DMAxRx, huart->DMA_ChannelRx, (uint32_t)&huart->USARTx->RDR, (uint32_t)huart->buffRx, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetDataLength(huart->DMAxRx, huart->DMA_ChannelRx, huart->sizeRx);
	LL_DMA_EnableChannel(huart->DMAxRx, huart->DMA_ChannelRx);
}

void sendToController(uint8_t *buf, uint32_t size)
{
	CONTROLLER_TX_EN;
	UART_RS485_BeginTransmit(&huartController, buf, size);
}

void sendToBms(uint8_t *buf, uint32_t size)
{
	BMS_TX_EN;
	UART_RS485_BeginTransmit(&huartBms, buf, size);
}

void UART_RS485_ITCallback(UART_HANDLE *huart)
{
	//Transmit
	if(LL_DMA_IsActiveFlag_TCx(huart->DMAxTx, huart->DMA_ChannelTx))
	{
		LL_DMA_ClearFlag_TCx(huart->DMAxTx, huart->DMA_ChannelTx);
		LL_DMA_DisableIT_TC(huart->DMAxTx, huart->DMA_ChannelTx);
		LL_DMA_DisableChannel(huart->DMAxTx, huart->DMA_ChannelTx);

	}

	//Using this to determine when the RS485 pin needs to be toggled back to RX mode
	if(LL_USART_IsActiveFlag_TC(huart->USARTx))
	{
		LL_USART_ClearFlag_TC(huart->USARTx);
		if(huart->USARTx == CONTROLLER_UART)
			CONTROLLER_RX_EN;
		if(huart->USARTx == BMS_UART)
			BMS_RX_EN;
	}

	//Dont process data if Overrun error
	if(LL_USART_IsActiveFlag_ORE(huart->USARTx))
 	{
		LL_USART_ReceiveData8(huart->USARTx);
		LL_USART_ClearFlag_ORE(huart->USARTx);
		LL_USART_ClearFlag_IDLE(huart->USARTx);
	}
	else
	{

		//Receive
		if(LL_USART_IsActiveFlag_IDLE(huart->USARTx))
		{
			LL_DMA_DisableChannel(huart->DMAxRx, huart->DMA_ChannelRx);

			/*
			if(huart->USARTx == CONTROLLER_UART)
			{
				uartTimeout = 5;
				receiveControlData(ctrlRxBuf, sizeof(ctrlRxBuf)-LL_DMA_GetDataLength(huart->DMAxRx, huart->DMA_ChannelRx));
				memset(ctrlRxBuf, 0, sizeof(ctrlRxBuf));
				LL_DMA_SetDataLength(huart->DMAxRx, huart->DMA_ChannelRx, sizeof(ctrlRxBuf));
				LL_DMA_ConfigAddresses(huart->DMAxRx, huart->DMA_ChannelRx, (uint32_t)&huart->USARTx->RDR, (uint32_t)ctrlRxBuf, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
			}

			if(huart->USARTx == BMS_UART)
			{
				uartTimeout = 5;
				receiveBmsData(bmsRxBuf, sizeof(bmsRxBuf)-LL_DMA_GetDataLength(huart->DMAxRx, huart->DMA_ChannelRx));
				memset(bmsRxBuf, 0, sizeof(bmsRxBuf));
				LL_DMA_SetDataLength(huart->DMAxRx, huart->DMA_ChannelRx, sizeof(bmsRxBuf));
				LL_DMA_ConfigAddresses(huart->DMAxRx, huart->DMA_ChannelRx, (uint32_t)&huart->USARTx->RDR, (uint32_t)bmsRxBuf, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
			}
*/			uartTimeout = 5;
			huart->cbRx(huart->buffRx, huart->sizeRx-LL_DMA_GetDataLength(huart->DMAxRx, huart->DMA_ChannelRx));
			memset(bmsRxBuf, 0, sizeof(bmsRxBuf));
			LL_DMA_SetDataLength(huart->DMAxRx, huart->DMA_ChannelRx, huart->sizeRx);
			LL_DMA_ConfigAddresses(huart->DMAxRx, huart->DMA_ChannelRx, (uint32_t)&huart->USARTx->RDR, (uint32_t)huart->buffRx, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);


			LL_DMA_EnableChannel(huart->DMAxRx, huart->DMA_ChannelRx);
			LL_USART_ClearFlag_IDLE(huart->USARTx);
		}
	}

}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
