/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "compiler/compiler.h"
/* USER CODE END Includes */

extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */
typedef enum
{
  USARTD_OK   = 0,
  USARTD_BUSY,
  USARTD_FAIL
} USARTD_StatusTypeDef;

/* Buffer used for reception */
#define RXBUFFERSIZE 64U
#define USART_DATA_MAX_PACKET_SIZE	64U  /* Endpoint IN & OUT Packet size */
uint8_t aRxBuffer[RXBUFFERSIZE];

uint16_t usart2_local_buff_offset, usart3_local_buff_offset;

/* USER CODE END Private defines */

void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void USART_Config(UART_HandleTypeDef* uartHandle);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

