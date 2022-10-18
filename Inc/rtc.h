/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    rtc.h
  * @brief   This file contains all the function prototypes for
  *          the rtc.c file
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
#ifndef __RTC_H__
#define __RTC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "platform/port.h"
#include "stm32_hal_legacy.h"
/* USER CODE END Includes */

extern RTC_HandleTypeDef hrtc;

/* USER CODE BEGIN Private defines */
#define RTC_ASYNCH_PREDIV_LSI  0x7F
#define RTC_SYNCH_PREDIV_LSI   0xF9
#define RTC_ASYNCH_PREDIV_LSE  0x7F
#define RTC_SYNCH_PREDIV_LSE   0x00FF
#define RTC_SET_VALUES	(0)
//static int RtcSynchPrediv;
int RtcSynchPrediv;
typedef struct
{
  uint8_t date[3];
  uint8_t time[3];
} DateTime_t;
/* USER CODE END Private defines */

void MX_RTC_Init(void);

/* USER CODE BEGIN Prototypes */
void RTC_DateTimeStamp(RTC_HandleTypeDef* rtcHandle, DateTime_t *Stamp);
void RTC_Handler(RTC_HandleTypeDef* rtcHandle, uint8_t* Buff);
void RTC_DateRegulate(RTC_HandleTypeDef* rtcHandle, uint8_t y, uint8_t m, uint8_t d, uint8_t dw);
void RTC_TimeRegulate(RTC_HandleTypeDef* rtcHandle, uint8_t hh, uint8_t mm, uint8_t ss, uint32_t Format);
void RTC_DateTimeRegulate(RTC_HandleTypeDef* rtcHandle, uint8_t y, uint8_t m, uint8_t d, uint8_t dw,
														uint8_t hh, uint8_t mm, uint8_t ss, uint32_t Format);
void enable_backup_rtc(void);
void disable_backup_rtc(void);
int8_t writeBkpRTC(uint8_t *data, uint16_t bytes, uint16_t offset);
uint8_t readBkpRTC(uint8_t *data, uint16_t bytes, uint16_t offset);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __RTC_H__ */

