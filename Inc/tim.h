/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
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
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim1;

extern TIM_HandleTypeDef htim3;

extern TIM_HandleTypeDef htim6;

extern TIM_HandleTypeDef htim7;

/* USER CODE BEGIN Private defines */
#define TIM_CLOCK  	2000U										//TimerClockSource/Prescaler = 84MHz/42000
//#define ALGO_FREQ   TIM_CLOCK/(htim1.Init.Period+1U) 			//IMU Algorithm frequency [Hz].
//#define ALGO_PERIOD 1000U/(TIM_CLOCK/(htim1.Init.Period+1U))	//IMU Algorithm period [ms].
//#define MOTIONFX_ENGINE_DELTATIME  (1.0)/(TIM_CLOCK/(htim1.Init.Period+1U))	//IMU Algorithm period [s].
#define FX_ALGO_FREQ 				100U						//MotionFX Algorithm frequency [Hz].
#define FX_ALGO_PERIOD				10							//MotionFX Algorithm period [ms].
#define MOTIONFX_ENGINE_DELTATIME	0.01f						//MotionFX Algorithm period [s].
#define MAG_CAL_ALGO_PERIOD 		40							//MotionFX Mag.Cal. Algorithm period [ms].
#define GC_ALGO_FREQ 				50U							//MotionGC Algorithm frequency [Hz].
#define AC_ALGO_FREQ 				50U							//MotionAC Algorithm frequency [Hz].
#define AR_ALGO_FREQ 				16U							//MotionAR Algorithm frequency [Hz].
#define AR_ALGO_PERIOD				(1000U/AR_ALGO_FREQ)		//MotionAR Algorithm period [ms].

#define DEFAULT_uhCCR1_Val  100 /* 10kHz/100 value */
#define DEFAULT_uhCCR2_Val  200 /* 10kHz/50 value */
#define DEFAULT_uhCCR3_Val  625 /* 10kHz/16 value */
#define DEFAULT_uhCCR4_Val  400 /* 10kHz/25 value */
//Value in seconds: ServiceTimerX_Timeout * 0.1
#define ServiceTimer1_Timeout 	3		//Used for push button debounce timer (300mS)
#define ServiceTimer2_Timeout	300		//Used for HTS221 heather timeout (30s)
#define ServiceTimer3_Timeout	36000	//Used for the HTS221 waiting time between two heater starts (1h)
#define ServiceTimer4_Timeout	6000	//used for the HTS221 waiting time to measures restart after the heater activation (10min)

typedef enum	service_timers
{
	STim_1,
	STim_2,
	STim_3,
	STim_4
} Service_Timer;

typedef struct
{
  uint16_t 			TimeOut;
  uint16_t 			Counter;
  bool 				Start;
  bool				Expired;
} ServiceTimer_st;

ServiceTimer_st ServiceTimer1, ServiceTimer2, ServiceTimer3, ServiceTimer4;
/* USER CODE END Private defines */

void MX_TIM1_Init(void);
void MX_TIM3_Init(void);
void MX_TIM6_Init(void);
void MX_TIM7_Init(void);

/* USER CODE BEGIN Prototypes */
void ServiceTimersInit(void);
void ServiceTimerStart(Service_Timer nTim);
void TIM_OC_Timers_Start(TIM_HandleTypeDef *htim);
void TIM_OC_Timers_Stop(TIM_HandleTypeDef *htim);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

