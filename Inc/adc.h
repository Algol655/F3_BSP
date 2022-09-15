/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN Private defines */
#define ADC_REFERENCE_VOLTAGE_MV				3300.0f
#define ADC_MAX_OUTPUT_VALUE					4095.0f
//Temperature sensor characteristics valid for STM32F105/107 family
#define TEMP_SENSOR_AVG_SLOPE_MV_PER_CELSIUS	4.3f
#define TEMP_SENSOR_VOLTAGE_MV_AT_25			1430.0f
#define AVG_SLOPE								(TEMP_SENSOR_AVG_SLOPE_MV_PER_CELSIUS/1000.0f)
#define V25										(TEMP_SENSOR_VOLTAGE_MV_AT_25/1000.0f)
//Temperature sensor calibration values for STM32F405/407/415/417 family
#define TEMP30_CAL_VALUE						((uint16_t*)((uint32_t)0x1FFF7A2C))
#define TEMP110_CAL_VALUE						((uint16_t*)((uint32_t)0x1FFF7A2E))
#define TEMP30									30.0f
#define TEMP110									110.0f

typedef enum ADCD_status_typedef
{
	ADCD_OK   = 0,
	ADCD_BUSY,
	ADCD_FAIL,
} ADCD_StatusTypeDef;

#define num_ad_chs  (3)			//Number of AD channels employed
#define num_anlg_mux_in (16)	//Number of analog signal to be converted
#define mux_channels_enabled (0b00000000000000000000000000111111)	//mux1 -> 0..15, mux2 -> 17..31
uint16_t ADCxConvertedValue[num_ad_chs];

/* USER CODE END Private defines */

void MX_ADC1_Init(void);

/* USER CODE BEGIN Prototypes */
void ADC_Config(ADC_HandleTypeDef* AdcHandle);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

