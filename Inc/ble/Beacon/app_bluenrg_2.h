/**
  ******************************************************************************
  * @file    app_bluenrg_2.h
  * @author  SRA Application Team
  * @brief   Header file for app_bluenrg_2.c
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APP_BLUENRG_2_H
#define APP_BLUENRG_2_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
//#include "stm32f4xx_nucleo.h"

/* USER CODE BEGIN Includes */
#include "application/MEMS_app.h"
/* USER CODE END Includes */

/* Exported Defines ----------------------------------------------------------*/

/* USER CODE BEGIN ED */
#define DEVICE_NAME   'S','e','n','s','u','s',' ','1','9','1'
//#define DEVICE_NAME   'S','N','S','0','1'
#define MY_BEACON	(1)
/* USER CODE END ED */

/* Exported Variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported Functions Prototypes ---------------------------------------------*/
void MX_BlueNRG_2_Init(void);
void MX_BlueNRG_2_Process(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif
#endif /* APP_BLUENRG_2_H */
