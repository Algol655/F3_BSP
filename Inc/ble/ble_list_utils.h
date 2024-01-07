/**
  ******************************************************************************
  * @file    ble_list_utils.h
  * @author  SRA Application Team
  * @brief   Header file
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
#ifndef BLE_LIST_UTILS_H
#define BLE_LIST_UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#if defined(STM32F405xx)
	#include "stm32f4xx_hal.h"
#elif defined(STM32F105xC)
	#include "stm32f1xx_hal.h"
#endif

#ifdef __cplusplus
}
#endif
#endif /* BLE_LIST_UTILS_H */
