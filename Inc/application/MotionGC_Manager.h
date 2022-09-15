/**
 ******************************************************************************
 * @file    motion_gc_manager.h
 * @author  MEMS Software Solutions Team
 * @brief   This file contains definitions for the motion_gc_manager.c file
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Software License Agreement SLA0077,
 * the "License". You may not use this component except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        www.st.com/sla0077
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MOTION_GC_MANAGER_H
#define MOTION_GC_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "application/MEMS_app.h"
#include "string.h"
#include "application/motion_gc.h"
#include "main.h"

/* Extern variables ----------------------------------------------------------*/
#if (IMU_PRESENT==1)
	MGC_knobs_t GC_Knobs;						//Added By Me!!!
	#pragma GCC diagnostic push
	#pragma GCC diagnostic ignored "-Wunused-variable"
	static MGC_knobs_t *pGC_Knobs = &GC_Knobs;	//Added By Me!!!
	#pragma GCC diagnostic pop
#endif
/* Exported Macros -----------------------------------------------------------*/
/* Exported Types ------------------------------------------------------------*/
/* Imported Variables --------------------------------------------------------*/
/* Exported Functions Prototypes ---------------------------------------------*/
void MotionGC_manager_init(float freq);
void MotionGC_manager_update(MGC_input_t *data_in, MGC_output_t *gyro_bias, int *bias_update);
void MotionGC_manager_get_knobs(MGC_knobs_t *knobs);
void MotionGC_manager_set_knobs(MGC_knobs_t *knobs);
void MotionGC_manager_get_params(MGC_output_t *gyro_bias);
void MotionGC_manager_set_params(MGC_output_t *gyro_bias);
void MotionGC_manager_set_frequency(float freq);
void MotionGC_manager_get_version(char *version, int *length);
#if (IMU_PRESENT==1)						//Defined in main.h
	void MotionGC_manager_compensate(SensorAxes_t *DataIn, SensorAxes_t *DataOut);
#endif
int16_t gyro_bias_to_mdps(float gyro_bias);

#ifdef __cplusplus
}
#endif

#endif /* MOTION_GC_MANAGER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
