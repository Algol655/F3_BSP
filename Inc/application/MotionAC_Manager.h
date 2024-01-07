/**
 ******************************************************************************
 * @file    motion_ac_manager.h
 * @author  MEMS Software Solutions Team
 * @brief   This file contains definitions for the motion_ac_manager.c file
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
#ifndef MOTION_AC_MANAGER_H
#define MOTION_AC_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "string.h"
#include "application/MEMS_app.h"		//Added By Me!!!
#include "application/motion_ac.h"
#include "main.h"

#define MOVE_THR_G  0.2f	//Recommended between 0.15 - 0.30 g, higher value will relax condition on data selection
							//for calibration but reduce the accuracy which will be around (moveThresh_g / 10)
							//Added By Me!!!
/* Extern variables ----------------------------------------------------------*/
#if (IMU_PRESENT==1)
	MAC_knobs_t Knobs;						//Added By Me!!!
	#pragma GCC diagnostic push
	#pragma GCC diagnostic ignored "-Wunused-variable"
	static MAC_knobs_t *pKnobs = &Knobs;	//Added By Me!!!
	#pragma GCC diagnostic pop
#endif
/* Exported Macros -----------------------------------------------------------*/
/* Exported Types ------------------------------------------------------------*/
/*typedef enum							//Moved in MEMS_app.h. Modified By Me!!!
{
  DYNAMIC_CALIBRATION = 0,
  SIX_POINT_CALIBRATION = 1
} MAC_calibration_mode_t;

typedef enum
{
  MAC_DISABLE_LIB = 0,
  MAC_ENABLE_LIB = 1
} MAC_enable_lib_t; */

/* Imported Variables --------------------------------------------------------*/
/* Exported Functions Prototypes ---------------------------------------------*/
void MotionAC_manager_init(MAC_enable_lib_t enable);	//Defined in MEMS_app.h Added By Me!!!
void MotionAC_manager_update(MAC_input_t *data_in, uint8_t *is_calibrated);
void MotionAC_manager_get_params(MAC_output_t *data_out);
void MotionAC_manager_get_version(char *version, int *length);
#if (IMU_PRESENT==1)									//Added By Me!!!
	void MotionAC_manager_compensate(SensorAxes_t *DataIn, SensorAxes_t *DataOut);	//Defined in LSM9DS1_Driver.h Added By Me!!!
#endif

int16_t acc_bias_to_mg(float acc_bias);
static const uint32_t ReportInterval = 1000/AC_ALGO_FREQ;	//Algorithm report interval [ms] Note: Must be between 10 and 50 ms
															//Added By Me!!!
#ifdef __cplusplus
}
#endif

#endif /* MOTION_AC_MANAGER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
