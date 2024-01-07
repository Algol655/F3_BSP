/**
  ******************************************************************************
  * @file    gatt_db.h
  * @author  SRA Application Team
  * @brief   Header file for gatt_db.c
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

#ifndef GATT_DB_H
#define GATT_DB_H

/* Includes ------------------------------------------------------------------*/
#include "hci.h"

/* Exported defines ----------------------------------------------------------*/
#define X_OFFSET 200
#define Y_OFFSET 50
#define Z_OFFSET 1000
/* Feature mask for Sensor fusion short precision */
#define FEATURE_MASK_SENSORFUSION_SHORT 0x00000100
/* Feature mask for e-compass */
#define FEATURE_MASK_ECOMPASS 0x00000040
/* Feature mask for hardware events */
#define FEATURE_MASK_ACC_EVENTS 0x00000400
/* Feature mask for Temperature1 */
#define FEATURE_MASK_TEMP1 0x00040000
/* Feature mask for Temperature2 */
#define FEATURE_MASK_TEMP2 0x00010000
/* Feature mask for Pressure */
#define FEATURE_MASK_PRESS 0x00100000
/* Feature mask for Humidity */
#define FEATURE_MASK_HUM   0x00080000
/* Feature mask for Accelerometer */
#define FEATURE_MASK_ACC   0x00800000
/* Feature mask for Gyroscope */
#define FEATURE_MASK_GRYO  0x00400000
/* Feature mask for Magnetometer */
#define FEATURE_MASK_MAG   0x00200000
/* Feature mask for Microphone */
#define FEATURE_MASK_MIC   0x04000000
/* Feature mask for BlueVoice */
#define FEATURE_MASK_BLUEVOICE   0x08000000
/* Feature mask for SourceLocalization */
#define FEATURE_MASK_DIR_OF_ARRIVAL 0x10000000
/* W2ST command for asking the calibration status */
#define W2ST_COMMAND_CAL_STATUS 0xFF
/* W2ST command for resetting the calibration */
#define W2ST_COMMAND_CAL_RESET  0x00
/* W2ST command for stopping the calibration process */
#define W2ST_COMMAND_CAL_STOP   0x01
/* W2ST command - SL sensitivity */
#define W2ST_COMMAND_SL_SENSITIVITY 0xCC
/* W2ST command - SL sensitivity Low */
#define W2ST_COMMAND_SL_LOW  0x00
/* W2ST command - SL sensitivity High */
#define W2ST_COMMAND_SL_HIGH  0x01

/**
 * @brief Number of application services
 */
#define NUMBER_OF_APPLICATION_SERVICES (2)

/**
 * @brief Define How Many quaterions you want to trasmit (from 1 to 3)
 *        In this sample application use only 1
 */
#define SEND_N_QUATERNIONS 1

/* Exported typedef ----------------------------------------------------------*/
/**
 * @brief Structure containing acceleration value of each axis.
 */
typedef struct {
  int32_t AXIS_X;
  int32_t AXIS_Y;
  int32_t AXIS_Z;
} AxesRaw_t;

enum {
  ACCELERATION_SERVICE_INDEX = 0,
  ENVIRONMENTAL_SERVICE_INDEX = 1
};

typedef enum
{
  NOACTIVITY  = 0x00,
  STATIONARY  = 0x01,
  WALKING     = 0x02,
  FASTWALKING = 0x03,
  JOGGING     = 0x04,
  BIKING      = 0x05,
  DRIVING     = 0x06
} MAR_Output_t;

/* Exported function prototypes ----------------------------------------------*/
tBleStatus Add_HWServW2ST_Service(void);
tBleStatus Add_SWServW2ST_Service(void);
tBleStatus Add_ConfigW2ST_Service(void);
tBleStatus Add_Time_Service(void);
#ifdef STM32L476xx
tBleStatus Add_ConsoleW2ST_Service(void);
#endif /* STM32L476xx */
void Read_Request_CB(uint16_t handle);
void Attribute_Modified_Request_CB(uint16_t Connection_Handle, uint16_t attr_handle,
                                   uint16_t Offset, uint8_t data_length, uint8_t *att_data);
tBleStatus Environmental_Update(int32_t press, int16_t temp, int16_t temp2, int16_t hum, int32_t co);
tBleStatus Acc_Update(AxesRaw_t *x_axes, AxesRaw_t *g_axes, AxesRaw_t *m_axes);
tBleStatus Quat_Update(AxesRaw_t *q_axes);
tBleStatus ECompass_Update(uint16_t angle);
tBleStatus ActivityRec_Update(MAR_Output_t ActivityCode);
tBleStatus Seconds_Update(void);
tBleStatus Minutes_Update(void);
void Time_Update(void);
#ifdef STM32L476xx
tBleStatus Term_Update(uint8_t *data, uint8_t length);
tBleStatus Term_Update_AfterRead(void);
#endif /* STM32L476xx */

#endif /* GATT_DB_H */
