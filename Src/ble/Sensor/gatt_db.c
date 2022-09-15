/**
  ******************************************************************************
  * @file    gatt_db.c
  * @author  SRA Application Team
  * @brief   Functions to build GATT DB and handle GATT events.
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

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <Sensor/app_bluenrg_2.h>
#include <Sensor/gatt_db.h>
#include "bluenrg1_aci.h"
#include "bluenrg1_hci_le.h"
#include "bluenrg1_gatt_aci.h"
#ifdef STM32L476xx
	#include <Sensor/OTA.h>
#endif /* STM32L476xx */

/* Private macros ------------------------------------------------------------*/
/** @brief Macro that stores Value into a buffer in Little Endian Format (2 bytes)*/
#define HOST_TO_LE_16(buf, val)    ( ((buf)[0] =  (uint8_t) (val)    ) , \
                                   ((buf)[1] =  (uint8_t) (val>>8) ) )

/** @brief Macro that stores Value into a buffer in Little Endian Format (4 bytes) */
#define HOST_TO_LE_32(buf, val)    ( ((buf)[0] =  (uint8_t) (val)     ) , \
                                   ((buf)[1] =  (uint8_t) (val>>8)  ) , \
                                   ((buf)[2] =  (uint8_t) (val>>16) ) , \
                                   ((buf)[3] =  (uint8_t) (val>>24) ) )

/** @brief Macro that stores Value into a buffer in Big Endian Format (4 bytes) */
#define HOST_TO_BE_32(buf, val)    ( ((buf)[3] =  (uint8_t) (val)     ) , \
                                   ((buf)[2] =  (uint8_t) (val>>8)  ) , \
                                   ((buf)[1] =  (uint8_t) (val>>16) ) , \
                                   ((buf)[0] =  (uint8_t) (val>>24) ) )

#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
        uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
            uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
                uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)

/************************************/
/* Hardware Characteristics Service */
/************************************/
#define COPY_HW_SENS_W2ST_SERVICE_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00, 0x00,0x01, 0x11,0xe1, 0x9a,0xb4, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_ENVIRONMENTAL_W2ST_CHAR_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00, 0x00,0x01, 0x11,0xe1, 0xac,0x36, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_ACC_GYRO_MAG_W2ST_CHAR_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x00,0xE0,0x00,0x00, 0x00,0x01, 0x11,0xe1, 0xac,0x36, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_ACC_EVENT_W2ST_CHAR_UUID(uuid_struct)     COPY_UUID_128(uuid_struct,0x00,0x00,0x04,0x00, 0x00,0x01, 0x11,0xe1, 0xac,0x36, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
/***********************************
* Software Characteristics Service *
*        MotionXX Section          *
************************************/
#define COPY_SW_SENS_W2ST_SERVICE_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00, 0x00,0x02, 0x11,0xe1, 0x9a,0xb4, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
/* Code for MotionFX integration - Start Section */
#define COPY_QUATERNIONS_W2ST_CHAR_UUID(uuid_struct)   COPY_UUID_128(uuid_struct,0x00,0x00,0x01,0x00, 0x00,0x01, 0x11,0xe1, 0xac,0x36, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_ECOMPASS_W2ST_CHAR_UUID(uuid_struct)      COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x40, 0x00,0x01, 0x11,0xe1, 0xac,0x36, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
/* Code for MotionFX integration - End Section */
/* Code for MotionAR integration - Start Section */
#define COPY_ACTIVITY_REC_W2ST_CHAR_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x10, 0x00,0x01, 0x11,0xe1, 0xac,0x36, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
/* Code for MotionAR integration - End Section */
/************************
* Configuration Service *
*************************/
#define COPY_CONFIG_SERVICE_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00, 0x00,0x0F, 0x11,0xe1, 0x9a,0xb4, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_CONFIG_W2ST_CHAR_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x02, 0x00,0x0F, 0x11,0xe1, 0xac,0x36, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
/************************
*     Time Service      *
*************************/
#define COPY_TIME_SERVICE_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x08,0x36,0x6e,0x80, 0xcf,0x3a, 0x11,0xe1, 0x9a,0xb4, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_TIME_CHAR_UUID(uuid_struct)     COPY_UUID_128(uuid_struct,0x09,0x36,0x6e,0x80, 0xcf,0x3a, 0x11,0xe1, 0xac,0x36, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_MINUTE_CHAR_UUID(uuid_struct)   COPY_UUID_128(uuid_struct,0x0a,0x36,0x6e,0x80, 0xcf,0x3a, 0x11,0xe1, 0xac,0x36, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
/************************
*      LED Service      *
*************************/
#define COPY_LED_SERVICE_UUID(uuid_struct)   COPY_UUID_128(uuid_struct,0x0b,0x36,0x6e,0x80, 0xcf,0x3a, 0x11,0xe1, 0x9a,0xb4, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_LED1_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x20,0x00,0x00,0x00, 0x00,0x01, 0x11,0xe1, 0xac,0x36, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)

#ifdef STM32L476xx
/* Console Service */
	#define COPY_CONSOLE_SERVICE_UUID(uuid_struct)   COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x0E,0x11,0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
	#define COPY_TERM_CHAR_UUID(uuid_struct)         COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x01,0x00,0x0E,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
	#define COPY_STDERR_CHAR_UUID(uuid_struct)       COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x02,0x00,0x0E,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
	#define MCR_FAST_TERM_UPDATE_FOR_OTA(data) aci_gatt_update_char_value(ConsoleW2STHandle, TermCharHandle, 0, 1 , data)
#endif /* STM32L476xx */
#define W2ST_MAX_CHAR_LEN 20

/* Private variables ---------------------------------------------------------*/
uint16_t HWServW2STHandle, EnvironmentalCharHandle, AccGyroMagCharHandle, AccEventCharHandle;
uint16_t SWServW2STHandle, QuaternionsCharHandle, ECompassCharHandle, ActivityRecCharHandle;
uint16_t ConfigServW2STHandle, ConfigCharHandle;
uint16_t timeServHandle, secondsCharHandle, minutesCharHandle;

#ifdef STM32L476xx
	uint16_t ConsoleW2STHandle, TermCharHandle, StdErrCharHandle;
	uint32_t SizeOfUpdateBlueFW=0;
	uint8_t  BufferToWrite[256];
	int32_t  BytesToWrite;
	static uint8_t LastTermBuffer[W2ST_MAX_CHAR_LEN];
	static uint8_t LastTermLen;
#endif /* STM32L476xx */

/* UUIDS */
Service_UUID_t service_uuid;
Char_UUID_t char_uuid;

extern AxesRaw_t x_axes;
extern AxesRaw_t g_axes;
extern AxesRaw_t m_axes;
extern AxesRaw_t q_axes;
extern uint16_t Angle;
extern MAR_Output_t Activity;

__IO uint8_t send_env;
__IO uint8_t send_mot;
__IO uint8_t send_quat;
__IO uint8_t send_ecomp;
__IO uint8_t send_ar;
__IO uint8_t send_scds;
__IO uint8_t send_mnts;

extern __IO uint16_t connection_handle;
//extern uint32_t start_time;
#ifdef STM32L476xx
	static uint32_t DebugConsoleCommandParsing(uint8_t * att_data, uint8_t data_length);
	static uint8_t  getBlueNRG2_Version(uint8_t *hwVersion, uint16_t *fwVersion);
#endif /* STM32L476xx */

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Add the 'HW' service (and the Environmental and AccGyr characteristics).
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_HWServW2ST_Service(void)
{
  tBleStatus ret;
  uint8_t uuid[16];
  /* num of characteristics of this service */
  uint8_t char_number = 3;
  /* number of attribute records that can be added to this service */
  uint8_t max_attribute_records = 1+(3*char_number);

  /* add HW_SENS_W2ST service */
  COPY_HW_SENS_W2ST_SERVICE_UUID(uuid);
  BLUENRG_memcpy(&service_uuid.Service_UUID_128, uuid, 16);
  ret = aci_gatt_add_service(UUID_TYPE_128, &service_uuid, PRIMARY_SERVICE,
                             max_attribute_records, &HWServW2STHandle);
  if (ret != BLE_STATUS_SUCCESS)
    return BLE_STATUS_ERROR;

  /* Fill the Environmental BLE Characteristc */
  COPY_ENVIRONMENTAL_W2ST_CHAR_UUID(uuid);
  uuid[13] |= 0x80; /* CO sensor */
  uuid[14] |= 0x01; /* Second Temperature */
  uuid[14] |= 0x04; /* One Temperature value */
  uuid[14] |= 0x08; /* Humidity */
  uuid[14] |= 0x10; /* Pressure value */
  uuid[15] |= 0x01; /* Luxmeter */
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, &char_uuid,
                           2+4+2+2+2+4+2,
                           CHAR_PROP_NOTIFY|CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &EnvironmentalCharHandle);
  if (ret != BLE_STATUS_SUCCESS)
    return BLE_STATUS_ERROR;
#if (IMU_PRESENT==1)
  /* Fill the AccGyroMag BLE Characteristc */
  COPY_ACC_GYRO_MAG_W2ST_CHAR_UUID(uuid);
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, &char_uuid,
                           2+3*3*2,
                           CHAR_PROP_NOTIFY,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &AccGyroMagCharHandle);
  if (ret != BLE_STATUS_SUCCESS)
    return BLE_STATUS_ERROR;

  /* Fill the Accelerometer BLE Characteristc */
  COPY_ACC_EVENT_W2ST_CHAR_UUID(uuid);
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, &char_uuid, 2+3,
                           CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 1, &AccEventCharHandle);

  if (ret != BLE_STATUS_SUCCESS)
    return BLE_STATUS_ERROR;
#endif
  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Add the SW Feature service using a vendor specific profile
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_SWServW2ST_Service(void)
{
#if (IMU_PRESENT==1)
  tBleStatus ret;
  uint8_t uuid[16];
  /* num of characteristics of this service */
  /* Sensor Fusion Short + ECompass + Activity Recognition */
  uint8_t char_number = 3;
  /* number of attribute records that can be added to this service */
  uint8_t max_attribute_records = 1+(3*char_number);

  /* add SW_SENS_W2ST service */
  COPY_SW_SENS_W2ST_SERVICE_UUID(uuid);
  BLUENRG_memcpy(&service_uuid.Service_UUID_128, uuid, 16);
  ret = aci_gatt_add_service(UUID_TYPE_128, &service_uuid, PRIMARY_SERVICE,
                             max_attribute_records, &SWServW2STHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  /* Code for MotionAR integration - Start Section */
  COPY_ACTIVITY_REC_W2ST_CHAR_UUID(uuid);
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(SWServW2STHandle, UUID_TYPE_128, &char_uuid, 2+1,
                           CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &ActivityRecCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  /* Code for MotionAR integration - End Section */

  /* Code for MotionFX integration - Start Section */
  /* Quaternions */
  COPY_QUATERNIONS_W2ST_CHAR_UUID(uuid);
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(SWServW2STHandle, UUID_TYPE_128, &char_uuid,
                           2+6*SEND_N_QUATERNIONS,
                           CHAR_PROP_NOTIFY,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &QuaternionsCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  /* ECompass */
  COPY_ECOMPASS_W2ST_CHAR_UUID(uuid);
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(SWServW2STHandle, UUID_TYPE_128, &char_uuid, 2+2,
                           CHAR_PROP_NOTIFY,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &ECompassCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  /* Code for MotionFX integration - End Section */

  return BLE_STATUS_SUCCESS;

fail:
#endif
  return BLE_STATUS_ERROR;
}

/**
 * @brief  Add the Config service using a vendor specific profile
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_ConfigW2ST_Service(void)
{
  tBleStatus ret;
  uint8_t uuid[16];

  COPY_CONFIG_SERVICE_UUID(uuid);
  BLUENRG_memcpy(&service_uuid.Service_UUID_128, uuid, 16);
  ret = aci_gatt_add_service(UUID_TYPE_128,  &service_uuid, PRIMARY_SERVICE, 1+3,&ConfigServW2STHandle);

  if (ret != BLE_STATUS_SUCCESS)
    goto fail;

  COPY_CONFIG_W2ST_CHAR_UUID(uuid);
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(ConfigServW2STHandle, UUID_TYPE_128, &char_uuid, W2ST_MAX_CHAR_LEN /* Max Dimension */,
                           CHAR_PROP_NOTIFY| CHAR_PROP_WRITE_WITHOUT_RESP,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 1, &ConfigCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  return BLE_STATUS_SUCCESS;

fail:
  return BLE_STATUS_ERROR;
}

/**
* @brief Add a time service using a vendor specific profile (See UM1873 STM Doc)
* @param None
* @retval Status
*/
tBleStatus Add_Time_Service(void)
{
	tBleStatus ret;
	uint8_t uuid[16];
	/* num of characteristics of this service */
	/* Seconds + Minutes */
	uint8_t char_number = 2;
	/* number of attribute records that can be added to this service */
	uint8_t max_attribute_records = 1+(3*char_number);

	COPY_TIME_SERVICE_UUID(uuid);
	BLUENRG_memcpy(&service_uuid.Service_UUID_128, uuid, 16);
	ret = aci_gatt_add_service(UUID_TYPE_128, &service_uuid, PRIMARY_SERVICE,
							   max_attribute_records, &timeServHandle);

	if (ret != BLE_STATUS_SUCCESS)
		goto fail;

	/* Fill the Seconds Characteristic */
	COPY_TIME_CHAR_UUID(uuid);
	BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
	ret = aci_gatt_add_char(timeServHandle, UUID_TYPE_128, &char_uuid, 2+4,
							CHAR_PROP_READ,
							ATTR_PERMISSION_NONE,
							GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
							16, 0, &secondsCharHandle);

	if (ret != BLE_STATUS_SUCCESS)
		goto fail;

	/* Fill the Minutes Characteristic */
	COPY_MINUTE_CHAR_UUID(uuid);
	BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
	ret = aci_gatt_add_char(timeServHandle, UUID_TYPE_128, &char_uuid, 2+4,
							CHAR_PROP_NOTIFY|CHAR_PROP_READ,
							ATTR_PERMISSION_NONE,
							GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
							16, 0, &minutesCharHandle);

	if (ret != BLE_STATUS_SUCCESS)
		goto fail;

	return BLE_STATUS_SUCCESS;
fail:
	return BLE_STATUS_ERROR ;
}

#ifdef STM32L476xx
/**
 * @brief  Add the Console service using a vendor specific profile
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_ConsoleW2ST_Service(void)
{
  /* num of characteristics of this service */
  uint8_t char_number = 2;
  /* number of attribute records that can be added to this service */
  uint8_t max_attribute_records = 1+(3*char_number);
  tBleStatus ret;

  uint8_t uuid[16];

  COPY_CONSOLE_SERVICE_UUID(uuid);
  BLUENRG_memcpy(&service_uuid.Service_UUID_128, uuid, 16);
  ret = aci_gatt_add_service(UUID_TYPE_128,  &service_uuid, PRIMARY_SERVICE,
                             max_attribute_records,
                             &ConsoleW2STHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  COPY_TERM_CHAR_UUID(uuid);
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(ConsoleW2STHandle, UUID_TYPE_128, &char_uuid, W2ST_MAX_CHAR_LEN,
                           CHAR_PROP_NOTIFY| CHAR_PROP_WRITE_WITHOUT_RESP | CHAR_PROP_WRITE | CHAR_PROP_READ ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 1, &TermCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  COPY_STDERR_CHAR_UUID(uuid);
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(ConsoleW2STHandle, UUID_TYPE_128, &char_uuid, W2ST_MAX_CHAR_LEN,
                           CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 1, &StdErrCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
     goto fail;
  }

  return BLE_STATUS_SUCCESS;

fail:
  return BLE_STATUS_ERROR;
}
#endif /* STM32L476xx */

/* Code for notifications integration - Start Section */
/**
 * @brief  Send a notification for answering to a configuration command for Accelerometer events
 * @param  uint32_t Feature Feature calibrated
 * @param  uint8_t Command Replay to this Command
 * @param  uint8_t data result to send back
 * @retval tBleStatus Status
 */
tBleStatus ConfigNotify_Update(uint32_t Feature,uint8_t Command,uint8_t data)
{
  tBleStatus ret;
  uint8_t buff[2+4+1+1];

  HOST_TO_LE_16(buff,(HAL_GetTick()>>3));	//The Time Stamp is on an 8mS basis (??)
  HOST_TO_BE_32(buff+2,Feature);
  buff[6] = Command;
  buff[7] = data;

  ret = aci_gatt_update_char_value (ConfigServW2STHandle, ConfigCharHandle, 0, 8,buff);
  if (ret != BLE_STATUS_SUCCESS){
	  PRINT_DBG("Error Updating Configuration Char\r\n");
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Send a notification When the DS3 detects one Acceleration event
 * @param  Command to Send
 * @retval tBleStatus Status
 */
tBleStatus AccEventNotify_Update(uint16_t Command, uint8_t dimByte)
{
  tBleStatus ret= BLE_STATUS_SUCCESS;
  uint8_t buff_2[2+2];
  uint8_t buff_3[2+3];

  switch(dimByte)
  {
  case 2:
	HOST_TO_LE_16(buff_2,(HAL_GetTick()>>3));
	HOST_TO_LE_16(buff_2+2,Command);
    ret = aci_gatt_update_char_value(HWServW2STHandle, AccEventCharHandle, 0, 2+2,buff_2);
    break;
  case 3:
	HOST_TO_LE_16(buff_3  ,(HAL_GetTick()>>3));
    buff_3[2]= 0;
    HOST_TO_LE_16(buff_3+3,Command);
    ret = aci_gatt_update_char_value(HWServW2STHandle, AccEventCharHandle, 0, 2+3,buff_3);
    break;
  }

  if (ret != BLE_STATUS_SUCCESS){
	  PRINT_DBG("Error Updating AccEventNotify_Update Char\r\n");
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}
/* Code for notifications integration - End Section */

/**
 * @brief  Update environmental characteristic value
 * @param  int32_t pressure value
 * @param  int16_t temperature value
 * @retval tBleStatus Status
 */
tBleStatus Environmental_Update(int32_t press, int16_t temp, int16_t temp2, int16_t hum, int32_t co)
{
  tBleStatus ret;
  int16_t lux = 100;
  uint8_t buff[2+2+4+2+2+2+4];

  HOST_TO_LE_16(buff, HAL_GetTick()>>3);
/* Call the function in the reverse order in which the features
 * appear in the Add_HWServW2ST_Service function
 */
  HOST_TO_LE_16(buff+2,lux);
  HOST_TO_LE_32(buff+4,press);
  HOST_TO_LE_16(buff+8,hum);
  HOST_TO_LE_16(buff+10,temp);
  HOST_TO_LE_16(buff+12,temp2);
  HOST_TO_LE_32(buff+14,co);

  ret = aci_gatt_update_char_value(HWServW2STHandle, EnvironmentalCharHandle,
                                   0, 2+2+4+2+2+2+4, buff);

  if (ret != BLE_STATUS_SUCCESS){
    PRINT_DBG("Error while updating TEMP characteristic: 0x%04X\r\n",ret) ;
    return BLE_STATUS_ERROR ;
  }

  return BLE_STATUS_SUCCESS;
}

#if (IMU_PRESENT==1)
/**
 * @brief  Update acceleration characteristic value
 * @param  AxesRaw_t structure containing acceleration value in mg.
 * @retval tBleStatus Status
 */
tBleStatus Acc_Update(AxesRaw_t *x_axes, AxesRaw_t *g_axes, AxesRaw_t *m_axes)
{
  tBleStatus ret;
  uint8_t buff[2+2*3*3];

  HOST_TO_LE_16(buff,(HAL_GetTick()>>3));	//The Time Stamp is on an 8mS basis (??)

  HOST_TO_LE_16(buff+2, -x_axes->AXIS_X);
  HOST_TO_LE_16(buff+4,  x_axes->AXIS_Y);
  HOST_TO_LE_16(buff+6, -x_axes->AXIS_Z);

  HOST_TO_LE_16(buff+8,  g_axes->AXIS_X);
  HOST_TO_LE_16(buff+10, g_axes->AXIS_Y);
  HOST_TO_LE_16(buff+12, g_axes->AXIS_Z);

  HOST_TO_LE_16(buff+14, m_axes->AXIS_X);
  HOST_TO_LE_16(buff+16, m_axes->AXIS_Y);
  HOST_TO_LE_16(buff+18, m_axes->AXIS_Z);

  ret = aci_gatt_update_char_value(HWServW2STHandle, AccGyroMagCharHandle,
				   	   	   	   	   0, 2+2*3*3, buff);
  if (ret != BLE_STATUS_SUCCESS){
    PRINT_DBG("Error while updating Acceleration characteristic: 0x%02X\r\n",ret) ;
    return BLE_STATUS_ERROR ;
  }

  return BLE_STATUS_SUCCESS;
}

/* Code for MotionFX integration - Start Section */
/**
 * @brief  Update quaternions characteristic value
 * @param  SensorAxes_t *data Structure containing the quaterions
 * @retval tBleStatus      Status
 */
tBleStatus Quat_Update(AxesRaw_t *data)
{
  tBleStatus ret;
  uint8_t buff[2+6*SEND_N_QUATERNIONS];

  HOST_TO_LE_16(buff,(HAL_GetTick()>>3));	//The Time Stamp is on an 8mS basis (??)

#if SEND_N_QUATERNIONS == 1
  HOST_TO_LE_16(buff+2,data[0].AXIS_X);
  HOST_TO_LE_16(buff+4,data[0].AXIS_Y);
  HOST_TO_LE_16(buff+6,data[0].AXIS_Z);
#elif SEND_N_QUATERNIONS == 2
  HOST_TO_LE_16(buff+2,data[0].AXIS_X);
  HOST_TO_LE_16(buff+4,data[0].AXIS_Y);
  HOST_TO_LE_16(buff+6,data[0].AXIS_Z);

  HOST_TO_LE_16(buff+8 ,data[1].AXIS_X);
  HOST_TO_LE_16(buff+10,data[1].AXIS_Y);
  HOST_TO_LE_16(buff+12,data[1].AXIS_Z);
#elif SEND_N_QUATERNIONS == 3
  HOST_TO_LE_16(buff+2,data[0].AXIS_X);
  HOST_TO_LE_16(buff+4,data[0].AXIS_Y);
  HOST_TO_LE_16(buff+6,data[0].AXIS_Z);

  HOST_TO_LE_16(buff+8 ,data[1].AXIS_X);
  HOST_TO_LE_16(buff+10,data[1].AXIS_Y);
  HOST_TO_LE_16(buff+12,data[1].AXIS_Z);

  HOST_TO_LE_16(buff+14,data[2].AXIS_X);
  HOST_TO_LE_16(buff+16,data[2].AXIS_Y);
  HOST_TO_LE_16(buff+18,data[2].AXIS_Z);
#else
#error SEND_N_QUATERNIONS could be only 1,2,3
#endif

  ret = aci_gatt_update_char_value(SWServW2STHandle, QuaternionsCharHandle,
				   	   	   	   	   0, 2+6*SEND_N_QUATERNIONS, buff);
  if (ret != BLE_STATUS_SUCCESS){
    PRINT_DBG("Error while updating Sensor Fusion characteristic: 0x%02X\r\n",ret) ;
    return BLE_STATUS_ERROR ;
  }

  ConfigNotify_Update(FEATURE_MASK_SENSORFUSION_SHORT,W2ST_COMMAND_CAL_STATUS,MagCalStatus ? 100: 0);
  ConfigNotify_Update(FEATURE_MASK_ECOMPASS,W2ST_COMMAND_CAL_STATUS,MagCalStatus ? 100: 0);
//ConfigNotify_Update(FEATURE_MASK_SENSORFUSION_SHORT,W2ST_COMMAND_CAL_STATUS,1 ? 100: 0);
//ConfigNotify_Update(FEATURE_MASK_ECOMPASS,W2ST_COMMAND_CAL_STATUS,1 ? 100: 0);

  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update E-Compass characteristic value
 * @param  uint16_t Angle To Magnetic North in cents of degree [0.00 -> 259,99]
 * @retval tBleStatus      Status
 */
tBleStatus ECompass_Update(uint16_t angle)
{
  tBleStatus ret;
  uint8_t buff[2+2];

  HOST_TO_LE_16(buff,(HAL_GetTick()>>3));	//The Time Stamp is on an 8mS basis (??)
  HOST_TO_LE_16(buff+2,angle);

  ret = aci_gatt_update_char_value(SWServW2STHandle, ECompassCharHandle, 0, 2+2, buff);

  if (ret != BLE_STATUS_SUCCESS){
	  PRINT_DBG("Error Updating E-Compass Char\r\n");
	  return BLE_STATUS_ERROR;
  }

  ConfigNotify_Update(FEATURE_MASK_SENSORFUSION_SHORT,W2ST_COMMAND_CAL_STATUS,MagCalStatus ? 100: 0);
  ConfigNotify_Update(FEATURE_MASK_ECOMPASS,W2ST_COMMAND_CAL_STATUS,MagCalStatus ? 100: 0);
//  ConfigNotify_Update(FEATURE_MASK_SENSORFUSION_SHORT,W2ST_COMMAND_CAL_STATUS,1 ? 100: 0);
//  ConfigNotify_Update(FEATURE_MASK_ECOMPASS,W2ST_COMMAND_CAL_STATUS,1 ? 100: 0);

  return BLE_STATUS_SUCCESS;
}
/* Code for MotionFX integration - End Section */

/* Code for MotionAR integration - Start Section */
/**
 * @brief  Update Activity Recognition value
 * @param  MAR_output_t ActivityCode Activity Recognized
 * @retval tBleStatus      Status
 */
tBleStatus ActivityRec_Update(MAR_Output_t ActivityCode)
{
  tBleStatus ret;
  uint8_t buff[2+ 1];

  HOST_TO_LE_16(buff,(HAL_GetTick()>>3));	//The Time Stamp is on an 8mS basis (??)
  buff[2] = ActivityCode;

  ret = aci_gatt_update_char_value(SWServW2STHandle, ActivityRecCharHandle, 0, 2+1, buff);

  if (ret != BLE_STATUS_SUCCESS){
	  PRINT_DBG("Error Updating ActivityRec Char\r\n");
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}
/* Code for MotionAR integration - End Section */
#endif	//IMU_PRESENT==1

/**
* @brief Update seconds characteristic value of Time service
* @param None
* @retval Status
*/
tBleStatus Seconds_Update(void)
{
	extern FLASH_DATA_ORG FlashDataOrg;
	tBleStatus ret;
	uint8_t buff[2+4];

	HOST_TO_LE_16(buff,(HAL_GetTick()>>3));			//The Time Stamp is on an 8mS basis (??)
	HOST_TO_LE_32(buff+2,FlashDataOrg.b_status.s1);	//Get the total uptime timer in seconds

	ret = aci_gatt_update_char_value(timeServHandle, secondsCharHandle, 0, 2+4, buff);

	if (ret != BLE_STATUS_SUCCESS){
		PRINT_DBG("Error while updating TIME characteristic.\n") ;
		return BLE_STATUS_ERROR ;
	}

	return BLE_STATUS_SUCCESS;
}

/**
* @brief Update minute characteristic value of Time service
* 		 Send a notification for a minute characteristic of time service
* @param None
* @retval Status
*/
tBleStatus Minutes_Update(void)
{
	extern FLASH_DATA_ORG FlashDataOrg;
	tBleStatus ret;
	uint8_t buff[2+4];
	uint32_t minuteValue;
	static uint32_t previousMinuteValue = 0;

	HOST_TO_LE_16(buff,(HAL_GetTick()>>3));

	minuteValue=FlashDataOrg.b_status.s1/60;

	if(minuteValue != previousMinuteValue)
	{
		/* memorize this "minute" value for future usage */
		previousMinuteValue = minuteValue;
		HOST_TO_LE_32(buff+2,minuteValue);

		ret = aci_gatt_update_char_value(timeServHandle, minutesCharHandle, 0, 2+4, buff);

		if (ret != BLE_STATUS_SUCCESS){
			PRINT_DBG("Error while updating TIME characteristic.\n") ;
			return BLE_STATUS_ERROR ;
		}
	}

	return BLE_STATUS_SUCCESS;
}

/**
* @brief Updates "Seconds and Minutes characteristics" values
* @param None
* @retval None
*/
void Time_Update()
{
	/* update "seconds and minutes characteristics" of time service */
	Seconds_Update();
	Minutes_Update();
}

#ifdef STM32L476xx
/**
 * @brief  Update Terminal characteristic value
 * @param  uint8_t    *data string to write
 * @param  uint8_t    length lengt of string to write
 * @retval tBleStatus Status
 */
tBleStatus Term_Update(uint8_t *data,uint8_t length)
{
  tBleStatus ret;
  uint8_t Offset;
  uint8_t DataToSend;

  /* Split the code in packages */
  for(Offset =0; Offset<length; Offset +=W2ST_MAX_CHAR_LEN){
    DataToSend = (length-Offset);
    DataToSend = (DataToSend>W2ST_MAX_CHAR_LEN) ?  W2ST_MAX_CHAR_LEN : DataToSend;

    /* keep a copy */
    memcpy(LastTermBuffer,data+Offset,DataToSend);
    LastTermLen = DataToSend;

    ret = aci_gatt_update_char_value(ConsoleW2STHandle, TermCharHandle, 0, DataToSend , data+Offset);
    if (ret != BLE_STATUS_SUCCESS) {
        PRINT_DBG("Term_Update: Error Updating Stdout Char\r\n");
      return BLE_STATUS_ERROR;
    }
    HAL_Delay(20);
  }

  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update Terminal characteristic value after a read request
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Term_Update_AfterRead(void)
{
  tBleStatus ret;

  ret = aci_gatt_update_char_value(ConsoleW2STHandle, TermCharHandle, 0, LastTermLen , LastTermBuffer);
  if (ret != BLE_STATUS_SUCCESS) {
    return BLE_STATUS_ERROR;
  }

  return BLE_STATUS_SUCCESS;
}
#endif /* STM32L476xx */

/**
 * @brief  Update the sensor value
 *
 * @param  Handle of the characteristic to update
 * @retval None
 */
void Read_Request_CB(uint16_t handle)
{
  tBleStatus ret;
#if (HUMIDITY_SENSOR_PRESENT==0)
  int16_t Hum_Out = 450;
  int16_t T2_Out = 234;
#endif

  if (handle == EnvironmentalCharHandle + 1)
  {
	 Environmental_Update(P0_Out, T_Out, T2_Out, Hum_Out, CO_Out);
  }
#if (IMU_PRESENT==1)
  else if(handle == AccGyroMagCharHandle + 1)
  {
     Acc_Update(&x_axes, &g_axes, &m_axes);
  }
  else if (handle == QuaternionsCharHandle + 1)
  {
	 Quat_Update(&q_axes);
  }
  else if (handle == ECompassCharHandle + 1)
  {
	 ECompass_Update(Angle);
  }
  else if (handle == ActivityRecCharHandle + 1)
  {
	 ActivityRec_Update(Activity);
  }
#endif	//IMU_PRESENT==1
  else if (handle == secondsCharHandle + 1)
  {
	 Seconds_Update();
  }
  else if (handle == minutesCharHandle + 1)
  {
	 Minutes_Update();
  }
#ifdef STM32L476xx
  else if (handle == TermCharHandle + 1) {
    /* Send again the last packet for Terminal */
    Term_Update_AfterRead();
  }
#endif /* STM32L476xx */

  if(connection_handle !=0)
  {
    ret = aci_gatt_allow_read(connection_handle);
    if (ret != BLE_STATUS_SUCCESS)
    {
      PRINT_DBG("aci_gatt_allow_read() failed: 0x%02x\r\n", ret);
    }
  }
}

/**
 * @brief  This function is called when there is a change on the gatt attribute.
 *         With this function it's possible to understand if one application
 *         is subscribed to the one service or not.
 *
 * @param  uint16_t att_handle Handle of the attribute
 * @param  uint8_t  *att_data attribute data
 * @param  uint8_t  data_length length of the data
 * @retval None
 */
void Attribute_Modified_Request_CB(uint16_t Connection_Handle, uint16_t attr_handle, uint16_t Offset, uint8_t data_length, uint8_t *att_data)
{
  if(attr_handle == EnvironmentalCharHandle + 2) {
    if (att_data[0] == 1) {
      send_env = TRUE;
    } else if (att_data[0] == 0){
      send_env = FALSE;
    }
  }
  else if (attr_handle == AccGyroMagCharHandle +2) {
    if (att_data[0] == 1) {
      send_mot = TRUE;
    } else if (att_data[0] == 0){
      send_mot = FALSE;
    }
  }
  else if (attr_handle == QuaternionsCharHandle +2) {
    if (att_data[0] == 1) {
      send_quat = TRUE;
    } else if (att_data[0] == 0){
      send_quat = FALSE;
    }
  }
  else if(attr_handle == ECompassCharHandle + 2){
    if (att_data[0] == 1) {
      send_ecomp = TRUE;
    } else if (att_data[0] == 0){
      send_ecomp = FALSE;
    }
  }
  else if(attr_handle == ActivityRecCharHandle + 2){
    if (att_data[0] == 1) {
      send_ar = TRUE;
    } else if (att_data[0] == 0){
      send_ar = FALSE;
    }
  }
  else if(attr_handle == secondsCharHandle + 2){
    if (att_data[0] == 1) {
      send_scds = TRUE;
    } else if (att_data[0] == 0){
    	send_scds = FALSE;
    }
  }
  else if(attr_handle == minutesCharHandle + 2){
    if (att_data[0] == 1) {
      send_mnts = TRUE;
    } else if (att_data[0] == 0){
    	send_mnts = FALSE;
    }
  }

#ifdef STM32L476xx
  else if (attr_handle == TermCharHandle + 1) {
    uint32_t SendBackData =1; /* By default Answer with the same message received */

    if (SizeOfUpdateBlueFW != 0) {
      /* X-CUBE-BLE2 firwmare update */
      int8_t RetValue = UpdateFWBlueNRG(&SizeOfUpdateBlueFW, att_data, data_length, 1);
      if (RetValue != 0) {
        MCR_FAST_TERM_UPDATE_FOR_OTA(((uint8_t *)&RetValue));
        if (RetValue == 1) {
          /* if OTA checked */
          BytesToWrite = sprintf((char *)BufferToWrite, "The Board will restart in 5 seconds\r\n");
          Term_Update(BufferToWrite,BytesToWrite);
          PRINT_DBG("The BlueNRG application will restart in 5 seconds\r\n");
          HAL_Delay(5000);
          HAL_NVIC_SystemReset();
        }
      }
      SendBackData = 0;
    }
    else {
      /* Received one write from Client on Terminal characteristc */
      SendBackData = DebugConsoleCommandParsing(att_data,data_length);
    }

    /* Send it back for testing */
    if (SendBackData) {
      Term_Update(att_data,data_length);
    }
  }
#endif /* STM32L476xx */
}

#ifdef STM32L476xx
/**
 * @brief  This function makes the parsing of the Debug Console Commands
 *
 * @param uint8_t *att_data attribute data
 * @param uint8_t data_length length of the data
 * @retval uint32_t SendItBack true/false
 */
static uint32_t DebugConsoleCommandParsing(uint8_t * att_data, uint8_t data_length)
{
  uint32_t SendBackData = 1;

  /* Help Command */
  if(!strncmp("help",(char *)(att_data),4)) {
    /* Print Legend */
    SendBackData=0;

    BytesToWrite =sprintf((char *)BufferToWrite,"Command:\r\n"
                          "info -> System Info\r\n"
                          "versionFw  -> FW Version\r\n"
                          "versionBle -> Ble Version\r\n");
    Term_Update(BufferToWrite,BytesToWrite);
  }
  else if (!strncmp("versionFw",(char *)(att_data),9)) {
    BytesToWrite = sprintf((char *)BufferToWrite,"%s_%s_%c.%c.%c\r\n",
                            "L476"
                            ,BLUENRG_PACKAGENAME,
                             PACK_VERSION_MAJOR,
                             PACK_VERSION_MINOR,
                             PACK_VERSION_PATCH);

    Term_Update(BufferToWrite,BytesToWrite);
    SendBackData=0;

  }
  else if (!strncmp("info",(char *)(att_data),4)) {
    SendBackData=0;

    BytesToWrite = sprintf((char *)BufferToWrite,
                           "\r\nSTMicroelectronics %s:\r\n"
                           "\tVersion %c.%c.%c\r\n"
                           "\tSTM32L476RG-Nucleo board"
                           "\r\n",
                           BLUENRG_PACKAGENAME,
                           PACK_VERSION_MAJOR, PACK_VERSION_MINOR, PACK_VERSION_PATCH);
    Term_Update(BufferToWrite, BytesToWrite);

    BytesToWrite = sprintf((char *)BufferToWrite,
                           "\t(HAL %d.%d.%d_%d)\r\n"
                           "\tCompiled %s %s"
#if defined (__IAR_SYSTEMS_ICC__)
                           " (IAR)\r\n",
#elif defined (__CC_ARM)
                           " (KEIL)\r\n",
#elif defined (__GNUC__)
                           " (STM32CubeIDE)\r\n",
#endif
                           (uint16_t)((HAL_GetHalVersion() >>24)),
                           (uint16_t)((HAL_GetHalVersion() >>16)&0xFF),
                           (uint16_t)((HAL_GetHalVersion() >> 8)&0xFF),
                           (uint16_t)((HAL_GetHalVersion()     )&0xFF),
                           __DATE__,__TIME__);
    Term_Update(BufferToWrite,BytesToWrite);
  }
  else if (!strncmp("upgradeFw", (char *)(att_data), 9)) {
    uint32_t uwCRCValue;
    uint8_t *PointerByte = (uint8_t*) &SizeOfUpdateBlueFW;

    SizeOfUpdateBlueFW = atoi((char *)(att_data + 9));
    PointerByte[0] = att_data[ 9];
    PointerByte[1] = att_data[10];
    PointerByte[2] = att_data[11];
    PointerByte[3] = att_data[12];

    /* Check the Maximum Possible OTA size */
    if (SizeOfUpdateBlueFW > OTA_MAX_PROG_SIZE) {
      PRINT_DBG("OTA %s SIZE=%ld > %d Max Allowed\r\n", BLUENRG_PACKAGENAME, (long)(SizeOfUpdateBlueFW), OTA_MAX_PROG_SIZE);
      /* Answer with a wrong CRC value for signaling the problem to BlueMS application */
      PointerByte[0] = att_data[13];
      PointerByte[1] = (att_data[14]!=0) ? 0 : 1; /* In order to be sure to have a wrong CRC */
      PointerByte[2] = att_data[15];
      PointerByte[3] = att_data[16];
      BytesToWrite = 4;
      Term_Update(BufferToWrite,BytesToWrite);
    }
    else {
      PointerByte = (uint8_t*) &uwCRCValue;
      PointerByte[0] = att_data[13];
      PointerByte[1] = att_data[14];
      PointerByte[2] = att_data[15];
      PointerByte[3] = att_data[16];

      PRINT_DBG("OTA %s SIZE=%ld uwCRCValue=%lx\r\n", BLUENRG_PACKAGENAME, (long)(SizeOfUpdateBlueFW), (long)(uwCRCValue));

      /* Reset the Flash */
      StartUpdateFWBlueNRG(SizeOfUpdateBlueFW, uwCRCValue);

      /* Reduce the connection interval */
      {
        int ret = aci_l2cap_connection_parameter_update_req(connection_handle,
                                                            10  /* interval_min  */,
                                                            10  /* interval_max  */,
                                                            0   /* slave_latency */,
                                                            400 /* timeout_multiplier */);
        /* Go to infinite loop if there is one error */
        if (ret != BLE_STATUS_SUCCESS) {
          while (1) {
            PRINT_DBG("Problem Changing the connection interval\r\n");
          }
        }
      }

      /* Signal that we are ready sending back the CRV value*/
      BufferToWrite[0] = PointerByte[0];
      BufferToWrite[1] = PointerByte[1];
      BufferToWrite[2] = PointerByte[2];
      BufferToWrite[3] = PointerByte[3];
      BytesToWrite = 4;
      Term_Update(BufferToWrite,BytesToWrite);
    }

    SendBackData=0;
  }
  else if (!strncmp("versionBle",(char *)(att_data),10)) {
    uint8_t  hwVersion = 0x00;
    uint16_t fwVersion = 0x0000;
    /* get the BlueNRG HW and FW versions */
    getBlueNRG2_Version(&hwVersion, &fwVersion);
    BytesToWrite = sprintf((char *)BufferToWrite,"%s_%d.%d.%c\r\n",
                           (hwVersion > 0x30) ? "BlueNRG-MS" : "BlueNRG",
                           (fwVersion>>8),
                           (fwVersion>>4)&0xF,
                           (hwVersion > 0x30) ? ('a'+(fwVersion&0xF)-1) : 'a');
    Term_Update(BufferToWrite,BytesToWrite);
    SendBackData=0;
  }

  return SendBackData;
}

/**
 * @brief  Get BlueNRG-2 hardware and firmware version
 *
 * @param  Hardware version
 * @param  Firmware version
 * @retval Status
 */
static uint8_t getBlueNRG2_Version(uint8_t *hwVersion, uint16_t *fwVersion)
{
  uint8_t status;
  uint8_t hci_version, lmp_pal_version;
  uint16_t hci_revision, manufacturer_name, lmp_pal_subversion;
  uint8_t DTM_version_major, DTM_version_minor, DTM_version_patch, DTM_variant, BTLE_Stack_version_major, BTLE_Stack_version_minor, BTLE_Stack_version_patch, BTLE_Stack_development;
  uint16_t DTM_Build_Number, BTLE_Stack_variant, BTLE_Stack_Build_Number;

  status = hci_read_local_version_information(&hci_version, &hci_revision, &lmp_pal_version,
				              &manufacturer_name, &lmp_pal_subversion);

  if (status == BLE_STATUS_SUCCESS) {
    *hwVersion = hci_revision >> 8;
  }
  else {
    PRINT_DBG("Error= %x \r\n", status);
  }

  status = aci_hal_get_firmware_details(&DTM_version_major,
                                        &DTM_version_minor,
                                        &DTM_version_patch,
                                        &DTM_variant,
                                        &DTM_Build_Number,
                                        &BTLE_Stack_version_major,
                                        &BTLE_Stack_version_minor,
                                        &BTLE_Stack_version_patch,
                                        &BTLE_Stack_development,
                                        &BTLE_Stack_variant,
                                        &BTLE_Stack_Build_Number);

  if (status == BLE_STATUS_SUCCESS) {
    *fwVersion = BTLE_Stack_version_major  << 8;  // Major Version Number
    *fwVersion |= BTLE_Stack_version_minor << 4;  // Minor Version Number
    *fwVersion |= BTLE_Stack_version_patch;       // Patch Version Number
  }
  else {
    PRINT_DBG("Error= %x \r\n", status);
  }

  return status;
}
#endif /* STM32L476xx */
