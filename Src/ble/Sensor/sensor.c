/**
  ******************************************************************************
  * @file    sensor.c
  * @author  SRA Application Team
  * @brief   Sensor init and sensor state machines
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
#include "OpModes.h"		//Added By Me!!!
#if (SENSOR_APP==1)			//Added By Me!!!

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <Sensor/gatt_db.h>
#include <Sensor/sensor.h>
#include "bluenrg1_gap.h"
#include "bluenrg1_gap_aci.h"
#include "bluenrg1_hci_le.h"
#include "hci_const.h"
#include "bluenrg1_gatt_aci.h"

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern uint8_t bdaddr[BDADDR_SIZE];
//extern uint8_t bnrg_expansion_board;
//extern uint16_t EnvironmentalCharHandle;
//extern uint16_t AccGyroMagCharHandle;

__IO uint8_t  set_connectable = 1;
__IO uint16_t connection_handle = 0;
__IO uint8_t  notification_enabled = FALSE;
__IO uint8_t  connected = FALSE;
__IO uint8_t  pairing = FALSE;
__IO uint8_t  paired = FALSE;

volatile uint8_t request_free_fall_notify = FALSE;

AxesRaw_t x_axes = {0, 0, 0};
AxesRaw_t g_axes = {0, 0, 0};
AxesRaw_t m_axes = {0, 0, 0};
AxesRaw_t q_axes[SEND_N_QUATERNIONS] = {{0, 0, 0}};
uint16_t Angle;
MAR_Output_t Activity;

/* Private function prototypes -----------------------------------------------*/
//void GAP_DisconnectionComplete_CB(void);
//void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle);

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Set_DeviceConnectable
 * @note   Puts the device in connectable mode
 * @param  None
 * @retval None
 */
void Set_DeviceConnectable(void)
{
  uint8_t ret;
  uint8_t local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,DEVICE_NAME};

  uint8_t manuf_data[26] =
  {
    2,0x0A,0x00, /* 0 dBm */  // Transmission Power
    8,0x09,DEVICE_NAME,  // Complete Name
    13,0xFF,0x01, /* SKD version */
    0x80,
    0x00,	/* AudioSync+AudioData */
    0x00,	/* ACC + Gyro + Mag + Environmental + Battery Info */
    0x00,	/* Hardware Events + MotionFX + SD Card Logging */
    0x00,	/*  */
    bdaddr[5], /* BLE MAC start -MSB first- */
    bdaddr[4],
    bdaddr[3],
    bdaddr[2],
    bdaddr[1],
    bdaddr[0]  /* BLE MAC stop */
  };

  manuf_data[16] |= 0x01;	// Luxmetro
//manuf_data[16] |= 0x02;	// Proximity
//manuf_data[16] |= 0x04;	// Mic
//manuf_data[16] |= 0x08;	// ADPCM Audio
//manuf_data[16] |= 0x10;	// Audio Source Localization
//manuf_data[16] |= 0x20;	// Switch Status
//manuf_data[16] |= 0x40;	// ADPCM Sync: parameters used for a proper ADPCM decoding
//manuf_data[16] |= 0x80;	// RFU: Reserved for Future Use

  manuf_data[17] |= 0x01;	// Second Temperature
//manuf_data[17] |= 0x02;	// Battery
  manuf_data[17] |= 0x04;	// One Temperature value
  manuf_data[17] |= 0x08;	// Humidity
  manuf_data[17] |= 0x10;	// Pressure value
  manuf_data[17] |= 0x20;	// Magnetometer
  manuf_data[17] |= 0x40;	// Gyroscope
  manuf_data[17] |= 0x80;	// Accelerometer

  manuf_data[18] |= 0x01;	// Sensor Fusion Compact
//manuf_data[18] |= 0x02;	// FreeFall
  manuf_data[18] |= 0x04;	// Accelerometer Events
//manuf_data[18] |= 0x08;	// Beam forming
//manuf_data[18] |= 0x10;	// SD Logging
//manuf_data[18] |= 0x20;	// OTA Reboot bit
//manuf_data[18] |= 0x40;	// Reboot bit
  manuf_data[18] |= 0x80;	// CO Sensor

//manuf_data[19] |= 0x01;	// Code for MotionPR integration (Pedometer)
//manuf_data[19] |= 0x02;	// Code for MotionGR integration (MemsGesture)
//manuf_data[19] |= 0x04;	// Code for MotionGR integration (ProximityGesture)
//manuf_data[19] |= 0x08;	// Code for MotionCP integration (Carry Position)
  manuf_data[19] |= 0x10;	// Code for MotionAR integration (Activity)
  manuf_data[19] |= 0x20;	// Code for MotionFX integration (ECompass)
  manuf_data[19] |= 0x40;	// Code for MotionID integration (Motion intensity)
//manuf_data[19] |= 0x80;	// Code for MotionFX integration (Sensor Fusion)

/*manuf_data[19] |= 0x0A;	// Code for MotionPE & MotionSD & MotionVC integration
  manuf_data[19] |= 0x0D;	// Tilt Measurement
  manuf_data[19] |= 0x0E;	// ALLMEMS1_MOTIONFA */

  hci_le_set_scan_response_data(0,NULL);

  PRINT_DBG("Set General Discoverable Mode.\r\n");

  /* disable scan response */
  hci_le_set_scan_response_data(0,NULL);
  ret = aci_gap_set_discoverable(ADV_DATA_TYPE,
		  	  	  	  	  	  	 /* 0, 0, */
                                 ADV_INTERV_MIN, ADV_INTERV_MAX,
                                 PUBLIC_ADDR,
                                 NO_WHITE_LIST_USE,
                                 sizeof(local_name), local_name, 0, NULL, 0, 0);

  aci_gap_update_adv_data(26, manuf_data);

  if(ret != BLE_STATUS_SUCCESS)
  {
    PRINT_DBG("aci_gap_set_discoverable() failed: 0x%02x\r\n", ret);
  }
  else
  {
    PRINT_DBG("aci_gap_set_discoverable() --> SUCCESS\r\n");
  }
}

/**
 * @brief  Callback processing the ACI events
 * @note   Inside this function each event must be identified and correctly
 *         parsed
 * @param  void* Pointer to the ACI packet
 * @retval None
 */
void APP_UserEvtRx(void *pData)
{
  uint32_t i;

  hci_spi_pckt *hci_pckt = (hci_spi_pckt *)pData;

  if(hci_pckt->type == HCI_EVENT_PKT)
  {
    hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;

    if(event_pckt->evt == EVT_LE_META_EVENT)
    {
      evt_le_meta_event *evt = (void *)event_pckt->data;

      for (i = 0; i < (sizeof(hci_le_meta_events_table)/sizeof(hci_le_meta_events_table_type)); i++)
      {
        if (evt->subevent == hci_le_meta_events_table[i].evt_code)
        {
          hci_le_meta_events_table[i].process((void *)evt->data);
        }
      }
    }
    else if(event_pckt->evt == EVT_VENDOR)
    {
      evt_blue_aci *blue_evt = (void*)event_pckt->data;

      for (i = 0; i < (sizeof(hci_vendor_specific_events_table)/sizeof(hci_vendor_specific_events_table_type)); i++)
      {
        if (blue_evt->ecode == hci_vendor_specific_events_table[i].evt_code)
        {
          hci_vendor_specific_events_table[i].process((void *)blue_evt->data);
        }
      }
    }
    else
    {
      for (i = 0; i < (sizeof(hci_events_table)/sizeof(hci_events_table_type)); i++)
      {
        if (event_pckt->evt == hci_events_table[i].evt_code)
        {
          hci_events_table[i].process((void *)event_pckt->data);
        }
      }
    }
  }
}
#endif	//SENSOR_APP==1 - Added By Me!!!
