/**
  ******************************************************************************
  * @file    bluenrg_conf.h
  * @author  SRA Application Team
  * @brief   BLE configuration file
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
#ifndef BLUENRG_CONF_H
#define BLUENRG_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#if defined(STM32F405xx)
	#include "stm32f4xx_hal.h"
#elif defined(STM32F105xC)
	#include "stm32f1xx_hal.h"
#endif
#include <string.h>
#include "OpModes.h"

/*---------- Print messages from BLE2 files at user level -----------*/
#define BLE2_DEBUG      0
/*---------- Print the data travelling over the SPI in the .csv format compatible with the ST BlueNRG GUI -----------*/
#define PRINT_CSV_FORMAT      0
/*---------- Print messages from BLE2 files at middleware level -----------*/
#define BLUENRG2_DEBUG      0
#if BEACON_APP
/*---------- Number of Bytes reserved for HCI Read Packet -----------*/
	#define HCI_READ_PACKET_SIZE      128
/*---------- Number of Bytes reserved for HCI Max Payload -----------*/
	#define HCI_MAX_PAYLOAD_SIZE      128
/*---------- Number of incoming packets added to the list of packets to read -----------*/
	#define HCI_READ_PACKET_NUM_MAX      32
#else
/*---------- Number of Bytes reserved for HCI Read Packet -----------*/
	#define HCI_READ_PACKET_SIZE      256
/*---------- Number of Bytes reserved for HCI Max Payload -----------*/
	#define HCI_MAX_PAYLOAD_SIZE      256
/*---------- Number of incoming packets added to the list of packets to read -----------*/
	#define HCI_READ_PACKET_NUM_MAX      64
#endif
/*---------- Scan Interval: time interval from when the Controller started its last scan until it begins the subsequent scan (for a number N, Time = N x 0.625 msec) -----------*/
#define SCAN_P      16384
/*---------- Scan Window: amount of time for the duration of the LE scan (for a number N, Time = N x 0.625 msec) -----------*/
#define SCAN_L      16384
/*---------- Supervision Timeout for the LE Link (for a number N, Time = N x 10 msec) -----------*/
#define SUPERV_TIMEOUT      60
/*---------- Minimum Connection Period (for a number N, Time = N x 1.25 msec) -----------*/
#define CONN_P1      40
/*---------- Maximum Connection Period (for a number N, Time = N x 1.25 msec) -----------*/
#define CONN_P2      40
/*---------- Minimum Connection Length (for a number N, Time = N x 0.625 msec) -----------*/
#define CONN_L1      2000
/*---------- Maximum Connection Length (for a number N, Time = N x 0.625 msec) -----------*/
#define CONN_L2      2000
/*---------- Advertising Type -----------*/
#if BEACON_APP
	#define ADV_DATA_TYPE	ADV_NONCONN_IND
/*---------- Minimum Advertising Interval (for a number N, Time = N x 0.625 msec) -----------*/
	#define ADV_INTERV_MIN      1600	//1 sec
//	#define ADV_INTERV_MIN      16384	//10,24 sec (It is the Max Value, 16384, 0x4000)
/*---------- Maximum Advertising Interval (for a number N, Time = N x 0.625 msec) -----------*/
	#define ADV_INTERV_MAX      1600	//1 sec
//	#define ADV_INTERV_MAX      16384	//10,24 sec (It is the Max Value, 16384, 0x4000)
#else
	#define ADV_DATA_TYPE	ADV_IND
/*---------- Minimum Advertising Interval (for a number N, Time = N x 0.625 msec) -----------*/
	#define ADV_INTERV_MIN      1600
/*---------- Maximum Advertising Interval (for a number N, Time = N x 0.625 msec) -----------*/
	#define ADV_INTERV_MAX      1600
#endif
/*---------- Minimum Connection Event Interval (for a number N, Time = N x 1.25 msec) -----------*/
#define L2CAP_INTERV_MIN      9
/*---------- Maximum Connection Event Interval (for a number N, Time = N x 1.25 msec) -----------*/
#define L2CAP_INTERV_MAX      20
/*---------- Timeout Multiplier (for a number N, Time = N x 10 msec) -----------*/
#define L2CAP_TIMEOUT_MULTIPLIER      600
/*---------- HCI Default Timeout -----------*/
#if BEACON_APP
	#define HCI_DEFAULT_TIMEOUT_MS        1000
#else
	#define HCI_DEFAULT_TIMEOUT_MS        20
#endif

#define BLUENRG_memcpy                memcpy
#define BLUENRG_memset                memset
#define BLUENRG_memcmp                memcmp

#if (BLE2_DEBUG == 1)
  #include <stdio.h>
  #define PRINT_DBG(...)              printf(__VA_ARGS__)
#else
  #define PRINT_DBG(...)
#endif

#if PRINT_CSV_FORMAT
  #include <stdio.h>
  #define PRINT_CSV(...)              printf(__VA_ARGS__)
  void print_csv_time(void);
#else
  #define PRINT_CSV(...)
#endif

#if BLUENRG2_DEBUG
  /**
   * User can change here printf with a custom implementation.
   * For example:
   * #define BLUENRG_PRINTF(...)   STBOX1_PRINTF(__VA_ARGS__)
   */
  #include <stdio.h>
  #define BLUENRG_PRINTF(...)         printf(__VA_ARGS__)
#else
  #define BLUENRG_PRINTF(...)
#endif

#ifdef __cplusplus
}
#endif
#endif /* BLUENRG_CONF_H */
