/**
******************************************************************************
* File Name          : CCS811.h
* Description        : This file contains the common defines of the application
******************************************************************************
* Created by Akash kapashia, 2017
* Modified by Tommaso Sabatini 2019
******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __CCS811_BASIC_H
#define __CCS811_BASIC_H

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
// User include starts here
#include "port.h"
#include "i2c.h"
// User include ends here

/************************************************************************/
/* CCS811 REGISTER DEFINITIONS                                          */
/************************************************************************/

//CCS811 Register Addresses							bit  7 -----> bit  0    | bit functions | R = Reserved
//													byte 7 -----> byte 0 if reg address is a multi-byte register (Mailbox)
#define CCS811_STATUS			((uint8_t)0x00)		// FW_MODE - APP_ERASE - APP_VERIFY - APP_VALID - DATA_READY - R - R - ERROR
#define CCS811_MEAS_MODE		((uint8_t)0x01)		// R - DRIVE_MODE - DRIVE_MODE - DRIVE_MODE - INTERRUPT - THRESH - R - R
#define CCS811_ALG_RESULT_DATA	((uint8_t)0x02)		// RAW_DATA - RAW_DATA - ERROR_ID - STATUS - eTVOC_L - eTVOC_H - eCO2_L - eCO2_H 
#define CCS811_RAW_DATA			((uint8_t)0x03)		// Byte0[7..2]: Current Selected 5:0 - Byte0[1..0], Byte1[7..0]: Raw ADC reading 9:0
#define CCS811_ENV_DATA			((uint8_t)0x05)		// Environment Data Register Base Address
#define CCS811_ENV_DATA_0		((uint8_t)0x05)		// Byte0[7..1]: Humidity % - Byte0[0]: 1/2 Humidity % Fraction
#define CCS811_ENV_DATA_1		((uint8_t)0x06)		// Byte1[7..0]: Humidity % Fraction
#define CCS811_ENV_DATA_2		((uint8_t)0x07)		// Byte2[7..1]: Temperature 25°C - Byte0[0]: 1/2 Temperature 25°C Fraction
#define CCS811_ENV_DATA_3		((uint8_t)0x08)		// Byte3[7..0]: Temperature 25°C Fraction
#define CCS811_THRESHOLDS		((uint8_t)0x10)		// Byte0: Low to Medium Threshold_H - Byte1: Low to Medium Threshold_L
													// Byte2: Medium to High Threshold_H - Medium to High Threshold_L
#define CCS811_BASELINE			((uint8_t)0x11)		// 
#define CCS811_HW_ID			((uint8_t)0x20)		// 1 - 0 - 0 - 0 - 0 - 0 - 0 - 1
#define CCS811_HW_VERSION		((uint8_t)0x21)		// 0x1X
#define CCS811_FW_BOOT_VERSION	((uint8_t)0x23)		// Byte[0..1]
#define CCS811_FW_APP_VERSION	((uint8_t)0x24)		// Byte[0..1]
#define CCS811_ERROR_ID			((uint8_t)0xE0)		// R - R - HEATER_SUPPLY - HEATER_FAULT - MAX_RES - MEASMODE_INV - READ_REG_INV - WRITE_REG_INV
#define CCS811_APP_START		((uint8_t)0xF4)		// 
#define CCS811_SW_RESET			((uint8_t)0xFF)		// 0x11 0xE5 0x72 0x8A

//CCS811 Register Default Values					bit 7 -----> bit 0    | bit functions |
//													byte 7 -----> byte 0 if reg address is a multi-byte register (Mailbox)
#define CCS811_MEAS_MODE_VAL	((uint8_t)0x10)		// 0x01 R - DRIVE_MODE - DRIVE_MODE - DRIVE_MODE - INTERRUPT - THRESH - R - R
#define CCS811_ENV_DATA_VAL_0	((uint8_t)0x00)		// 0x05 Byte0[7..1]: Humidity % - Byte0[0]: 1/2 Humidity % Fraction
#define CCS811_ENV_DATA_VAL_1	((uint8_t)0x00)		// 0x06 Byte1[7..0]: Humidity % Fraction
#define CCS811_ENV_DATA_VAL_2	((uint8_t)0x00)		// 0x07 Byte2[7..1]: Temperature 25°C - Byte0[0]: 1/2 Temperature 25°C Fraction
#define CCS811_ENV_DATA_VAL_3	((uint8_t)0x00)		// 0x08 Byte3[7..0]: Temperature 25°C Fraction
#define CCS811_THRESHOLDS_VAL	((uint32_t)0x00000000)	// Byte0: Low to Medium Threshold_H - Byte1: Low to Medium Threshold_L
#define CCS811_APP_START_VAL	((uint8_t)0xF4)		//
													// Byte2: Medium to High Threshold_H - Medium to High Threshold_L
#define CCS811_SW_RESET_VAL		((uint32_t)0x8A72E511)	// 0x11 0xE5 0x72 0x8A

#define CCS811_WHO_AM_I_VAL		((uint8_t)0x81)

//#define CCS811_BADDR 			((uint8_t)0x5A) 	//7-bit unshifted default I2C Address
#define CCS811_BADDR 			((uint8_t)0xB4) 	//7-bit shifted default I2C Address

//#define APPLICATION_RUN_CYCLE	  (5)	//Moved in port.h
#define BURN_IN_TIME              ((48*60*60)/APPLICATION_RUN_CYCLE)//48 Hours
#define RUN_IN_TIME               ((40*60)/APPLICATION_RUN_CYCLE)//40 Minutes
#define NEW_MODE_RUN_IN_TIME      ((10*60)/APPLICATION_RUN_CYCLE)//10 Minutes
#define BASELINE_EARLYLIFE_PERIOD ((500*60*60)/APPLICATION_RUN_CYCLE)//500 Hours
#define BASELINE_EL_STORE_PERIOD  ((24*60*60)/APPLICATION_RUN_CYCLE)//24 Hours
#define BASELINE_AEL_STORE_PERIOD ((7*BASELINE_EL_STORE_PERIOD)/APPLICATION_RUN_CYCLE)
#define CALIB_TEMP_HUM            ((30*60)/APPLICATION_RUN_CYCLE)//30 Minutes

typedef enum
{
	CCS811_OK	 = 0x00,
	CCS811_ERROR = 0x01
} CCS811_Error_et;

typedef struct
{
	uint16_t	eCO2;
	uint16_t	eCO2_mean;	//8h average
	uint16_t	eTVOC;
	uint16_t	eTVOC_mean;	//1h average
	uint8_t		Status;
	uint8_t		ErrorID;
	uint16_t	RawData;
} CCS811_MeasureTypeDef_st;

bool Store_CCS811_Baseline;

/** @defgroup CCS811_My_Function_Prototypes
* @{
*/
extern void Sleep(uint32_t Delay);
//extern void Write_Flash(uint32_t data, uint8_t);
CCS811_Error_et	CCS811_ReadReg(uint8_t B_Addr, uint8_t RegAddr, uint16_t NumByteToRead, uint8_t *Data);
CCS811_Error_et	CCS811_WriteReg(uint8_t B_Addr, uint8_t RegAddr, uint16_t NumByteToWrite, uint8_t *Data);
CCS811_Error_et	CCS811_WriteReg_DMA(uint8_t B_Addr, uint8_t RegAddr, uint16_t NumByteToWrite, uint8_t *Data);
CCS811_Error_et	CCS811_Init(uint8_t B_Addr, uint8_t RegAddr, uint8_t *RegValues, uint8_t Size);
CCS811_Error_et	MX_CCS811_Init(void);
CCS811_Error_et setDriveMode(uint8_t B_Addr, uint8_t mode);
CCS811_Error_et	CCS811_Get_Measurement(CCS811_MeasureTypeDef_st *Measurement_Value);
CCS811_Error_et	CCS811_SetEnvironmentalData(float relativeHumidity, float temperature);
CCS811_Error_et	CCS811_SoftRST(void);
CCS811_Error_et	CCS811_Sleep(void);
CCS811_Error_et	CCS811_Save_Baseline(bool SaveToFlash);
CCS811_Error_et	CCS811_Restore_Baseline(bool RestoreFromFlash);
FlagStatus checkForError(void);
uint32_t CCS811_Get_Sensor_Resistance(void);
/**
* @}
*/

/*
unsigned int getBaseline(void);
FlagStatus dataAvailable(void);
void enableInterrupts(void);
void disableInterrupts(void);
void sleep(void);
uint32_t get_Sensor_Resistance(void);
void restore_Baseline(void); */

#ifdef __cplusplus
}
#endif

#endif  
