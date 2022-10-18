/**
******************************************************************************
* File Name          : ENS160.h
* Description        : This file contains the common defines of the application
******************************************************************************
* Created by Tommaso Sabatini 2022
******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __ENS160_BASIC_H
#define __ENS160_BASIC_H

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
// User include starts here
#include "port.h"
#include "i2c.h"
// User include ends here

/************************************************************************/
/* ENS160 REGISTER DEFINITIONS                                          */
/************************************************************************/

//ENS160 Register Addresses							bit  7 -----> bit  0    | bit functions | R = Reserved
//													byte 7 -----> byte 0 if reg address is a multi-byte register (Mailbox)
#define ENS160_PART_ID			((uint8_t)0x00)		// [7..0]: Lower Byte of Part ID. [15..8]: Upper Byte of Part ID
#define ENS160_OPMODE			((uint8_t)0x10)		// Operating mode: 0x00: DEEP SLEEP mode. 0x01: IDLE mode: 0x02: STANDARD Gas Sensing Mode. 0xF0: RESET
#define ENS160_CONFIG			((uint8_t)0x11)		// R - INTPOL - INT_CFG - R - INTGPR - R - INTDAT - INTEN
#define ENS160_COMMAND			((uint8_t)0x12)		// [7..0]: Command
#define ENS160_TEMP_IN			((uint8_t)0x13)		// [7..0]: Lower Byte of TEMP_IN. [15..8]: Upper Byte of TEMP_IN
#define ENS160_RH_IN			((uint8_t)0x15)		// [7..0]: Lower Byte of RH_IN. [15..8]: Upper Byte of RH_IN
#define ENS160_DEVICE_STATUS	((uint8_t)0x20)		// STATAS - STATER - R - R - VALIDITY FLAG - NEWDAT - NEWGPR
#define ENS160_DATA_AQI			((uint8_t)0x21)		// [2..0]: Air Quality Index according to UBA [1..5]. [7..3]: Reserved
#define ENS160_DATA_TVOC		((uint8_t)0x22)		// [7..0]: Lower Byte of DATA_TVOC. [15..8]: Upper Byte of DATA_TVOC
//#define ENS160_DATA_ETOH		((uint8_t)0x22)		// [7..0]: Lower Byte of DATA_ETH. [15..8]: Upper Byte of DATA_ETH
#define ENS160_DATA_ECO2		((uint8_t)0x24)		// [7..0]: Lower Byte of DATA_ECO2. [15..8]: Upper Byte of DATA_ECO2
#define ENS160_DATA_BL			((uint8_t)0x28)		// [7..0]: Lower Byte of DATA_ETOH. [15..8]: Upper Byte of DATA_ETOH
#define ENS160_DATA_T			((uint8_t)0x30)		// [7..0]: Lower Byte of DATA_T. [15..8]: Upper Byte of DATA_T
#define ENS160_DATA_RH			((uint8_t)0x32)		// [7..0]: Lower Byte of DATA_RH. [15..8]: Upper Byte of DATA_RH
#define ENS160_DATA_MISR		((uint8_t)0x38)		// [7..0]: DATA_MISR Calculated checksum of the previous transaction
#define ENS160_GPR_WRITE_0		((uint8_t)0x40)		// General Purpose WRITE Register Byte 0
#define ENS160_GPR_WRITE_1		((uint8_t)0x41)		// General Purpose WRITE Register Byte 1
#define ENS160_GPR_WRITE_2		((uint8_t)0x42)		// General Purpose WRITE Register Byte 2
#define ENS160_GPR_WRITE_3		((uint8_t)0x43)		// General Purpose WRITE Register Byte 3
#define ENS160_GPR_WRITE_4		((uint8_t)0x44)		// General Purpose WRITE Register Byte 4
#define ENS160_GPR_WRITE_5		((uint8_t)0x45)		// General Purpose WRITE Register Byte 5
#define ENS160_GPR_WRITE_6		((uint8_t)0x46)		// General Purpose WRITE Register Byte 6
#define ENS160_GPR_WRITE_7		((uint8_t)0x47)		// General Purpose WRITE Register Byte 7

#define ENS160_GPR_READ_0		((uint8_t)0x48)		// General Purpose READ Register Byte 0
#define ENS160_GPR_READ_4		((uint8_t)0x4C)		// General Purpose READ Register Byte 4
#define ENS160_GPR_READ_6		((uint8_t)0x4E)		// General Purpose READ Register Byte 6
#define ENS160_GPR_READ_7		((uint8_t)0x4F)		// General Purpose READ Register Byte 7

//ENS160 Register Default Values					bit 7 -----> bit 0    | bit functions |
//													byte 7 -----> byte 0 if reg address is a multi-byte register (Mailbox)
#define ENS160_OPMODE_VAL		((uint8_t)0x02)		// Operating mode: 0x00: DEEP SLEEP mode. 0x01: IDLE mode: 0x02: STANDARD Gas Sensing Mode. 0xF0: RESET
#define ENS160_CONFIG_VAL		((uint8_t)0x00)		// R - INTPOL - INT_CFG - R - INTGPR - R - INTDAT - INTEN
#define ENS160_COMMAND_VAL		((uint8_t)0x00)		// [7..0]: Command
#define ENS160_TEMP_IN_VAL_0	((uint8_t)0x4A)		// Lower Byte of TEMP_IN	((20°C + 273.15) * 64)
#define ENS160_TEMP_IN_VAL_1	((uint8_t)0x49)		// Upper Byte of TEMP_IN	((20°C + 273.15) * 64)
#define ENS160_RH_IN_VAL_0		((uint8_t)0x00)		// Lower Byte of RH_IN	(50% * 512)
#define ENS160_RH_IN_VAL_1		((uint8_t)0x64)		// Upper Byte of RH_IN	(50% * 512)
#define ENS160_GPR_WRITE_0_VAL	((uint8_t)0x00)		// General Purpose WRITE Register bit 0
#define ENS160_GPR_WRITE_1_VAL	((uint8_t)0x00)		// General Purpose WRITE Register bit 1
#define ENS160_GPR_WRITE_2_VAL	((uint8_t)0x00)		// General Purpose WRITE Register bit 2
#define ENS160_GPR_WRITE_3_VAL	((uint8_t)0x00)		// General Purpose WRITE Register bit 3
#define ENS160_GPR_WRITE_4_VAL	((uint8_t)0x00)		// General Purpose WRITE Register bit 4
#define ENS160_GPR_WRITE_5_VAL	((uint8_t)0x00)		// General Purpose WRITE Register bit 5
#define ENS160_GPR_WRITE_6_VAL	((uint8_t)0x00)		// General Purpose WRITE Register bit 6
#define ENS160_GPR_WRITE_7_VAL	((uint8_t)0x00)		// General Purpose WRITE Register bit 7

#define ENS160_WHO_AM_I_VAL		((uint16_t)0x0160)
//#define ENS160_BADDR 			((uint8_t)0x53) 	//7-bit unshifted default I2C Address
#define ENS160_BADDR 			((uint8_t)0xA6) 	//7-bit shifted default I2C Address

//ENS160 data register fields
#define ENS160_COMMAND_NOP			((uint8_t)0x00)
#define ENS160_COMMAND_CLRGPR		((uint8_t)0xCC)
#define ENS160_COMMAND_GET_APPVER	((uint8_t)0x0E)
#define ENS160_COMMAND_SETTH		((uint8_t)0x02)
#define ENS160_COMMAND_SETSEQ		((uint8_t)0xC2)

#define ENS160_OPMODE_RESET			((uint8_t)0xF0)
#define ENS160_OPMODE_DEP_SLEEP		((uint8_t)0x00)
#define ENS160_OPMODE_IDLE			((uint8_t)0x01)
#define ENS160_OPMODE_STD			((uint8_t)0x02)
#define ENS160_OPMODE_INTERMEDIATE	((uint8_t)0x03)
#define ENS160_OPMODE_CUSTOM		((uint8_t)0xC0)
#define ENS160_OPMODE_D0			((uint8_t)0xD0)
#define ENS160_OPMODE_D1			((uint8_t)0xD1)
#define ENS160_OPMODE_BOOTLOADER	((uint8_t)0xB0)

#define ENS160_BL_CMD_START			((uint8_t)0x02)
#define ENS160_BL_CMD_ERASE_APP		((uint8_t)0x04)
#define ENS160_BL_CMD_ERASE_BLINE	((uint8_t)0x06)
#define ENS160_BL_CMD_WRITE			((uint8_t)0x08)
#define ENS160_BL_CMD_VERIFY		((uint8_t)0x0A)
#define ENS160_BL_CMD_GET_BLVER		((uint8_t)0x0C)
#define ENS160_BL_CMD_GET_APPVER	((uint8_t)0x0E)
#define ENS160_BL_CMD_EXITBL		((uint8_t)0x12)

#define ENS160_DATA_STATUS_NEWGPR	((uint8_t)0x01)
#define ENS160_DATA_STATUS_NEWDAT	((uint8_t)0x02)

#define CONVERT_RS_RAW2OHMS_I(x) 	(1 << ((x) >> 11))
#define CONVERT_RS_RAW2OHMS_F(x) 	(pow (2, (float)(x) / 2048))

#define BURN_IN_TIME              ((48*60*60)/APPLICATION_RUN_CYCLE)//48 Hours
#define RUN_IN_TIME               ((40*60)/APPLICATION_RUN_CYCLE)//40 Minutes
#define NEW_MODE_RUN_IN_TIME      ((10*60)/APPLICATION_RUN_CYCLE)//10 Minutes
#define BASELINE_EARLYLIFE_PERIOD ((500*60*60)/APPLICATION_RUN_CYCLE)//500 Hours
#define BASELINE_EL_STORE_PERIOD  ((24*60*60)/APPLICATION_RUN_CYCLE)//24 Hours
#define BASELINE_AEL_STORE_PERIOD ((7*BASELINE_EL_STORE_PERIOD)/APPLICATION_RUN_CYCLE)
#define CALIB_TEMP_HUM            ((30*60)/APPLICATION_RUN_CYCLE)//30 Minutes

typedef enum
{
	ENS160_OK	 = 0x00,
	ENS160_ERROR = 0x01
} ENS160_Error_et;

typedef struct
{
	uint16_t	eCO2;
	uint16_t	eCO2_mean;	//8h average
	uint16_t	eTVOC;
	uint16_t	eTVOC_mean;	//1h average
	uint8_t		AQI_UBA;
	uint8_t		Status;
	uint8_t		ErrorID;
	uint16_t	RawData_0;
	uint16_t	RawData_1;
	uint16_t	RawData_2;
	uint16_t	RawData_3;
	uint16_t	BaseLine_0;
	uint16_t	BaseLine_1;
	uint16_t	BaseLine_2;
	uint16_t	BaseLine_3;
	uint8_t		Misr;
	uint16_t	ETH;
	uint16_t	ETH_mean;	//Ethanol 1h average
} ENS160_MeasureTypeDef_st;

bool Store_ENS160_Baseline;

/** @defgroup ENS160_My_Function_Prototypes
* @{
*/
extern void Sleep(uint32_t Delay);
//extern void Write_Flash(uint32_t data, uint8_t);
void eCO2_MovingAverage(uint16_t *in, uint16_t *out, uint16_t length);
void eTVOC_MovingAverage(uint16_t *in, uint16_t *out, uint16_t length);
ENS160_Error_et ENS160_ReadReg(uint8_t B_Addr, uint8_t RegAddr, uint16_t NumByteToRead, uint8_t *Data);
ENS160_Error_et ENS160_WriteReg(uint8_t B_Addr, uint8_t RegAddr, uint16_t NumByteToWrite, uint8_t *Data);
ENS160_Error_et ENS160_WriteReg_DMA(uint8_t B_Addr, uint8_t RegAddr, uint16_t NumByteToWrite, uint8_t *Data);
ENS160_Error_et ENS160_Init(uint8_t B_Addr, uint8_t RegAddr, uint8_t* RegValues, uint8_t Size);
ENS160_Error_et ENS160_Get_Dev(uint8_t B_Addr, uint8_t *buff);
ENS160_Error_et ENS160_Idle_Mode(uint8_t B_Addr);
ENS160_Error_et ENS160_Std_Mode(uint8_t B_Addr);
ENS160_Error_et ENS160_Get_FW_Ver(uint8_t B_Addr, uint8_t *buff, bool Check_FW_Ver);
ENS160_Error_et ENS160_SetEnvironmentalData(float relativeHumidity, float temperature);
ENS160_Error_et	ENS160_Get_Measurement(ENS160_MeasureTypeDef_st *Measurement_Value);
ENS160_Error_et	ENS160_Get_Raw_Data(ENS160_MeasureTypeDef_st *Measurement_Value);
ENS160_Error_et	ENS160_SoftRST();
ENS160_Error_et MX_ENS160_Init();
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
