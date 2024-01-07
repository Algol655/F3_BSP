/**
  ******************************************************************************
  * @file    LPS22HB_Driver.h
  * @author  Sensors Software Solution Team
  * @brief   This file contains all the functions prototypes for the
  *          LPS22HB_Driver.c driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LPS22HB_DRIVER_H
#define LPS22HB_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "port.h"
#include "i2c.h"

/**
  * @brief  Error type.
*/
typedef enum
{
	LPS22HB_OK = (uint8_t)0,
	LPS22HB_ERROR = !LPS22HB_OK
} LPS22HB_Error_et;

typedef struct
{
	uint8_t bit0 : 1;
	uint8_t bit1 : 1;
	uint8_t bit2 : 1;
	uint8_t bit3 : 1;
	uint8_t bit4 : 1;
	uint8_t bit5 : 1;
	uint8_t bit6 : 1;
	uint8_t bit7 : 1;
} Bitwise_t;

#define PROPERTY_DISABLE	(0U)
#define PROPERTY_ENABLE		(1U)

/** @defgroup LPS22HB_Infos
  * @{
  *
  */

//	LPS22HB Measure Type definition.
typedef struct
{
  int16_t Tout;
  int16_t Tout_DailyMin;
  int16_t Tout_DailyMax;
  int32_t Pout;
  int32_t Pout_DailyMin;
  int32_t Pout_DailyMax;
} LPS22HB_MeasureTypeDef_st;

//	LPS22HB Driver Version Info structure definition.
#define LPS22HB_DriverVersion_Major (uint8_t)1
#define LPS22HB_DriverVersion_Minor (uint8_t)2
#define LPS22HB_DriverVersion_Point (uint8_t)5
typedef struct
{
  uint8_t   Major;
  uint8_t   Minor;
  uint8_t 	Point;
} LPS22HB_DriverVersion_st;

//	Bitfield positioning.
#define LPS22HB_BIT(x) ((uint8_t)x)

/** I2C Device Address 8 bit format: SA0=0 **/
#define LPS22HB_I2C_ADD_H	0xB9U
#define LPS22HB_I2C_ADD_L	0xB8U
#define LPS22HB_BADDR		0xB8U
/** I2C Device Address 8 bit format: SA0=1 **/
//#define LPS22HB_I2C_ADD_H	0xBBU
//#define LPS22HB_I2C_ADD_L	0xBAU
//#define LPS22HB_BADDR		0xBAU

/** Device Identification (Who am I) **/
#define LPS22HB_ID			0xB1U

/**
  * @}
  *
  */

#define LPS22HB_INTERRUPT_CFG  0x0BU
typedef struct
{
	uint8_t pe        : 2; /* ple + phe -> pe */
	uint8_t lir       : 1;
	uint8_t diff_en   : 1;
	uint8_t reset_az  : 1;
	uint8_t autozero  : 1;
	uint8_t reset_arp : 1;
	uint8_t autorifp  : 1;
} LPS22HB_InterruptCfg_t;

#define LPS22HB_THS_P_L  		0x0CU
#define LPS22HB_THS_P_H  		0x0DU
#define LPS22HB_WHO_AM_I 		0x0FU
#define LPS22HB_CTRL_REG1		0x10U
typedef struct
{
	uint8_t sim         : 1;
	uint8_t bdu         : 1;
	uint8_t lpfp        : 2; /* en_lpfp + lpfp_cfg -> lpfp */
	uint8_t odr         : 3;
	uint8_t not_used_01 : 1;
} LPS22HB_CtrlReg1_t;

#define LPS22HB_CTRL_REG2		0x11U
typedef struct
{
	uint8_t one_shot   	: 1;
	uint8_t not_used_01	: 1;
	uint8_t swreset    	: 1;
	uint8_t i2c_dis    	: 1;
	uint8_t if_add_inc 	: 1;
	uint8_t stop_on_fth	: 1;
	uint8_t fifo_en    	: 1;
	uint8_t boot       	: 1;
} LPS22HB_CtrlReg2_t;

#define LPS22HB_CTRL_REG3		0x12U
typedef struct
{
	uint8_t int_s   : 2;
	uint8_t drdy    : 1;
	uint8_t f_ovr   : 1;
	uint8_t f_fth   : 1;
	uint8_t f_fss5  : 1;
	uint8_t pp_od   : 1;
	uint8_t int_h_l : 1;
} LPS22HB_CtrlReg3_t;


#define LPS22HB_FIFO_CTRL		0x14U
typedef struct
{
	uint8_t wtm    : 5;
	uint8_t f_mode : 3;
} LPS22HB_FifoCtrl_t;

#define LPS22HB_REF_P_XL		0x15U
#define LPS22HB_REF_P_L 		0x16U
#define LPS22HB_REF_P_H 		0x17U
#define LPS22HB_RPDS_L  		0x18U
#define LPS22HB_RPDS_H  		0x19U
		
#define LPS22HB_RES_CONF		0x1AU
typedef struct
{
	uint8_t lc_en       : 1;
	uint8_t not_used_01 : 7;
} LPS22HB_ResConf_t;

#define LPS22HB_INT_SOURCE		0x25U
typedef struct
{
	uint8_t ph          : 1;
	uint8_t pl          : 1;
	uint8_t ia          : 1;
	uint8_t not_used_01 : 4;
	uint8_t boot_status : 1;
} LPS22HB_IntSource_t;

#define LPS22HB_FIFO_STATUS		0x26U
typedef struct
{
	uint8_t fss      : 6;
	uint8_t ovr      : 1;
	uint8_t fth_fifo : 1;
} LPS22HB_FifoStatus_t;

#define LPS22HB_STATUS			0x27U
typedef struct
{
	uint8_t p_da        : 1;
	uint8_t t_da        : 1;
	uint8_t not_used_02 : 2;
	uint8_t p_or        : 1;
	uint8_t t_or        : 1;
	uint8_t not_used_01 : 2;
} LPS22HB_Status_t;

#define LPS22HB_PRESS_OUT_XL	0x28U
#define LPS22HB_PRESS_OUT_L 	0x29U
#define LPS22HB_PRESS_OUT_H 	0x2AU
#define LPS22HB_TEMP_OUT_L  	0x2BU
#define LPS22HB_TEMP_OUT_H  	0x2CU
#define LPS22HB_LPFP_RES    	0x33U

/**
  * @defgroup LPS22HB_Register_Union
  * @brief    This union group all the registers having a bit-field
  *           description.
  *           This union is useful but it's not needed by the driver.
  *           REMOVING this union you are compliant with:
  *           MISRA-C 2012 [Rule 19.2] -> " Union are not allowed "
  * @{
  */
typedef union
{
	LPS22HB_InterruptCfg_t interrupt_cfg;
	LPS22HB_CtrlReg1_t     ctrl_reg1;
	LPS22HB_CtrlReg2_t     ctrl_reg2;
	LPS22HB_CtrlReg3_t     ctrl_reg3;
	LPS22HB_FifoCtrl_t     fifo_ctrl;
	LPS22HB_ResConf_t      res_conf;
	LPS22HB_IntSource_t    int_source;
	LPS22HB_FifoStatus_t   fifo_status;
	LPS22HB_Status_t       status;
	Bitwise_t              bitwise;
	uint8_t                byte;
} LPS22HB_Driver_t;
/**
  * @}
  *
  */

#ifndef __weak
#define __weak __attribute__((weak))
#endif /* __weak */

/*
 * These are the basic platform dependent I/O routines to read
 * and write device registers connected on a standard bus.
 * The driver keeps offering a default implementation based on function
 * pointers to read/write routines for backward compatibility.
 * The __weak directive allows the final application to overwrite
 * them with a custom implementation.
 */
LPS22HB_Error_et LPS22HB_ReadReg(uint8_t B_Addr, uint8_t RegAddr, uint8_t *Data, uint16_t Len);
LPS22HB_Error_et LPS22HB_WriteReg(uint8_t B_Addr, uint8_t RegAddr, uint8_t *Data, uint16_t Len);

float32_t LPS22HB_LSB_2_HPa(int32_t lsb);

float32_t LPS22HB_LSB_2_DegC(int16_t lsb);

LPS22HB_Error_et LPS22HB_AutozeroRst_Set(uint8_t B_Addr, uint8_t val);
LPS22HB_Error_et LPS22HB_AutozeroRst_Get(uint8_t B_Addr, uint8_t *val);

LPS22HB_Error_et LPS22HB_Autozero_Set(uint8_t B_Addr, uint8_t val);
LPS22HB_Error_et LPS22HB_Autozero_Get(uint8_t B_Addr, uint8_t *val);

LPS22HB_Error_et LPS22HB_PressSnapRst_Set(uint8_t B_Addr, uint8_t val);
LPS22HB_Error_et LPS22HB_PressSnapRst_Get(uint8_t B_Addr, uint8_t *val);

LPS22HB_Error_et LPS22HB_PressSnap_Set(uint8_t B_Addr, uint8_t val);
LPS22HB_Error_et LPS22HB_PressSnap_Get(uint8_t B_Addr, uint8_t *val);

LPS22HB_Error_et LPS22HB_BlockDataUpdate_Set(uint8_t B_Addr, uint8_t val);
LPS22HB_Error_et LPS22HB_BlockDataUpdate_Get(uint8_t B_Addr, uint8_t *val);

typedef enum
{
	LPS22HB_LPF_ODR_DIV_2  = 0,
	LPS22HB_LPF_ODR_DIV_9  = 2,
	LPS22HB_LPF_ODR_DIV_20 = 3,
} LPS22HB_LPFp_t;
LPS22HB_Error_et LPS22HB_LPF_Mode_Set(uint8_t B_Addr, LPS22HB_LPFp_t val);
LPS22HB_Error_et LPS22HB_LPF_Mode_Get(uint8_t B_Addr, LPS22HB_LPFp_t *val);

typedef enum
{
	LPS22HB_POWER_DOWN  = 0,
	LPS22HB_ODR_1_Hz    = 1,
	LPS22HB_ODR_10_Hz   = 2,
	LPS22HB_ODR_25_Hz   = 3,
	LPS22HB_ODR_50_Hz   = 4,
	LPS22HB_ODR_75_Hz   = 5,
} LPS22HB_ODR_t;
LPS22HB_Error_et LPS22HB_DataRate_Set(uint8_t B_Addr, LPS22HB_ODR_t val);
LPS22HB_Error_et LPS22HB_DataRate_Get(uint8_t B_Addr, LPS22HB_ODR_t *val);

LPS22HB_Error_et LPS22HB_OneShootTrigger_Set(uint8_t B_Addr, uint8_t val);
LPS22HB_Error_et LPS22HB_OneShootTrigger_Get(uint8_t B_Addr, uint8_t *val);

LPS22HB_Error_et LPS22HB_PressRef_Set(uint8_t B_Addr, int32_t val);
LPS22HB_Error_et LPS22HB_PressRef_Get(uint8_t B_Addr, int32_t *val);

LPS22HB_Error_et LPS22HB_PressOffset_Set(uint8_t B_Addr, int16_t val);
LPS22HB_Error_et LPS22HB_PressOffset_Get(uint8_t B_Addr, int16_t *val);

LPS22HB_Error_et LPS22HB_PressDataReady_Get(uint8_t B_Addr, uint8_t *val);

LPS22HB_Error_et LPS22HB_TempDataReady_Get(uint8_t B_Addr, uint8_t *val);

LPS22HB_Error_et LPS22HB_PressDataOvr_Get(uint8_t B_Addr, uint8_t *val);

LPS22HB_Error_et LPS22HB_TempDataOvr_Get(uint8_t B_Addr, uint8_t *val);

LPS22HB_Error_et LPS22HB_PressRaw_Get(uint8_t B_Addr, uint32_t *buff);

LPS22HB_Error_et LPS22HB_TempRaw_Get(uint8_t B_Addr, int16_t *buff);

LPS22HB_Error_et LPS22HB_LPF_Rst_Get(uint8_t B_Addr, uint8_t *buff);

LPS22HB_Error_et LPS22HB_DeviceId_Get(uint8_t B_Addr, uint8_t *buff);

LPS22HB_Error_et LPS22HB_SwRst_Set(uint8_t B_Addr, uint8_t val);
LPS22HB_Error_et LPS22HB_SwRst_Get(uint8_t B_Addr, uint8_t *val);

LPS22HB_Error_et LPS22HB_Boot_Set(uint8_t B_Addr, uint8_t val);
LPS22HB_Error_et LPS22HB_Boot_Get(uint8_t B_Addr, uint8_t *val);

LPS22HB_Error_et LPS22HB_LowPwr_Set(uint8_t B_Addr, uint8_t val);
LPS22HB_Error_et LPS22HB_LowPwr_Get(uint8_t B_Addr, uint8_t *val);

LPS22HB_Error_et LPS22HB_BootStatus_Get(uint8_t B_Addr, uint8_t *val);

typedef struct
{
	LPS22HB_FifoStatus_t	fifo_status;
	LPS22HB_Status_t		status;
} LPS22HB_DevStat_t;
LPS22HB_Error_et LPS22HB_DevStatus_Get(uint8_t B_Addr, LPS22HB_DevStat_t *val);

typedef enum
{
	LPS22HB_NO_THRESHOLD = 0,
	LPS22HB_POSITIVE     = 1,
	LPS22HB_NEGATIVE     = 2,
	LPS22HB_BOTH         = 3,
} LPS22HB_pe_t;
LPS22HB_Error_et LPS22HB_InterruptThresholdSign_Set(uint8_t B_Addr, LPS22HB_pe_t val);
LPS22HB_Error_et LPS22HB_InterruptThresholdSign_Get(uint8_t B_Addr, LPS22HB_pe_t *val);

typedef enum
{
	LPS22HB_INT_PULSED  = 0,
	LPS22HB_INT_LATCHED = 1,
} LPS22HB_lir_t;
LPS22HB_Error_et LPS22HB_IntNotificationMode_Set(uint8_t B_Addr, LPS22HB_lir_t val);
LPS22HB_Error_et LPS22HB_IntNotificationMode_Get(uint8_t B_Addr, LPS22HB_lir_t *val);

LPS22HB_Error_et LPS22HB_IntGenerationSet(uint8_t B_Addr, uint8_t val);
LPS22HB_Error_et LPS22HB_IntGenerationGet(uint8_t B_Addr, uint8_t *val);

LPS22HB_Error_et LPS22HB_IntThreshold_Set(uint8_t B_Addr, uint16_t val);
LPS22HB_Error_et LPS22HB_IntThreshold_Get(uint8_t B_Addr, uint16_t *val);

typedef enum
{
	LPS22HB_DRDY_OR_FIFO_FLAGS = 0,
	LPS22HB_HIGH_PRES_INT      = 1,
	LPS22HB_LOW_PRES_INT       = 2,
	LPS22HB_EVERY_PRES_INT     = 3,
} LPS22HB_int_s_t;
LPS22HB_Error_et LPS22HB_IntPinMode_Set(uint8_t B_Addr, LPS22HB_int_s_t val);
LPS22HB_Error_et LPS22HB_IntPinMode_Get(uint8_t B_Addr, LPS22HB_int_s_t *val);

LPS22HB_Error_et LPS22HB_DrdyOnInt_Set(uint8_t B_Addr, uint8_t val);
LPS22HB_Error_et LPS22HB_DrdyOnInt_Get(uint8_t B_Addr, uint8_t *val);

LPS22HB_Error_et LPS22HB_FifoOvrOnInt_Set(uint8_t B_Addr, uint8_t val);
LPS22HB_Error_et LPS22HB_FifoOvrOnInt_Get(uint8_t B_Addr, uint8_t *val);

LPS22HB_Error_et LPS22HB_FifoThresholdOnInt_Set(uint8_t B_Addr, uint8_t val);
LPS22HB_Error_et LPS22HB_FifoThresholdOnInt_Get(uint8_t B_Addr, uint8_t *val);

LPS22HB_Error_et LPS22HB_FifoFullOnInt_Set(uint8_t B_Addr, uint8_t val);
LPS22HB_Error_et LPS22HB_FifoFullOnInt_Get(uint8_t B_Addr, uint8_t *val);

typedef enum
{
	LPS22HB_PUSH_PULL  = 0,
	LPS22HB_OPEN_DRAIN = 1,
} LPS22HB_pp_od_t;
LPS22HB_Error_et LPS22HB_PinMode_Set(uint8_t B_Addr, LPS22HB_pp_od_t val);
LPS22HB_Error_et LPS22HB_PinMode_Get(uint8_t B_Addr, LPS22HB_pp_od_t *val);

typedef enum
{
	LPS22HB_ACTIVE_HIGH = 0,
	LPS22HB_ACTIVE_LOW = 1,
} LPS22HB_int_h_l_t;
LPS22HB_Error_et LPS22HB_IntPol_Set(uint8_t B_Addr, LPS22HB_int_h_l_t val);
LPS22HB_Error_et LPS22HB_IntPol_Get(uint8_t B_Addr, LPS22HB_int_h_l_t *val);

LPS22HB_Error_et LPS22HB_IntSource_Get(uint8_t B_Addr, LPS22HB_IntSource_t *val);

LPS22HB_Error_et LPS22HB_IntOnPressHigh_Get(uint8_t B_Addr, uint8_t *val);

LPS22HB_Error_et LPS22HB_IntOnPressLow_Get(uint8_t B_Addr, uint8_t *val);

LPS22HB_Error_et LPS22HB_IntEvent_Get(uint8_t B_Addr, uint8_t *val);

LPS22HB_Error_et LPS22HB_StopOnFifoThreshold_Set(uint8_t B_Addr, uint8_t val);
LPS22HB_Error_et LPS22HB_StopOnFifoThreshold_Get(uint8_t B_Addr, uint8_t *val);

LPS22HB_Error_et LPS22HB_Fifo_Set(uint8_t B_Addr, uint8_t val);
LPS22HB_Error_et LPS22HB_Fifo_Get(uint8_t B_Addr, uint8_t *val);

LPS22HB_Error_et LPS22HB_FifoWatermark_Set(uint8_t B_Addr, uint8_t val);
LPS22HB_Error_et LPS22HB_FifoWatermark_Get(uint8_t B_Addr, uint8_t *val);

typedef enum
{
	LPS22HB_BYPASS_MODE           = 0,
	LPS22HB_FIFO_MODE             = 1,
	LPS22HB_STREAM_MODE           = 2,
	LPS22HB_STREAM_TO_FIFO_MODE   = 3,
	LPS22HB_BYPASS_TO_STREAM_MODE = 4,
	LPS22HB_DYNAMIC_STREAM_MODE   = 6,
	LPS22HB_BYPASS_TO_FIFO_MODE   = 7,
} LPS22HB_f_mode_t;
LPS22HB_Error_et LPS22HB_FifoMode_Set(uint8_t B_Addr, LPS22HB_f_mode_t val);
LPS22HB_Error_et LPS22HB_FifoMode_Get(uint8_t B_Addr, LPS22HB_f_mode_t *val);

LPS22HB_Error_et LPS22HB_FifoDataLevel_Get(uint8_t B_Addr, uint8_t *val);

LPS22HB_Error_et LPS22HB_FifoOvrFlag_Get(uint8_t B_Addr, uint8_t *val);

LPS22HB_Error_et LPS22HB_FifoFthFlag_Get(uint8_t B_Addr, uint8_t *val);

typedef enum
{
	LPS22HB_SPI_4_WIRE = 0,
	LPS22HB_SPI_3_WIRE = 1,
} LPS22HB_sim_t;
LPS22HB_Error_et LPS22HB_SPI_Mode_Set(uint8_t B_Addr, LPS22HB_sim_t val);
LPS22HB_Error_et LPS22HB_SPI_Mode_Get(uint8_t B_Addr, LPS22HB_sim_t *val);

typedef enum
{
	LPS22HB_I2C_ENABLE = 0,
	LPS22HB_I2C_DISABLE = 1,
} LPS22HB_i2c_dis_t;
LPS22HB_Error_et LPS22HB_I2C_Interface_Set(uint8_t B_Addr, LPS22HB_i2c_dis_t val);
LPS22HB_Error_et LPS22HB_I2C_Interface_Get(uint8_t B_Addr, LPS22HB_i2c_dis_t *val);

LPS22HB_Error_et LPS22HB_AutoAddInc_Set(uint8_t B_Addr, uint8_t val);
LPS22HB_Error_et LPS22HB_AutoAddInc_Get(uint8_t B_Addr, uint8_t *val);

/**
  *@}
  *
  */
/**
* @brief Registers Init Values.
* Values write in the registers in the MX_LPS22HB_Init() function
*/
//LPS22HB Register Addresses Values	
#define LPS22HB_INTERRUPT_CFG_VAL	(uint8_t)0x00U	// 0x0B - Interrupt mode for pressure acquisition configuration. (r/w)
#define LPS22HB_THS_P_L_VAL			(uint8_t)0x00U	// 0x0C - User-defined threshold value for pressure interrupt event (Least significant bits). (r/w)
#define LPS22HB_THS_P_H_VAL			(uint8_t)0x00U	// 0x0D - User-defined threshold value for pressure interrupt event (Most significant bits). (r/w)
#define LPS22HB_WHO_AM_I_VAL		(uint8_t)0xB1U	// 0x0F - Device Who am I (r)
#define LPS22HB_CTRL_REG1_VAL		(uint8_t)0x1EU	// 0x10 - Control register 1 (r/w)
#define LPS22HB_CTRL_REG2_VAL		(uint8_t)0x90U	// 0x11 - Control register 2 (r/w)
#define LPS22HB_CTRL_REG3_VAL		(uint8_t)0x00U	// 0x12 - Control register 3 - INT_DRDY pin control register (r/w)
#define LPS22HB_FIFO_CTRL_VAL		(uint8_t)0x00U	// 0x14 - FIFO control register (r/w)
#define LPS22HB_REF_P_XL_VAL		(uint8_t)0x00U	// 0x15 - Reference pressure (LSB data) (r/w)
#define LPS22HB_REF_P_L_VAL			(uint8_t)0x00U	// 0x16 - Reference pressure (middle part) (r/w)
#define LPS22HB_REF_P_H_VAL			(uint8_t)0x00U	// 0x17 - Reference pressure (MSB part) (r/w)
#define LPS22HB_RPDS_L_VAL			(uint8_t)0x00U	// 0x18 - Pressure offset (LSB data) (r/w)
#define LPS22HB_RPDS_H_VAL			(uint8_t)0x00U	// 0x19 - Pressure offset (MSB data) (r/w)
#define LPS22HB_RES_CONF_VAL		(uint8_t)0x00U	// 0x1A - Low-power mode configuration. Read this register before writing it in order to know bit1 and not to change it (r/w)
#define LPS22HB_INT_SOURCE_VAL		(uint8_t)0x25U	// 0x25 - Interrupt source (r)
#define LPS22HB_FIFO_STATUS_VAL		(uint8_t)0x26U	// 0x26 - FIFO_STATUS (r)
#define LPS22HB_STATUS_VAL			(uint8_t)0x27U	// 0x27 - STATUS register (r)
#define LPS22HB_PRESS_OUT_XL_VAL	(uint8_t)0x28U	// 0x28 - Pressure output value (LSB) (r)
#define LPS22HB_PRESS_OUT_L_VAL		(uint8_t)0x29U	// 0x29 - Pressure output value (mid part) (r)
#define LPS22HB_PRESS_OUT_H_VAL		(uint8_t)0x2AU	// 0x2A - Pressure output value (MSB) (r)
#define LPS22HB_TEMP_OUT_L_VAL		(uint8_t)0x2BU	// 0x2B - Temperature output value (LSB) (r)
#define LPS22HB_TEMP_OUT_H_VAL		(uint8_t)0x2CU	// 0x2C - Temperature output value (MSB) (r)
#define LPS22HB_LPFP_RES_VAL		(uint8_t)0x33U	// 0x33 - Low-pass filter reset register. (r)

/**
* @}brief LPS22HB Pressure & Temperature Min Max Init Values.	//Added By Me!!!
*/
#define LPS22HB_UPPER_P_LIMIT	(1100*100)
#define LPS22HB_LOWER_P_LIMIT	(500*100)
#define LPS22HB_UPPER_T_LIMIT	(80*10)
#define LPS22HB_LOWER_T_LIMIT	(-40*10)

/** @defgroup LPS22HB_My_Function_Prototypes
* @{
*/
LPS22HB_Error_et MX_LPS22HB_Init(void);
LPS22HB_Error_et LPS22HB_Get_Measurement(uint8_t B_Addr, LPS22HB_MeasureTypeDef_st *Measurement_Value);
LPS22HB_Error_et LPS22HB_Set_Reference_Pressure(uint8_t B_Addr, int32_t P_ref);
/**
* @}
*/

#ifdef __cplusplus
}
#endif

#endif /* LPS22HB_DRIVER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
