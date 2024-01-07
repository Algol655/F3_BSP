/*
 * VEML7700_Driver.h
 *
 *  Created on: Sep 16, 2023
 *      Author: Tommaso Sabatini
 */

#ifndef PLATFORM_VEML7700_DRIVER_H_
#define PLATFORM_VEML7700_DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
// User include starts here
#include "port.h"
#include "i2c.h"
// User include ends here

/**
* Brief  Error type.
*/
typedef enum
{
	VEML7700_OK = (uint8_t)0,
	VEML7700_ERROR = !VEML7700_OK
} VEML7700_Error_et;

/**
* VEML7700 I2C Slave address
*/
//#define VEML7700_BADDR				0x10	//7-bit unshifted I2C Address
#define VEML7700_BADDR				0x20	//7-bit shifted I2C Address

/**
* VEML7700 Command Registers (I2C Register Address offset)
* REGISTER NAME					COMMAND CODE (I2C Register Address)
*/
#define VEML7700_ALS_CONF_0_REG	(uint8_t)0x00	// Light configuration register. r/w
#define VEML7700_ALS_WH_REG		(uint8_t)0x01	// Light high threshold for irq. r/w
#define VEML7700_ALS_WL_REG    	(uint8_t)0x02	// Light low threshold for irq. r/w
#define VEML7700_PWR_SAVE_REG	(uint8_t)0x03	// Power save register. r/w
#define VEML7700_ALS_REG		(uint8_t)0x04	// The light data output. r
#define VEML7700_WHITE_REG		(uint8_t)0x05	// The white light data output. r
#define VEML7700_ALS_INT_REG	(uint8_t)0x06	// What IRQ (if any). r

/**
*	Command Register 0 Values
*/
//ALS_GAIN: Gain Selection. Bit 12..11
#define VEML7700_ALS_GAIN_x1	(uint8_t)0x00	//ALS gain x 1
#define VEML7700_ALS_GAIN_x2	(uint8_t)0x01	//ALS gain x 2
#define VEML7700_ALS_GAIN_d8	(uint8_t)0x02	//ALS gain x 1/8
#define VEML7700_ALS_GAIN_d4	(uint8_t)0x03	//ALS gain x 1/4
#define ALS_SM_MASK 		(uint16_t)0x1800	//Select bits 12..11
#define ALS_SM_SHIFT			(uint8_t)0x0B	//Shift 11 bits
//ALS_IT: ALS integration time setting. Bit 9..6
#define VEML7700_ALS_IT_25ms	(uint8_t)0x0C	//ALS integration time 25ms
#define VEML7700_ALS_IT_50ms	(uint8_t)0x08	//ALS integration time 50ms
#define VEML7700_ALS_IT_100ms	(uint8_t)0x00	//ALS integration time 100ms
#define VEML7700_ALS_IT_200ms	(uint8_t)0x01	//ALS integration time 200ms
#define VEML7700_ALS_IT_400ms	(uint8_t)0x02	//ALS integration time 400ms
#define VEML7700_ALS_IT_800ms	(uint8_t)0x03	//ALS integration time 800ms
#define ALS_IT_MASK 		(uint16_t)0x03C0	//Select bits 9..6
#define ALS_IT_SHIFT			(uint8_t)0x06	//Shift 6 bits
//ALS_PERS: ALS persistence protect number setting. Bit 5..4
#define VEML7700_ALS_PERS_1		(uint8_t)0x00	//ALS irq persistence 1 sample
#define VEML7700_ALS_PERS_2		(uint8_t)0x01	//ALS irq persistence 2 samples
#define VEML7700_ALS_PERS_4		(uint8_t)0x10	//ALS irq persistence 4 samples
#define VEML7700_ALS_PERS_8		(uint8_t)0x11	//ALS irq persistence 8 samples
#define ALS_PERS_MASK 		(uint16_t)0x0030	//Select bits 5..4
#define ALS_PERS_SHIFT			(uint8_t)0x06	//Shift 4 bits
//ALS_PERS: ALS interrupt enable setting. Bit 1
#define VEML7700_ALS_INT_DIS	(uint8_t)0x00
#define VEML7700_ALS_INT_EN		(uint8_t)0x01
#define ALS_INT_EN_MASK 	(uint16_t)0x0002	//Select bits 1
#define ALS_INT_EN_SHIFT		(uint8_t)0x01	//Shift 1 bits
//ALS_PERS: ALS shut down setting. Bit 0
#define VEML7700_ALS_SD_PWR_ON		(uint8_t)0x00
#define VEML7700_ALS_SD_SHUT_DWN	(uint8_t)0x01
#define ALS_SD_MASK 		(uint16_t)0x0001	//Select bit 0
#define ALS_SD_EN_SHIFT			(uint8_t)0x00	//Shift 0 bits

/**
*	Command Register 3 Values
*/
//PSM: Power saving mode; see table “Refresh time”. Bit 2..1
#define VEML7700_ALS_PSM_1		(uint8_t)0x00	//Power saving mode 1
#define VEML7700_ALS_PSM_2		(uint8_t)0x01	//Power saving mode 2
#define VEML7700_ALS_PSM_3		(uint8_t)0x10	//Power saving mode 3
#define VEML7700_ALS_PSM_4		(uint8_t)0x11	//Power saving mode 4
#define ALS_PSM_MASK 		(uint16_t)0x0006	//Select bits 2..1
#define ALS_PSM_SHIFT			(uint8_t)0x01	//Shift 1 bits
//PSM_EN: Power saving mode enable setting. Bit 0
#define VEML7700_ALS_PSM_DIS	(uint8_t)0x00
#define VEML7700_ALS_PSM_EN		(uint8_t)0x01
#define ALS_PSM_EN_MASK 	(uint16_t)0x0001	//Select bit 0
#define ALS_PSM_EN_SHIFT		(uint8_t)0x00	//Shift 0 bits

/**
*	Command Register 6 Values
*/
//IF: Interrupt Status. Bit 15..14
#define ALS_IF_L_MASK 		(uint16_t)0x8000	//Select bits 15
#define ALS_IF_L_SHIF			(uint8_t)0x0F	//Shift 15 bits
#define ALS_IF_H_MASK 		(uint16_t)0x4000	//Select bits 14
#define ALS_IF_H_SHIF			(uint8_t)0x0E	//Shift 14 bits

#define VEML7700_CONF_0_DEFAULT ((VEML7700_ALS_GAIN_x2 << ALS_SM_SHIFT) | \
								(VEML7700_ALS_IT_100ms << ALS_IT_SHIFT) | \
								(VEML7700_ALS_PERS_1 << ALS_PERS_SHIFT) | \
								(VEML7700_ALS_INT_DIS << ALS_INT_EN_SHIFT) | \
								(VEML7700_ALS_SD_SHUT_DWN))
#define VEML7700_CONF_1_DEFAULT	0x0000
#define VEML7700_CONF_2_DEFAULT	0xFFFF
#define VEML7700_CONF_3_DEFAULT ((VEML7700_ALS_PSM_2 << ALS_PSM_SHIFT) | \
								(VEML7700_ALS_PSM_EN))

typedef struct
{
	float corrected_ALS_lux;	//ALS channel that follows the so-called human eye curve very well.
	float white_lux;			//White channel, which offers a much higher responsivity for a much wider wavelength spectrum
	uint16_t raw_ALS;			//Raw ALS channel (Command 0x04)
	uint16_t raw_white;			//Raw white channel (Command 0x05)
} VEML7700_MeasureTypeDef_st;

/** Options for lux reading method */
typedef enum
{
	VEML_RAW_ALS,
	VEML_RAW_WHITE,
	VEML_LUX_NORMAL,
	VEML_LUX_CORRECTED,
	VEML_LUX_AUTO,
	VEML_LUX_NORMAL_NOWAIT,
	VEML_LUX_CORRECTED_NOWAIT
} VEML7700_MeasureMode_t;

/*VEML7700_Error_et VEML7700_readLux(luxMethod method);
VEML7700_Error_et VEML7700_readALS(bool wait);
VEML7700_Error_et VEML7700_readWhite(bool wait);
VEML7700_Error_et VEML7700_enable(bool enable);
VEML7700_Error_et VEML7700_setPersistence(uint8_t pers);
VEML7700_Error_et VEML7700_getPersistence(void);
VEML7700_Error_et VEML7700_Get_Measurement(I2C_HandleTypeDef *hi2c, VEML7700_MeasureTypeDef_st *reading); */
VEML7700_Error_et VEML7700_Geat_Val(I2C_HandleTypeDef *hi2c, VEML7700_MeasureMode_t param, uint16_t *val);
VEML7700_Error_et VEML7700_Get_Measurement(I2C_HandleTypeDef *hi2c, VEML7700_MeasureTypeDef_st *reading);
VEML7700_Error_et MX_VEML7700_Init(void);

/*
VEML7700_Error_et VEML7700_WhoAmI( I2C_HandleTypeDef *hi2c, uint16_t *idval );
VEML7700_Error_et VEML7700_ShutDown( I2C_HandleTypeDef *hi2c, bool sd ) ;
//VEML7700_Error_et VEML7700_UVVAL( I2C_HandleTypeDef *hi2c, VEML7700_PARAM_t param, uint16_t *val );
VEML7700_Error_et VEML7700_getUVA(I2C_HandleTypeDef *hi2c, float *uva);
VEML7700_Error_et VEML7700_getUVB(I2C_HandleTypeDef *hi2c, float *uvb);
VEML7700_Error_et VEML7700_calculateUVIndex(I2C_HandleTypeDef *hi2c, float *uvindex);
VEML7700_Error_et VEML7700_Get_Measurement(I2C_HandleTypeDef *hi2c, VEML7700_MeasureTypeDef_st *reading);
VEML7700_Error_et MX_VEML7700_Init(void); */

/**
* Brief Registers Init Values
*/
//VEML7700 Register Addresses Values				bit    15..0
#define VEML7700_ALS_CONF_0_VAL		(uint16_t)0x8000	// Res-Res-Res - ALS_GAIN - Res - ALS_IT - ALS_PERS - Res-Res - ALS_INT_EN - ALS_SD
#define VEML7700_ALS_WH_VAL			(uint16_t)0x0000	//
#define VEML7700_ALS_WL_VAL    		(uint16_t)0xFFFF	//
#define VEML7700_PWR_SAVE_VAL		(uint16_t)0x0002	// Res-Res-Res-Res-Res-Res-Res-Res-Res-Res-Res-Res - PSM - PSM_EN
#define VEML7700_ALS_VAL			(uint16_t)0x0000	// Read Only
#define VEML7700_WHITE_VAL			(uint16_t)0x0000	// Read Only
#define VEML7700_ALS_INT_VAL		(uint16_t)0x0000	// Read Only

#ifdef __cplusplus
}
#endif

#endif /* PLATFORM_VEML7700_DRIVER_H_ */
