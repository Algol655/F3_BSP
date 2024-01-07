 /***
 
	Description: VEML6075 driver
	License: GNU General Public License
	Maintainer: S54MTB
*/

/******************************************************************************
  * @file    veml6075.h
  * @author  S54MTB
  * @version V1.0.0
  * @date    12-January-2018
  * @brief   veml6075 driver header file
  ******************************************************************************
  * @attention
  *
  * <h2><center>© Copyright (c) 2018 Scidrom 
	* This program is free software: you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation, either version 3 of the License, or
  * (at your option) any later version.
	*
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program.  If not, see <https://www.gnu.org/licenses/>.  *
  ******************************************************************************
  */


#ifndef __VEML6075__
#define __VEML6075__

// User include starts here
#include "port.h"
#include "i2c.h"
// User include ends here

#ifdef __cplusplus
extern "C" {
#endif

/**
* @brief  Error type.
*/
typedef enum {VEML6075_OK = (uint8_t)0, VEML6075_ERROR = !VEML6075_OK} VEML6075_Error_et;

/**!
	* VEML6075 I2C Slave address
	*/
//#define VEML6075_BADDR				0x10	//7-bit unshifted I2C Address
#define VEML6075_BADDR				0x20	//7-bit shifted I2C Address

/**!
	* VEML6075 data registers
	*/
#define VEML6075_UV_CONF_REG		(uint8_t)0x00
#define VEML6075_UVA_DATA_REG		(uint8_t)0x07
#define VEML6075_DARK_DATA_REG    	(uint8_t)0x08
#define VEML6075_UVB_DATA_REG		(uint8_t)0x09
#define VEML6075_UVCOMP1_DATA_REG	(uint8_t)0x0a
#define VEML6075_UVCOMP2_DATA_REG	(uint8_t)0x0b
#define VEML6075_ID_REG				(uint8_t)0x0c

#define VEML6075_ID_REGRESPONSE		0x0026

/**!
	*	Register values define : CONF */
#define VEML6075_CONF_SD 			0x01
#define VEML6075_CONF_UV_AF_AUTO	0x00
#define VEML6075_CONF_UV_AF_FORCE	0x02
#define VEML6075_CONF_UV_TRIG_NO	0x00
#define VEML6075_CONF_UV_TRIG_ONCE	0x04
#define VEML6075_CONF_HD			0x08
#define VEML6075_CONF_UV_IT_MASK	0x70
#define VEML6075_CONF_UV_IT_50MS	0x00
#define VEML6075_CONF_UV_IT_100MS	0x10
#define VEML6075_CONF_UV_IT_200MS	0x20
#define VEML6075_CONF_UV_IT_400MS	0x30
#define VEML6075_CONF_UV_IT_800MS	0x40
#define VEML6075_CONF_DEFAULT (VEML6075_CONF_UV_AF_AUTO | VEML6075_CONF_UV_TRIG_NO | VEML6075_CONF_UV_IT_100MS)


// Taken from application note:
// http://www.vishay.com/docs/84339/designingveml6075.pdf

#define VEML6075_UVI_UVA_VIS_COEFF	(2.22)
#define VEML6075_UVI_UVA_IR_COEFF	(1.33)
#define VEML6075_UVI_UVB_VIS_COEFF	(2.95)
#define VEML6075_UVI_UVB_IR_COEFF	(1.74)
#define VEML6075_UVI_UVA_RESPONSE	(1.0/909.0)
#define VEML6075_UVI_UVB_RESPONSE	(1.0/800.0)


typedef enum
{
	VEML6075_PARAM_UVA,
	VEML6075_PARAM_UVB,
	VEML6075_PARAM_UVCOMP1,
	VEML6075_PARAM_UVCOMP2,
	VEML6075_PARAM_DARK
} VEML6075_PARAM_t;

typedef struct 
{
	float32_t uva;
	float32_t uvb;
	float32_t UVI;
} VEML6075_MeasureTypeDef_st;

VEML6075_Error_et VEML6075_WhoAmI( I2C_HandleTypeDef *hi2c, uint16_t *idval );
VEML6075_Error_et VEML6075_ShutDown( I2C_HandleTypeDef *hi2c, bool sd ) ;
//VEML6075_Error_et VEML6075_UVVAL( I2C_HandleTypeDef *hi2c, VEML6075_PARAM_t param, uint16_t *val );
VEML6075_Error_et VEML6075_getUVA(I2C_HandleTypeDef *hi2c, float *uva);
VEML6075_Error_et VEML6075_getUVB(I2C_HandleTypeDef *hi2c, float *uvb);
VEML6075_Error_et VEML6075_calculateUVIndex(I2C_HandleTypeDef *hi2c, float *uvindex);
VEML6075_Error_et VEML6075_Get_Measurement(I2C_HandleTypeDef *hi2c, VEML6075_MeasureTypeDef_st *reading);
VEML6075_Error_et MX_VEML6075_Init();

/**
* @brief Registers Init Values.						//Added By Me!!!
* \code
* Values write in the registers in the lps25hb_setup() function
* \endcode
*/
//VEML6075 Register Addresses Values					bit 7 -----> bit 0    | bit functions |
#define VEML6075_UV_CONF_VAL		(uint16_t)0x0000	// Res - UV_IT2 - UV_IT1 - UV_IT0 - HD - UV_TRIG - UV_AF - SD
#define VEML6075_UVA_DATA_VAL		(uint16_t)0x0000	// Read Only
#define VEML6075_DARK_DATA_VAL    	(uint16_t)0x0000	// Read Only
#define VEML6075_UVB_DATA_VAL		(uint16_t)0x0000	// Read Only
#define VEML6075_UVCOMP1_DATA_VAL	(uint16_t)0x0000	// Read Only
#define VEML6075_UVCOMP2_DATA_VAL	(uint16_t)0x0000	// Read Only
#define VEML6075_ID_VAL				(uint16_t)0x0000	// Read Only: 0x0026

#endif

