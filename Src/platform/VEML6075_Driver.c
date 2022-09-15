/*** 
	Description: VEML6075 driver
	License: GNU General Public License
	Maintainer: S54MTB
*/

/******************************************************************************
  * @file    VEML6075.c
  * @author  S54MTB
  * @version V1.0.0
  * @date    14-January-2018
  * @brief   VEML6075 driver
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
  * along with this program.  If not, see <https://www.gnu.org/licenses/>.  
  ******************************************************************************
  */

#include "platform/VEML6075_Driver.h"

/*
 * The VEML6075 has 16-bit registers used to set up the measurements as 
 * well as pick up the measurement results.
 */

/*
 * VEML6075_ReadRegister() - Read register from VEML6075
 * @hi2c:  handle to I2C interface
 * @adr: I2C device address
 * @reg: Register address
 * @val: 16-bit register value from the VEML6075
 * Returns HAL status or HAL_ERROR for invalid parameters.
 */
static VEML6075_Error_et VEML6075_ReadRegister(I2C_HandleTypeDef *hi2c,	uint8_t adr, uint8_t reg, uint16_t *regval)
{
	uint8_t val[2];
	VEML6075_Error_et status;
	
	status = HAL_I2C_Mem_Read(hi2c, adr<<1, reg , I2C_MEMADD_SIZE_8BIT, val, 2, 100);

	if (status == VEML6075_OK)
	{
		*regval = val[0] | val[1] << 8;
	}
	return status;
}

/*
 * VEML6075_WriteRegister() - Write VEML6075 register
 * @hi2c:  handle to I2C interface
 * @adr: I2C device address
 * @reg: Register address
 * @val: 8-bit register value from the VEML6075
 * Returns HAL status or HAL_ERROR for invalid parameters.
 */
static VEML6075_Error_et VEML6075_WriteRegister(I2C_HandleTypeDef *hi2c, uint8_t adr, uint8_t reg, uint16_t regval)
{	
	uint8_t val[2];
	val[1] = (regval >> 8) & 0xff;
	val[0] = regval & 0xff;
	VEML6075_Error_et status = HAL_I2C_Mem_Write(hi2c, adr<<1, reg,	I2C_MEMADD_SIZE_8BIT, val, 2, 100);
	return status;
}

/*!
 * \brief Init VEML6075 Sensor - init
 * returns: HAL Status
 */
VEML6075_Error_et VEML6075_Init(void)
{
	uint16_t id;

	VEML6075_ShutDown( &hi2c1, true );
	HAL_Delay(50);
	VEML6075_ShutDown( &hi2c1, false );
	HAL_Delay(500);
	if (VEML6075_WriteRegister(&hi2c1, VEML6075_BADDR, VEML6075_UV_CONF_REG, VEML6075_UV_CONF_VAL))
		return VEML6075_ERROR;
	VEML6075_WhoAmI( &hi2c1, &id );

	return (id == VEML6075_ID_REGRESPONSE) ? VEML6075_OK : VEML6075_ERROR;
}

VEML6075_Error_et MX_VEML6075_Init()
{
	if (VEML6075_Init())
		return VEML6075_ERROR;
	return VEML6075_OK;
}

VEML6075_Error_et VEML6075_WhoAmI( I2C_HandleTypeDef *hi2c, uint16_t *idval )
{
	return VEML6075_ReadRegister(hi2c, VEML6075_BADDR, VEML6075_ID_REG, idval); 
}

VEML6075_Error_et VEML6075_UVVAL( I2C_HandleTypeDef *hi2c, VEML6075_PARAM_t param, uint16_t *val )
{
	switch (param)
	{
		case VEML6075_PARAM_UVA:
			return VEML6075_ReadRegister(hi2c, VEML6075_BADDR, VEML6075_UVA_DATA_REG, val); 
		//break;

		case VEML6075_PARAM_UVB:
			return VEML6075_ReadRegister(hi2c, VEML6075_BADDR, VEML6075_UVB_DATA_REG, val); 
		//break;

		case VEML6075_PARAM_UVCOMP1:
			return VEML6075_ReadRegister(hi2c, VEML6075_BADDR, VEML6075_UVCOMP1_DATA_REG, val); 
		//break;

		case VEML6075_PARAM_UVCOMP2:
			return VEML6075_ReadRegister(hi2c, VEML6075_BADDR, VEML6075_UVCOMP2_DATA_REG, val); 
		//break;

		case VEML6075_PARAM_DARK:
			return VEML6075_ReadRegister(hi2c, VEML6075_BADDR, VEML6075_DARK_DATA_REG, val); 
		//break;
	}
	
	// invalid parameter if we reached here
	return VEML6075_ERROR;
}

VEML6075_Error_et VEML6075_ShutDown( I2C_HandleTypeDef *hi2c, bool sd )
{
	uint16_t regval = VEML6075_CONF_DEFAULT | (sd ? VEML6075_CONF_SD : 0);
	
	return VEML6075_WriteRegister(hi2c, VEML6075_BADDR, VEML6075_UV_CONF_REG, regval); 
}

/*
Readouts indexes and identifiers...
First column are subscriptors from 
http://www.vishay.com/docs/84339/designingveml6075.pdf

vis ---  0a  -- VEML6075_UVCOMP1_DATA_REG  VEML6075_PARAM_UVCOMP1
dark --  08  -- VEML6075_DARK_DATA_REG     VEML6075_PARAM_DARK
uva ---  07  -- VEML6075_UVA_DATA_REG      VEML6075_PARAM_UVA
uvb ---  09  -- VEML6075_UVB_DATA_REG      VEML6075_PARAM_UVB
ir ----  0b  -- VEML6075_UVCOMP2_DATA_REG  VEML6075_PARAM_UVCOMP2
*/

/*!
 * \brief Get UVA reading from VEML6075 Sensor
 * returns: HAL Status
 */	
VEML6075_Error_et VEML6075_getUVA(I2C_HandleTypeDef *hi2c, float *uva)
{
	VEML6075_Error_et status;
	float comp_vis;
	float comp_ir;
	float comp_uva;
	static uint16_t raw_vis=0, raw_ir=0, raw_uva=0, raw_dark=0;
  
	status = VEML6075_UVVAL( hi2c, VEML6075_PARAM_UVA, &raw_uva);
	if (status == VEML6075_OK)
	{		
		// get rest of the readouts
		status += VEML6075_UVVAL( hi2c, VEML6075_PARAM_UVCOMP1, &raw_vis);
		status += VEML6075_UVVAL( hi2c, VEML6075_PARAM_UVCOMP2, &raw_ir);
		status += VEML6075_UVVAL( hi2c, VEML6075_PARAM_DARK, &raw_dark);
	} else return VEML6075_ERROR;

	comp_vis = (float)raw_vis - (float)raw_dark;
	comp_ir  = (float)raw_ir - (float)raw_dark;
	comp_uva = (float)raw_uva - (float)raw_dark;

	comp_uva -= (VEML6075_UVI_UVA_VIS_COEFF * comp_vis) - (VEML6075_UVI_UVA_IR_COEFF * comp_ir);

	*uva = comp_uva;
  
	return status;
}

/*!
 * \brief Get UVB reading from VEML6075 Sensor
 * returns: HAL Status
 */	
VEML6075_Error_et VEML6075_getUVB(I2C_HandleTypeDef *hi2c, float *uvb)
{
	VEML6075_Error_et status;
	float comp_vis;
	float comp_ir;
	float comp_uvb;
	static uint16_t raw_vis=0, raw_ir=0, raw_uvb=0, raw_dark=0;
  
	status = VEML6075_UVVAL( hi2c, VEML6075_PARAM_UVB, &raw_uvb);
	if (status == VEML6075_OK)
	{		
		// get rest of the readouts
		status += VEML6075_UVVAL( hi2c, VEML6075_PARAM_UVCOMP1, &raw_vis);
		status += VEML6075_UVVAL( hi2c, VEML6075_PARAM_UVCOMP2, &raw_ir);
		status += VEML6075_UVVAL( hi2c, VEML6075_PARAM_DARK, &raw_dark);
	} else return VEML6075_ERROR;

	comp_vis = (float)raw_vis - (float)raw_dark;
	comp_ir  = (float)raw_ir - (float)raw_dark;
	comp_uvb = (float)raw_uvb - (float)raw_dark;

	comp_uvb -= (VEML6075_UVI_UVB_VIS_COEFF * comp_vis) - (VEML6075_UVI_UVB_IR_COEFF * comp_ir);

	*uvb = comp_uvb;
  
	return status;
}

/*!
 * \brief Calculate UV index from compensated VEML6075 Sensor UV readings
 * returns: HAL Status
 */	
VEML6075_Error_et VEML6075_calculateUVIndex(I2C_HandleTypeDef *hi2c, float *uvindex)
{
	static float uva_weighted = 0;
	static float uvb_weighted = 0;
	VEML6075_Error_et status;

	status = VEML6075_getUVA(hi2c, &uva_weighted);
	uva_weighted *= VEML6075_UVI_UVA_RESPONSE;
	status += VEML6075_getUVB(hi2c, &uvb_weighted);
	uvb_weighted *= VEML6075_UVI_UVB_RESPONSE;
  
	if (status == VEML6075_OK)
	{
		*uvindex = (uva_weighted + uvb_weighted) / 2.0;
	}

	return status;
}

/*!
 * \brief Prepare UV, UVB and UV index readings in one single pass
 * returns: HAL Status
 */	
VEML6075_Error_et VEML6075_Get_Measurement(I2C_HandleTypeDef *hi2c, VEML6075_MeasureTypeDef_st *reading)
{
	VEML6075_Error_et status;
	float comp_vis;
	float comp_ir;
	float comp_uva;
	float comp_uvb;
	float uva_weighted;
	float uvb_weighted;

	static uint16_t raw_vis=0, raw_ir=0, raw_uva=0, raw_uvb=0, raw_dark=0;
  
	status = VEML6075_UVVAL( hi2c, VEML6075_PARAM_UVA, &raw_uva);
	if (status == VEML6075_OK)
	{		
	   // get rest of the readouts  
		status += VEML6075_UVVAL( hi2c, VEML6075_PARAM_UVCOMP1, &raw_vis);
		status += VEML6075_UVVAL( hi2c, VEML6075_PARAM_UVCOMP2, &raw_ir);
		status += VEML6075_UVVAL( hi2c, VEML6075_PARAM_DARK, &raw_dark);
		status += VEML6075_UVVAL( hi2c, VEML6075_PARAM_UVB, &raw_uvb);
	} else return VEML6075_ERROR;

	comp_vis = (float)raw_vis - (float)raw_dark;
	comp_ir  = (float)raw_ir - (float)raw_dark;
	comp_uva = (float)raw_uva - (float)raw_dark;
	comp_uvb = (float)raw_uvb - (float)raw_dark;
  
	comp_uva -= (VEML6075_UVI_UVA_VIS_COEFF * comp_vis) - (VEML6075_UVI_UVA_IR_COEFF * comp_ir);
	comp_uvb -= (VEML6075_UVI_UVB_VIS_COEFF * comp_vis) - (VEML6075_UVI_UVB_IR_COEFF * comp_ir);

	uva_weighted = comp_uva*VEML6075_UVI_UVA_RESPONSE;
	uvb_weighted = comp_uvb*VEML6075_UVI_UVB_RESPONSE;

	reading->uva = comp_uva;
	reading->uvb = comp_uvb;
	reading->uvindex = (uva_weighted + uvb_weighted) / 2.0;
  
	return status;
}
