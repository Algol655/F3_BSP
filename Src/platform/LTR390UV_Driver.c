/*
 * LTR390UV_Driver.c
 *
 *  Created on: Oct 10, 2023
 *      Author: Tommaso Sabatini
 */

#include "platform/LTR390UV_Driver.h"

/**brief LTR390UV_Private_Variables
 *
 */
uint8_t LTR390UV_1[1] = {
							LTR390UV_MAIN_CTRL_VAL		// 0x00
						};
uint8_t LTR390UV_2[2] =	{
							LTR390UV_MEAS_RATE_VAL,		// 0x04
							LTR390UV_GAIN_VAL			// 0x05
						};
uint8_t LTR390UV_3[8] =	{
							LTR390UV_INT_CFG_VAL,		// 0x19
							LTR390UV_INT_PST_VAL,		// 0x1A
							LTR390UV_THRESH_UP_0_VAL,	// 0x21
							LTR390UV_THRESH_UP_1_VAL,	// 0x22
							LTR390UV_THRESH_UP_2_VAL,	// 0x23
							LTR390UV_THRESH_LOW_0_VAL,	// 0x24
							LTR390UV_THRESH_LOW_1_VAL,	// 0x25
							LTR390UV_THRESH_LOW_2_VAL   // 0x26
						};

uint8_t a_gain[5]  = {1,3,6,9,18};
//float32_t a_int[6] = {0.03125,0.25,0.5,1.0,2.0,4.0};
float32_t a_int[6] = {4.0,2.0,1.0,0.5,0.25,0.03125};
float32_t W_Fac = 1.0;	//W_Fac = 1 for NO window or clear window glass.
						//W_Fac > 1 device under tinted window glass. Calibrate under white LED

/** @brief LTR390UV_Private_Functions
 */
/**
  * Function Name	: LTR390UV_ReadReg
  * Description		: Generic Reading function. It must be full-filled with either
  *         	  	: I2C or SPI reading functions
  * Input       	: Register Address
  * Output      	: Data Read
  * Return      	: Status [LTR390UV_ERROR, LTR390UV_OK]
  */
LTR390UV_Error_et LTR390UV_ReadReg(uint8_t B_Addr, uint8_t RegAddr, uint8_t *Data, uint16_t Len)
{
//	if ( Len > 1 ) RegAddr |= 0x80;

	if (I2C_ReadData(B_Addr, RegAddr, Data, Len))
		return LTR390UV_ERROR;
	else
		return LTR390UV_OK;
}

/*******************************************************************************
* Function Name	: LTR390UV_WriteReg
* Description  	: Generic Writing function in normal mode. It must be full-filled with either
*         		: I2C or SPI writing function
* Input       	: Register Address, Data to be written
* Output      	: None
* Return      	: Status [LTR390UV_ERROR, LTR390UV_OK]
*******************************************************************************/
LTR390UV_Error_et LTR390UV_WriteReg(uint8_t B_Addr, uint8_t RegAddr, uint8_t *Data, uint16_t Len)
{
//	if ( Len > 1 ) RegAddr |= 0x80;

	if (I2C_WriteData(B_Addr, RegAddr, Data, Len))
		return LTR390UV_ERROR;
	else
		return LTR390UV_OK;
}

/*******************************************************************************
* Function Name	: LTR390UV_WriteReg
* Description   : Generic Writing function in DMA mode. It must be full-filled with either
*         		: I2C or SPI writing function
* Input       	: Register Address, Data to be written
* Output      	: None
* Return      	: Status [LTR390UV_ERROR, LTR390UV_OK]
*******************************************************************************/
LTR390UV_Error_et LTR390UV_WriteReg_DMA(uint8_t B_Addr, uint8_t RegAddr, uint8_t *Data, uint16_t Len)
{
//	if ( Len > 1 ) RegAddr |= 0x80;

	if (I2C_WriteData_DMA(B_Addr, RegAddr, Data, Len))
		return LTR390UV_ERROR;
	else
		return LTR390UV_OK;
}

/**
  * @brief  Device ID [get]
  *
  * @param  B_Addr I2C Address
  * @param  buff   Buffer that stores data read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LTR390UV_Error_et LTR390UV_DeviceId_Get(uint8_t B_Addr, uint8_t *buff)
{
	LTR390UV_Error_et ret;

	ret =  LTR390UV_ReadReg(B_Addr, LTR390UV_PART_ID, (uint8_t *) buff, 1);

	return ret;
}

/**
  * @brief  Software reset. Restore the default values in user registers [set]
  *
  * @param  B_Addr I2C Address
  * @param  val    Change the values of swreset in reg LTR390UV_MAIN_CTRL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LTR390UV_Error_et LTR390UV_SwRst_Set(uint8_t B_Addr, uint8_t val)
{
	LTR390UV_MainCtrl_t main_ctrl_reg;
	LTR390UV_Error_et ret;

	ret = LTR390UV_ReadReg(B_Addr, LTR390UV_MAIN_CTRL, (uint8_t *)&main_ctrl_reg, 1);

	if (ret == 0)
	{
		main_ctrl_reg.sw_reset = val;
		ret = LTR390UV_WriteReg(B_Addr, LTR390UV_MAIN_CTRL, (uint8_t *)&main_ctrl_reg, 1);
	}

	return ret;
}

/**
  * @brief  ALS/UVS Measurement Rate, ALS/UVS Resolution [set]
  *
  * @param  B_Addr I2C Address
  * @param  rate   Change the values of als_uvs_meas_rate in reg ALS_UVS_MEAS_RATE
  * @param  res    Change the values of als_uvs_resolution in reg ALS_UVS_MEAS_RATE
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LTR390UV_Error_et LTR390UV_ALS_UVS_Rate_Res_Set(uint8_t B_Addr, uint8_t rate, uint8_t res)
{
	LTR390UV_MeasRate_t meas_rate_reg;
	LTR390UV_Error_et ret;

	ret = LTR390UV_ReadReg(B_Addr, LTR390UV_MEAS_RATE, (uint8_t *)&meas_rate_reg, 1);

	if (ret == 0)
	{
		meas_rate_reg.als_uvs_meas_rate = rate;
		meas_rate_reg.als_uvs_resolution = res;
		ret = LTR390UV_WriteReg(B_Addr, LTR390UV_MEAS_RATE, (uint8_t *)&meas_rate_reg, 1);
	}

	return ret;
}

/**
  * @brief  ALS/UVS Gain [set]
  *
  * @param  B_Addr I2C Address
  * @param  rate   Change the values of als_uvs_gain in reg LTR390UV_GAIN
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LTR390UV_Error_et LTR390UV_ALS_UVS_Gain_Set(uint8_t B_Addr, uint8_t gain)
{
	LTR390UV_Gain_t gain_reg;
	LTR390UV_Error_et ret;

	ret = LTR390UV_ReadReg(B_Addr, LTR390UV_GAIN, (uint8_t *)&gain_reg, 1);

	if (ret == 0)
	{
		gain_reg.als_uvs_gain = gain;
		ret = LTR390UV_WriteReg(B_Addr, LTR390UV_GAIN, (uint8_t *)&gain_reg, 1);
	}

	return ret;
}

/**
  * @brief  UVS Mode: 0 -> ALS Mode. 1 -> UVS Mode [set]
  *
  * @param  B_Addr I2C Address
  * @param  val    Change the values of als_uvs_en in reg LTR390UV_MAIN_CTRL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LTR390UV_Error_et LTR390UV_ALS_UVS_Set(uint8_t B_Addr, uint8_t val)
{
	LTR390UV_MainCtrl_t main_ctrl_reg;
	LTR390UV_Error_et ret;

	ret = LTR390UV_ReadReg(B_Addr, LTR390UV_MAIN_CTRL, (uint8_t *)&main_ctrl_reg, 1);

	if (ret == 0)
	{
		main_ctrl_reg.uvs_mode = val;
		ret = LTR390UV_WriteReg(B_Addr, LTR390UV_MAIN_CTRL, (uint8_t *)&main_ctrl_reg, 1);
	}

	return ret;
}

/**
  * @brief  Ultraviolet Light Sensor (UVS) output value [get]
  *
  * @param  B_Addr I2C Address
  * @param  buff   Buffer that stores data read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LTR390UV_Error_et LTR390UV_RawUVS_Get(uint8_t B_Addr, uint32_t *buff)
{
	uint8_t reg[3] = {0,0,0};
	uint32_t tmp = 0;
	uint8_t i;

	if(LTR390UV_ReadReg(B_Addr, LTR390UV_UVSDATA_0, reg, 3))
		return LTR390UV_ERROR;

	/* Build the raw data */
	for(i = 0; i < 3; i++)
		tmp |= (((uint32_t)reg[i]) << (8 * i));

	*buff = tmp;

	return LTR390UV_OK;
}

/**
  * @brief  Ambient Light Sensor (ALS) output value [get]
  *
  * @param  B_Addr I2C Address
  * @param  buff   Buffer that stores data read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LTR390UV_Error_et LTR390UV_RawALS_Get(uint8_t B_Addr, uint32_t *buff)
{
	uint8_t reg[3] = {0,0,0};
	uint32_t tmp = 0;
	uint8_t i;

	if(LTR390UV_ReadReg(B_Addr, LTR390UV_ALSDATA_0, reg, 3))
		return LTR390UV_ERROR;

	/* Build the raw data */
	for(i = 0; i < 3; i++)
		tmp |= (((uint32_t)reg[i]) << (8 * i));

	*buff = tmp;

	return LTR390UV_OK;
}

/**
* @brief    Get the LTR390UV Ambient Light Sensor (ALS) value in Lux.
* @param    *handle Device handle.
* @param    The buffer to empty with tAmbient Light Sensor (ALS) value in Lux
* @retval   Error Code [LTR390UV_ERROR, LTR390UV_OK]
*/
LTR390UV_Error_et LTR390UV_Lux_Get(uint8_t B_Addr, float32_t *buff)
{
	uint32_t raw_als = 0;
	LTR390UV_MeasRate_t meas_rate_reg;
	LTR390UV_Gain_t gain_reg;

	if(LTR390UV_RawALS_Get(B_Addr, &raw_als))
		return LTR390UV_ERROR;
	if(LTR390UV_ReadReg(B_Addr, LTR390UV_MEAS_RATE, (uint8_t *)&meas_rate_reg, 1))
		return LTR390UV_ERROR;
	if(LTR390UV_ReadReg(B_Addr, LTR390UV_GAIN, (uint8_t *)&gain_reg, 1))
		return LTR390UV_ERROR;

    *buff = ((0.6*raw_als)/(a_gain[gain_reg.als_uvs_gain]*a_int[meas_rate_reg.als_uvs_resolution])*W_Fac);

	return LTR390UV_OK;
}

/**
* @brief    Get the LTR390UV Ultraviolet Light Sensor (UVS) value index
* @param    *handle Device handle.
* @param    The buffer to empty with the Ultraviolet Light Sensor (UVS) value index
* @retval   Error Code [LTR390UV_ERROR, LTR390UV_OK]
*/
LTR390UV_Error_et LTR390UV_UVI_Get(uint8_t B_Addr, float32_t *buff)
{
	uint32_t raw_uvi;

	if(LTR390UV_RawUVS_Get(B_Addr, &raw_uvi))
		return LTR390UV_ERROR;

    *buff = ((((float32_t)raw_uvi)/((float32_t)LTR390UV_UV_SENSITIVITY))*W_Fac);

	return LTR390UV_OK;
}

/**
* @brief  Get the values of the last single measurement.
* @param  *handle Device handle.
* @param  LTR390UV report structure
* @retval Error Code [LTR390UV_ERROR, LTR390UV_OK]
*/
LTR390UV_Error_et LTR390UV_Get_Measurement(uint8_t B_Addr, LTR390UV_MeasureTypeDef_st *Measurement_Value)
{
	uint32_t UVSout;
	uint32_t ALSout;
	float32_t UVIout;
	float32_t LUXout;
	static bool toggle = false;
	static bool high_gain = true;

	toggle = !toggle;
	if (toggle)		//ALS measurement
	{
		if(LTR390UV_RawALS_Get(B_Addr, &ALSout))
			return LTR390UV_ERROR;
		Measurement_Value->ALS_RawOut=ALSout;

		if(LTR390UV_Lux_Get(B_Addr, &LUXout))
			return LTR390UV_ERROR;
		/*
		 * The ALS is initially programmed for maximum sensitivity (ALS Gain=18, Resolution=20bit).
		 * In this case the maximum measurable illuminance is 8738.125 lux. To avoid overflow problems,
		 * when the illuminance exceeds 8465 lux, the low sensitivity mode is set (ALS Gain=3, Resolution=18bit).
		 */
		if (LUXout > 8465.0)
		{	//Set Low Gain Mode
			if (high_gain)
			{
				if(LTR390UV_ALS_UVS_Rate_Res_Set(B_Addr, 5, 2))
					return LTR390UV_ERROR;
				if(LTR390UV_ALS_UVS_Gain_Set(B_Addr, 1))
					return LTR390UV_ERROR;
				high_gain = false;
			}
		} else
		{	//Set High Gain Mode
			if (!high_gain)
			{
				if(LTR390UV_ALS_UVS_Rate_Res_Set(B_Addr, 5, 0))
					return LTR390UV_ERROR;
				if(LTR390UV_ALS_UVS_Gain_Set(B_Addr, 4))
					return LTR390UV_ERROR;
				high_gain = true;
			}
		}

		Measurement_Value->Lux=LUXout;

		if (LTR390UV_ALS_UVS_Set(B_Addr, 1))	//Set in UVS Mode for the next cycle
			return LTR390UV_ERROR;
		/*
		 * The formula for calculating the UV index (UVI) is specified in the data sheet only for maximum sensitivity.
		 * Therefore, in UVS mode, the maximum sensitivity is always set ((ALS Gain=18, Resolution=20bit).
		 */
		if (!high_gain)
		{	//Set High Gain Mode
			if(LTR390UV_ALS_UVS_Rate_Res_Set(B_Addr, 5, 0))
				return LTR390UV_ERROR;
			if(LTR390UV_ALS_UVS_Gain_Set(B_Addr, 4))
				return LTR390UV_ERROR;
		}

	} else	//UVS measurement
	{
		if(LTR390UV_RawUVS_Get(B_Addr, &UVSout))
			return LTR390UV_ERROR;
		Measurement_Value->UVS_RawOut=UVSout;

		if(LTR390UV_UVI_Get(B_Addr, &UVIout))
			return LTR390UV_ERROR;
		Measurement_Value->UVI=UVIout;

		if (LTR390UV_ALS_UVS_Set(B_Addr, 0))	//Set in ALS Mode for the next cycle
			return LTR390UV_ERROR;
		/*
		 * If the low gain configuration was set in the ALS configuration then restore the initial settings
		 */
		if (!high_gain)
		{	//Set Low Gain Mode
			if(LTR390UV_ALS_UVS_Rate_Res_Set(B_Addr, 5, 2))
				return LTR390UV_ERROR;
			if(LTR390UV_ALS_UVS_Gain_Set(B_Addr, 1))
				return LTR390UV_ERROR;
			Sleep(i2c_delay);
		}

	}

	return LTR390UV_OK;
}

/*******************************************************************************
* Function Name	: MX_LTR390UV_Init
* Description   : LTR390UV Global init
* Input       	: None
* Output      	: None
* Return      	: Status [LTR390UV_ERROR, LTR390UV_OK]
*******************************************************************************/
LTR390UV_Error_et MX_LTR390UV_Init()
{
	uint8_t tmp;

	if(LTR390UV_SwRst_Set(LTR390UV_BADDR, 1))
		return LTR390UV_ERROR;
	Sleep(10);

	if (LTR390UV_WriteReg_DMA(LTR390UV_BADDR, 0x00, &LTR390UV_1[0], 1))
		return LTR390UV_ERROR;
	Sleep(i2c_delay);

	if (LTR390UV_WriteReg_DMA(LTR390UV_BADDR, 0x04, &LTR390UV_2[0], 2))
		return LTR390UV_ERROR;
	Sleep(i2c_delay);

	if (LTR390UV_WriteReg_DMA(LTR390UV_BADDR, 0x19, &LTR390UV_3[0], 8))
		return LTR390UV_ERROR;
	Sleep(i2c_delay);

	if (LTR390UV_DeviceId_Get(LTR390UV_BADDR, &tmp))
		return LTR390UV_ERROR;
	if (tmp != LTR390UV_PART_ID_VAL)
		return LTR390UV_ERROR;
/*	Sleep(i2c_delay);

	if (LTR390UV_DataRate_Get(LTR390UV_BADDR, &tmp))
		return LTR390UV_ERROR;
	if (tmp != LTR390UV_ODR_1_Hz)
		return LTR390UV_ERROR; */

	return LTR390UV_OK;
}

