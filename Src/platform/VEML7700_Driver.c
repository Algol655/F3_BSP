/*
 * VEML7700_Driver.c
 *
 *  Created on: Sep 17, 2023
 *      Author: Tommaso Sabatini
 */

#include "platform/VEML7700_Driver.h"

#ifdef __cplusplus
extern "C" {
#endif

uint16_t veml7700_conf_0_default = (uint16_t)VEML7700_CONF_0_DEFAULT;
uint16_t veml7700_conf_1_default = (uint16_t)VEML7700_CONF_1_DEFAULT;
uint16_t veml7700_conf_2_default = (uint16_t)VEML7700_CONF_2_DEFAULT;
uint16_t veml7700_conf_3_default = (uint16_t)VEML7700_CONF_3_DEFAULT;
//static uint8_t veml7700_als_conf_0[3];
//static uint8_t veml7700_als_wh[3];
//static uint8_t veml7700_als_wl[3];
//static uint8_t veml7700_pwr_save[3];

/*
 * VEML7700_ReadRegister() - Read register from VEML7700
 * @hi2c:  handle to I2C interface
 * @adr: I2C device address
 * @reg: Register address
 * @val: 16-bit register value from the VEML7700
 * Returns HAL status or HAL_ERROR for invalid parameters.
 */
VEML7700_Error_et VEML7700_ReadRegister(I2C_HandleTypeDef *hi2c, uint8_t adr, uint8_t reg, uint16_t *regval)
{
	uint8_t val[2];
	VEML7700_Error_et status;

//	status = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(adr << 1), &reg, 1, 100);
//	Sleep(10);
	status = HAL_I2C_Mem_Read(hi2c, adr, reg, I2C_MEMADD_SIZE_8BIT, val, 2, 100);
//	SET_BIT(hi2c->Instance->CR1, I2C_CR1_STOP);
//	status = HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(adr << 1), val, 2, 100);

	if (status == VEML7700_OK)
	{
		*regval =  (val[1] << 8 | val[0]);
	}
	return status;
}

/*
 * VEML7700_WriteRegister() - Write VEML7700 register
 * @hi2c:  handle to I2C interface
 * @adr: I2C device address
 * @reg: Register address
 * @val: 8-bit register value from the VEML7700
 * Returns HAL status or HAL_ERROR for invalid parameters.
 */
VEML7700_Error_et VEML7700_WriteRegister(I2C_HandleTypeDef *hi2c, uint8_t adr, uint8_t reg, uint16_t regval)
{
	uint8_t val[2];

	val[1] = (regval >> 8) & 0xff;
	val[0] = regval & 0xff;

	return (int8_t)HAL_I2C_Mem_Write(hi2c, adr, reg, I2C_MEMADD_SIZE_8BIT, val, 2, 100);
}

/**
 * Execute one write transaction on the I2C bus, sending a given number of
 * bytes. The bytes in the supplied buffer must be sent to the given address. If
 * the slave device does not acknowledge any of the bytes, an error shall be
 * returned.
 *
 * @param address 7-bit I2C address to write to
 * @param data    pointer to the buffer containing the data to write
 * @param count   number of bytes to read from the buffer and send over I2C
 * @returns 0 on success, error code otherwise
 */
/*VEML7700_Error_et VEML7700_WriteRegister(uint8_t address, const uint8_t* data, uint16_t count)
{
	return (int8_t)HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(address << 1), (uint8_t*)data, count, 100);
} */

/**
 * Sleep for a given number of microseconds. The function should delay the
 * execution for at least the given time, but may also sleep longer.
 *
 * @param useconds the sleep time in microseconds
 */
void VEML7700_sleep_usec(uint32_t useconds)
{
	usleep(useconds);
}

VEML7700_Error_et VEML7700_Get_Val(I2C_HandleTypeDef *hi2c, VEML7700_MeasureMode_t param, uint16_t *val)
{
	switch (param)
	{
		case VEML_RAW_ALS:
			return VEML7700_ReadRegister(hi2c, VEML7700_BADDR, VEML7700_ALS_REG, val);
			//break;

		case VEML_RAW_WHITE:
			return VEML7700_ReadRegister(hi2c, VEML7700_BADDR, VEML7700_WHITE_REG, val);
			//break;

/*		case VEML6075_PARAM_UVCOMP1:
			return VEML6075_ReadRegister(hi2c, VEML6075_BADDR, VEML6075_UVCOMP1_DATA_REG, val);
			//break;

		case VEML6075_PARAM_UVCOMP2:
			return VEML6075_ReadRegister(hi2c, VEML6075_BADDR, VEML6075_UVCOMP2_DATA_REG, val);
			//break;

		case VEML6075_PARAM_DARK:
			return VEML6075_ReadRegister(hi2c, VEML6075_BADDR, VEML6075_DARK_DATA_REG, val);
			//break; */
		default:
			break;
	}

	// invalid parameter if we reached here
	return VEML7700_ERROR;
}

/*
 *	@brief Read the raw ALS data
 *	@param none
 *	@returns none
 */
VEML7700_Error_et VEML7700_Get_Measurement(I2C_HandleTypeDef *hi2c, VEML7700_MeasureTypeDef_st *reading)
{
	static uint16_t raw_als=0, raw_white=0;

	BIT_CLEAR(veml7700_conf_0_default,0);

/*	if (VEML7700_WriteRegister(VEML7700_BADDR, veml7700_als_conf_0, 3))
		return VEML7700_ERROR; */
	if (VEML7700_WriteRegister(&hi2c1, VEML7700_BADDR, VEML7700_ALS_CONF_0_REG, veml7700_conf_0_default))
		return VEML7700_ERROR;

	Sleep(5);

	if (VEML7700_Get_Val(&hi2c1, VEML_RAW_ALS, &raw_als))
		return VEML7700_ERROR;
	if (VEML7700_Get_Val(&hi2c1, VEML_RAW_WHITE, &raw_white))
		return VEML7700_ERROR;

	reading->raw_ALS = raw_als;
	reading->raw_white = raw_white;

	BIT_SET(veml7700_conf_0_default,0);

/*	if (VEML7700_WriteRegister(VEML7700_BADDR, veml7700_als_conf_0, 3))
		return VEML7700_ERROR; */
	if (VEML7700_WriteRegister(&hi2c1, VEML7700_BADDR, VEML7700_ALS_CONF_0_REG, veml7700_conf_0_default))
		return VEML7700_ERROR;

	return VEML7700_OK;
}

VEML7700_Error_et MX_VEML7700_Init()
{
	if (VEML7700_WriteRegister(&hi2c1, VEML7700_BADDR, VEML7700_ALS_CONF_0_REG, VEML7700_CONF_0_DEFAULT))
		return VEML7700_ERROR;
	if (VEML7700_WriteRegister(&hi2c1, VEML7700_BADDR, VEML7700_ALS_WH_REG, VEML7700_CONF_1_DEFAULT))
		return VEML7700_ERROR;
	if (VEML7700_WriteRegister(&hi2c1, VEML7700_BADDR, VEML7700_ALS_WL_REG, VEML7700_CONF_2_DEFAULT))
		return VEML7700_ERROR;
	if (VEML7700_WriteRegister(&hi2c1, VEML7700_BADDR, VEML7700_PWR_SAVE_REG, VEML7700_CONF_3_DEFAULT))
		return VEML7700_ERROR;

/*	veml7700_als_conf_0[0] = VEML7700_ALS_CONF_0_REG;
	memcpy(&veml7700_als_conf_0[1], &veml7700_conf_0_default, 2);
	if (VEML7700_WriteRegister(VEML7700_BADDR, veml7700_als_conf_0, 3))
		return VEML7700_ERROR;

	Sleep(5);

	veml7700_als_wh[0] = VEML7700_ALS_WH_REG;
	memcpy(&veml7700_als_wh[1], &veml7700_conf_1_default, 2);
	if (VEML7700_WriteRegister(VEML7700_BADDR, veml7700_als_wh, 3))
		return VEML7700_ERROR;

	veml7700_als_wl[0] = VEML7700_ALS_WL_REG;
	memcpy(&veml7700_als_wl[1], &veml7700_conf_2_default, 2);
	if (VEML7700_WriteRegister(VEML7700_BADDR, veml7700_als_wl, 3))
		return VEML7700_ERROR;

	veml7700_pwr_save[0] = VEML7700_PWR_SAVE_REG;
	memcpy(&veml7700_pwr_save[1], &veml7700_conf_3_default, 2);
	if (VEML7700_WriteRegister(VEML7700_BADDR, veml7700_pwr_save, 3))
		return VEML7700_ERROR; */

	return VEML7700_OK;
}

#ifdef __cplusplus
}
#endif
