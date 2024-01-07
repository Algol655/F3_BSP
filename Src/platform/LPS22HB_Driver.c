/**
  ******************************************************************************
  * @file    LPS22HB_Driver.c
  * @author  Sensors Software Solution Team
  * @brief   LPS22HB driver file
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

#include "platform/LPS22HB_Driver.h"

/**
  * @defgroup    LPS22HB
  * @brief       This file provides a set of functions needed to drive the
  *              ultra-compact piezoresistive absolute pressure sensor.
  * @{
  *
  */

/**
  * @defgroup    LPS22HB_Interfaces_functions
  * @brief       This section provide a set of functions used to read and
  *              write a generic register of the device.
  * @{
  *
  */

/**
  * @brief  Read generic device register
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  reg   register to read
  * @param  data  pointer to buffer that store the data read(ptr)
  * @param  len   number of consecutive register to read
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
/*LPS22HB_Error_et __weak LPS22HB_ReadReg(uint8_t B_Addr, uint8_t RegAddr, uint8_t *Data, uint16_t Len)
{
	LPS22HB_Error_et ret;
	
	ret = ctx->read_reg(ctx->handle, reg, data, Len);
	
	return ret;
} */

/**
  * @brief  Write generic device register
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  reg   register to write
  * @param  data  pointer to data to write in register reg(ptr)
  * @param  len   number of consecutive register to write
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
/*LPS22HB_Error_et __weak LPS22HB_WriteReg(uint8_t B_Addr, uint8_t RegAddr, uint8_t *Data, uint16_t Len)
{
	LPS22HB_Error_et ret;
	
	ret = ctx->write_reg(ctx->handle, reg, data, Len);
	
	return ret;
} */

/** @defgroup LPS22HB_My_Private_Variables
 * @{
*/
uint8_t LPS22HB_1[3] = 	{
							LPS22HB_INTERRUPT_CFG_VAL,	// 0x0B
							LPS22HB_THS_P_L_VAL,		// 0x0C
							LPS22HB_THS_P_H_VAL			// 0x0D
						};
uint8_t LPS22HB_2[3] = 	{
							LPS22HB_CTRL_REG1_VAL,		// 0x10 
							LPS22HB_CTRL_REG2_VAL,		// 0x11 	
							LPS22HB_CTRL_REG3_VAL 		// 0x12	
						};
uint8_t LPS22HB_3[7] = 	{
							LPS22HB_FIFO_CTRL_VAL,		// 0x14	 
							LPS22HB_REF_P_XL_VAL,		// 0x15	
							LPS22HB_REF_P_L_VAL,		// 0x16	
							LPS22HB_REF_P_H_VAL,		// 0x17
							LPS22HB_RPDS_L_VAL,			// 0x18
							LPS22HB_RPDS_H_VAL,			// 0x19
							LPS22HB_RES_CONF_VAL		// 0x1A
						};
/**
 * @}
*/
/** @defgroup LPS22HB_My_Private_Functions
 * @{
*/
/**
  * Function Name	: LPS22HB_ReadReg
  * Description		: Generic Reading function. It must be full-filled with either
  *         	  	: I2C or SPI reading functions
  * Input       	: Register Address
  * Output      	: Data Read
  * Return      	: Status [LPS22HB_ERROR, LPS22HB_OK]
  */
LPS22HB_Error_et LPS22HB_ReadReg(uint8_t B_Addr, uint8_t RegAddr, uint8_t *Data, uint16_t Len)
{
	if ( Len > 1 ) RegAddr |= 0x80;

	if (I2C_ReadData(B_Addr, RegAddr, Data, Len))
		return LPS22HB_ERROR;
	else
		return LPS22HB_OK;
}

/*******************************************************************************
* Function Name	: LPS22HB_WriteReg
* Description  	: Generic Writing function in normal mode. It must be full-filled with either
*         		: I2C or SPI writing function
* Input       	: Register Address, Data to be written
* Output      	: None
* Return      	: Status [LPS22HB_ERROR, LPS22HB_OK]
*******************************************************************************/
LPS22HB_Error_et LPS22HB_WriteReg(uint8_t B_Addr, uint8_t RegAddr, uint8_t *Data, uint16_t Len)
{
	if ( Len > 1 ) RegAddr |= 0x80;

	if (I2C_WriteData(B_Addr, RegAddr, Data, Len))
		return LPS22HB_ERROR;
	else
		return LPS22HB_OK;
}

/*******************************************************************************
* Function Name	: LPS22HB_WriteReg
* Description   : Generic Writing function in DMA mode. It must be full-filled with either
*         		: I2C or SPI writing function
* Input       	: Register Address, Data to be written
* Output      	: None
* Return      	: Status [LPS22HB_ERROR, LPS22HB_OK]
*******************************************************************************/
LPS22HB_Error_et LPS22HB_WriteReg_DMA(uint8_t B_Addr, uint8_t RegAddr, uint8_t *Data, uint16_t Len)
{
	if ( Len > 1 ) RegAddr |= 0x80;

	if (I2C_WriteData_DMA(B_Addr, RegAddr, Data, Len))
		return LPS22HB_ERROR;
	else
		return LPS22HB_OK;
}

/**
* @brief    Get the LPS22HB Pressure value in hPA.
* @detail   The data are expressed as PRESS_OUT_H/_L/_XL in 2’s complement.
            Pout(hPA)=PRESS_OUT / 4096
* @param    *handle Device handle.
* @param    The buffer to empty with the pressure value that must be divided by 100 to get the value in hPA
* @retval   Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_Pressure(uint8_t B_Addr, uint32_t* Pout)
{
	uint32_t raw_press;

	if(LPS22HB_PressRaw_Get(B_Addr, &raw_press))
		return LPS22HB_ERROR;

	*Pout = (raw_press*100)/4096;

	return LPS22HB_OK;
}

/**
* @brief    Get the Temperature value in °C.
* @detail   Temperature data are expressed as TEMP_OUT_H&TEMP_OUT_L as 2’s complement number.
*           Tout(degC)=TEMP_OUT/100
* @param    *handle Device handle.
* @param    Buffer to empty with the temperature value that must be divided by 10 to get the value in °C
* @retval   Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_Temperature(uint8_t B_Addr, int16_t* Tout)
{
	int16_t raw_data;

	if(LPS22HB_TempRaw_Get(B_Addr, &raw_data))
		return LPS22HB_ERROR;

	*Tout = (raw_data*10)/100;

	return LPS22HB_OK;
}

/**
* @brief  Get the values of the last single measurement.
* @param  *handle Device handle.
* @param  Pressure and temperature tmp
* @retval Error Code [LPS22HB_ERROR, LPS22HB_OK]
*/
LPS22HB_Error_et LPS22HB_Get_Measurement(uint8_t B_Addr, LPS22HB_MeasureTypeDef_st *Measurement_Value)
{
	int16_t Tout;
	uint32_t Pout;

	if(LPS22HB_Get_Temperature(B_Addr, &Tout))
		return LPS22HB_ERROR;

	Measurement_Value->Tout=Tout;

	if(LPS22HB_Get_Pressure(B_Addr, &Pout))
		return LPS22HB_ERROR;

	Measurement_Value->Pout=Pout;

	return LPS22HB_OK;
}

/*******************************************************************************
* Function Name	: MX_LPS22HB_Init
* Description   : LPS22HB Global init
*         		: I2C or SPI writing function
* Input       	: None
* Output      	: None
* Return      	: Status [LPS22HB_ERROR, LPS22HB_OK]
*******************************************************************************/
LPS22HB_Error_et MX_LPS22HB_Init()
{
	uint8_t tmp;

	if (LPS22HB_WriteReg_DMA(LPS22HB_BADDR, 0x0B, &LPS22HB_1[0], 3))
		return LPS22HB_ERROR;
	Sleep(i2c_delay);

	if (LPS22HB_WriteReg_DMA(LPS22HB_BADDR, 0x10, &LPS22HB_2[0], 3))
		return LPS22HB_ERROR;
	Sleep(i2c_delay);

	if (LPS22HB_WriteReg_DMA(LPS22HB_BADDR, 0x14, &LPS22HB_3[0], 7))
		return LPS22HB_ERROR;
	Sleep(i2c_delay);

	if (LPS22HB_DeviceId_Get(LPS22HB_BADDR, &tmp))
		return LPS22HB_ERROR;
	if (tmp != LPS22HB_WHO_AM_I_VAL)
		return LPS22HB_ERROR;
/*	Sleep(i2c_delay);

	if (LPS22HB_DataRate_Get(LPS22HB_BADDR, &tmp))
		return LPS22HB_ERROR;
	if (tmp != LPS22HB_ODR_1_Hz)
		return LPS22HB_ERROR; */

	return LPS22HB_OK;
}
/**
 * @}
*/

/**
  * @}
  *
  */

/**
  * @defgroup    LPS22HB_Sensitivity
  * @brief       These functions convert raw-data into engineering units.
  * @{
  *
  */
float32_t LPS22HB_LSB_2_HPa(int32_t lsb)
{
	return ((float32_t)lsb / 1048576.0f);
}

float32_t LPS22HB_LSB_2_DegC(int16_t lsb)
{
	return ((float32_t)lsb / 100.0f);
}

/**
  * @}
  *
  */

/**
  * @defgroup    LPS22HB_data_generation_c
  * @brief       This section group all the functions concerning data
  *              generation
  * @{
  *
  */

/**
  * @brief  Reset Autozero function.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of reset_az in reg INTERRUPT_CFG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_AutozeroRst_Set(uint8_t B_Addr, uint8_t val)
{
	LPS22HB_InterruptCfg_t interrupt_cfg;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_INTERRUPT_CFG, (uint8_t *)&interrupt_cfg, 1);
	
	if (ret == 0)
	{
		interrupt_cfg.reset_az = val;
		ret = LPS22HB_WriteReg(B_Addr, LPS22HB_INTERRUPT_CFG, (uint8_t *)&interrupt_cfg, 1);
	}
	
	return ret;
}

/**
  * @brief  Reset Autozero function.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of reset_az in reg INTERRUPT_CFG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_AutozeroRst_Get(uint8_t B_Addr, uint8_t *val)
{
	LPS22HB_InterruptCfg_t interrupt_cfg;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_INTERRUPT_CFG, (uint8_t *)&interrupt_cfg, 1);
	*val = interrupt_cfg.reset_az;
	
	return ret;
}

/**
  * @brief  Enable Autozero function.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of autozero in reg INTERRUPT_CFG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_Autozero_Set(uint8_t B_Addr, uint8_t val)
{
	LPS22HB_InterruptCfg_t interrupt_cfg;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_INTERRUPT_CFG, (uint8_t *)&interrupt_cfg, 1);
	
	if (ret == 0)
	{
		interrupt_cfg.autozero = val;
		ret = LPS22HB_WriteReg(B_Addr, LPS22HB_INTERRUPT_CFG, (uint8_t *)&interrupt_cfg, 1);
	}
	
	return ret;
}

/**
  * @brief  Enable Autozero function.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of autozero in reg INTERRUPT_CFG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_Autozero_Get(uint8_t B_Addr, uint8_t *val)
{
	LPS22HB_InterruptCfg_t interrupt_cfg;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_INTERRUPT_CFG, (uint8_t *)&interrupt_cfg, 1);
	*val = interrupt_cfg.autozero;
	
	return ret;
}

/**
  * @brief  Reset AutoRifP function.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of reset_arp in reg INTERRUPT_CFG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_PressSnapRst_Set(uint8_t B_Addr, uint8_t val)
{
	LPS22HB_InterruptCfg_t interrupt_cfg;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_INTERRUPT_CFG, (uint8_t *)&interrupt_cfg, 1);
	
	if (ret == 0)
	{
		interrupt_cfg.reset_arp = val;
		ret = LPS22HB_WriteReg(B_Addr, LPS22HB_INTERRUPT_CFG, (uint8_t *)&interrupt_cfg, 1);
	}
	
	return ret;
}

/**
  * @brief  Reset AutoRifP function.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of reset_arp in reg INTERRUPT_CFG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_PressSnapRst_Get(uint8_t B_Addr, uint8_t *val)
{
	LPS22HB_InterruptCfg_t interrupt_cfg;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_INTERRUPT_CFG, (uint8_t *)&interrupt_cfg, 1);
	*val = interrupt_cfg.reset_arp;
	
	return ret;
}

/**
  * @brief  Enable AutoRifP function.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of autorifp in reg INTERRUPT_CFG.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_PressSnap_Set(uint8_t B_Addr, uint8_t val)
{
	LPS22HB_InterruptCfg_t interrupt_cfg;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_INTERRUPT_CFG, (uint8_t *)&interrupt_cfg, 1);
	
	if (ret == 0)
	{
		interrupt_cfg.autorifp = val;
		ret = LPS22HB_WriteReg(B_Addr, LPS22HB_INTERRUPT_CFG, (uint8_t *)&interrupt_cfg, 1);
	}
	
	return ret;
}

/**
  * @brief  Enable AutoRifP function.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of autorifp in reg INTERRUPT_CFG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_PressSnap_Get(uint8_t B_Addr, uint8_t *val)
{
	LPS22HB_InterruptCfg_t interrupt_cfg;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_INTERRUPT_CFG, (uint8_t *)&interrupt_cfg, 1);
	*val = interrupt_cfg.autorifp;
	
	return ret;
}

/**
  * @brief  Block data update.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of bdu in reg CTRL_REG1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_BlockDataUpdate_Set(uint8_t B_Addr, uint8_t val)
{
	LPS22HB_CtrlReg1_t ctrl_reg1;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_CTRL_REG1, (uint8_t *)&ctrl_reg1, 1);
	
	if (ret == 0)
	{
		ctrl_reg1.bdu = val;
		ret = LPS22HB_WriteReg(B_Addr, LPS22HB_CTRL_REG1, (uint8_t *)&ctrl_reg1, 1);
	}
	
	return ret;
}

/**
  * @brief  Block data update.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of bdu in reg CTRL_REG1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_BlockDataUpdate_Get(uint8_t B_Addr, uint8_t *val)
{
	LPS22HB_CtrlReg1_t ctrl_reg1;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_CTRL_REG1, (uint8_t *)&ctrl_reg1, 1);
	*val = ctrl_reg1.bdu;
	
	return ret;
}

/**
  * @brief  Low-pass bandwidth selection.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of lpfp in reg CTRL_REG1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_LPF_Mode_Set(uint8_t B_Addr, LPS22HB_LPFp_t val)
{
	LPS22HB_CtrlReg1_t ctrl_reg1;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_CTRL_REG1, (uint8_t *)&ctrl_reg1, 1);
	
	if (ret == 0)
	{
		ctrl_reg1.lpfp = (uint8_t)val;
		ret = LPS22HB_WriteReg(B_Addr, LPS22HB_CTRL_REG1, (uint8_t *)&ctrl_reg1, 1);
	}
	
	return ret;
}

/**
  * @brief   Low-pass bandwidth selection.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Get the values of lpfp in reg CTRL_REG1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_LPF_Mode_Get(uint8_t B_Addr, LPS22HB_LPFp_t *val)
{
	LPS22HB_CtrlReg1_t ctrl_reg1;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_CTRL_REG1, (uint8_t *)&ctrl_reg1, 1);
	
	switch (ctrl_reg1.lpfp)
	{
		case LPS22HB_LPF_ODR_DIV_2:
			*val = LPS22HB_LPF_ODR_DIV_2;
		break;
	
		case LPS22HB_LPF_ODR_DIV_9:
			*val = LPS22HB_LPF_ODR_DIV_9;
		break;
	
		case LPS22HB_LPF_ODR_DIV_20:
			*val = LPS22HB_LPF_ODR_DIV_20;
		break;
	
		default:
			*val = LPS22HB_LPF_ODR_DIV_2;
		break;
	}
	
	return ret;
}

/**
  * @brief  Output data rate selection.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of odr in reg CTRL_REG1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_DataRate_Set(uint8_t B_Addr, LPS22HB_ODR_t val)
{
	LPS22HB_CtrlReg1_t ctrl_reg1;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_CTRL_REG1, (uint8_t *)&ctrl_reg1, 1);
	
	if (ret == 0)
	{
		ctrl_reg1.odr = (uint8_t)val;
		ret = LPS22HB_WriteReg(B_Addr, LPS22HB_CTRL_REG1, (uint8_t *)&ctrl_reg1, 1);
	}
	
	return ret;
}

/**
  * @brief  Output data rate selection.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Get the values of odr in reg CTRL_REG1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_DataRate_Get(uint8_t B_Addr, LPS22HB_ODR_t *val)
{
	LPS22HB_CtrlReg1_t ctrl_reg1;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_CTRL_REG1, (uint8_t *)&ctrl_reg1, 1);
	
	switch (ctrl_reg1.odr)
	{
		case LPS22HB_POWER_DOWN:
			*val = LPS22HB_POWER_DOWN;
		break;
	
		case LPS22HB_ODR_1_Hz:
			*val = LPS22HB_ODR_1_Hz;
		break;
	
		case LPS22HB_ODR_10_Hz:
			*val = LPS22HB_ODR_10_Hz;
		break;
	
		case LPS22HB_ODR_25_Hz:
			*val = LPS22HB_ODR_25_Hz;
		break;
	
		case LPS22HB_ODR_50_Hz:
			*val = LPS22HB_ODR_50_Hz;
		break;
	
		case LPS22HB_ODR_75_Hz:
			*val = LPS22HB_ODR_75_Hz;
		break;
	
		default:
			*val = LPS22HB_ODR_1_Hz;
		break;
	}
	
	return ret;
}

/**
  * @brief  One-shot mode. Device perform a single measure.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of one_shot in reg CTRL_REG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_OneShootTrigger_Set(uint8_t B_Addr, uint8_t val)
{
	LPS22HB_CtrlReg2_t ctrl_reg2;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_CTRL_REG2, (uint8_t *)&ctrl_reg2, 1);
	
	if (ret == 0)
	{
		ctrl_reg2.one_shot = val;
		ret = LPS22HB_WriteReg(B_Addr, LPS22HB_CTRL_REG2, (uint8_t *)&ctrl_reg2, 1);
	}
	
	return ret;
}

/**
  * @brief  One-shot mode. Device perform a single measure.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of one_shot in reg CTRL_REG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_OneShootTrigger_Get(uint8_t B_Addr, uint8_t *val)
{
	LPS22HB_CtrlReg2_t ctrl_reg2;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_CTRL_REG2, (uint8_t *)&ctrl_reg2, 1);
	*val = ctrl_reg2.one_shot;
	
	return ret;
}

/**
  * @brief  pressure_ref:   The Reference pressure value is a 24-bit data
  *         expressed as 2’s complement. The value is used when AUTOZERO
  *         or AUTORIFP function is enabled.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  buff   Buffer that contains data to write
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_PressRef_Set(uint8_t B_Addr, int32_t val)
{
	uint8_t buff[3];
	LPS22HB_Error_et ret;
	
	buff[2] = (uint8_t)((uint32_t)val / 65536U);
	buff[1] = (uint8_t)((uint32_t)val - (buff[2] * 65536U)) / 256U;
	buff[0] = (uint8_t)((uint32_t)val - (buff[2] * 65536U) - (buff[1] * 256U));
	ret =  LPS22HB_WriteReg(B_Addr, LPS22HB_REF_P_XL, buff, 3);
	
	return ret;
}

/**
  * @brief  pressure_ref:   The Reference pressure value is a 24-bit data
  *         expressed as 2’s complement. The value is used when AUTOZERO
  *         or AUTORIFP function is enabled.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  buff   Buffer that stores data read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_PressRef_Get(uint8_t B_Addr, int32_t *val)
{
	uint8_t buff[3];
	LPS22HB_Error_et ret;
	
	ret =  LPS22HB_ReadReg(B_Addr, LPS22HB_REF_P_XL, buff, 3);
	*val = (int32_t)buff[2];
	*val = (*val * 256) + (int32_t)buff[1];
	*val = (*val * 256) + (int32_t)buff[0];
	
	return ret;
}

/**
  * @brief  The pressure offset value is 16-bit data that can be used to
  *         implement one-point calibration (OPC) after soldering.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  buff   Buffer that contains data to write
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_PressOffset_Set(uint8_t B_Addr, int16_t val)
{
	uint8_t buff[2];
	LPS22HB_Error_et ret;
	
	buff[1] = (uint8_t)((uint16_t)val / 256U);
	buff[0] = (uint8_t)((uint16_t)val - (buff[1] * 256U));
	ret =  LPS22HB_WriteReg(B_Addr, LPS22HB_RPDS_L, buff, 2);
	
	return ret;
}

/**
  * @brief  The pressure offset value is 16-bit data that can be used to
  *         implement one-point calibration (OPC) after soldering.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  buff   Buffer that stores data read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_PressOffset_Get(uint8_t B_Addr, int16_t *val)
{
	uint8_t buff[2];
	LPS22HB_Error_et ret;
	
	ret =  LPS22HB_ReadReg(B_Addr, LPS22HB_RPDS_L, buff, 2);
	*val = (int16_t)buff[1];
	*val = (*val * 256) + (int16_t)buff[0];
	
	return ret;
}

/**
  * @brief  Pressure data available.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of p_da in reg STATUS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_PressDataReady_Get(uint8_t B_Addr, uint8_t *val)
{
	LPS22HB_Status_t status;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_STATUS, (uint8_t *)&status, 1);
	*val = status.p_da;
	
	return ret;
}

/**
  * @brief  Temperature data available.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of t_da in reg STATUS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_TempDataReady_Get(uint8_t B_Addr, uint8_t *val)
{
	LPS22HB_Status_t status;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_STATUS, (uint8_t *)&status, 1);
	*val = status.t_da;
	
	return ret;
}

/**
  * @brief  Pressure data overrun.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of p_or in reg STATUS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_PressDataOvr_Get(uint8_t B_Addr, uint8_t *val)
{
	LPS22HB_Status_t status;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_STATUS, (uint8_t *)&status, 1);
	*val = status.p_or;
	
	return ret;
}

/**
  * @brief  Temperature data overrun.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of t_or in reg STATUS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_TempDataOvr_Get(uint8_t B_Addr, uint8_t *val)
{
	LPS22HB_Status_t status;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_STATUS, (uint8_t *)&status, 1);
	*val = status.t_or;
	
	return ret;
}

/**
  * @brief  Pressure output value[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  buff   Buffer that stores data read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_PressRaw_Get(uint8_t B_Addr, uint32_t *buff)
{
	uint8_t reg[3];
	uint32_t tmp = 0;
	uint8_t i;

	if(LPS22HB_ReadReg(B_Addr, LPS22HB_PRESS_OUT_XL, reg, 3))
		return LPS22HB_ERROR;

	/* Build the raw data */
	for(i = 0; i < 3; i++)
		tmp |= (((uint32_t)reg[i]) << (8 * i));

	/* convert the 2's complement 24 bit to 2's complement 32 bit */
	if(tmp & 0x00800000)
		tmp |= 0xFF000000;

	*buff = ((int32_t)tmp);

	return LPS22HB_OK;
/*
 * Original implementation...
 */
/*	uint8_t reg[3];
	LPS22HB_Error_et ret;
	
	ret =  LPS22HB_ReadReg(B_Addr, LPS22HB_PRESS_OUT_XL, reg, 3);
	*buff = reg[2];
	*buff = (*buff * 256) + reg[1];
	*buff = (*buff * 256) + reg[0];
	*buff *= 256;
	
	return ret; */
}

/**
  * @brief  temperature_raw:   Temperature output value[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  buff   Buffer that stores data read.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_TempRaw_Get(uint8_t B_Addr, int16_t *buff)
{
	uint8_t reg[2];
	LPS22HB_Error_et ret;
	
	ret =  LPS22HB_ReadReg(B_Addr, LPS22HB_TEMP_OUT_L, (uint8_t *) reg, 2);
	*buff = reg[1];
	*buff = (*buff * 256) + reg[0];
	
	return ret;
}

/**
  * @brief  Low-pass filter reset register. If the LPFP is active, in
  *         order to avoid the transitory phase, the filter can be
  *         reset by reading this register before generating pressure
  *         measurements.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  buff   Buffer that stores data read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_LPF_Rst_Get(uint8_t B_Addr, uint8_t *buff)
{
	LPS22HB_Error_et ret;
	
	ret =  LPS22HB_ReadReg(B_Addr, LPS22HB_LPFP_RES, (uint8_t *) buff, 1);
	
	return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup    LPS22HB_common
  * @brief       This section group common useful functions
  * @{
  *
  */

/**
  * @brief  Device Who am I[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  buff   Buffer that stores data read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_DeviceId_Get(uint8_t B_Addr, uint8_t *buff)
{
	LPS22HB_Error_et ret;
	
	ret =  LPS22HB_ReadReg(B_Addr, LPS22HB_WHO_AM_I, (uint8_t *) buff, 1);
	
	return ret;
}

/**
  * @brief  Software reset. Restore the default values in user registers[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of swreset in reg CTRL_REG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_SwRst_Set(uint8_t B_Addr, uint8_t val)
{
	LPS22HB_CtrlReg2_t ctrl_reg2;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_CTRL_REG2, (uint8_t *)&ctrl_reg2, 1);
	
	if (ret == 0)
	{
		ctrl_reg2.swreset = val;
		ret = LPS22HB_WriteReg(B_Addr, LPS22HB_CTRL_REG2, (uint8_t *)&ctrl_reg2, 1);
	}
	
	return ret;
}

/**
  * @brief  Software reset. Restore the default values in user registers[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of swreset in reg CTRL_REG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_SwRst_Get(uint8_t B_Addr, uint8_t *val)
{
	LPS22HB_CtrlReg2_t ctrl_reg2;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_CTRL_REG2, (uint8_t *)&ctrl_reg2, 1);
	*val = ctrl_reg2.swreset;
	
	return ret;
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of boot in reg CTRL_REG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_Boot_Set(uint8_t B_Addr, uint8_t val)
{
	LPS22HB_CtrlReg2_t ctrl_reg2;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_CTRL_REG2, (uint8_t *)&ctrl_reg2, 1);
	
	if (ret == 0)
	{
		ctrl_reg2.boot = val;
		ret = LPS22HB_WriteReg(B_Addr, LPS22HB_CTRL_REG2, (uint8_t *)&ctrl_reg2, 1);
	}
	
	return ret;
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of boot in reg CTRL_REG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_Boot_Get(uint8_t B_Addr, uint8_t *val)
{
	LPS22HB_CtrlReg2_t ctrl_reg2;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_CTRL_REG2, (uint8_t *)&ctrl_reg2, 1);
	*val = ctrl_reg2.boot;
	
	return ret;
}

/**
  * @brief  Low current mode.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of lc_en in reg RES_CONF
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_LowPwr_Set(uint8_t B_Addr, uint8_t val)
{
	LPS22HB_ResConf_t res_conf;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_RES_CONF, (uint8_t *)&res_conf, 1);
	
	if (ret == 0)
	{
		res_conf.lc_en = val;
		ret = LPS22HB_WriteReg(B_Addr, LPS22HB_RES_CONF, (uint8_t *)&res_conf, 1);
	}
	
	return ret;
}

/**
  * @brief  Low current mode.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of lc_en in reg RES_CONF
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_LowPwr_Get(uint8_t B_Addr, uint8_t *val)
{
	LPS22HB_ResConf_t res_conf;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_RES_CONF, (uint8_t *)&res_conf, 1);
	*val = res_conf.lc_en;
	
	return ret;
}

/**
  * @brief  If ‘1’ indicates that the Boot (Reboot) phase is running.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of boot_status in reg INT_SOURCE
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_BootStatus_Get(uint8_t B_Addr, uint8_t *val)
{
	LPS22HB_IntSource_t int_source;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_INT_SOURCE, (uint8_t *)&int_source, 1);
	*val = int_source.boot_status;
	
	return ret;
}

/**
  * @brief  All the status bit, FIFO and data generation[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Structure of registers from FIFO_STATUS to STATUS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_DevStatus_Get(uint8_t B_Addr, LPS22HB_DevStat_t *val)
{
	LPS22HB_Error_et ret;
	
	ret =  LPS22HB_ReadReg(B_Addr, LPS22HB_FIFO_STATUS, (uint8_t *) val, 2);
	
	return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup    LPS22HB_interrupts
  * @brief       This section group all the functions that manage interrupts
  * @{
  *
  */

/**
  * @brief  Enable interrupt generation on pressure low/high event.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of pe in reg INTERRUPT_CFG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_InterruptThresholdSign_Set(uint8_t B_Addr, LPS22HB_pe_t val)
{
	LPS22HB_InterruptCfg_t interrupt_cfg;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_INTERRUPT_CFG, (uint8_t *)&interrupt_cfg, 1);
	
	if (ret == 0)
	{
		interrupt_cfg.pe = (uint8_t)val;
		ret = LPS22HB_WriteReg(B_Addr, LPS22HB_INTERRUPT_CFG, (uint8_t *)&interrupt_cfg, 1);
	}
	
	return ret;
}

/**
  * @brief  Enable interrupt generation on pressure low/high event.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Get the values of pe in reg INTERRUPT_CFG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_InterruptThresholdSign_Get(uint8_t B_Addr, LPS22HB_pe_t *val)
{
	LPS22HB_InterruptCfg_t interrupt_cfg;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_INTERRUPT_CFG, (uint8_t *)&interrupt_cfg, 1);
	
	switch (interrupt_cfg.pe)
	{
		case LPS22HB_NO_THRESHOLD:
			*val = LPS22HB_NO_THRESHOLD;
		break;
	
		case LPS22HB_POSITIVE:
			*val = LPS22HB_POSITIVE;
		break;
	
		case LPS22HB_NEGATIVE:
			*val = LPS22HB_NEGATIVE;
		break;
	
		case LPS22HB_BOTH:
			*val = LPS22HB_BOTH;
		break;
	
		default:
			*val = LPS22HB_NO_THRESHOLD;
		break;
	}
	
	return ret;
}

/**
  * @brief  Interrupt request to the INT_SOURCE (25h) register
  *         mode (pulsed / latched) [set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of lir in reg INTERRUPT_CFG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_IntNotificationMode_Set(uint8_t B_Addr, LPS22HB_lir_t val)
{
	LPS22HB_InterruptCfg_t interrupt_cfg;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_INTERRUPT_CFG, (uint8_t *)&interrupt_cfg, 1);
	
	if (ret == 0)
	{
		interrupt_cfg.lir = (uint8_t)val;
		ret = LPS22HB_WriteReg(B_Addr, LPS22HB_INTERRUPT_CFG, (uint8_t *)&interrupt_cfg, 1);
	}
	
	return ret;
}

/**
  * @brief   Interrupt request to the INT_SOURCE (25h) register
  *          mode (pulsed / latched) [get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Get the values of lir in reg INTERRUPT_CFG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_IntNotificationMode_Get(uint8_t B_Addr, LPS22HB_lir_t *val)
{
	LPS22HB_InterruptCfg_t interrupt_cfg;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_INTERRUPT_CFG, (uint8_t *)&interrupt_cfg, 1);
	
	switch (interrupt_cfg.lir)
	{
		case LPS22HB_INT_PULSED:
			*val = LPS22HB_INT_PULSED;
		break;
	
		case LPS22HB_INT_LATCHED:
			*val = LPS22HB_INT_LATCHED;
		break;
	
		default:
			*val = LPS22HB_INT_PULSED;
		break;
	}
	
	return ret;
}

/**
  * @brief  Enable interrupt generation.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of diff_en in reg INTERRUPT_CFG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_IntGenerationSet(uint8_t B_Addr, uint8_t val)
{
	LPS22HB_InterruptCfg_t interrupt_cfg;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_INTERRUPT_CFG, (uint8_t *)&interrupt_cfg, 1);
	
	if (ret == 0)
	{
		interrupt_cfg.diff_en = val;
		ret = LPS22HB_WriteReg(B_Addr, LPS22HB_INTERRUPT_CFG, (uint8_t *)&interrupt_cfg, 1);
	}
	
	return ret;
}

/**
  * @brief  Enable interrupt generation.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of diff_en in reg INTERRUPT_CFG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_IntGenerationGet(uint8_t B_Addr, uint8_t *val)
{
	LPS22HB_InterruptCfg_t interrupt_cfg;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_INTERRUPT_CFG, (uint8_t *)&interrupt_cfg, 1);
	*val = interrupt_cfg.diff_en;
	
	return ret;
}

/**
  * @brief  User-defined threshold value for pressure interrupt event[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  buff   Buffer that contains data to write
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_IntThreshold_Set(uint8_t B_Addr, uint16_t val)
{
	uint8_t buff[2];
	LPS22HB_Error_et ret;
	
	buff[1] = (uint8_t)(val / 256U);
	buff[0] = (uint8_t)(val - (buff[1] * 256U));
	ret =  LPS22HB_WriteReg(B_Addr, LPS22HB_THS_P_L, (uint8_t *) buff, 2);
	
	return ret;
}

/**
  * @brief  User-defined threshold value for pressure interrupt event[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  buff   Buffer that stores data read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_IntThreshold_Get(uint8_t B_Addr, uint16_t *val)
{
	uint8_t buff[2];
	LPS22HB_Error_et ret;
	
	ret =  LPS22HB_ReadReg(B_Addr, LPS22HB_THS_P_L, (uint8_t *) buff, 2);
	*val = buff[1];
	*val = (*val * 256) + buff[0];
	
	return ret;
}

/**
  * @brief  Data signal on INT_DRDY pin control bits.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of int_s in reg CTRL_REG3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_IntPinMode_Set(uint8_t B_Addr, LPS22HB_int_s_t val)
{
	LPS22HB_CtrlReg3_t ctrl_reg3;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_CTRL_REG3, (uint8_t *)&ctrl_reg3, 1);
	
	if (ret == 0)
	{
		ctrl_reg3.int_s = (uint8_t)val;
		ret = LPS22HB_WriteReg(B_Addr, LPS22HB_CTRL_REG3, (uint8_t *)&ctrl_reg3, 1);
	}
	
	return ret;
}

/**
  * @brief  Data signal on INT_DRDY pin control bits.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Get the values of int_s in reg CTRL_REG3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_IntPinMode_Get(uint8_t B_Addr, LPS22HB_int_s_t *val)
{
	LPS22HB_CtrlReg3_t ctrl_reg3;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_CTRL_REG3, (uint8_t *)&ctrl_reg3, 1);
	
	switch (ctrl_reg3.int_s)
	{
		case LPS22HB_DRDY_OR_FIFO_FLAGS:
			*val = LPS22HB_DRDY_OR_FIFO_FLAGS;
		break;
	
		case LPS22HB_HIGH_PRES_INT:
			*val = LPS22HB_HIGH_PRES_INT;
		break;
	
		case LPS22HB_LOW_PRES_INT:
			*val = LPS22HB_LOW_PRES_INT;
		break;
	
		case LPS22HB_EVERY_PRES_INT:
			*val = LPS22HB_EVERY_PRES_INT;
		break;
	
		default:
			*val = LPS22HB_DRDY_OR_FIFO_FLAGS;
		break;
	}
	
	return ret;
}

/**
  * @brief  Data-ready signal on INT_DRDY pin.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of drdy in reg CTRL_REG3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_DrdyOnInt_Set(uint8_t B_Addr, uint8_t val)
{
	LPS22HB_CtrlReg3_t ctrl_reg3;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_CTRL_REG3, (uint8_t *)&ctrl_reg3, 1);
	
	if (ret == 0)
	{
		ctrl_reg3.drdy = val;
		ret = LPS22HB_WriteReg(B_Addr, LPS22HB_CTRL_REG3, (uint8_t *)&ctrl_reg3, 1);
	}
	
	return ret;
}

/**
  * @brief  Data-ready signal on INT_DRDY pin.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of drdy in reg CTRL_REG3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_DrdyOnInt_Get(uint8_t B_Addr, uint8_t *val)
{
	LPS22HB_CtrlReg3_t ctrl_reg3;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_CTRL_REG3, (uint8_t *)&ctrl_reg3, 1);
	*val = ctrl_reg3.drdy;
	
	return ret;
}

/**
  * @brief  FIFO overrun interrupt on INT_DRDY pin.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of f_ovr in reg CTRL_REG3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_FifoOvrOnInt_Set(uint8_t B_Addr, uint8_t val)
{
	LPS22HB_CtrlReg3_t ctrl_reg3;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_CTRL_REG3, (uint8_t *)&ctrl_reg3, 1);
	
	if (ret == 0)
	{
		ctrl_reg3.f_ovr = val;
		ret = LPS22HB_WriteReg(B_Addr, LPS22HB_CTRL_REG3, (uint8_t *)&ctrl_reg3, 1);
	}
	
	return ret;
}

/**
  * @brief  FIFO overrun interrupt on INT_DRDY pin.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of f_ovr in reg CTRL_REG3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_FifoOvrOnInt_Get(uint8_t B_Addr, uint8_t *val)
{
	LPS22HB_CtrlReg3_t ctrl_reg3;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_CTRL_REG3, (uint8_t *)&ctrl_reg3, 1);
	*val = ctrl_reg3.f_ovr;
	
	return ret;
}

/**
  * @brief  FIFO watermark status on INT_DRDY pin.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of f_fth in reg CTRL_REG3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_FifoThresholdOnInt_Set(uint8_t B_Addr, uint8_t val)
{
	LPS22HB_CtrlReg3_t ctrl_reg3;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_CTRL_REG3, (uint8_t *)&ctrl_reg3, 1);
	
	if (ret == 0)
	{
		ctrl_reg3.f_fth = val;
		ret = LPS22HB_WriteReg(B_Addr, LPS22HB_CTRL_REG3, (uint8_t *)&ctrl_reg3, 1);
	}
	
	return ret;
}

/**
  * @brief   FIFO watermark status on INT_DRDY pin.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of f_fth in reg CTRL_REG3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_FifoThresholdOnInt_Get(uint8_t B_Addr, uint8_t *val)
{
	LPS22HB_CtrlReg3_t ctrl_reg3;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_CTRL_REG3, (uint8_t *)&ctrl_reg3, 1);
	*val = ctrl_reg3.f_fth;
	
	return ret;
}

/**
  * @brief  FIFO full flag on INT_DRDY pin.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of f_fss5 in reg CTRL_REG3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_FifoFullOnInt_Set(uint8_t B_Addr, uint8_t val)
{
	LPS22HB_CtrlReg3_t ctrl_reg3;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_CTRL_REG3, (uint8_t *)&ctrl_reg3, 1);
	
	if (ret == 0)
	{
		ctrl_reg3.f_fss5 = val;
		ret = LPS22HB_WriteReg(B_Addr, LPS22HB_CTRL_REG3, (uint8_t *)&ctrl_reg3, 1);
	}
	
	return ret;
}

/**
  * @brief  FIFO full flag on INT_DRDY pin.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of f_fss5 in reg CTRL_REG3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_FifoFullOnInt_Get(uint8_t B_Addr, uint8_t *val)
{
	LPS22HB_CtrlReg3_t ctrl_reg3;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_CTRL_REG3, (uint8_t *)&ctrl_reg3, 1);
	*val = ctrl_reg3.f_fss5;
	
	return ret;
}

/**
  * @brief  Push-pull/open drain selection on interrupt pads.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of pp_od in reg CTRL_REG3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_PinMode_Set(uint8_t B_Addr, LPS22HB_pp_od_t val)
{
	LPS22HB_CtrlReg3_t ctrl_reg3;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_CTRL_REG3, (uint8_t *)&ctrl_reg3, 1);
	
	if (ret == 0)
	{
		ctrl_reg3.pp_od = (uint8_t)val;
		ret = LPS22HB_WriteReg(B_Addr, LPS22HB_CTRL_REG3, (uint8_t *)&ctrl_reg3, 1);
	}
	
	return ret;
}

/**
  * @brief  Push-pull/open drain selection on interrupt pads.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Get the values of pp_od in reg CTRL_REG3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_PinMode_Get(uint8_t B_Addr, LPS22HB_pp_od_t *val)
{
	LPS22HB_CtrlReg3_t ctrl_reg3;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_CTRL_REG3, (uint8_t *)&ctrl_reg3, 1);
	
	switch (ctrl_reg3.pp_od)
	{
		case LPS22HB_PUSH_PULL:
			*val = LPS22HB_PUSH_PULL;
		break;
	
		case LPS22HB_OPEN_DRAIN:
			*val = LPS22HB_OPEN_DRAIN;
		break;
	
		default:
			*val = LPS22HB_PUSH_PULL;
		break;
	}
	
	return ret;
}

/**
  * @brief  Interrupt active-high/low.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of int_h_l in reg CTRL_REG3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_IntPol_Set(uint8_t B_Addr, LPS22HB_int_h_l_t val)
{
	LPS22HB_CtrlReg3_t ctrl_reg3;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_CTRL_REG3, (uint8_t *)&ctrl_reg3, 1);
	
	if (ret == 0)
	{
		ctrl_reg3.int_h_l = (uint8_t)val;
		ret = LPS22HB_WriteReg(B_Addr, LPS22HB_CTRL_REG3, (uint8_t *)&ctrl_reg3, 1);
	}
	
	return ret;
}

/**
  * @brief  Interrupt active-high/low.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Get the values of int_h_l in reg CTRL_REG3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_IntPol_Get(uint8_t B_Addr, LPS22HB_int_h_l_t *val)
{
	LPS22HB_CtrlReg3_t ctrl_reg3;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_CTRL_REG3, (uint8_t *)&ctrl_reg3, 1);
	
	switch (ctrl_reg3.int_h_l)
	{
		case LPS22HB_ACTIVE_HIGH:
			*val = LPS22HB_ACTIVE_HIGH;
		break;
	
		case LPS22HB_ACTIVE_LOW:
			*val = LPS22HB_ACTIVE_LOW;
		break;
	
		default:
			*val = LPS22HB_ACTIVE_HIGH;
		break;
	}
	
	return ret;
}

/**
  * @brief  Interrupt source register[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Register INT_SOURCE
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_IntSource_Get(uint8_t B_Addr, LPS22HB_IntSource_t *val)
{
	LPS22HB_Error_et ret;
	
	ret =  LPS22HB_ReadReg(B_Addr, LPS22HB_INT_SOURCE, (uint8_t *) val, 1);
	
	return ret;
}

/**
  * @brief  Differential pressure high interrupt flag.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of ph in reg INT_SOURCE
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_IntOnPressHigh_Get(uint8_t B_Addr, uint8_t *val)
{
	LPS22HB_IntSource_t int_source;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_INT_SOURCE,
						(uint8_t *)&int_source, 1);
	*val = int_source.ph;
	
	return ret;
}

/**
  * @brief  Differential pressure low interrupt flag.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of pl in reg INT_SOURCE
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_IntOnPressLow_Get(uint8_t B_Addr, uint8_t *val)
{
	LPS22HB_IntSource_t int_source;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_INT_SOURCE,
						(uint8_t *)&int_source, 1);
	*val = int_source.pl;
	
	return ret;
}

/**
  * @brief  Interrupt active flag.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of ia in reg INT_SOURCE
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_IntEvent_Get(uint8_t B_Addr, uint8_t *val)
{
	LPS22HB_IntSource_t int_source;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_INT_SOURCE,
						(uint8_t *)&int_source, 1);
	*val = int_source.ia;
	
	return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup    LPS22HB_fifo
  * @brief       This section group all the functions concerning the
  *              fifo usage
  * @{
  *
  */

/**
  * @brief   Stop on FIFO watermark. Enable FIFO watermark level use.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of stop_on_fth in reg CTRL_REG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_StopOnFifoThreshold_Set(uint8_t B_Addr, uint8_t val)
{
	LPS22HB_CtrlReg2_t ctrl_reg2;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_CTRL_REG2, (uint8_t *)&ctrl_reg2, 1);
	
	if (ret == 0)
	{
		ctrl_reg2.stop_on_fth = val;
		ret = LPS22HB_WriteReg(B_Addr, LPS22HB_CTRL_REG2, (uint8_t *)&ctrl_reg2, 1);
	}
	
	return ret;
}

/**
  * @brief   Stop on FIFO watermark. Enable FIFO watermark level use.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of stop_on_fth in reg CTRL_REG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_StopOnFifoThreshold_Get(uint8_t B_Addr, uint8_t *val)
{
	LPS22HB_CtrlReg2_t ctrl_reg2;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_CTRL_REG2, (uint8_t *)&ctrl_reg2, 1);
	*val = ctrl_reg2.stop_on_fth;
	
	return ret;
}

/**
  * @brief  FIFO enable.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of fifo_en in reg CTRL_REG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_Fifo_Set(uint8_t B_Addr, uint8_t val)
{
	LPS22HB_CtrlReg2_t ctrl_reg2;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_CTRL_REG2, (uint8_t *)&ctrl_reg2, 1);
	
	if (ret == 0)
	{
		ctrl_reg2.fifo_en = val;
		ret = LPS22HB_WriteReg(B_Addr, LPS22HB_CTRL_REG2, (uint8_t *)&ctrl_reg2, 1);
	}
	
	return ret;
}

/**
  * @brief  FIFO enable.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of fifo_en in reg CTRL_REG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_Fifo_Get(uint8_t B_Addr, uint8_t *val)
{
	LPS22HB_CtrlReg2_t ctrl_reg2;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_CTRL_REG2, (uint8_t *)&ctrl_reg2, 1);
	*val = ctrl_reg2.fifo_en;
	
	return ret;
}

/**
  * @brief  FIFO watermark level selection.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of wtm in reg FIFO_CTRL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_FifoWatermark_Set(uint8_t B_Addr, uint8_t val)
{
	LPS22HB_FifoCtrl_t fifo_ctrl;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_FIFO_CTRL, (uint8_t *)&fifo_ctrl, 1);
	
	if (ret == 0)
	{
		fifo_ctrl.wtm = val;
		ret = LPS22HB_WriteReg(B_Addr, LPS22HB_FIFO_CTRL, (uint8_t *)&fifo_ctrl, 1);
	}
	
	return ret;
}

/**
  * @brief  FIFO watermark level selection.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of wtm in reg FIFO_CTRL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_FifoWatermark_Get(uint8_t B_Addr, uint8_t *val)
{
	LPS22HB_FifoCtrl_t fifo_ctrl;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_FIFO_CTRL, (uint8_t *)&fifo_ctrl, 1);
	*val = fifo_ctrl.wtm;
	
	return ret;
}

/**
  * @brief  FIFO mode selection.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of f_mode in reg FIFO_CTRL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_FifoMode_Set(uint8_t B_Addr, LPS22HB_f_mode_t val)
{
	LPS22HB_FifoCtrl_t fifo_ctrl;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_FIFO_CTRL, (uint8_t *)&fifo_ctrl, 1);
	
	if (ret == 0)
	{
		fifo_ctrl.f_mode = (uint8_t)val;
		ret = LPS22HB_WriteReg(B_Addr, LPS22HB_FIFO_CTRL, (uint8_t *)&fifo_ctrl, 1);
	}
	
	return ret;
}

/**
  * @brief  FIFO mode selection.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Get the values of f_mode in reg FIFO_CTRL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_FifoMode_Get(uint8_t B_Addr, LPS22HB_f_mode_t *val)
{
	LPS22HB_FifoCtrl_t fifo_ctrl;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_FIFO_CTRL, (uint8_t *)&fifo_ctrl, 1);
	
	switch (fifo_ctrl.f_mode)
	{
		case LPS22HB_BYPASS_MODE:
			*val = LPS22HB_BYPASS_MODE;
		break;
	
		case LPS22HB_FIFO_MODE:
			*val = LPS22HB_FIFO_MODE;
		break;
	
		case LPS22HB_STREAM_MODE:
			*val = LPS22HB_STREAM_MODE;
		break;
	
		case LPS22HB_STREAM_TO_FIFO_MODE:
			*val = LPS22HB_STREAM_TO_FIFO_MODE;
		break;
	
		case LPS22HB_BYPASS_TO_STREAM_MODE:
			*val = LPS22HB_BYPASS_TO_STREAM_MODE;
		break;
	
		case LPS22HB_DYNAMIC_STREAM_MODE:
			*val = LPS22HB_DYNAMIC_STREAM_MODE;
		break;
	
		case LPS22HB_BYPASS_TO_FIFO_MODE:
			*val = LPS22HB_BYPASS_TO_FIFO_MODE;
		break;
	
		default:
			*val = LPS22HB_BYPASS_MODE;
		break;
	}
	
	return ret;
}

/**
  * @brief  FIFO stored data level.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of fss in reg FIFO_STATUS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_FifoDataLevel_Get(uint8_t B_Addr, uint8_t *val)
{
	LPS22HB_FifoStatus_t fifo_status;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_FIFO_STATUS, (uint8_t *)&fifo_status, 1);
	*val = fifo_status.fss;
	
	return ret;
}

/**
  * @brief  FIFO overrun status.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of ovr in reg FIFO_STATUS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_FifoOvrFlag_Get(uint8_t B_Addr, uint8_t *val)
{
	LPS22HB_FifoStatus_t fifo_status;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_FIFO_STATUS, (uint8_t *)&fifo_status, 1);
	*val = fifo_status.ovr;
	
	return ret;
}

/**
  * @brief  FIFO watermark status.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of fth_fifo in reg FIFO_STATUS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_FifoFthFlag_Get(uint8_t B_Addr, uint8_t *val)
{
	LPS22HB_FifoStatus_t fifo_status;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_FIFO_STATUS, (uint8_t *)&fifo_status, 1);
	*val = fifo_status.fth_fifo;
	
	return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup    LPS22HB_serial_interface
  * @brief       This section group all the functions concerning serial
  *              interface management
  * @{
  *
  */

/**
  * @brief  SPI Serial Interface Mode selection.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of sim in reg CTRL_REG1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_SPI_Mode_Set(uint8_t B_Addr, LPS22HB_sim_t val)
{
	LPS22HB_CtrlReg1_t ctrl_reg1;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_CTRL_REG1, (uint8_t *)&ctrl_reg1, 1);
	
	if (ret == 0)
	{
		ctrl_reg1.sim = (uint8_t)val;
		ret = LPS22HB_WriteReg(B_Addr, LPS22HB_CTRL_REG1, (uint8_t *)&ctrl_reg1, 1);
	}
	
	return ret;
}

/**
  * @brief  SPI Serial Interface Mode selection.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Get the values of sim in reg CTRL_REG1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_SPI_Mode_Get(uint8_t B_Addr, LPS22HB_sim_t *val)
{
	LPS22HB_CtrlReg1_t ctrl_reg1;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_CTRL_REG1, (uint8_t *)&ctrl_reg1, 1);
	
	switch (ctrl_reg1.sim)
	{
		case LPS22HB_SPI_4_WIRE:
			*val = LPS22HB_SPI_4_WIRE;
		break;
	
		case LPS22HB_SPI_3_WIRE:
			*val = LPS22HB_SPI_3_WIRE;
		break;
	
		default:
			*val = LPS22HB_SPI_4_WIRE;
		break;
	}
	
	return ret;
}

/**
  * @brief  Disable I2C interface.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of i2c_dis in reg CTRL_REG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_I2C_Interface_Set(uint8_t B_Addr, LPS22HB_i2c_dis_t val)
{
	LPS22HB_CtrlReg2_t ctrl_reg2;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_CTRL_REG2, (uint8_t *)&ctrl_reg2, 1);
	
	if (ret == 0)
	{
		ctrl_reg2.i2c_dis = (uint8_t)val;
		ret = LPS22HB_WriteReg(B_Addr, LPS22HB_CTRL_REG2, (uint8_t *)&ctrl_reg2, 1);
	}
	
	return ret;
}

/**
  * @brief  Disable I2C interface.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Get the values of i2c_dis in reg CTRL_REG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_I2C_Interface_Get(uint8_t B_Addr, LPS22HB_i2c_dis_t *val)
{
	LPS22HB_CtrlReg2_t ctrl_reg2;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_CTRL_REG2, (uint8_t *)&ctrl_reg2, 1);
	
	switch (ctrl_reg2.i2c_dis)
	{
		case LPS22HB_I2C_ENABLE:
			*val = LPS22HB_I2C_ENABLE;
		break;
	
		case LPS22HB_I2C_DISABLE:
			*val = LPS22HB_I2C_DISABLE;
		break;
	
		default:
			*val = LPS22HB_I2C_ENABLE;
		break;
	}
	
	return ret;
}

/**
  * @brief  Register address automatically incremented during a
  *         multiple byte access with a serial interface (I2C or SPI).[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of if_add_inc in reg CTRL_REG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_AutoAddInc_Set(uint8_t B_Addr, uint8_t val)
{
	LPS22HB_CtrlReg2_t ctrl_reg2;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_CTRL_REG2, (uint8_t *)&ctrl_reg2, 1);
	
	if (ret == 0)
	{
		ctrl_reg2.if_add_inc = val;
		ret = LPS22HB_WriteReg(B_Addr, LPS22HB_CTRL_REG2, (uint8_t *)&ctrl_reg2, 1);
	}
	
	return ret;
}

/**
  * @brief  Register address automatically incremented during a
  *         multiple byte access with a serial interface (I2C or SPI).[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of if_add_inc in reg CTRL_REG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LPS22HB_Error_et LPS22HB_AutoAddInc_Get(uint8_t B_Addr, uint8_t *val)
{
	LPS22HB_CtrlReg2_t ctrl_reg2;
	LPS22HB_Error_et ret;
	
	ret = LPS22HB_ReadReg(B_Addr, LPS22HB_CTRL_REG2, (uint8_t *)&ctrl_reg2, 1);
	*val = ctrl_reg2.if_add_inc;
	
	return ret;
}

/**
  * @}
  *
  */

/**
  * @}
  *
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
