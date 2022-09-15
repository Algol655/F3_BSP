/*
 ******************************************************************************
 * @file    LSM9DS1_Driver.c
 * @author  Sensors Software Solution Team
 * @brief   LSM9DS1 driver file
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 * Last committed from 
 * GITHUB/STMicroelectronics/STMems_Standard_C_drivers/lsm9ds1: 2020-07-03
 ******************************************************************************
 */
 
#include "platform/LSM9DS1_Driver.h"

/** @defgroup LSM9DS1_My_Private_Variables
* @{
*/
// Accelerometer and gyroscope r/w registers
uint8_t LSMDS9_AG_1[10] = 	{
   								LSM9DS1_ACT_THS_VAL,			//0x04
   								LSM9DS1_ACT_DUR_VAL,            //0x05
   								LSM9DS1_INT_GEN_CFG_XL_VAL,     //0x06
   								LSM9DS1_INT_GEN_THS_X_XL_VAL,   //0x07
   								LSM9DS1_INT_GEN_THS_Y_XL_VAL,   //0x08
   								LSM9DS1_INT_GEN_THS_Z_XL_VAL,   //0x09
   								LSM9DS1_INT_GEN_DUR_XL_VAL,     //0x0a
   								LSM9DS1_REFERENCE_G_VAL,        //0x0b
   								LSM9DS1_INT1_CTRL_VAL,          //0x0c
   								LSM9DS1_INT2_CTRL_VAL           //0x0d
							};
uint8_t LSMDS9_AG_2[4] = 	{
								LSM9DS1_CTRL_REG1_G_VAL,        //0x10
   								LSM9DS1_CTRL_REG2_G_VAL,        //0x11
   								LSM9DS1_CTRL_REG3_G_VAL,        //0x12
   								LSM9DS1_ORIENT_CFG_G_VAL        //0x13
   							};
uint8_t LSMDS9_AG_3[7] = 	{
   								LSM9DS1_CTRL_REG4_VAL,          //0x1e
   								LSM9DS1_CTRL_REG5_XL_VAL,       //0x1f
   								LSM9DS1_CTRL_REG6_XL_VAL,       //0x20
   								LSM9DS1_CTRL_REG7_XL_VAL,       //0x21
   								LSM9DS1_CTRL_REG8_VAL,          //0x22
   								LSM9DS1_CTRL_REG9_VAL,          //0x23
   								LSM9DS1_CTRL_REG10_VAL          //0x24
   							};
uint8_t LSMDS9_AG_4[8] = 	{
   								LSM9DS1_INT_GEN_CFG_G_VAL,      //0x30
   								LSM9DS1_INT_GEN_THS_XH_G_VAL,   //0x31
   								LSM9DS1_INT_GEN_THS_XL_G_VAL,   //0x32
   								LSM9DS1_INT_GEN_THS_YH_G_VAL,   //0x33
   								LSM9DS1_INT_GEN_THS_YL_G_VAL,   //0x34
   								LSM9DS1_INT_GEN_THS_ZH_G_VAL,   //0x35
   								LSM9DS1_INT_GEN_THS_ZL_G_VAL,   //0x36
   								LSM9DS1_INT_GEN_DUR_G_VAL       //0x37
   							};

// Magnetometer r/w registers
uint8_t LSMDS9_M_1[6] =		{
   								LSM9DS1_OFFSET_X_REG_L_M_VAL,   //0x05
   								LSM9DS1_OFFSET_X_REG_H_M_VAL,   //0x06
   								LSM9DS1_OFFSET_Y_REG_L_M_VAL,   //0x07
   								LSM9DS1_OFFSET_Y_REG_H_M_VAL,   //0x08
   								LSM9DS1_OFFSET_Z_REG_L_M_VAL,   //0x09
   								LSM9DS1_OFFSET_Z_REG_H_M_VAL,   //0x0a
   							};
uint8_t LSMDS9_M_2[5] =		{
   								LSM9DS1_CTRL_REG1_M_VAL,        //0x20
   								LSM9DS1_CTRL_REG2_M_VAL,        //0x21
   								LSM9DS1_CTRL_REG3_M_VAL,        //0x22
   								LSM9DS1_CTRL_REG4_M_VAL,        //0x23
   								LSM9DS1_CTRL_REG5_M_VAL         //0x24
   							};

uint8_t LSMDS9_AG_5[1] = 	{
   								LSM9DS1_FIFO_CTRL_VAL          	//0x2e
   							};
uint8_t LSMDS9_M_3[1] =		{
   								LSM9DS1_INT_CFG_M_VAL,          //0x30
   							};
/**
* @}
*/

/**
  * @defgroup    LSM9DS1
  * @brief       This file provides a set of functions needed to drive the
  *              lsm9ds1 enhanced inertial module.
  * @{
  *
  */

/**
  * @defgroup    LSM9DS1_Interfaces_Functions
  * @brief       This section provide a set of functions used to read and
  *              write a generic register of the device.
  *              MANDATORY: return 0 -> no Error.
  * @{
  *
  */

/**
  * @brief  Read generic device register
  *
  * @param  B_Addr   read / write interface definitions(ptr)
  * @param  reg   register to read
  * @param  data  pointer to buffer that store the data read(ptr)
  * @param  len   number of consecutive register to read
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */ /*
LSM9DS1_Error_et LSM9DS1_ReadReg(LSM9DS1_CTX_t* B_Addr, uint8_t reg, uint8_t* data, uint16_t len)
{
  LSM9DS1_Error_et ret;
  ret = B_Addr->read_reg(B_Addr->handle, reg, data, len);
  return ret;
} */
LSM9DS1_Error_et LSM9DS1_ReadReg(uint8_t B_Addr, uint8_t RegAddr, uint16_t NumByteToRead, uint8_t *Data)
{

//if ( NumByteToRead > 1 ) RegAddr |= 0x80;
  if (I2C_ReadData(B_Addr, RegAddr, &Data[0], NumByteToRead))
    return LSM9DS1_ERROR;
  else
    return LSM9DS1_OK;
}

/**
  * @brief  Write generic device register
  *
  * @param  B_Addr   read / write interface definitions(ptr)
  * @param  reg   register to write
  * @param  data  pointer to data to write in register reg(ptr)
  * @param  len   number of consecutive register to write
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */ /*
LSM9DS1_Error_et LSM9DS1_WriteReg(LSM9DS1_CTX_t* B_Addr, uint8_t reg, uint8_t* data,  uint16_t len)
{
  LSM9DS1_Error_et ret;
  ret = B_Addr->write_reg(B_Addr->handle, reg, data, len);
  return ret;
} */
LSM9DS1_Error_et LSM9DS1_WriteReg(uint8_t B_Addr, uint8_t RegAddr, uint16_t NumByteToWrite, uint8_t *Data)
{

//if ( NumByteToWrite > 1 ) RegAddr |= 0x80;
  if (I2C_WriteData(B_Addr, RegAddr, &Data[0], NumByteToWrite))
    return LSM9DS1_ERROR;
  else
    return LSM9DS1_OK;
}

/*******************************************************************************
* Function Name	: LSM9DS1_WriteReg
* Description   : Generic Writing function in DMA mode. It must be fullfilled with either
*         		: I2C or SPI writing function
* Input       	: Register Address, Data to be written
* Output      	: None
* Return      	: Status [LSM9DS1_ERROR, LSM9DS1_OK]
*******************************************************************************/
LSM9DS1_Error_et LSM9DS1_WriteReg_DMA(uint8_t B_Addr, uint8_t RegAddr, uint16_t NumByteToWrite, uint8_t *Data)
{

//if ( NumByteToWrite > 1 ) RegAddr |= 0x80;
  if (I2C_WriteData_DMA(B_Addr, RegAddr, &Data[0], NumByteToWrite))
    return LSM9DS1_ERROR;
  else
    return LSM9DS1_OK;
}

/*******************************************************************************
* Function Name	: LSM9DS1_Init
* Description   : Generic Writing function. It must be fullfilled with either
*         		: I2C or SPI writing function
* Input       	: Register Address, Data to be written
* Output      	: None
* Return      	: Status [LSM9DS1_ERROR, LSM9DS1_OK]
*******************************************************************************/
LSM9DS1_Error_et LSM9DS1_Init(uint8_t B_Addr, uint8_t RegAddr, uint8_t* RegValues, uint8_t Size)
{
	if (LSM9DS1_WriteReg(B_Addr, RegAddr, Size, &RegValues[0]))
		return LSM9DS1_ERROR;

	return LSM9DS1_OK;
}

/*******************************************************************************
* Function Name	: MX_LSM9DS1_Init
* Description   : LSM9DS1 Global init
*         		: I2C or SPI writing function
* Input       	: None
* Output      	: None
* Return      	: Status [LSM9DS1_ERROR, LSM9DS1_OK]
*******************************************************************************/
LSM9DS1_Error_et MX_LSM9DS1_Init()
{
	LSM9DS1_ID_t DevID;

	// Accelerometer and gyroscope r/w registers
	if (LSM9DS1_Dev_Id_Get(LSM9DS1_MAG_I2C_ADD_L, LSM9DS1_IMU_I2C_ADD_L, &DevID))
		return LSM9DS1_ERROR;
	if ((DevID.imu != LSM9DS1_WHO_AM_I_VAL) || (DevID.mag != LSM9DS1_WHO_AM_I_M_VAL))
		return LSM9DS1_ERROR;

	if (LSM9DS1_Init(LSM9DS1_IMU_I2C_ADD_L, LSM9DS1_ACT_THS, &LSMDS9_AG_1[0], 10))
		return LSM9DS1_ERROR;
	HAL_Delay(i2c_delay);
//	LSM9DS1_ReadReg(LSM9DS1_IMU_I2C_ADD_L, LSM9DS1_INT1_CTRL, 1, &tmp);			//Test

	if (LSM9DS1_Init(LSM9DS1_IMU_I2C_ADD_L, LSM9DS1_CTRL_REG1_G, &LSMDS9_AG_2[0], 4))
		return LSM9DS1_ERROR;
	HAL_Delay(i2c_delay);
//	LSM9DS1_ReadReg(LSM9DS1_IMU_I2C_ADD_L, LSM9DS1_CTRL_REG3_G, 1, &tmp);		//Test

	if (LSM9DS1_Init(LSM9DS1_IMU_I2C_ADD_L, LSM9DS1_CTRL_REG4, &LSMDS9_AG_3[0], 7))
		return LSM9DS1_ERROR;
	HAL_Delay(i2c_delay);
//	LSM9DS1_ReadReg(LSM9DS1_IMU_I2C_ADD_L, LSM9DS1_CTRL_REG8, 1, &tmp);			//Test

	if (LSM9DS1_Init(LSM9DS1_IMU_I2C_ADD_L, LSM9DS1_INT_GEN_CFG_G, &LSMDS9_AG_4[0], 8))
		return LSM9DS1_ERROR;
	HAL_Delay(i2c_delay);
//	LSM9DS1_ReadReg(LSM9DS1_IMU_I2C_ADD_L, LSM9DS1_INT_GEN_THS_ZL_G, 1, &tmp);	//Test

	if (LSM9DS1_WriteReg(LSM9DS1_IMU_I2C_ADD_L, LSM9DS1_FIFO_CTRL, 1, (uint8_t*)LSM9DS1_FIFO_CTRL_VAL))
		return LSM9DS1_ERROR;
	HAL_Delay(i2c_delay);

	// Magnetometer r/w registers
	if (LSM9DS1_Init(LSM9DS1_MAG_I2C_ADD_L, LSM9DS1_OFFSET_X_REG_L_M, &LSMDS9_M_1[0], 6))
		return LSM9DS1_ERROR;
	HAL_Delay(i2c_delay);
//	LSM9DS1_ReadReg(LSM9DS1_MAG_I2C_ADD_L, LSM9DS1_OFFSET_Z_REG_L_M, 1, &tmp);	//Test

	if (LSM9DS1_Init(LSM9DS1_MAG_I2C_ADD_L, LSM9DS1_CTRL_REG1_M, &LSMDS9_M_2[0], 5))
		return LSM9DS1_ERROR;
	HAL_Delay(i2c_delay);
//	LSM9DS1_ReadReg(LSM9DS1_MAG_I2C_ADD_L, LSM9DS1_CTRL_REG4_M, 1, &tmp);		//Test

	if (LSM9DS1_WriteReg(LSM9DS1_MAG_I2C_ADD_L, LSM9DS1_INT_CFG_M, 1, (uint8_t*)LSM9DS1_INT_CFG_M_VAL))
		return LSM9DS1_ERROR;

	return LSM9DS1_OK;
}

/**
 * @brief 	Get the LSM9DS1 temperature sensor
 * @param 	temperature_raw pointer where the raw values of the axes are written
 * @param 	temperature pointer where the values of the axes in °C are written
 * 			(LSM9DS1 temperature sensor resolution is 1 °C, while
 * 			 LPS25HB temperature sensor resolution is 1/10 °C !!)
 * @retval 	LSM9DS1_OK in case of success
 * @retval 	LSM9DS1_ERROR in case of failure
 */
LSM9DS1_Error_et LSM9DS1_Temperature_Get(int16_t *temperature_raw, int16_t *temperature)
{

  uint8_t regValue[2];
  int16_t dataRaw;

  /* Read raw data from LSM9DS1 Gyro output registers */
  if (LSM9DS1_Temperature_Raw_Get(LSM9DS1_IMU_I2C_ADD_L, regValue))
	  return LSM9DS1_ERROR;

  /* Format the data. */
  dataRaw = ((((int16_t )regValue[1]) << 8) + (int16_t)regValue[0]);
  *temperature_raw = dataRaw;

  /* Calculate the data. */		//Done in LSM9DS1_From_Fsxxxdps_To_°C(dataRaw[X]) functions
  *temperature = (int16_t)LSM9DS1_From_LSB_To_Celsius(dataRaw);

  return LSM9DS1_OK;
}

/**
 * @brief Get the LSM9DS1 gyroscope sensor axes
 * @param angular_velocity_raw pointer where the raw values of the axes are written
 * @param angular_velocity pointer where the values of the axes in mdps are written
 * @retval LSM9DS1_OK in case of success
 * @retval LSM9DS1_ERROR in case of failure
 */
LSM9DS1_Error_et LSM9DS1_Angular_Rate_Get(SensorAxesRaw_t *angular_velocity_raw, SensorAxes_t *angular_velocity)
{

  uint8_t regValue[6];
  int16_t dataRaw[3];
  LSM9DS1_GY_FS_t Gy_FS;
//  float   sensitivity = 0;

  /* Read raw data from LSM9DS1 Gyro output registers */
  if (LSM9DS1_Angular_Rate_Raw_Get(LSM9DS1_IMU_I2C_ADD_L, regValue))
	  return LSM9DS1_ERROR;

  /* Format the data. */
  dataRaw[0] = ((((int16_t )regValue[1]) << 8) + (int16_t)regValue[0]);
  dataRaw[1] = ((((int16_t )regValue[3]) << 8) + (int16_t)regValue[2]);
  dataRaw[2] = ((((int16_t )regValue[5]) << 8) + (int16_t)regValue[4]);
  /* Set the raw data. */
  angular_velocity_raw->AXIS_X = dataRaw[0];
  angular_velocity_raw->AXIS_Y = dataRaw[1];
  angular_velocity_raw->AXIS_Z = dataRaw[2];

  /* Get LSM9DS1 actual sensitivity. */
  if (LSM9DS1_Gy_Full_Scale_Get(LSM9DS1_IMU_I2C_ADD_L, &Gy_FS))
	  return LSM9DS1_ERROR;

  switch (Gy_FS)
  {
  	  case LSM9DS1_245dps:
  	  {
  		  angular_velocity->AXIS_X = (int32_t )LSM9DS1_From_Fs245dps_To_mdps(dataRaw[0]);
  		  angular_velocity->AXIS_Y = (int32_t )LSM9DS1_From_Fs245dps_To_mdps(dataRaw[1]);
  		  angular_velocity->AXIS_Z = (int32_t )LSM9DS1_From_Fs245dps_To_mdps(dataRaw[2]);
  		  break;
  	  }
  	  case LSM9DS1_500dps:
  	  {
  		  angular_velocity->AXIS_X = (int32_t )LSM9DS1_From_Fs500dps_To_mdps(dataRaw[0]);
  		  angular_velocity->AXIS_Y = (int32_t )LSM9DS1_From_Fs500dps_To_mdps(dataRaw[1]);
  		  angular_velocity->AXIS_Z = (int32_t )LSM9DS1_From_Fs500dps_To_mdps(dataRaw[2]);
  		  break;
  	  }
  	  case LSM9DS1_2000dps:
  	  {
  		  angular_velocity->AXIS_X = (int32_t )LSM9DS1_From_Fs2000dps_To_mdps(dataRaw[0]);
  		  angular_velocity->AXIS_Y = (int32_t )LSM9DS1_From_Fs2000dps_To_mdps(dataRaw[1]);
  		  angular_velocity->AXIS_Z = (int32_t )LSM9DS1_From_Fs2000dps_To_mdps(dataRaw[2]);
  		  break;
  	  }
  	  default:
  	  {
  		  angular_velocity->AXIS_X = -1;
  		  angular_velocity->AXIS_Y = -1;
  		  angular_velocity->AXIS_Z = -1;
  		  return LSM9DS1_ERROR;
  	  }
  }

  /* Calculate the data. */		//Done in LSM9DS1_From_Fsxxxdps_To_mdps(dataRaw[X]) functions
//  angular_velocity->AXIS_X = (int32_t )(dataRaw[0] * sensitivity);
//  angular_velocity->AXIS_Y = (int32_t )(dataRaw[1] * sensitivity);
//  angular_velocity->AXIS_Z = (int32_t )(dataRaw[2] * sensitivity);

  return LSM9DS1_OK;
}

/**
 * @brief Get the LSM9DS1 accelerometer sensor axes
 * @param acceleration_raw pointer where the raw values of the axes are written
 * @param acceletion pointer where the values of the axes in mg are written
 * @retval LSM9DS1_OK in case of success
 * @retval LSM9DS1_ERROR in case of failure
 */
LSM9DS1_Error_et LSM9DS1_Acceleration_Get(SensorAxesRaw_t *acceleration_raw, SensorAxes_t *acceleration)
{

  uint8_t regValue[6];
  int16_t dataRaw[3];
  LSM9DS1_XL_FS_t XL_FS;
//  float   sensitivity = 0;

  /* Read raw data from LSM9DS1 Accelerometer output registers */
  if (LSM9DS1_Acceleration_Raw_Get(LSM9DS1_IMU_I2C_ADD_L, regValue))
	  return LSM9DS1_ERROR;

  /* Format the data. */
  dataRaw[0] = ((((int16_t )regValue[1]) << 8) + (int16_t)regValue[0]);
  dataRaw[1] = ((((int16_t )regValue[3]) << 8) + (int16_t)regValue[2]);
  dataRaw[2] = ((((int16_t )regValue[5]) << 8) + (int16_t)regValue[4]);
  /* Set the raw data. */
  acceleration_raw->AXIS_X = dataRaw[0];
  acceleration_raw->AXIS_Y = dataRaw[1];
  acceleration_raw->AXIS_Z = dataRaw[2];

  /* Get LSM9DS1 actual sensitivity. */
  if (LSM9DS1_Xl_Full_Scale_Get(LSM9DS1_IMU_I2C_ADD_L, &XL_FS))
	  return LSM9DS1_ERROR;

  switch (XL_FS)
  {
  	  case LSM9DS1_2g:
  	  {
  		  acceleration->AXIS_X = (int32_t )LSM9DS1_From_Fs2g_To_mg(dataRaw[0]);
  		  acceleration->AXIS_Y = (int32_t )LSM9DS1_From_Fs2g_To_mg(dataRaw[1]);
  		  acceleration->AXIS_Z = (int32_t )LSM9DS1_From_Fs2g_To_mg(dataRaw[2]);
  		  break;
  	  }
  	  case LSM9DS1_4g:
  	  {
  		  acceleration->AXIS_X = (int32_t )LSM9DS1_From_Fs4g_To_mg(dataRaw[0]);
  		  acceleration->AXIS_Y = (int32_t )LSM9DS1_From_Fs4g_To_mg(dataRaw[1]);
  		  acceleration->AXIS_Z = (int32_t )LSM9DS1_From_Fs4g_To_mg(dataRaw[2]);
  		  break;
  	  }
  	  case LSM9DS1_8g:
  	  {
  		  acceleration->AXIS_X = (int32_t )LSM9DS1_From_Fs8g_To_mg(dataRaw[0]);
  		  acceleration->AXIS_Y = (int32_t )LSM9DS1_From_Fs8g_To_mg(dataRaw[1]);
  		  acceleration->AXIS_Z = (int32_t )LSM9DS1_From_Fs8g_To_mg(dataRaw[2]);
  		  break;
  	  }
  	  case LSM9DS1_16g:
  	  {
  		  acceleration->AXIS_X = (int32_t )LSM9DS1_From_Fs16g_To_mg(dataRaw[0]);
  		  acceleration->AXIS_Y = (int32_t )LSM9DS1_From_Fs16g_To_mg(dataRaw[1]);
  		  acceleration->AXIS_Z = (int32_t )LSM9DS1_From_Fs16g_To_mg(dataRaw[2]);
  		  break;
  	  }
  	  default:
  	  {
  		  acceleration->AXIS_X = -1;
  		  acceleration->AXIS_Y = -1;
  		  acceleration->AXIS_Z = -1;
  		  return LSM9DS1_ERROR;
  	  }
  }

  /* Calculate the data. */		//Done in LSM9DS1_From_Fsxxg_To_mg(dataRaw[X]) functions
//  acceleration->AXIS_X = (int32_t )(dataRaw[0] * sensitivity);
//  acceleration->AXIS_Y = (int32_t )(dataRaw[1] * sensitivity);
//  acceleration->AXIS_Z = (int32_t )(dataRaw[2] * sensitivity);

  return LSM9DS1_OK;
}

/**
 * @brief Get the LSM9DS1 magnetic field sensor axes
 * @param magnetic_field_raw pointer where the raw values of the axes are written
 * @param magnetic_field pointer where the values of the axes in mGauss are written
 * @retval LSM9DS1_OK in case of success
 * @retval LSM9DS1_ERROR in case of failure
 */
LSM9DS1_Error_et LSM9DS1_Magnetic_Get(SensorAxesRaw_t *magnetic_field_raw, SensorAxes_t *magnetic_field)
{

  uint8_t regValue[6];
  int16_t dataRaw[3];
  LSM9DS1_MAG_FS_t Mag_FS;
//  float   sensitivity = 0;

  /* Read raw data from LSM9DS1 Magnetometer output registers */
  if (LSM9DS1_Magnetic_Raw_Get(LSM9DS1_MAG_I2C_ADD_L, regValue))
	  return LSM9DS1_ERROR;

  /* Format the data. */
  dataRaw[0] = ((((int16_t )regValue[1]) << 8) + (int16_t)regValue[0]);
  dataRaw[1] = ((((int16_t )regValue[3]) << 8) + (int16_t)regValue[2]);
  dataRaw[2] = ((((int16_t )regValue[5]) << 8) + (int16_t)regValue[4]);
  /* Set the raw data. */
  magnetic_field_raw->AXIS_X = dataRaw[0];
  magnetic_field_raw->AXIS_Y = dataRaw[1];
  magnetic_field_raw->AXIS_Z = dataRaw[2];

  /* Get LSM9DS1 actual sensitivity. */
  if (LSM9DS1_Mag_Full_Scale_Get(LSM9DS1_MAG_I2C_ADD_L, &Mag_FS))
	  return LSM9DS1_ERROR;

  switch (Mag_FS)
  {
  	  case LSM9DS1_4Ga:
  	  {
  		  magnetic_field->AXIS_X = (int32_t )LSM9DS1_From_Fs4gauss_To_mG(dataRaw[0]);
  		  magnetic_field->AXIS_Y = (int32_t )LSM9DS1_From_Fs4gauss_To_mG(dataRaw[1]);
  		  magnetic_field->AXIS_Z = (int32_t )LSM9DS1_From_Fs4gauss_To_mG(dataRaw[2]);
  		  break;
  	  }
  	  case LSM9DS1_8Ga:
  	  {
  		  magnetic_field->AXIS_X = (int32_t )LSM9DS1_From_Fs8gauss_To_mG(dataRaw[0]);
  		  magnetic_field->AXIS_Y = (int32_t )LSM9DS1_From_Fs8gauss_To_mG(dataRaw[1]);
  		  magnetic_field->AXIS_Z = (int32_t )LSM9DS1_From_Fs8gauss_To_mG(dataRaw[2]);
  		  break;
  	  }
  	  case LSM9DS1_12Ga:
  	  {
  		  magnetic_field->AXIS_X = (int32_t )LSM9DS1_From_Fs12gauss_To_mG(dataRaw[0]);
  		  magnetic_field->AXIS_Y = (int32_t )LSM9DS1_From_Fs12gauss_To_mG(dataRaw[1]);
  		  magnetic_field->AXIS_Z = (int32_t )LSM9DS1_From_Fs12gauss_To_mG(dataRaw[2]);
  		  break;
  	  }
  	  case LSM9DS1_16Ga:
  	  {
  		  magnetic_field->AXIS_X = (int32_t )LSM9DS1_From_Fs16gauss_To_mG(dataRaw[0]);
  		  magnetic_field->AXIS_Y = (int32_t )LSM9DS1_From_Fs16gauss_To_mG(dataRaw[1]);
  		  magnetic_field->AXIS_Z = (int32_t )LSM9DS1_From_Fs16gauss_To_mG(dataRaw[2]);
  		  break;
  	  }
  	  default:
  	  {
  		  magnetic_field->AXIS_X = -1;
  		  magnetic_field->AXIS_Y = -1;
  		  magnetic_field->AXIS_Z = -1;
  		  return LSM9DS1_ERROR;
  	  }
  }

  /* Calculate the data. */		//Done in LSM9DS1_From_Fsxxg_To_mg(dataRaw[X]) functions
//  magnetic_field->AXIS_X = (int32_t )(dataRaw[0] * sensitivity);
//  magnetic_field->AXIS_Y = (int32_t )(dataRaw[1] * sensitivity);
//  magnetic_field->AXIS_Z = (int32_t )(dataRaw[2] * sensitivity);

  return LSM9DS1_OK;
}

/**
* @brief  Get the values of the last measurement.
* @param  Measurement_Value stores all IMU measurement
* @retval Status [LSM9DS1_ERROR, LSM9DS1_OK]
*/
LSM9DS1_Error_et LSM9DS1_Get_Measurement(LSM9DS1_MeasureTypeDef_st *Measurement_Value)
{
  int16_t 			T_out, TRow_out;
  SensorAxesRaw_t 	GyRaw_out, AccRaw_out, MagRaw_out;
  SensorAxes_t 		Gy_out, Acc_out, Mag_out;

  if(LSM9DS1_Temperature_Get(&TRow_out, &T_out))
    return LSM9DS1_ERROR;

  Measurement_Value->T_Out = T_out;

  if(LSM9DS1_Angular_Rate_Get(&GyRaw_out, &Gy_out))
    return LSM9DS1_ERROR;

  Measurement_Value->GyRaw_Out = GyRaw_out;
  Measurement_Value->Gy_Out = Gy_out;

  if(LSM9DS1_Acceleration_Get(&AccRaw_out, &Acc_out))
    return LSM9DS1_ERROR;

  Measurement_Value->AccRaw_Out = AccRaw_out;
  Measurement_Value->Acc_Out = Acc_out;

  if(LSM9DS1_Magnetic_Get(&MagRaw_out, &Mag_out))
    return LSM9DS1_ERROR;

  Measurement_Value->MagRaw_Out = MagRaw_out;
  Measurement_Value->Mag_Out = Mag_out;

  return LSM9DS1_OK;
}

/**
  * @}
  *
  */

/**
  * @defgroup    LSM9DS1_Sensitivity
  * @brief       These functions convert raw-data into engineering units.
  * @{
  *
  */

double_t LSM9DS1_From_Fs2g_To_mg(int16_t lsb)
{
  return ((double_t)lsb *0.061);
}

double_t LSM9DS1_From_Fs4g_To_mg(int16_t lsb)
{
  return ((double_t)lsb *0.122);
}

double_t LSM9DS1_From_Fs8g_To_mg(int16_t lsb)
{
  return ((double_t)lsb *0.244);
}

double_t LSM9DS1_From_Fs16g_To_mg(int16_t lsb)
{
  return ((double_t)lsb *0.732);
}

double_t LSM9DS1_From_Fs245dps_To_mdps(int16_t lsb)
{
  return ((double_t)lsb *8.75);
}

double_t LSM9DS1_From_Fs500dps_To_mdps(int16_t lsb)
{
  return ((double_t)lsb *17.50);
}

double_t LSM9DS1_From_Fs2000dps_To_mdps(int16_t lsb)
{
  return ((double_t)lsb *70.0);
}

double_t LSM9DS1_From_Fs4gauss_To_mG(int16_t lsb)
{
  return ((double_t)lsb *0.14);
}

double_t LSM9DS1_From_Fs8gauss_To_mG(int16_t lsb)
{
  return ((double_t)lsb *0.29);
}

double_t LSM9DS1_From_Fs12gauss_To_mG(int16_t lsb)
{
  return ((double_t)lsb *0.43);
}

double_t LSM9DS1_From_Fs16gauss_To_mG(int16_t lsb)
{
  return ((double_t)lsb *0.58);
}

double_t LSM9DS1_From_LSB_To_Celsius(int16_t lsb)
{
//return (((double_t)lsb / 16.0) + 25.0);
  return (((double_t)lsb / 16.00) + LSM9DS1_TEMP_BIAS);
}

/**
  * @}
  *
  */

/**
  * @defgroup   LSM9DS1_Data_generation
  * @brief      This section groups all the functions concerning data
  *             generation
  * @{
  *
  */

/**
  * @brief  Gyroscope full-scale selection.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "fs_g" in reg LSM9DS1.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Gy_Full_Scale_Set(uint8_t B_Addr, LSM9DS1_GY_FS_t val)
{
  LSM9DS1_CTRL_REG1_G_t ctrl_reg1_g;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG1_G, 1, (uint8_t*)&ctrl_reg1_g);
  if(ret == 0)
  {
    ctrl_reg1_g.fs_g = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_CTRL_REG1_G, 1, (uint8_t*)&ctrl_reg1_g);
  }
  return ret;
}

/**
  * @brief  Gyroscope full-scale selection.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Get the values of fs_g in reg CTRL_REG1_G.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Gy_Full_Scale_Get(uint8_t B_Addr, LSM9DS1_GY_FS_t *val)
{
  LSM9DS1_CTRL_REG1_G_t ctrl_reg1_g;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG1_G, 1, (uint8_t*)&ctrl_reg1_g);
  switch (ctrl_reg1_g.fs_g)
  {
    case LSM9DS1_245dps:
      *val = LSM9DS1_245dps;
      break;
    case LSM9DS1_500dps:
      *val = LSM9DS1_500dps;
      break;
    case LSM9DS1_2000dps:
      *val = LSM9DS1_2000dps;
      break;
    default:
      *val = LSM9DS1_245dps;
      break;
  }
  return ret;
}

/**
  * @brief  Data rate selection when both the accelerometer and gyroscope
  *         are activated.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "odr_g" in reg LSM9DS1.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_IMU_Data_Rate_Set(uint8_t B_Addr, lsm9ds1_imu_odr_t val)
{
  LSM9DS1_CTRL_REG1_G_t ctrl_reg1_g;
  LSM9DS1_CTRL_REG6_XL_t ctrl_reg6_xl;
  LSM9DS1_CTRL_REG3_G_t ctrl_reg3_g;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG1_G, 1, (uint8_t*)&ctrl_reg1_g);
  if(ret == 0)
  {
    ctrl_reg1_g.odr_g = (uint8_t)val & 0x07U;
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_CTRL_REG1_G, 1, (uint8_t*)&ctrl_reg1_g);
  }
  if(ret == 0)
  {
	ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG6_XL, 1, (uint8_t*)&ctrl_reg6_xl);
  }
  if(ret == 0)
  {
    ctrl_reg6_xl.odr_xl = (((uint8_t)val & 0x70U) >> 4);
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_CTRL_REG6_XL, 1, (uint8_t*)&ctrl_reg6_xl);
  }
  if(ret == 0)
  {
    ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG3_G, 1, (uint8_t*)&ctrl_reg3_g);
  }
  if(ret == 0)
  {
    ctrl_reg3_g.lp_mode = (((uint8_t)val & 0x80U) >> 7);
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_CTRL_REG3_G, 1, (uint8_t*)&ctrl_reg3_g);
  }

  return ret;
}

/**
  * @brief  Data rate selection when both the accelerometer and gyroscope
  *         are activated.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Get the values of odr_g in reg CTRL_REG1_G.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_IMU_Data_Rate_Get(uint8_t B_Addr, lsm9ds1_imu_odr_t *val)
{
  LSM9DS1_CTRL_REG1_G_t ctrl_reg1_g;
  LSM9DS1_CTRL_REG6_XL_t ctrl_reg6_xl;
  LSM9DS1_CTRL_REG3_G_t ctrl_reg3_g;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG1_G, 1, (uint8_t*)&ctrl_reg1_g);
  if(ret == 0)
  {
    ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG6_XL, 1, (uint8_t*)&ctrl_reg6_xl);
  }
  if(ret == 0)
  {
    ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG3_G, 1, (uint8_t*)&ctrl_reg3_g);
  }
  switch ((ctrl_reg3_g.lp_mode << 7) | (ctrl_reg6_xl.odr_xl << 4) | ctrl_reg1_g.odr_g)
  {
    case LSM9DS1_IMU_OFF:
      *val = LSM9DS1_IMU_OFF;
      break;
    case LSM9DS1_GY_OFF_XL_10Hz:
      *val = LSM9DS1_GY_OFF_XL_10Hz;
      break;
    case LSM9DS1_GY_OFF_XL_50Hz:
      *val = LSM9DS1_GY_OFF_XL_50Hz;
      break;
    case LSM9DS1_GY_OFF_XL_119Hz:
      *val = LSM9DS1_GY_OFF_XL_119Hz;
      break;
    case LSM9DS1_GY_OFF_XL_238Hz:
      *val = LSM9DS1_GY_OFF_XL_238Hz;
      break;
    case LSM9DS1_GY_OFF_XL_476Hz:
      *val = LSM9DS1_GY_OFF_XL_476Hz;
      break;
    case LSM9DS1_GY_OFF_XL_952Hz:
      *val = LSM9DS1_GY_OFF_XL_952Hz;
      break;
    case LSM9DS1_XL_OFF_GY_14Hz9:
      *val = LSM9DS1_XL_OFF_GY_14Hz9;
      break;
    case LSM9DS1_XL_OFF_GY_59Hz5:
      *val = LSM9DS1_XL_OFF_GY_59Hz5;
      break;
    case LSM9DS1_XL_OFF_GY_119Hz:
      *val = LSM9DS1_XL_OFF_GY_119Hz;
      break;
    case LSM9DS1_XL_OFF_GY_238Hz:
      *val = LSM9DS1_XL_OFF_GY_238Hz;
      break;
    case LSM9DS1_XL_OFF_GY_476Hz:
      *val = LSM9DS1_XL_OFF_GY_476Hz;
      break;
    case LSM9DS1_XL_OFF_GY_952Hz:
      *val = LSM9DS1_XL_OFF_GY_952Hz;
      break;
    case LSM9DS1_IMU_14Hz9:
      *val = LSM9DS1_IMU_14Hz9;
      break;
    case LSM9DS1_IMU_59Hz5:
      *val = LSM9DS1_IMU_59Hz5;
      break;
    case LSM9DS1_IMU_119Hz:
      *val = LSM9DS1_IMU_119Hz;
      break;
    case LSM9DS1_IMU_238Hz:
      *val = LSM9DS1_IMU_238Hz;
      break;
    case LSM9DS1_IMU_476Hz:
      *val = LSM9DS1_IMU_476Hz;
      break;
    case LSM9DS1_IMU_952Hz:
      *val = LSM9DS1_IMU_952Hz;
      break;
    case LSM9DS1_XL_OFF_GY_14Hz9_LP:
      *val = LSM9DS1_XL_OFF_GY_14Hz9_LP;
      break;
    case LSM9DS1_XL_OFF_GY_59Hz5_LP:
      *val = LSM9DS1_XL_OFF_GY_59Hz5_LP;
      break;
    case LSM9DS1_XL_OFF_GY_119Hz_LP:
      *val = LSM9DS1_XL_OFF_GY_119Hz_LP;
      break;
    case LSM9DS1_IMU_14Hz9_LP:
      *val = LSM9DS1_IMU_14Hz9_LP;
      break;
    case LSM9DS1_IMU_59Hz5_LP:
      *val = LSM9DS1_IMU_59Hz5_LP;
      break;
    case LSM9DS1_IMU_119Hz_LP:
      *val = LSM9DS1_IMU_119Hz_LP;
      break;
    default:
      *val = LSM9DS1_IMU_OFF;
    break;
  }

  return ret;
}

/**
  * @brief   Configure gyro orientation.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Directional user orientation selection.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Gy_Orient_Set(uint8_t B_Addr, LSM9DS1_GY_ORIENT_t val)
{
  LSM9DS1_ORIENT_CFG_G_t orient_cfg_g;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_ORIENT_CFG_G, 1, (uint8_t*)&orient_cfg_g);
  if(ret == 0) 
  {
    orient_cfg_g.orient   = val.orient;
    orient_cfg_g.signx_g  = val.signx_g;
    orient_cfg_g.signy_g  = val.signy_g;
    orient_cfg_g.signz_g  = val.signz_g;
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_ORIENT_CFG_G, 1, (uint8_t*)&orient_cfg_g);
  }
  return ret;
}

/**
  * @brief   Configure gyro orientation.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val     Directional user orientation selection.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Gy_Orient_Get(uint8_t B_Addr, LSM9DS1_GY_ORIENT_t *val)
{
  LSM9DS1_ORIENT_CFG_G_t orient_cfg_g;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_ORIENT_CFG_G, 1, (uint8_t*)&orient_cfg_g);
  val->orient = orient_cfg_g.orient;
  val->signz_g = orient_cfg_g.signz_g;
  val->signy_g = orient_cfg_g.signy_g;
  val->signx_g = orient_cfg_g.signx_g;

  return ret;
}

/**
  * @brief  Accelerometer new data available.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Iet the values of "xlda" in reg STATUS_REG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Xl_Flag_Data_Ready_Get(uint8_t B_Addr, uint8_t *val)
{
  LSM9DS1_STATUS_REG_t status_reg;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_STATUS_REG, 1, (uint8_t*)&status_reg);
  *val = status_reg.xlda;

  return ret;
}

/**
  * @brief  Gyroscope new data available.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Iet the values of "gda" in reg STATUS_REG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Gy_Flag_Data_Ready_Get(uint8_t B_Addr, uint8_t *val)
{
  LSM9DS1_STATUS_REG_t status_reg;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_STATUS_REG, 1, (uint8_t*)&status_reg);
  *val = status_reg.gda;

  return ret;
}

/**
  * @brief  Temperature new data available.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Iet the values of "tda" in reg STATUS_REG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Temp_Flag_Data_Ready_Get(uint8_t B_Addr, uint8_t *val)
{
  LSM9DS1_STATUS_REG_t status_reg;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_STATUS_REG, 1, (uint8_t*)&status_reg);
  *val = status_reg.tda;

  return ret;
}

/**
  * @brief  Enable gyroscope axis.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val     Gyroscope’s pitch axis (X) output enable.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Gy_Axis_Set(uint8_t B_Addr, LSM9DS1_GY_AXIS_t val)
{
  LSM9DS1_CTRL_REG4_t ctrl_reg4;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG4, 1, (uint8_t*)&ctrl_reg4);
  if(ret == 0) 
  {
    ctrl_reg4.xen_g = val.xen_g;
    ctrl_reg4.yen_g = val.yen_g;
    ctrl_reg4.zen_g = val.zen_g;
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_CTRL_REG4, 1, (uint8_t*)&ctrl_reg4);
  }
  return ret;
}

/**
  * @brief  Enable gyroscope axis.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val     Gyroscope’s pitch axis (X) output enable.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Gy_Axis_Get(uint8_t B_Addr, LSM9DS1_GY_AXIS_t *val)
{
  LSM9DS1_CTRL_REG4_t ctrl_reg4;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG4, 1, (uint8_t*)&ctrl_reg4);
  val->xen_g = ctrl_reg4.xen_g;
  val->yen_g = ctrl_reg4.yen_g;
  val->zen_g = ctrl_reg4.zen_g;

  return ret;
}

/**
  * @brief  Enable accelerometer axis.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val     Accelerometer’s X-axis output enable.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Xl_Axis_Set(uint8_t B_Addr, LSM9DS1_XL_AXIS_t val)
{
  LSM9DS1_CTRL_REG5_XL_t ctrl_reg5_xl;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG5_XL, 1, (uint8_t*)&ctrl_reg5_xl);
  if(ret == 0) 
  {
    ctrl_reg5_xl.xen_xl = val.xen_xl;
    ctrl_reg5_xl.yen_xl = val.yen_xl;
    ctrl_reg5_xl.zen_xl = val.zen_xl;
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_CTRL_REG5_XL, 1, (uint8_t*)&ctrl_reg5_xl);
  }
  return ret;
}

/**
  * @brief  Enable accelerometer axis.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val     Accelerometer’s X-axis output enable.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Xl_Axis_Get(uint8_t B_Addr, LSM9DS1_XL_AXIS_t *val)
{
  LSM9DS1_CTRL_REG5_XL_t ctrl_reg5_xl;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG5_XL, 1, (uint8_t*)&ctrl_reg5_xl);
  val->xen_xl = ctrl_reg5_xl.xen_xl;
  val->yen_xl = ctrl_reg5_xl.yen_xl;
  val->zen_xl = ctrl_reg5_xl.zen_xl;

  return ret;
}

/**
  * @brief  Decimation of acceleration data on OUT REG and FIFO.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "dec" in reg LSM9DS1.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Xl_Decimation_Set(uint8_t B_Addr, LSM9DS1_DEC_t val)
{
  LSM9DS1_CTRL_REG5_XL_t ctrl_reg5_xl;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG5_XL, 1, (uint8_t*)&ctrl_reg5_xl);
  if(ret == 0)
  {
    ctrl_reg5_xl.dec = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_CTRL_REG5_XL, 1, (uint8_t*)&ctrl_reg5_xl);
  }
  return ret;
}

/**
  * @brief  Decimation of acceleration data on OUT REG and FIFO.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Get the values of dec in reg CTRL_REG5_XL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Xl_Decimation_Get(uint8_t B_Addr, LSM9DS1_DEC_t *val)
{
  LSM9DS1_CTRL_REG5_XL_t ctrl_reg5_xl;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG5_XL, 1, (uint8_t*)&ctrl_reg5_xl);
  switch (ctrl_reg5_xl.dec)
  {
    case LSM9DS1_NO_DECIMATION:
      *val = LSM9DS1_NO_DECIMATION;
      break;
    case LSM9DS1_EVERY_2_SAMPLES:
      *val = LSM9DS1_EVERY_2_SAMPLES;
      break;
    case LSM9DS1_EVERY_4_SAMPLES:
      *val = LSM9DS1_EVERY_4_SAMPLES;
      break;
    case LSM9DS1_EVERY_8_SAMPLES:
      *val = LSM9DS1_EVERY_8_SAMPLES;
      break;
    default:
      *val = LSM9DS1_NO_DECIMATION;
      break;
  }
  return ret;
}

/**
  * @brief  Accelerometer full-scale selection.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "fs_xl" in reg LSM9DS1.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Xl_Full_Scale_Set(uint8_t B_Addr, LSM9DS1_XL_FS_t val)
{
  LSM9DS1_CTRL_REG6_XL_t ctrl_reg6_xl;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG6_XL, 1, (uint8_t*)&ctrl_reg6_xl);
  if(ret == 0)
  {
    ctrl_reg6_xl.fs_xl = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_CTRL_REG6_XL, 1, (uint8_t*)&ctrl_reg6_xl);
  }
  return ret;
}

/**
  * @brief  Accelerometer full-scale selection.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Get the values of fs_xl in reg CTRL_REG6_XL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Xl_Full_Scale_Get(uint8_t B_Addr, LSM9DS1_XL_FS_t *val)
{
  LSM9DS1_CTRL_REG6_XL_t ctrl_reg6_xl;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG6_XL, 1, (uint8_t*)&ctrl_reg6_xl);
  switch (ctrl_reg6_xl.fs_xl)
  {
    case LSM9DS1_2g:
      *val = LSM9DS1_2g;
      break;
    case LSM9DS1_16g:
      *val = LSM9DS1_16g;
      break;
    case LSM9DS1_4g:
      *val = LSM9DS1_4g;
      break;
    case LSM9DS1_8g:
      *val = LSM9DS1_8g;
      break;
    default:
      *val = LSM9DS1_2g;
      break;
  }
  return ret;
}

/**
  * @brief  Blockdataupdate.[set]
  *
  * @param  B_Addr_mag   Read / write magnetometer interface definitions.(ptr)
  * @param  B_Addr_imu   Read / write imu interface definitions.(ptr)
  * @param  val       Change the values of bdu in reg CTRL_REG8.
  * @retval           Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Block_Data_Update_Set(uint8_t B_Addr_mag, uint8_t B_Addr_imu, uint8_t val)
{
  LSM9DS1_CTRL_REG8_t ctrl_reg8;
  LSM9DS1_CTRL_REG5_M_t ctrl_reg5_m;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr_imu, LSM9DS1_CTRL_REG8, 1, (uint8_t*)&ctrl_reg8);
  if(ret == 0)
  {
    ctrl_reg8.bdu = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr_imu, LSM9DS1_CTRL_REG8, 1, (uint8_t*)&ctrl_reg8);
  }
  if(ret == 0)
  {
    ret = LSM9DS1_ReadReg(B_Addr_mag, LSM9DS1_CTRL_REG5_M, 1, (uint8_t*)&ctrl_reg5_m);
  }
  if(ret == 0)
  {
    ctrl_reg5_m.fast_read = (uint8_t)(~val);
    ctrl_reg5_m.bdu = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr_mag, LSM9DS1_CTRL_REG5_M, 1, (uint8_t*)&ctrl_reg5_m);
  }

  return ret;
}

/**
  * @brief  Blockdataupdate.[get]
  *
  * @param  B_Addr_mag   Read / write magnetometer interface definitions.(ptr)
  * @param  B_Addr_imu   Read / write imu interface definitions.(ptr)
  * @param  val       Get the values of bdu in reg CTRL_REG8.(ptr)
  * @retval           Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Block_Data_Update_Get(uint8_t B_Addr_mag, uint8_t B_Addr_imu, uint8_t *val)
{
  LSM9DS1_CTRL_REG8_t ctrl_reg8;
  LSM9DS1_CTRL_REG5_M_t ctrl_reg5_m;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr_imu, LSM9DS1_CTRL_REG8, 1, (uint8_t*)&ctrl_reg8);
  if(ret == 0)
  {
    ret = LSM9DS1_ReadReg(B_Addr_mag, LSM9DS1_CTRL_REG5_M, 1, (uint8_t*)&ctrl_reg5_m);
    *val = (uint8_t)(ctrl_reg5_m.bdu & ctrl_reg8.bdu);
  }
  return ret;
}

/**
  * @brief  This register is a 16-bit register and represents the X offset
  *         used to compensate environmental effects (data is expressed as
  *         two’s complement).[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores data to be write.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Mag_Offset_Set(uint8_t B_Addr, uint8_t *buff)
{
  LSM9DS1_Error_et ret;
  ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_OFFSET_X_REG_L_M, 6, buff);
  return ret;
}

/**
  * @brief  This register is a 16-bit register and represents the X offset
  *         used to compensate environmental effects (data is expressed as
  *         two’s complement).[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Mag_Offset_Get(uint8_t B_Addr, uint8_t *buff)
{
  LSM9DS1_Error_et ret;
  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_OFFSET_X_REG_L_M, 6, buff);
  return ret;
}

/**
  * @brief  Magnetometer data rate selection.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "fast_odr" in reg LSM9DS1.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Mag_Data_Rate_Set(uint8_t B_Addr, LSM9DS1_MAG_DATA_RATE_t val)
{
  LSM9DS1_CTRL_REG1_M_t ctrl_reg1_m;
  LSM9DS1_CTRL_REG3_M_t ctrl_reg3_m;
  LSM9DS1_CTRL_REG4_M_t ctrl_reg4_m;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG1_M, 1, (uint8_t*)&ctrl_reg1_m);
  if(ret == 0)
  {
    ctrl_reg1_m.fast_odr = (((uint8_t)val & 0x08U) >> 3);
    ctrl_reg1_m._do = ((uint8_t)val & 0x07U);
    ctrl_reg1_m.om = (((uint8_t)val & 0x30U) >> 4);
    ctrl_reg1_m.temp_comp = PROPERTY_ENABLE;
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_CTRL_REG1_M, 1, (uint8_t*)&ctrl_reg1_m);
  }
  if(ret == 0)
  {
    ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG3_M, 1, (uint8_t*)&ctrl_reg3_m);
  }
  if(ret == 0)
  {
    ctrl_reg3_m.md = (((uint8_t)val & 0xC0U) >> 6);
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_CTRL_REG3_M, 1, (uint8_t*)&ctrl_reg3_m);
  }
  if(ret == 0)
  {
    ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG4_M, 1, (uint8_t*)&ctrl_reg4_m);
  }
  if(ret == 0)
  {
    ctrl_reg4_m.omz = (((uint8_t)val & 0x30U) >> 4);
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_CTRL_REG4_M, 1, (uint8_t*)&ctrl_reg4_m);
  }
  return ret;
}

/**
  * @brief  Magnetometer data rate selection.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Get the values of fast_odr in reg CTRL_REG1_M.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Mag_Data_Rate_Get(uint8_t B_Addr, LSM9DS1_MAG_DATA_RATE_t *val)
{
  LSM9DS1_CTRL_REG1_M_t ctrl_reg1_m;
  LSM9DS1_CTRL_REG3_M_t ctrl_reg3_m;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG1_M, 1, (uint8_t*)&ctrl_reg1_m);
  if(ret == 0)
  {
    ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG3_M, 1, (uint8_t*)&ctrl_reg3_m);
  }
  switch ((ctrl_reg3_m.md << 6) | (ctrl_reg1_m.om << 4) | (ctrl_reg1_m.fast_odr << 3) | ctrl_reg1_m._do)
  {
    case LSM9DS1_MAG_POWER_DOWN:
      *val = LSM9DS1_MAG_POWER_DOWN;
      break;
    case LSM9DS1_MAG_LP_0Hz625:
      *val = LSM9DS1_MAG_LP_0Hz625;
      break;
    case LSM9DS1_MAG_LP_1Hz25:
      *val = LSM9DS1_MAG_LP_1Hz25;
      break;
    case LSM9DS1_MAG_LP_2Hz5:
      *val = LSM9DS1_MAG_LP_2Hz5;
      break;
    case LSM9DS1_MAG_LP_5Hz:
      *val = LSM9DS1_MAG_LP_5Hz;
      break;
    case LSM9DS1_MAG_LP_10Hz:
      *val = LSM9DS1_MAG_LP_10Hz;
      break;
    case LSM9DS1_MAG_LP_20Hz:
      *val = LSM9DS1_MAG_LP_20Hz;
      break;
    case LSM9DS1_MAG_LP_40Hz:
      *val = LSM9DS1_MAG_LP_40Hz;
      break;
    case LSM9DS1_MAG_LP_80Hz:
      *val = LSM9DS1_MAG_LP_80Hz;
      break;
    case LSM9DS1_MAG_MP_0Hz625:
      *val = LSM9DS1_MAG_MP_0Hz625;
      break;
    case LSM9DS1_MAG_MP_1Hz25:
      *val = LSM9DS1_MAG_MP_1Hz25;
      break;
    case LSM9DS1_MAG_MP_2Hz5:
      *val = LSM9DS1_MAG_MP_2Hz5;
      break;
    case LSM9DS1_MAG_MP_5Hz:
      *val = LSM9DS1_MAG_MP_5Hz;
      break;
    case LSM9DS1_MAG_MP_10Hz:
      *val = LSM9DS1_MAG_MP_10Hz;
      break;
    case LSM9DS1_MAG_MP_20Hz:
      *val = LSM9DS1_MAG_MP_20Hz;
      break;
    case LSM9DS1_MAG_MP_40Hz:
      *val = LSM9DS1_MAG_MP_40Hz;
      break;
    case LSM9DS1_MAG_MP_80Hz:
      *val = LSM9DS1_MAG_MP_80Hz;
      break;
    case LSM9DS1_MAG_HP_0Hz625:
      *val = LSM9DS1_MAG_HP_0Hz625;
      break;
    case LSM9DS1_MAG_HP_1Hz25:
      *val = LSM9DS1_MAG_HP_1Hz25;
      break;
    case LSM9DS1_MAG_HP_2Hz5:
      *val = LSM9DS1_MAG_HP_2Hz5;
      break;
    case LSM9DS1_MAG_HP_5Hz:
      *val = LSM9DS1_MAG_HP_5Hz;
      break;
    case LSM9DS1_MAG_HP_10Hz:
      *val = LSM9DS1_MAG_HP_10Hz;
      break;
    case LSM9DS1_MAG_HP_20Hz:
      *val = LSM9DS1_MAG_HP_20Hz;
      break;
    case LSM9DS1_MAG_HP_40Hz:
      *val = LSM9DS1_MAG_HP_40Hz;
      break;
    case LSM9DS1_MAG_HP_80Hz:
      *val = LSM9DS1_MAG_HP_80Hz;
      break;
    case LSM9DS1_MAG_UHP_0Hz625:
      *val = LSM9DS1_MAG_UHP_0Hz625;
      break;
    case LSM9DS1_MAG_UHP_1Hz25:
      *val = LSM9DS1_MAG_UHP_1Hz25;
      break;
    case LSM9DS1_MAG_UHP_2Hz5:
      *val = LSM9DS1_MAG_UHP_2Hz5;
      break;
    case LSM9DS1_MAG_UHP_5Hz:
      *val = LSM9DS1_MAG_UHP_5Hz;
      break;
    case LSM9DS1_MAG_UHP_10Hz:
      *val = LSM9DS1_MAG_UHP_10Hz;
      break;
    case LSM9DS1_MAG_UHP_20Hz:
      *val = LSM9DS1_MAG_UHP_20Hz;
      break;
    case LSM9DS1_MAG_UHP_40Hz:
      *val = LSM9DS1_MAG_UHP_40Hz;
      break;
    case LSM9DS1_MAG_UHP_80Hz:
      *val = LSM9DS1_MAG_UHP_80Hz;
      break;
    case LSM9DS1_MAG_UHP_155Hz:
      *val = LSM9DS1_MAG_UHP_155Hz;
      break;
    case LSM9DS1_MAG_HP_300Hz:
      *val = LSM9DS1_MAG_HP_300Hz;
      break;
    case LSM9DS1_MAG_MP_560Hz:
      *val = LSM9DS1_MAG_MP_560Hz;
      break;
    case LSM9DS1_MAG_LP_1000Hz:
      *val = LSM9DS1_MAG_LP_1000Hz;
      break;
    case LSM9DS1_MAG_ONE_SHOT:
      *val = LSM9DS1_MAG_ONE_SHOT;
      break;
    default:
      *val = LSM9DS1_MAG_POWER_DOWN;
      break;
  }
  return ret;
}

/**
  * @brief  Magnetometer full Scale Selection.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "fs" in reg LSM9DS1.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Mag_Full_Scale_Set(uint8_t B_Addr, LSM9DS1_MAG_FS_t val)
{
  LSM9DS1_CTRL_REG2_M_t ctrl_reg2_m;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG2_M, 1, (uint8_t*)&ctrl_reg2_m);
  if(ret == 0)
  {
    ctrl_reg2_m.fs = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_CTRL_REG2_M, 1, (uint8_t*)&ctrl_reg2_m);
  }
  return ret;
}

/**
  * @brief  Magnetometer full scale selection.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Get the values of fs in reg CTRL_REG2_M.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Mag_Full_Scale_Get(uint8_t B_Addr, LSM9DS1_MAG_FS_t *val)
{
  LSM9DS1_CTRL_REG2_M_t ctrl_reg2_m;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG2_M, 1, (uint8_t*)&ctrl_reg2_m);
  switch (ctrl_reg2_m.fs)
  {
    case LSM9DS1_4Ga:
      *val = LSM9DS1_4Ga;
      break;
    case LSM9DS1_8Ga:
      *val = LSM9DS1_8Ga;
      break;
    case LSM9DS1_12Ga:
      *val = LSM9DS1_12Ga;
      break;
    case LSM9DS1_16Ga:
      *val = LSM9DS1_16Ga;
      break;
    default:
      *val = LSM9DS1_4Ga;
      break;
  }
  return ret;
}

/**
  * @brief  New data available from magnetometer.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Iet the values of "zyxda" in reg STATUS_REG_M.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Mag_Flag_Data_Ready_Get(uint8_t B_Addr, uint8_t *val)
{
  LSM9DS1_STATUS_REG_M_t status_reg_m;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_STATUS_REG_M, 1, (uint8_t*)&status_reg_m);
  *val = status_reg_m.zyxda;

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup     LSM9DS1_Dataoutput
  * @brief        This section groups all the data output functions.
  * @{
  *
  */

/**
  * @brief  Temperature data output register (r). L and H registers
  *         together express a 16-bit word in two’s complement.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores the data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Temperature_Raw_Get(uint8_t B_Addr, uint8_t *buff)
{
  LSM9DS1_Error_et ret;
  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_OUT_TEMP_L, 2, buff);
  return ret;
}

/**
  * @brief  Angular rate sensor. The value is expressed as a 16-bit word in
  *         two’s complement.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores the data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Angular_Rate_Raw_Get(uint8_t B_Addr, uint8_t *buff)
{
  LSM9DS1_Error_et ret;
  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_OUT_X_L_G, 6, buff);
  return ret;
}

/**
  * @brief  Linear acceleration output register. The value is expressed as
  *         a 16-bit word in two’s complement.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores the data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Acceleration_Raw_Get(uint8_t B_Addr, uint8_t *buff)
{
  LSM9DS1_Error_et ret;
  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_OUT_X_L_XL, 6, buff);
  return ret;
}

/**
  * @brief  Magnetic sensor. The value is expressed as a 16-bit word in
  *         two’s complement.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores the data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Magnetic_Raw_Get(uint8_t B_Addr, uint8_t *buff)
{
  LSM9DS1_Error_et ret;
  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_OUT_X_L_M, 6, buff);
  return ret;
}

/**
  * @brief  Internal measurement range overflow on magnetic value.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Iet the values of "mroi" in reg INT_SRC_M.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Magnetic_Overflow_Get(uint8_t B_Addr, uint8_t *val)
{
  LSM9DS1_INT_SRC_M_t int_src_m;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_SRC_M, 1, (uint8_t*)&int_src_m);
  *val = int_src_m.mroi;

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup    LSM9DS1_Common
  * @brief       This section groups common usefull functions.
  * @{
  *
  */

/**
  * @brief  DeviceWhoamI.[get]
  *
  * @param  B_Addr_mag   Read / write magnetometer interface definitions.(ptr)
  * @param  B_Addr_imu   Read / write imu interface definitions.(ptr)
  * @param  buff      Buffer that stores the data read.(ptr)
  * @retval           Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Dev_Id_Get(uint8_t B_Addr_mag, uint8_t B_Addr_imu, LSM9DS1_ID_t *buff)
{
  LSM9DS1_Error_et ret;
  ret = LSM9DS1_ReadReg(B_Addr_imu, LSM9DS1_WHO_AM_I, 1, (uint8_t*)&(buff->imu));
  if(ret == 0)
  {
    ret = LSM9DS1_ReadReg(B_Addr_mag, LSM9DS1_WHO_AM_I_M, 1, (uint8_t*)&(buff->mag));
  }
  return ret;
}

/**
  * @brief  Device status register.[get]
  *
  * @param  B_Addr_mag   Read / write magnetometer interface definitions.(ptr)
  * @param  B_Addr_imu   Read / write imu interface definitions.(ptr)
  * @param  val       Device status registers.(ptr)
  * @retval           Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Dev_Status_Get(uint8_t B_Addr_mag, uint8_t B_Addr_imu, LSM9DS1_STATUS_t *val)
{
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr_imu, LSM9DS1_STATUS_REG, 1, (uint8_t*)&(val->status_imu));
  if(ret == 0)
  {
    ret = LSM9DS1_ReadReg(B_Addr_mag, LSM9DS1_STATUS_REG_M, 1, (uint8_t*)&(val->status_mag));
  }

  return ret;
}

/**
  * @brief  Software reset. Restore the default values in user registers.[set]
  *
  * @param  B_Addr_mag   Read / write magnetometer interface definitions.(ptr)
  * @param  B_Addr_imu   Read / write imu interface definitions.(ptr)
  * @param  val    Change the values of sw_reset in reg CTRL_REG8.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Dev_Reset_Set(uint8_t B_Addr_mag, uint8_t B_Addr_imu, uint8_t val)
{
  LSM9DS1_CTRL_REG2_M_t ctrl_reg2_m;
  LSM9DS1_CTRL_REG8_t ctrl_reg8;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr_imu, LSM9DS1_CTRL_REG8, 1, (uint8_t*)&ctrl_reg8);
  if(ret == 0)
  {
    ctrl_reg8.sw_reset = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr_imu, LSM9DS1_CTRL_REG8, 1, (uint8_t*)&ctrl_reg8);
  }
  if(ret == 0)
  {
    ret = LSM9DS1_ReadReg(B_Addr_mag, LSM9DS1_CTRL_REG2_M, 1, (uint8_t*)&ctrl_reg2_m);
  }
  if(ret == 0)
  {
    ctrl_reg2_m.soft_rst = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr_mag, LSM9DS1_CTRL_REG2_M, 1, (uint8_t*)&ctrl_reg2_m);
  }

  return ret;
}

/**
  * @brief  Software reset. Restore the default values in user registers.[get]
  *
  * @param  B_Addr_mag   Read / write magnetometer interface definitions.(ptr)
  * @param  B_Addr_imu   Read / write imu interface definitions.(ptr)
  * @param  val       Get the values of sw_reset in reg CTRL_REG8.(ptr)
  * @retval           Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Dev_Reset_Get(uint8_t B_Addr_mag, uint8_t B_Addr_imu, uint8_t *val)
{
  LSM9DS1_CTRL_REG2_M_t ctrl_reg2_m;
  LSM9DS1_CTRL_REG8_t ctrl_reg8;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr_imu, LSM9DS1_CTRL_REG8, 1, (uint8_t*)&ctrl_reg8);
  if(ret == 0)
  {
    ret = LSM9DS1_ReadReg(B_Addr_mag, LSM9DS1_CTRL_REG2_M, 1, (uint8_t*)&ctrl_reg2_m);
    *val = (uint8_t)(ctrl_reg2_m.soft_rst | ctrl_reg8.sw_reset);
  }
  return ret;
}

/**
  * @brief  Big/Little Endian data selection.[set]
  *
  * @param  B_Addr_mag   Read / write magnetometer interface definitions.(ptr)
  * @param  B_Addr_imu   Read / write imu interface definitions.(ptr)
  * @param  val       Change the values of "ble" in reg LSM9DS1.
  * @retval           Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Dev_Data_Format_Set(uint8_t B_Addr_mag, uint8_t B_Addr_imu, LSM9DS1_BLE_t val)
{
  LSM9DS1_CTRL_REG8_t ctrl_reg8;
  LSM9DS1_CTRL_REG4_M_t ctrl_reg4_m;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr_imu, LSM9DS1_CTRL_REG8, 1, (uint8_t*)&ctrl_reg8);
  if(ret == 0)
  {
    ctrl_reg8.ble = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr_imu, LSM9DS1_CTRL_REG8, 1, (uint8_t*)&ctrl_reg8);
  }
  if(ret == 0)
  {
    ret = LSM9DS1_ReadReg(B_Addr_mag, LSM9DS1_CTRL_REG4_M, 1, (uint8_t*)&ctrl_reg4_m);
  }
  if(ret == 0)
  {
    ctrl_reg4_m.ble = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr_mag, LSM9DS1_CTRL_REG4_M, 1, (uint8_t*)&ctrl_reg4_m);
  }
  return ret;
}

/**
  * @brief  Big/Little Endian data selection.[get]
  *
  * @param  B_Addr_mag   Read / write magnetometer interface definitions.(ptr)
  * @param  B_Addr_imu   Read / write imu interface definitions.(ptr)
  * @param  val       Get the values of ble in reg CTRL_REG8.(ptr)
  * @retval           Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Dev_Data_Format_Get(uint8_t B_Addr_mag, uint8_t B_Addr_imu, LSM9DS1_BLE_t *val)
{
  LSM9DS1_CTRL_REG8_t ctrl_reg8;
  LSM9DS1_CTRL_REG4_M_t ctrl_reg4_m;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr_imu, LSM9DS1_CTRL_REG8, 1, (uint8_t*)&ctrl_reg8);
  if(ret == 0)
  {
    ret = LSM9DS1_ReadReg(B_Addr_mag, LSM9DS1_CTRL_REG4_M, 1, (uint8_t*)&ctrl_reg4_m);
  }
  switch (ctrl_reg8.ble & ctrl_reg4_m.ble)
  {
    case LSM9DS1_LSB_LOW_ADDRESS:
      *val = LSM9DS1_LSB_LOW_ADDRESS;
      break;
    case LSM9DS1_MSB_LOW_ADDRESS:
      *val = LSM9DS1_MSB_LOW_ADDRESS;
      break;
    default:
      *val = LSM9DS1_LSB_LOW_ADDRESS;
      break;
  }
  return ret;
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters.[set]
  *
  * @param  B_Addr_mag   Read / write magnetometer interface definitions.(ptr)
  * @param  B_Addr_imu   Read / write imu interface definitions.(ptr)
  * @param  val       Change the values of boot in reg CTRL_REG8.
  * @retval           Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Dev_Boot_Set(uint8_t B_Addr_mag, uint8_t B_Addr_imu, uint8_t val)
{
  LSM9DS1_CTRL_REG8_t ctrl_reg8;
  LSM9DS1_CTRL_REG2_M_t ctrl_reg2_m;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr_imu, LSM9DS1_CTRL_REG8, 1, (uint8_t*)&ctrl_reg8);
  if(ret == 0)
  {
    ctrl_reg8.boot = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr_imu, LSM9DS1_CTRL_REG8, 1, (uint8_t*)&ctrl_reg8);
  }
  if(ret == 0)
  {
    ret = LSM9DS1_ReadReg(B_Addr_mag, LSM9DS1_CTRL_REG2_M, 1, (uint8_t*)&ctrl_reg2_m);
  }
  if(ret == 0)
  {
    ctrl_reg2_m.reboot = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr_mag, LSM9DS1_CTRL_REG2_M, 1, (uint8_t*)&ctrl_reg2_m);
  }
  return ret;
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters.[get]
  *
  * @param  B_Addr_mag   Read / write magnetometer interface definitions.(ptr)
  * @param  B_Addr_imu   Read / write imu interface definitions.(ptr)
  * @param  val       Get the values of boot in reg CTRL_REG8.(ptr)
  * @retval           Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Dev_Boot_Get(uint8_t B_Addr_mag, uint8_t B_Addr_imu, uint8_t *val)
{
  LSM9DS1_CTRL_REG2_M_t ctrl_reg2_m;
  LSM9DS1_CTRL_REG8_t ctrl_reg8;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr_imu, LSM9DS1_CTRL_REG8, 1, (uint8_t*)&ctrl_reg8);
  if(ret == 0)
  {
  ret = LSM9DS1_ReadReg(B_Addr_mag, LSM9DS1_CTRL_REG2_M, 1, (uint8_t*)&ctrl_reg2_m);
    *val = (uint8_t)ctrl_reg2_m.reboot & ctrl_reg8.boot;
  }
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup    LSM9DS1_Filters
  * @brief       This section group all the functions concerning the
                 filters configuration
  * @{
  *
  */

/**
  * @brief  Reference value for gyroscope’s digital high-pass filter.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores data to be write.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Gy_Filter_Reference_Set(uint8_t B_Addr, uint8_t *buff)
{
  LSM9DS1_Error_et ret;
  ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_REFERENCE_G, 1, buff);
  return ret;
}

/**
  * @brief  Reference value for gyroscope’s digital high-pass filter.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Gy_Filter_Reference_Get(uint8_t B_Addr, uint8_t *buff)
{
  LSM9DS1_Error_et ret;
  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_REFERENCE_G, 1, buff);
  return ret;
}

/**
  * @brief  Gyroscope lowpass filter bandwidth selection.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "bw_g" in reg LSM9DS1.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Gy_Filter_LP_Bandwidth_Set(uint8_t B_Addr, LSM9DS1_GY_LP_BW_t val)
{
  LSM9DS1_CTRL_REG1_G_t ctrl_reg1_g;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG1_G, 1, (uint8_t*)&ctrl_reg1_g);
  if(ret == 0)
  {
    ctrl_reg1_g.bw_g = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_CTRL_REG1_G, 1, (uint8_t*)&ctrl_reg1_g);
  }
  return ret;
}

/**
  * @brief  Gyroscope lowpass filter bandwidth selection.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Get the values of bw_g in reg CTRL_REG1_G.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Gy_Filter_LP_Bandwidth_Get(uint8_t B_Addr, LSM9DS1_GY_LP_BW_t *val)
{
  LSM9DS1_CTRL_REG1_G_t ctrl_reg1_g;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG1_G, 1, (uint8_t*)&ctrl_reg1_g);
  switch (ctrl_reg1_g.bw_g)
  {
    case LSM9DS1_LP_STRONG:
      *val = LSM9DS1_LP_STRONG;
      break;
    case LSM9DS1_LP_MEDIUM:
      *val = LSM9DS1_LP_MEDIUM;
      break;
    case LSM9DS1_LP_LIGHT:
      *val = LSM9DS1_LP_LIGHT;
      break;
    case LSM9DS1_LP_ULTRA_LIGHT:
      *val = LSM9DS1_LP_ULTRA_LIGHT;
      break;
    default:
      *val = LSM9DS1_LP_STRONG;
      break;
  }
  return ret;
}

/**
  * @brief  Gyro output filter path configuration.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "out_sel" in reg LSM9DS1.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Gy_Filter_Out_Path_Set(uint8_t B_Addr, LSM9DS1_GY_OUT_PATH_t val)
{
  LSM9DS1_CTRL_REG2_G_t ctrl_reg2_g;
  LSM9DS1_CTRL_REG3_G_t ctrl_reg3_g;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG2_G, 1, (uint8_t*)&ctrl_reg2_g);
  if(ret == 0)
  {
    ctrl_reg2_g.out_sel = ((uint8_t)val & 0x03U);
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_CTRL_REG2_G, 1, (uint8_t*)&ctrl_reg2_g);
  }
  if(ret == 0)
  {
    ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG3_G, 1, (uint8_t*)&ctrl_reg3_g);
  }
  if(ret == 0)
  {
    ctrl_reg3_g.hp_en = (((uint8_t)val & 0x10U) >> 4 );
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_CTRL_REG3_G, 1, (uint8_t*)&ctrl_reg3_g);
  }

  return ret;
}

/**
  * @brief  Gyro output filter path configuration.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Get the values of out_sel in reg CTRL_REG2_G.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Gy_Filter_Out_Path_Get(uint8_t B_Addr, LSM9DS1_GY_OUT_PATH_t *val)
{
  LSM9DS1_CTRL_REG2_G_t ctrl_reg2_g;
  LSM9DS1_CTRL_REG3_G_t ctrl_reg3_g;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG2_G, 1, (uint8_t*)&ctrl_reg2_g);
  if(ret == 0){
    ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG3_G, 1, (uint8_t*)&ctrl_reg3_g);
  }
  switch ((ctrl_reg3_g.hp_en << 4) | ctrl_reg2_g.out_sel)
  {
    case LSM9DS1_LPF1_OUT:
      *val = LSM9DS1_LPF1_OUT;
      break;
    case LSM9DS1_LPF1_HPF_OUT:
      *val = LSM9DS1_LPF1_HPF_OUT;
      break;
    case LSM9DS1_LPF1_LPF2_OUT:
      *val = LSM9DS1_LPF1_LPF2_OUT;
      break;
    case LSM9DS1_LPF1_HPF_LPF2_OUT:
      *val = LSM9DS1_LPF1_HPF_LPF2_OUT;
      break;
    default:
      *val = LSM9DS1_LPF1_OUT;
      break;
  }

  return ret;
}

/**
  * @brief  Gyro interrupt filter path configuration.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "int_sel" in reg LSM9DS1.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Gy_Filter_Int_Path_Set(uint8_t B_Addr, LSM9DS1_GY_INT_PATH_t val)
{
  LSM9DS1_CTRL_REG2_G_t ctrl_reg2_g;
  LSM9DS1_CTRL_REG3_G_t ctrl_reg3_g;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG2_G, 1, (uint8_t*)&ctrl_reg2_g);
  if(ret == 0)
  {
    ctrl_reg2_g.int_sel = ((uint8_t)val & 0x03U);
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_CTRL_REG2_G, 1, (uint8_t*)&ctrl_reg2_g);
  }
  if(ret == 0)
  {
    ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG3_G, 1, (uint8_t*)&ctrl_reg3_g);
  }
  if(ret == 0)
  {
    ctrl_reg3_g.hp_en = (((uint8_t)val & 0x10U) >> 4 );
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_CTRL_REG3_G, 1, (uint8_t*)&ctrl_reg3_g);
  }
  return ret;
}

/**
  * @brief  Gyro interrupt filter path configuration.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Get the values of int_sel in reg CTRL_REG2_G.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Gy_Filter_Int_Path_Get(uint8_t B_Addr, LSM9DS1_GY_INT_PATH_t *val)
{
  LSM9DS1_CTRL_REG2_G_t ctrl_reg2_g;
  LSM9DS1_CTRL_REG3_G_t ctrl_reg3_g;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG2_G, 1, (uint8_t*)&ctrl_reg2_g);
  if(ret == 0)
  {
    ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG3_G, 1, (uint8_t*)&ctrl_reg3_g);
  }
  switch ( (ctrl_reg3_g.hp_en << 4) | ctrl_reg2_g.int_sel)
  {
    case LSM9DS1_LPF1_INT:
      *val = LSM9DS1_LPF1_INT;
      break;
    case LSM9DS1_LPF1_HPF_INT:
      *val = LSM9DS1_LPF1_HPF_INT;
      break;
    case LSM9DS1_LPF1_LPF2_INT:
      *val = LSM9DS1_LPF1_LPF2_INT;
      break;
    case LSM9DS1_LPF1_HPF_LPF2_INT:
      *val = LSM9DS1_LPF1_HPF_LPF2_INT;
      break;
    default:
      *val = LSM9DS1_LPF1_INT;
      break;
  }
  return ret;
}

/**
  * @brief  Gyroscope high-pass filter bandwidth selection.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "hpcf_g" in reg LSM9DS1.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Gy_Filter_HP_Bandwidth_Set(uint8_t B_Addr, LSM9DS1_GY_HP_BW_t val)
{
  LSM9DS1_CTRL_REG3_G_t ctrl_reg3_g;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG3_G, 1, (uint8_t*)&ctrl_reg3_g);
  if(ret == 0)
  {
    ctrl_reg3_g.hpcf_g = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_CTRL_REG3_G, 1, (uint8_t*)&ctrl_reg3_g);
  }
  return ret;
}

/**
  * @brief  Gyroscope high-pass filter bandwidth selection.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Get the values of hpcf_g in reg CTRL_REG3_G.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Gy_Filter_HP_Bandwidth_Get(uint8_t B_Addr, LSM9DS1_GY_HP_BW_t *val)
{
  LSM9DS1_CTRL_REG3_G_t ctrl_reg3_g;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG3_G, 1, (uint8_t*)&ctrl_reg3_g);
  switch (ctrl_reg3_g.hpcf_g)
  {
    case LSM9DS1_HP_EXTREME:
      *val = LSM9DS1_HP_EXTREME;
      break;
    case LSM9DS1_HP_ULTRA_STRONG:
      *val = LSM9DS1_HP_ULTRA_STRONG;
      break;
    case LSM9DS1_HP_STRONG:
      *val = LSM9DS1_HP_STRONG;
      break;
    case LSM9DS1_HP_ULTRA_HIGH:
      *val = LSM9DS1_HP_ULTRA_HIGH;
      break;
    case LSM9DS1_HP_HIGH:
      *val = LSM9DS1_HP_HIGH;
      break;
    case LSM9DS1_HP_MEDIUM:
      *val = LSM9DS1_HP_MEDIUM;
      break;
    case LSM9DS1_HP_LOW:
      *val = LSM9DS1_HP_LOW;
      break;
    case LSM9DS1_HP_ULTRA_LOW:
      *val = LSM9DS1_HP_ULTRA_LOW;
      break;
    case LSM9DS1_HP_LIGHT:
      *val = LSM9DS1_HP_LIGHT;
      break;
    case LSM9DS1_HP_ULTRA_LIGHT:
      *val = LSM9DS1_HP_ULTRA_LIGHT;
      break;
    default:
      *val = LSM9DS1_HP_EXTREME;
      break;
  }
  return ret;
}

/**
  * @brief  Configure accelerometer anti aliasing filter.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "bw_xl" in reg LSM9DS1.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Xl_Filter_Aalias_Bandwidth_Set(uint8_t B_Addr, LSM9DS1_XL_AA_BW_t val)
{
  LSM9DS1_CTRL_REG6_XL_t ctrl_reg6_xl;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG6_XL, 1, (uint8_t*)&ctrl_reg6_xl);
  if(ret == 0)
  {
    ctrl_reg6_xl.bw_xl = ((uint8_t)val & 0x03U);
    ctrl_reg6_xl.bw_scal_odr = (((uint8_t)val & 0x10U) >> 4 );
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_CTRL_REG6_XL, 1, (uint8_t*)&ctrl_reg6_xl);
  }
  return ret;
}

/**
  * @brief  Configure accelerometer anti aliasing filter.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Get the values of bw_xl in reg CTRL_REG6_XL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Xl_Filter_Aalias_Bandwidth_Get(uint8_t B_Addr, LSM9DS1_XL_AA_BW_t *val)
{
  LSM9DS1_CTRL_REG6_XL_t ctrl_reg6_xl;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG6_XL, 1, (uint8_t*)&ctrl_reg6_xl);
  switch ((ctrl_reg6_xl.bw_scal_odr << 4) | ctrl_reg6_xl.bw_xl)
  {
    case LSM9DS1_AUTO:
      *val = LSM9DS1_AUTO;
      break;
    case LSM9DS1_408Hz:
      *val = LSM9DS1_408Hz;
      break;
    case LSM9DS1_211Hz:
      *val = LSM9DS1_211Hz;
      break;
    case LSM9DS1_105Hz:
      *val = LSM9DS1_105Hz;
      break;
    case LSM9DS1_50Hz:
      *val = LSM9DS1_50Hz;
      break;
    default:
      *val = LSM9DS1_AUTO;
      break;
  }
  return ret;
}

/**
  * @brief  Configure HP accelerometer filter.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "hpis1" in reg LSM9DS1.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Xl_Filter_Int_Path_Set(uint8_t B_Addr, LSM9DS1_XL_HP_PATH_t val)
{
  LSM9DS1_CTRL_REG7_XL_t ctrl_reg7_xl;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG7_XL, 1, (uint8_t*)&ctrl_reg7_xl);
  if(ret == 0)
  {
    ctrl_reg7_xl.hpis1 = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_CTRL_REG7_XL, 1, (uint8_t*)&ctrl_reg7_xl);
  }
  return ret;
}

/**
  * @brief  .[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Get the values of hpis1 in reg CTRL_REG7_XL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Xl_Filter_Int_Path_Get(uint8_t B_Addr, LSM9DS1_XL_HP_PATH_t *val)
{
  LSM9DS1_CTRL_REG7_XL_t ctrl_reg7_xl;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG7_XL, 1, (uint8_t*)&ctrl_reg7_xl);
  switch (ctrl_reg7_xl.hpis1)
  {
    case LSM9DS1_HP_DIS:
      *val = LSM9DS1_HP_DIS;
      break;
    case LSM9DS1_HP_EN:
      *val = LSM9DS1_HP_EN;
      break;
    default:
      *val = LSM9DS1_HP_DIS;
      break;
  }
  return ret;
}

/**
  * @brief  Accelerometer output filter path configuration.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "fds" in reg LSM9DS1.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Xl_Filter_Out_Path_Set(uint8_t B_Addr, LSM9DS1_XL_OUT_PATH_t val)
{
  LSM9DS1_CTRL_REG7_XL_t ctrl_reg7_xl;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG7_XL, 1, (uint8_t*)&ctrl_reg7_xl);
  if(ret == 0)
  {
    ctrl_reg7_xl.fds = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_CTRL_REG7_XL, 1, (uint8_t*)&ctrl_reg7_xl);
  }
  return ret;
}

/**
  * @brief  Accelerometer output filter path configuration.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Get the values of fds in reg CTRL_REG7_XL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Xl_Filter_Out_Path_Get(uint8_t B_Addr, LSM9DS1_XL_OUT_PATH_t *val)
{
  LSM9DS1_CTRL_REG7_XL_t ctrl_reg7_xl;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG7_XL, 1, (uint8_t*)&ctrl_reg7_xl);
  switch (ctrl_reg7_xl.fds)
  {
    case LSM9DS1_LP_OUT:
      *val = LSM9DS1_LP_OUT;
      break;
    case LSM9DS1_HP_OUT:
      *val = LSM9DS1_HP_OUT;
      break;
    default:
      *val = LSM9DS1_LP_OUT;
      break;
  }
  return ret;
}

/**
  * @brief  Accelerometer digital filter low pass cutoff frequency
  *         selection.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "dcf" in reg LSM9DS1.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Xl_Filter_LP_Bandwidth_Set(uint8_t B_Addr, LSM9DS1_XL_LP_BW_t val)
{
  LSM9DS1_CTRL_REG7_XL_t ctrl_reg7_xl;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG7_XL, 1, (uint8_t*)&ctrl_reg7_xl);
  if(ret == 0)
  {
    ctrl_reg7_xl.hr = ((uint8_t)val & 0x03U) >> 4;
    ctrl_reg7_xl.dcf = ((uint8_t)val & 0x10U);
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_CTRL_REG7_XL, 1, (uint8_t*)&ctrl_reg7_xl);
  }
  return ret;
}

/**
  * @brief  Accelerometer digital filter low pass cutoff frequency
  *         selection.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Get the values of dcf in reg CTRL_REG7_XL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Xl_Filter_LP_Bandwidth_Get(uint8_t B_Addr, LSM9DS1_XL_LP_BW_t *val)
{
  LSM9DS1_CTRL_REG7_XL_t ctrl_reg7_xl;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG7_XL, 1, (uint8_t*)&ctrl_reg7_xl);
  switch ((ctrl_reg7_xl.hr << 4) + ctrl_reg7_xl.dcf)
  {
    case LSM9DS1_LP_DISABLE:
      *val = LSM9DS1_LP_DISABLE;
      break;
    case LSM9DS1_LP_ODR_DIV_50:
      *val = LSM9DS1_LP_ODR_DIV_50;
      break;
    case LSM9DS1_LP_ODR_DIV_100:
      *val = LSM9DS1_LP_ODR_DIV_100;
      break;
    case LSM9DS1_LP_ODR_DIV_9:
      *val = LSM9DS1_LP_ODR_DIV_9;
      break;
    case LSM9DS1_LP_ODR_DIV_400:
      *val = LSM9DS1_LP_ODR_DIV_400;
      break;
    default:
      *val = LSM9DS1_LP_DISABLE;
      break;
  }
  return ret;
}

/**
  * @brief  Accelerometer digital filter high pass cutoff frequency
  *         selection.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "dcf" in reg LSM9DS1.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Xl_Filter_HP_Bandwidth_Set(uint8_t B_Addr, LSM9DS1_XL_HP_BW_t val)
{
  LSM9DS1_CTRL_REG7_XL_t ctrl_reg7_xl;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG7_XL, 1, (uint8_t*)&ctrl_reg7_xl);
  if(ret == 0)
  {
    ctrl_reg7_xl.dcf = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_CTRL_REG7_XL, 1, (uint8_t*)&ctrl_reg7_xl);
  }
  return ret;
}

/**
  * @brief  Accelerometer digital filter high pass cutoff frequency
  *         selection.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Get the values of dcf in reg CTRL_REG7_XL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Xl_Filter_HP_Bandwidth_Get(uint8_t B_Addr, LSM9DS1_XL_HP_BW_t *val)
{
  LSM9DS1_CTRL_REG7_XL_t ctrl_reg7_xl;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG7_XL, 1, (uint8_t*)&ctrl_reg7_xl);

  switch (ctrl_reg7_xl.dcf)
  {
    case LSM9DS1_HP_ODR_DIV_50:
      *val = LSM9DS1_HP_ODR_DIV_50;
      break;
    case LSM9DS1_HP_ODR_DIV_100:
      *val = LSM9DS1_HP_ODR_DIV_100;
      break;
    case LSM9DS1_HP_ODR_DIV_9:
      *val = LSM9DS1_HP_ODR_DIV_9;
      break;
    case LSM9DS1_HP_ODR_DIV_400:
      *val = LSM9DS1_HP_ODR_DIV_400;
      break;
    default:
      *val = LSM9DS1_HP_ODR_DIV_50;
      break;
  }
  return ret;
}

/**
  * @brief  Mask DRDY on pin (both XL & Gyro) until filter settling ends.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Change the values of drdy_mask_bit in reg CTRL_REG9.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Filter_Settling_Mask_Set(uint8_t B_Addr, uint8_t val)
{
  LSM9DS1_CTRL_REG9_t ctrl_reg9;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG9, 1, (uint8_t*)&ctrl_reg9);
  if(ret == 0)
  {
    ctrl_reg9.drdy_mask_bit = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_CTRL_REG9, 1, (uint8_t*)&ctrl_reg9);
  }
  return ret;
}

/**
  * @brief  Mask DRDY on pin (both XL & Gyro) until filter settling ends.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Get the values of drdy_mask_bit in reg CTRL_REG9.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Filter_Settling_Mask_Get(uint8_t B_Addr, uint8_t *val)
{
  LSM9DS1_CTRL_REG9_t ctrl_reg9;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG9, 1, (uint8_t*)&ctrl_reg9);
  *val = (uint8_t)ctrl_reg9.drdy_mask_bit;

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup     LSM9DS1_Serial_interface
  * @brief        This section groups all the functions concerning main
  *               serial interface management (not auxiliary)
  * @{
  *
  */

/**
  * @brief  Register address automatically incremented during a multiple
  *         byte access with a serial interface.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "if_add_inc" in reg LSM9DS1.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Auto_Increment_Set(uint8_t B_Addr, uint8_t val)
{
  LSM9DS1_CTRL_REG8_t ctrl_reg8;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG8, 1, (uint8_t*)&ctrl_reg8);
  if(ret == 0)
  {
    ctrl_reg8.if_add_inc = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_CTRL_REG8, 1, (uint8_t*)&ctrl_reg8);
  }
  return ret;
}

/**
  * @brief  Register address automatically incremented during a multiple
  *         byte access with a serial interface.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Get the values of if_add_inc in reg CTRL_REG8.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Auto_Increment_Get(uint8_t B_Addr, uint8_t *val)
{
  LSM9DS1_CTRL_REG8_t ctrl_reg8;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG8, 1, (uint8_t*)&ctrl_reg8);
  *val = (uint8_t)ctrl_reg8.if_add_inc;
  return ret;
}

/**
  * @brief  SPI Serial Interface Mode selection.[set]
  *
  * @param  B_Addr_mag   Read / write magnetometer interface definitions.(ptr)
  * @param  B_Addr_imu   Read / write imu interface definitions.(ptr)
  * @param  val       Change the values of "sim" in reg LSM9DS1.
  * @retval           Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_SPI_Mode_Set(uint8_t B_Addr_mag, uint8_t B_Addr_imu, LSM9DS1_SIM_t val)
{
  LSM9DS1_CTRL_REG3_M_t ctrl_reg3_m;
  LSM9DS1_CTRL_REG8_t ctrl_reg8;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr_imu, LSM9DS1_CTRL_REG8, 1, (uint8_t*)&ctrl_reg8);
  if(ret == 0)
  {
    ctrl_reg8.sim = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr_imu, LSM9DS1_CTRL_REG8, 1, (uint8_t*)&ctrl_reg8);
  }
  if(ret == 0)
  {
      ret = LSM9DS1_ReadReg(B_Addr_mag, LSM9DS1_CTRL_REG3_M, 1, (uint8_t*)&ctrl_reg3_m);
  }
  if(ret == 0)
  {
    ctrl_reg3_m.sim = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr_mag, LSM9DS1_CTRL_REG3_M, 1, (uint8_t*)&ctrl_reg3_m);
  }
  return ret;
}

/**
  * @brief  SPI Serial Interface Mode selection.[get]
  *
  * @param  B_Addr_mag   Read / write magnetometer interface definitions.(ptr)
  * @param  B_Addr_imu   Read / write imu interface definitions.(ptr)
  * @param  val       Get the values of sim in reg CTRL_REG8.(ptr)
  * @retval           Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_SPI_Mode_Get(uint8_t B_Addr_mag, uint8_t B_Addr_imu, LSM9DS1_SIM_t *val)
{
  LSM9DS1_CTRL_REG3_M_t ctrl_reg3_m;
  LSM9DS1_CTRL_REG8_t ctrl_reg8;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr_imu, LSM9DS1_CTRL_REG8, 1, (uint8_t*)&ctrl_reg8);
  if(ret == 0)
  {
      ret = LSM9DS1_ReadReg(B_Addr_mag, LSM9DS1_CTRL_REG3_M, 1, (uint8_t*)&ctrl_reg3_m);
  }
  switch (ctrl_reg8.sim & ctrl_reg3_m.sim)
  {
    case LSM9DS1_SPI_4_WIRE:
      *val = LSM9DS1_SPI_4_WIRE;
      break;
    case LSM9DS1_SPI_3_WIRE:
      *val = LSM9DS1_SPI_3_WIRE;
      break;
    default:
      *val = LSM9DS1_SPI_4_WIRE;
      break;
  }
  return ret;
}

/**
  * @brief  Enable / Disable I2C interface.[set]
  *
  * @param  B_Addr_mag   Read / write magnetometer interface definitions.(ptr)
  * @param  B_Addr_imu   Read / write imu interface definitions.(ptr)
  * @param  val       Change the values of "i2c_disable" in reg LSM9DS1.
  * @retval           Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_I2C_Interface_Set(uint8_t B_Addr_mag, uint8_t B_Addr_imu, LSM9DS1_I2C_DIS_t val)
{
  LSM9DS1_CTRL_REG3_M_t ctrl_reg3_m;
  LSM9DS1_CTRL_REG9_t ctrl_reg9;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr_imu, LSM9DS1_CTRL_REG9, 1, (uint8_t*)&ctrl_reg9);
  if(ret == 0)
  {
    ctrl_reg9.i2c_disable = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr_imu, LSM9DS1_CTRL_REG9, 1, (uint8_t*)&ctrl_reg9);
  }
  if(ret == 0)
  {
    ret = LSM9DS1_ReadReg(B_Addr_mag, LSM9DS1_CTRL_REG3_M, 1, (uint8_t*)&ctrl_reg3_m);
  }
  if(ret == 0)
  {
    ctrl_reg3_m.i2c_disable = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr_mag, LSM9DS1_CTRL_REG3_M, 1, (uint8_t*)&ctrl_reg3_m);
  }
  return ret;
}

/**
  * @brief  Enable / Disable I2C interface.[get]
  *
  * @param  B_Addr_mag   Read / write magnetometer interface definitions.(ptr)
  * @param  B_Addr_imu   Read / write imu interface definitions.(ptr)
  * @param  val       Get the values of i2c_disable in reg CTRL_REG9.(ptr)
  * @retval           Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_I2C_Interface_Get(uint8_t B_Addr_mag, uint8_t B_Addr_imu, LSM9DS1_I2C_DIS_t *val)
{
  LSM9DS1_CTRL_REG3_M_t ctrl_reg3_m;
  LSM9DS1_CTRL_REG9_t ctrl_reg9;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr_imu, LSM9DS1_CTRL_REG9, 1, (uint8_t*)&ctrl_reg9);
  if(ret == 0)
  {
    ret = LSM9DS1_ReadReg(B_Addr_mag, LSM9DS1_CTRL_REG3_M, 1, (uint8_t*)&ctrl_reg3_m);
  }
  switch (ctrl_reg9.i2c_disable & ctrl_reg3_m.i2c_disable)
  {
    case LSM9DS1_I2C_ENABLE:
      *val = LSM9DS1_I2C_ENABLE;
      break;
    case LSM9DS1_I2C_DISABLE:
      *val = LSM9DS1_I2C_DISABLE;
      break;
    default:
      *val = LSM9DS1_I2C_ENABLE;
      break;
  }
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup     LSM9DS1_Interrupt_pins
  * @brief        This section groups all the functions that manage
  *               interrupt pins
  * @{
  *
  */

/**
  * @brief  AND/OR combination of accelerometer’s interrupt events.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "aoi_xl" in reg LSM9DS1.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Pin_Logic_Set(uint8_t B_Addr, LSM9DS1_PIN_LOGIC_t val)
{
  LSM9DS1_INT_GEN_CFG_XL_t int_gen_cfg_xl;
  LSM9DS1_INT_GEN_CFG_G_t int_gen_cfg_g;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_GEN_CFG_XL, 1, (uint8_t*)&int_gen_cfg_xl);
  if(ret == 0)
  {
    int_gen_cfg_xl.aoi_xl = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_INT_GEN_CFG_XL, 1, (uint8_t*)&int_gen_cfg_xl);
  }
  if(ret == 0)
  {
	ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_GEN_CFG_G, 1, (uint8_t*)&int_gen_cfg_g);
  }
  if(ret == 0)
  {
    int_gen_cfg_g.aoi_g = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_INT_GEN_CFG_G, 1, (uint8_t*)&int_gen_cfg_g);
  }

  return ret;
}

/**
  * @brief  AND/OR combination of accelerometer’s interrupt events.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Get the values of aoi_xl in reg INT_GEN_CFG_XL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Pin_Logic_Get(uint8_t B_Addr, LSM9DS1_PIN_LOGIC_t *val)
{
  LSM9DS1_INT_GEN_CFG_XL_t int_gen_cfg_xl;
  LSM9DS1_INT_GEN_CFG_G_t int_gen_cfg_g;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_GEN_CFG_XL, 1, (uint8_t*)&int_gen_cfg_xl);
  if(ret == 0)
  {
    ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_GEN_CFG_G, 1, (uint8_t*)&int_gen_cfg_g);
  }
  switch (int_gen_cfg_xl.aoi_xl & int_gen_cfg_g.aoi_g)
  {
    case LSM9DS1_LOGIC_OR:
      *val = LSM9DS1_LOGIC_OR;
      break;
    case LSM9DS1_LOGIC_AND:
      *val = LSM9DS1_LOGIC_AND;
      break;
    default:
      *val = LSM9DS1_LOGIC_OR;
      break;
  }
  return ret;
}

/**
  * @brief  Route a signal on INT 1_A/G pin.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val     Accelerometer data ready on INT 1_A/G pin.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Pin_Int1_Route_Set(uint8_t B_Addr, LSM9DS1_PIN_INT1_ROUTE_t val)
{
  LSM9DS1_INT1_CTRL_t int1_ctrl;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT1_CTRL, 1, (uint8_t*)&int1_ctrl);
  if(ret == 0) 
  {
    int1_ctrl.int1_drdy_xl  = val.int1_drdy_xl;
    int1_ctrl.int1_drdy_g  = val.int1_drdy_g;
    int1_ctrl.int1_boot  = val.int1_boot;
    int1_ctrl.int1_fth  = val.int1_fth;
    int1_ctrl.int1_ovr  = val.int1_ovr;
    int1_ctrl.int1_fss5  = val.int1_fss5;
    int1_ctrl.int1_ig_xl  = val.int1_ig_xl;
    int1_ctrl.int1_ig_g  = val.int1_ig_g;
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_INT1_CTRL, 1, (uint8_t*)&int1_ctrl);
  }
  return ret;
}

/**
  * @brief  Route a signal on INT 1_A/G pin.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val     Accelerometer data ready on INT 1_A/G pin.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Pin_Int1_Route_Get(uint8_t B_Addr, LSM9DS1_PIN_INT1_ROUTE_t *val)
{
  LSM9DS1_INT1_CTRL_t int1_ctrl;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT1_CTRL, 1, (uint8_t*)&int1_ctrl);

  val->int1_drdy_xl = int1_ctrl.int1_drdy_xl;
  val->int1_drdy_g = int1_ctrl.int1_drdy_g;
  val->int1_boot = int1_ctrl.int1_boot;
  val->int1_fth = int1_ctrl.int1_fth;
  val->int1_ovr = int1_ctrl.int1_ovr;
  val->int1_fss5 = int1_ctrl.int1_fss5;
  val->int1_ig_xl = int1_ctrl.int1_ig_xl;
  val->int1_ig_g = int1_ctrl.int1_ig_g;

  return ret;
}

/**
  * @brief  Route a signal on INT 2_A/G pin.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val     Accelerometer data ready on INT2_A/G pin.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Pin_Int2_Route_Set(uint8_t B_Addr, LSM9DS1_PIN_INT2_ROUTE_t val)
{
  LSM9DS1_INT2_CTRL_t int2_ctrl;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT2_CTRL, 1, (uint8_t*)&int2_ctrl);
  if(ret == 0) 
  {
    int2_ctrl.int2_drdy_xl  = val.int2_drdy_xl;
    int2_ctrl.int2_inact  = val.int2_inact;
    int2_ctrl.int2_drdy_g  = val.int2_drdy_g;
    int2_ctrl.int2_drdy_temp  = val.int2_drdy_temp;
    int2_ctrl.int2_fth  = val.int2_fth;
    int2_ctrl.int2_ovr  = val.int2_ovr;
    int2_ctrl.int2_fss5  = val.int2_fss5;
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_INT2_CTRL, 1, (uint8_t*)&int2_ctrl);
  }
  return ret;
}

/**
  * @brief  Route a signal on INT 2_A/G pin.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val     Accelerometer data ready on INT2_A/G pin.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Pin_Int2_Route_Get(uint8_t B_Addr, LSM9DS1_PIN_INT2_ROUTE_t *val)
{
  LSM9DS1_INT2_CTRL_t int2_ctrl;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT2_CTRL, 1, (uint8_t*)&int2_ctrl);
  val->int2_drdy_xl = int2_ctrl.int2_drdy_xl;
  val->int2_inact = int2_ctrl.int2_inact;
  val->int2_fss5 = int2_ctrl.int2_fss5;
  val->int2_ovr = int2_ctrl.int2_ovr;
  val->int2_fth = int2_ctrl.int2_fth;
  val->int2_drdy_temp = int2_ctrl.int2_drdy_temp;
  val->int2_drdy_g = int2_ctrl.int2_drdy_g;

  return ret;
}

/**
  * @brief  Latched/pulsed interrupt.[set]
  *
  * @param  B_Addr_mag   Read / write magnetometer interface definitions.(ptr)
  * @param  B_Addr_imu   Read / write imu interface definitions.(ptr)
  * @param  val       Change the values of "lir_xl1" in reg LSM9DS1.
  * @retval           Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Pin_Notification_Set(uint8_t B_Addr_mag, uint8_t B_Addr_imu, LSM9DS1_LIR_t val)
{
  LSM9DS1_INT_GEN_CFG_G_t int_gen_cfg_g;
  LSM9DS1_INT_CFG_M_t int_cfg_m;
  LSM9DS1_CTRL_REG4_t ctrl_reg4;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr_imu, LSM9DS1_CTRL_REG4, 1, (uint8_t*)&ctrl_reg4);
  if(ret == 0)
  {
    ctrl_reg4.lir_xl1 = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr_imu, LSM9DS1_CTRL_REG4, 1, (uint8_t*)&ctrl_reg4);
  }
  if(ret == 0)
  {
    ret = LSM9DS1_ReadReg(B_Addr_imu, LSM9DS1_INT_GEN_CFG_G, 1, (uint8_t*)&int_gen_cfg_g);
  }
  if(ret == 0)
  {
    int_gen_cfg_g.lir_g = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr_imu, LSM9DS1_INT_GEN_CFG_G, 1, (uint8_t*)&int_gen_cfg_g);
  }
  if(ret == 0)
  {
    ret = LSM9DS1_ReadReg(B_Addr_mag, LSM9DS1_INT_CFG_M, 1, (uint8_t*)&int_cfg_m);
  }
  if(ret == 0)
  {
    int_cfg_m.iel = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr_mag, LSM9DS1_INT_CFG_M, 1, (uint8_t*)&int_cfg_m);
  }

  return ret;
}

/**
  * @brief  Configure the interrupt notification mode.[get]
  *
  * @param  B_Addr_mag   Read / write magnetometer interface definitions.(ptr)
  * @param  B_Addr_imu   Read / write imu interface definitions.(ptr)
  * @param  val       Get the values of iel in reg INT_CFG_M.(ptr)
  * @retval           Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Pin_Notification_Get(uint8_t B_Addr_mag, uint8_t B_Addr_imu, LSM9DS1_LIR_t *val)
{
  LSM9DS1_INT_CFG_M_t int_cfg_m;
  LSM9DS1_INT_GEN_CFG_G_t int_gen_cfg_g;
  LSM9DS1_CTRL_REG4_t ctrl_reg4;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr_imu, LSM9DS1_CTRL_REG4, 1, (uint8_t*)&ctrl_reg4);
  if(ret == 0)
  {
    ret = LSM9DS1_ReadReg(B_Addr_imu, LSM9DS1_INT_GEN_CFG_G, 1, (uint8_t*)&int_gen_cfg_g);
  }
  if(ret == 0)
  {
    ret = LSM9DS1_ReadReg(B_Addr_mag, LSM9DS1_INT_CFG_M, 1, (uint8_t*)&int_cfg_m);
  }
  switch (int_cfg_m.iel & int_gen_cfg_g.lir_g & ctrl_reg4.lir_xl1)
  {
    case LSM9DS1_INT_LATCHED:
      *val = LSM9DS1_INT_LATCHED;
      break;
    case LSM9DS1_INT_PULSED:
      *val = LSM9DS1_INT_PULSED;
      break;
    default:
      *val = LSM9DS1_INT_LATCHED;
      break;
  }

  return ret;
}
/**
  * @brief  Push-pull/open drain selection on interrupt pads.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "pp_od" in reg LSM9DS1.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Pin_Mode_Set(uint8_t B_Addr, LSM9DS1_PP_OD_t val)
{
  LSM9DS1_CTRL_REG8_t ctrl_reg8;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG8, 1, (uint8_t*)&ctrl_reg8);
  if(ret == 0){
    ctrl_reg8.pp_od = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_CTRL_REG8, 1, (uint8_t*)&ctrl_reg8);
  }
  return ret;
}

/**
  * @brief  Push-pull/open drain selection on interrupt pads.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Get the values of pp_od in reg CTRL_REG8.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Pin_Mode_Get(uint8_t B_Addr, LSM9DS1_PP_OD_t *val)
{
  LSM9DS1_CTRL_REG8_t ctrl_reg8;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG8, 1, (uint8_t*)&ctrl_reg8);
  switch (ctrl_reg8.pp_od)
  {
    case LSM9DS1_PUSH_PULL:
      *val = LSM9DS1_PUSH_PULL;
      break;
    case LSM9DS1_OPEN_DRAIN:
      *val = LSM9DS1_OPEN_DRAIN;
      break;
    default:
      *val = LSM9DS1_PUSH_PULL;
      break;
  }
  return ret;
}

/**
  * @brief  Route a signal on INT_M pin.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val      Interrupt enable on the INT_M pin.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Pin_INT_M_Route_Set(uint8_t B_Addr, LSM9DS1_PIN_M_ROUTE_t val)
{
  LSM9DS1_INT_CFG_M_t int_cfg_m;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_CFG_M, 1, (uint8_t*)&int_cfg_m);
  if(ret == 0) 
  {
    int_cfg_m.ien  = val.ien;
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_INT_CFG_M, 1, (uint8_t*)&int_cfg_m);
  }
  return ret;
}

/**
  * @brief  Route a signal on INT_M pin.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val      Interrupt enable on the INT_M pin.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Pin_INT_M_Route_Get(uint8_t B_Addr, LSM9DS1_PIN_M_ROUTE_t *val)
{
  LSM9DS1_INT_CFG_M_t int_cfg_m;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_CFG_M, 1, (uint8_t*)&int_cfg_m);
  val->ien = int_cfg_m.ien;

  return ret;
}

/**
  * @brief  Configure the interrupt pin polarity.[set]
  *
  * @param  B_Addr_mag   Read / write magnetometer interface definitions.(ptr)
  * @param  B_Addr_imu   Read / write imu interface definitions.(ptr)
  * @param  val       Change the values of "iea" in reg LSM9DS1.
  * @retval           Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Pin_Polarity_Set(uint8_t B_Addr_mag, uint8_t B_Addr_imu, LSM9DS1_POLARITY_t val)
{
  LSM9DS1_INT_CFG_M_t int_cfg_m;
  LSM9DS1_CTRL_REG8_t ctrl_reg8;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr_mag, LSM9DS1_INT_CFG_M, 1, (uint8_t*)&int_cfg_m);
  if(ret == 0)
  {
    int_cfg_m.iea = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr_mag, LSM9DS1_INT_CFG_M, 1, (uint8_t*)&int_cfg_m);
  }
  if(ret == 0)
  {
    ret = LSM9DS1_ReadReg(B_Addr_imu, LSM9DS1_CTRL_REG8, 1, (uint8_t*)&ctrl_reg8);
  }
  if(ret == 0)
  {
    ctrl_reg8.h_lactive = (uint8_t)(~val);
    ret = LSM9DS1_WriteReg(B_Addr_imu, LSM9DS1_CTRL_REG8, 1, (uint8_t*)&ctrl_reg8);
  }

  return ret;
}

/**
  * @brief  Configure the interrupt pin polarity.[get]
  *
  * @param  B_Addr_mag   Read / write magnetometer interface definitions.(ptr)
  * @param  B_Addr_imu   Read / write imu interface definitions.(ptr)
  * @param  val       Get the values of iea in reg INT_CFG_M.(ptr)
  * @retval           Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Pin_Polarity_Get(uint8_t B_Addr_mag, uint8_t B_Addr_imu, LSM9DS1_POLARITY_t *val)
{
  LSM9DS1_INT_CFG_M_t int_cfg_m;
  LSM9DS1_CTRL_REG8_t ctrl_reg8;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr_mag, LSM9DS1_INT_CFG_M, 1, (uint8_t*)&int_cfg_m);
  if(ret == 0)
  {
    ret = LSM9DS1_ReadReg(B_Addr_imu, LSM9DS1_CTRL_REG8, 1, (uint8_t*)&ctrl_reg8);
  }
  switch (int_cfg_m.iea & (~ctrl_reg8.h_lactive))
  {
    case LSM9DS1_ACTIVE_LOW:
      *val = LSM9DS1_ACTIVE_LOW;
      break;
    case LSM9DS1_ACTIVE_HIGH:
      *val = LSM9DS1_ACTIVE_HIGH;
      break;
    default:
      *val = LSM9DS1_ACTIVE_LOW;
      break;
  }
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup     LSM9DS1_Interrupt_on_threshold
  * @brief        This section group all the functions concerning the
  *               interrupt on threshold configuration
  * @{
  *
  */

/**
  * @brief  Enable interrupt generation on threshold event.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Enable interrupt generation on accelerometer’s X-axis
  *                low event.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Xl_Trshld_Axis_Set(uint8_t B_Addr, LSM9DS1_XL_TRSHLD_EN_t val)
{
  LSM9DS1_INT_GEN_CFG_XL_t int_gen_cfg_xl;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_GEN_CFG_XL, 1, (uint8_t*)&int_gen_cfg_xl);
  if(ret == 0) 
  {
    int_gen_cfg_xl.xlie_xl  = val.xlie_xl;
    int_gen_cfg_xl.xhie_xl  = val.xhie_xl;
    int_gen_cfg_xl.ylie_xl  = val.ylie_xl;
    int_gen_cfg_xl.zhie_xl  = val.zhie_xl;
    int_gen_cfg_xl.yhie_xl  = val.yhie_xl;
    int_gen_cfg_xl.zlie_xl  = val.zlie_xl;
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_INT_GEN_CFG_XL, 1, (uint8_t*)&int_gen_cfg_xl);
  }

  return ret;
}

/**
  * @brief  Enable interrupt generation on threshold event.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Enable interrupt generation on accelerometer’s X-axis
  *                low event.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Xl_Trshld_Axis_Get(uint8_t B_Addr, LSM9DS1_XL_TRSHLD_EN_t *val)
{
  LSM9DS1_INT_GEN_CFG_XL_t int_gen_cfg_xl;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_GEN_CFG_XL, 1, (uint8_t*)&int_gen_cfg_xl);
  val->xlie_xl = int_gen_cfg_xl.xlie_xl;
  val->xhie_xl = int_gen_cfg_xl.xhie_xl;
  val->ylie_xl = int_gen_cfg_xl.ylie_xl;
  val->yhie_xl = int_gen_cfg_xl.yhie_xl;
  val->zlie_xl = int_gen_cfg_xl.zlie_xl;
  val->zhie_xl = int_gen_cfg_xl.zhie_xl;

  return ret;
}

/**
  * @brief  Axis interrupt threshold.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores data to be write.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Xl_Trshld_Set(uint8_t B_Addr, uint8_t *buff)
{
  LSM9DS1_Error_et ret;
  ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_INT_GEN_THS_X_XL, 3, buff);
  return ret;
}

/**
  * @brief  Axis interrupt threshold.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Xl_Trshld_Get(uint8_t B_Addr, uint8_t *buff)
{
  LSM9DS1_Error_et ret;
  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_GEN_THS_X_XL, 3, buff);
  return ret;
}

/**
  * @brief  Enter/exit interrupt duration value.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Change the values of dur_xl in reg INT_GEN_DUR_XL.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Xl_Trshld_Min_Sample_Set(uint8_t B_Addr, uint8_t val)
{
  LSM9DS1_INT_GEN_DUR_XL_t int_gen_dur_xl;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_GEN_DUR_XL, 1, (uint8_t*)&int_gen_dur_xl);
  if(ret == 0)
  {
    int_gen_dur_xl.dur_xl = (uint8_t)val;
    if (val != 0x00U)
	{
      int_gen_dur_xl.wait_xl = PROPERTY_ENABLE;
    } else 
	{
      int_gen_dur_xl.wait_xl = PROPERTY_DISABLE;
    }
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_INT_GEN_DUR_XL, 1, (uint8_t*)&int_gen_dur_xl);
  }

  return ret;
}

/**
  * @brief  Enter/exit interrupt duration value.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Get the values of dur_xl in reg INT_GEN_DUR_XL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Xl_Trshld_Min_Sample_Get(uint8_t B_Addr, uint8_t *val)
{
  LSM9DS1_INT_GEN_DUR_XL_t int_gen_dur_xl;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_GEN_DUR_XL, 1, (uint8_t*)&int_gen_dur_xl);
  *val = (uint8_t)int_gen_dur_xl.dur_xl;

  return ret;
}

/**
  * @brief  Angular rate sensor interrupt on threshold source.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Pitch(X)low.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Gy_Trshld_Src_Get(uint8_t B_Addr, LSM9DS1_GY_TRSHLD_SRC_t *val)
{
  LSM9DS1_INT_GEN_SRC_G_t int_gen_src_g;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_GEN_SRC_G, 1, (uint8_t*)&int_gen_src_g);
  val->xl_g = int_gen_src_g.xl_g;
  val->xh_g = int_gen_src_g.xh_g;
  val->yl_g = int_gen_src_g.yl_g;
  val->yh_g = int_gen_src_g.yh_g;
  val->zl_g = int_gen_src_g.zl_g;
  val->zh_g = int_gen_src_g.zh_g;
  val->ia_g = int_gen_src_g.ia_g;

  return ret;
}

/**
  * @brief  Accelerometer interrupt on threshold source.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val     Accelerometer’s X low. event.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Xl_Trshld_Src_Get(uint8_t B_Addr, LSM9DS1_XL_TRSHLD_SRC_t *val)
{
  LSM9DS1_INT_GEN_SRC_XL_t int_gen_src_xl;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_GEN_SRC_XL, 1, (uint8_t*)&int_gen_src_xl);
  val->xl_xl = int_gen_src_xl.xl_xl;
  val->xh_xl = int_gen_src_xl.xh_xl;
  val->yl_xl = int_gen_src_xl.yl_xl;
  val->yh_xl = int_gen_src_xl.yh_xl;
  val->zl_xl = int_gen_src_xl.zl_xl;
  val->zh_xl = int_gen_src_xl.zh_xl;
  val->ia_xl = int_gen_src_xl.ia_xl;

  return ret;
}

/**
  * @brief  Enable interrupt generation on threshold event.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Enable interrupt generation on gyroscope’s pitch
  *                (X) axis low event.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Gy_Trshld_Axis_Set(uint8_t B_Addr, LSM9DS1_GY_TRSHLD_EN_t val)
{
  LSM9DS1_INT_GEN_CFG_G_t int_gen_cfg_g;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_GEN_CFG_G, 1, (uint8_t*)&int_gen_cfg_g);
  if(ret == 0) 
  {
    int_gen_cfg_g.xlie_g  = val.xlie_g;
    int_gen_cfg_g.xhie_g  = val.xhie_g;
    int_gen_cfg_g.ylie_g  = val.ylie_g;
    int_gen_cfg_g.yhie_g  = val.yhie_g;
    int_gen_cfg_g.zlie_g  = val.zlie_g;
    int_gen_cfg_g.zhie_g  = val.zhie_g;
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_INT_GEN_CFG_G, 1, (uint8_t*)&int_gen_cfg_g);
  }
  return ret;
}

/**
  * @brief  Enable interrupt generation on threshold event.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val     Enable interrupt generation on gyroscope’s pitch
  *                 (X) axis low event.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Gy_Trshld_Axis_Get(uint8_t B_Addr, LSM9DS1_GY_TRSHLD_EN_t *val)
{
  LSM9DS1_INT_GEN_CFG_G_t int_gen_cfg_g;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_GEN_CFG_G, 1, (uint8_t*)&int_gen_cfg_g);
  val->xlie_g = int_gen_cfg_g.xlie_g;
  val->xhie_g = int_gen_cfg_g.xhie_g;
  val->ylie_g = int_gen_cfg_g.ylie_g;
  val->yhie_g = int_gen_cfg_g.yhie_g;
  val->zlie_g = int_gen_cfg_g.zlie_g;
  val->zhie_g = int_gen_cfg_g.zhie_g;
  return ret;
}

/**
  * @brief  Decrement or reset counter mode selection.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "dcrm_g" in reg LSM9DS1.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Gy_Trshld_Mode_Set(uint8_t B_Addr, LSM9DS1_DCRM_G_t val)
{
  LSM9DS1_INT_GEN_THS_XH_G_t int_gen_ths_xh_g;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_GEN_THS_XH_G, 1, (uint8_t*)&int_gen_ths_xh_g);
  if(ret == 0)
  {
    int_gen_ths_xh_g.dcrm_g = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_INT_GEN_THS_XH_G, 1, (uint8_t*)&int_gen_ths_xh_g);
  }
  return ret;
}

/**
  * @brief  Decrement or reset counter mode selection.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Get the values of dcrm_g in reg INT_GEN_THS_XH_G.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Gy_Trshld_Mode_Get(uint8_t B_Addr, LSM9DS1_DCRM_G_t *val)
{
  LSM9DS1_INT_GEN_THS_XH_G_t int_gen_ths_xh_g;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_GEN_THS_XH_G, 1, (uint8_t*)&int_gen_ths_xh_g);
  switch (int_gen_ths_xh_g.dcrm_g)
  {
    case LSM9DS1_RESET_MODE:
      *val = LSM9DS1_RESET_MODE;
      break;
    case LSM9DS1_DECREMENT_MODE:
      *val = LSM9DS1_DECREMENT_MODE;
      break;
    default:
      *val = LSM9DS1_RESET_MODE;
      break;
  }
  return ret;
}

/**
  * @brief  Angular rate sensor interrupt threshold on pitch (X) axis.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Change the values of ths_g_x in reg INT_GEN_THS_XH_G.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Gy_Trshld_X_Set(uint8_t B_Addr, uint16_t val)
{
  LSM9DS1_INT_GEN_THS_XH_G_t int_gen_ths_xh_g;
  LSM9DS1_INT_GEN_THS_XL_G_t int_gen_ths_xl_g;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_GEN_THS_XH_G, 1, (uint8_t*)&int_gen_ths_xh_g);
  if(ret == 0)
  {
    int_gen_ths_xh_g.ths_g_x = (uint8_t)((val & 0x7F00U) >> 8);
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_INT_GEN_THS_XH_G, 1, (uint8_t*)&int_gen_ths_xh_g);
  }
  if(ret == 0)
  {
    ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_GEN_THS_XL_G, 1, (uint8_t*)&int_gen_ths_xl_g);
  }
  if(ret == 0)
  {
    int_gen_ths_xl_g.ths_g_x = (uint8_t)(val & 0x00FFU);
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_INT_GEN_THS_XL_G, 1, (uint8_t*)&int_gen_ths_xl_g);
  }

  return ret;
}

/**
  * @brief  Angular rate sensor interrupt threshold on pitch (X) axis.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Get the values of ths_g_x in reg INT_GEN_THS_XH_G.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Gy_Trshld_X_Get(uint8_t B_Addr, uint16_t *val)
{
  LSM9DS1_INT_GEN_THS_XH_G_t int_gen_ths_xh_g;
  LSM9DS1_INT_GEN_THS_XL_G_t int_gen_ths_xl_g;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_GEN_THS_XH_G, 1, (uint8_t*)&int_gen_ths_xh_g);
  if(ret == 0)
  {
    ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_GEN_THS_XL_G, 1, (uint8_t*)&int_gen_ths_xl_g);
  }
  *val = int_gen_ths_xh_g.ths_g_x;
  *val = *val << 8;
  *val += int_gen_ths_xl_g.ths_g_x;
  return ret;
}


/**
  * @brief  Angular rate sensor interrupt threshold on roll (Y) axis.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Change the values of ths_g_y in reg INT_GEN_THS_YH_G.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Gy_Trshld_Y_Set(uint8_t B_Addr, uint16_t val)
{
  LSM9DS1_INT_GEN_THS_YH_G_t int_gen_ths_yh_g;
  LSM9DS1_INT_GEN_THS_YL_G_t int_gen_ths_yl_g;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_GEN_THS_YH_G, 1, (uint8_t*)&int_gen_ths_yh_g);
  if(ret == 0)
  {
    int_gen_ths_yh_g.ths_g_y = (uint8_t)((val & 0x7F00U) >> 8);
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_INT_GEN_THS_YH_G, 1, (uint8_t*)&int_gen_ths_yh_g);
  }
  if(ret == 0)
  {
    ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_GEN_THS_YL_G, 1, (uint8_t*)&int_gen_ths_yl_g);
  }
  if(ret == 0)
  {
    int_gen_ths_yl_g.ths_g_y = (uint8_t)(val & 0x00FFU);
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_INT_GEN_THS_YL_G, 1, (uint8_t*)&int_gen_ths_yl_g);
  }

  return ret;
}

/**
  * @brief  Angular rate sensor interrupt threshold on roll (Y) axis.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Get the values of ths_g_y in reg INT_GEN_THS_YH_G.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Gy_Trshld_Y_Get(uint8_t B_Addr, uint16_t *val)
{
  LSM9DS1_INT_GEN_THS_YH_G_t int_gen_ths_yh_g;
  LSM9DS1_INT_GEN_THS_YL_G_t int_gen_ths_yl_g;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_GEN_THS_YH_G, 1, (uint8_t*)&int_gen_ths_yh_g);
  if(ret == 0)
  {
    ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_GEN_THS_YL_G, 1, (uint8_t*)&int_gen_ths_yl_g);
  }
  *val = (uint8_t)int_gen_ths_yh_g.ths_g_y;
  *val = *val << 8;
  *val += int_gen_ths_yl_g.ths_g_y;
  return ret;
}

/**
  * @brief  Angular rate sensor interrupt thresholds on yaw (Z) axis.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Change the values of ths_g_z in reg INT_GEN_THS_ZH_G.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Gy_Trshld_Z_Set(uint8_t B_Addr, uint16_t val)
{
  LSM9DS1_INT_GEN_THS_ZH_G_t int_gen_ths_zh_g;
  LSM9DS1_INT_GEN_THS_ZL_G_t int_gen_ths_zl_g;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_GEN_THS_ZH_G, 1, (uint8_t*)&int_gen_ths_zh_g);
  if(ret == 0)
  {
    int_gen_ths_zh_g.ths_g_z = (uint8_t)((val & 0x7F00U) >> 8);
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_INT_GEN_THS_ZH_G, 1, (uint8_t*)&int_gen_ths_zh_g);
  }
  if(ret == 0)
  {
    ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_GEN_THS_ZL_G, 1, (uint8_t*)&int_gen_ths_zl_g);
  }
  if(ret == 0)
  {
    int_gen_ths_zl_g.ths_g_z = (uint8_t)(val & 0x00FFU);
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_INT_GEN_THS_ZL_G, 1, (uint8_t*)&int_gen_ths_zl_g);
  }

  return ret;
}

/**
  * @brief  Angular rate sensor interrupt thresholds on yaw (Z) axis.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Get the values of ths_g_z in reg INT_GEN_THS_ZH_G.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Gy_Trshld_Z_Get(uint8_t B_Addr, uint16_t *val)
{
  LSM9DS1_INT_GEN_THS_ZH_G_t int_gen_ths_zh_g;
  LSM9DS1_INT_GEN_THS_ZL_G_t int_gen_ths_zl_g;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_GEN_THS_ZH_G, 1, (uint8_t*)&int_gen_ths_zh_g);
  if(ret == 0)
  {
    ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_GEN_THS_ZL_G, 1, (uint8_t*)&int_gen_ths_zl_g);
  }
  *val = int_gen_ths_zh_g.ths_g_z;
  *val = *val << 8;
  *val += int_gen_ths_zl_g.ths_g_z;

  return ret;
}

/**
  * @brief  Enter/exit interrupt duration value.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Change the values of dur_g in reg INT_GEN_DUR_G.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Gy_Trshld_Min_Sample_Set(uint8_t B_Addr, uint8_t val)
{
  LSM9DS1_INT_GEN_DUR_G_t int_gen_dur_g;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_GEN_DUR_G, 1, (uint8_t*)&int_gen_dur_g);
  if(ret == 0)
  {
    if (val != 0x00U)
	{
      int_gen_dur_g.wait_g = PROPERTY_ENABLE;
    } else 
	{
      int_gen_dur_g.wait_g = PROPERTY_DISABLE;
    }
    int_gen_dur_g.dur_g = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_INT_GEN_DUR_G, 1, (uint8_t*)&int_gen_dur_g);
  }

  return ret;
}

/**
  * @brief  Enter/exit interrupt duration value.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Get the values of dur_g in reg INT_GEN_DUR_G.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Gy_Trshld_Min_Sample_Get(uint8_t B_Addr, uint8_t *val)
{
  LSM9DS1_INT_GEN_DUR_G_t int_gen_dur_g;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_GEN_DUR_G, 1, (uint8_t*)&int_gen_dur_g);
  *val = (uint8_t)int_gen_dur_g.dur_g;

  return ret;
}

/**
  * @brief  Enable interrupt generation on threshold event.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val     Enable interrupt generation on Z-axis. .
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Mag_Trshld_Axis_Set(uint8_t B_Addr, LSM9DS1_MAG_TRSHLD_AXIS_t val)
{
  LSM9DS1_INT_CFG_M_t int_cfg_m;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_CFG_M, 1, (uint8_t*)&int_cfg_m);
  if(ret == 0) {
    int_cfg_m.zien  = val.zien;
    int_cfg_m.xien  = val.xien;
    int_cfg_m.yien  = val.yien;
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_INT_CFG_M, 1, (uint8_t*)&int_cfg_m);
  }

  return ret;
}

/**
  * @brief  Enable interrupt generation on threshold event.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val     Enable interrupt generation on Z-axis. .(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Mag_Trshld_Axis_Get(uint8_t B_Addr, LSM9DS1_MAG_TRSHLD_AXIS_t *val)
{
  LSM9DS1_INT_CFG_M_t int_cfg_m;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_CFG_M, 1, (uint8_t*)&int_cfg_m);
  val->zien = int_cfg_m.zien;
  val->yien = int_cfg_m.yien;
  val->xien = int_cfg_m.xien;

  return ret;
}

/**
  * @brief  Magnetometer interrupt on threshold source.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val     This bit signals when the interrupt event occurs.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Mag_Trshld_Src_Get(uint8_t B_Addr, LSM9DS1_MAG_TRSHLD_SRC_t *val)
{
  LSM9DS1_INT_SRC_M_t int_src_m;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_SRC_M, 1, (uint8_t*)&int_src_m);
  val->_int = int_src_m._int;
  val->nth_z = int_src_m.nth_z;
  val->nth_y = int_src_m.nth_y;
  val->nth_x = int_src_m.nth_x;
  val->pth_z = int_src_m.pth_z;
  val->pth_y = int_src_m.pth_y;
  val->pth_x = int_src_m.pth_x;

  return ret;
}

/**
  * @brief  The value is expressed in 15-bit unsigned.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Iet the values of "ths" in reg INT_THS_L_M.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Mag_Trshld_Get(uint8_t B_Addr, uint8_t *val)
{
  LSM9DS1_INT_THS_L_M_t int_ths_l_m;
  LSM9DS1_INT_THS_H_M_t int_ths_h_m;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_THS_L_M, 1, (uint8_t*)&int_ths_l_m);
  if(ret == 0){
    ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_THS_H_M, 1, (uint8_t*)&int_ths_h_m);
  }

  *val = int_ths_h_m.ths;
  *val = *val << 8;
  *val += int_ths_l_m.ths;

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup     LSM9DS1_ Activity/Inactivity_detection
  * @brief        This section groups all the functions concerning
  *               activity/inactivity detection.
  * @{
  *
  */

/**
  * @brief  Inactivity threshold.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Change the values of act_ths in reg ACT_THS.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Act_Trshld_Set(uint8_t B_Addr, uint8_t val)
{
  LSM9DS1_ACT_THS_t act_ths;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_ACT_THS, 1, (uint8_t*)&act_ths);
  if(ret == 0)
  {
    act_ths.act_ths = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_ACT_THS, 1, (uint8_t*)&act_ths);
  }
  return ret;
}

/**
  * @brief  Inactivity threshold.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Get the values of act_ths in reg ACT_THS.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Act_Trshld_Get(uint8_t B_Addr, uint8_t *val)
{
  LSM9DS1_ACT_THS_t act_ths;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_ACT_THS, 1, (uint8_t*)&act_ths);
  *val = (uint8_t)act_ths.act_ths;

  return ret;
}

/**
  * @brief  Gyroscope operating mode during inactivity.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "sleep_on_inact_en" in reg LSM9DS1.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Act_Mode_Set(uint8_t B_Addr, LSM9DS1_ACT_MODE_t val)
{
  LSM9DS1_ACT_THS_t act_ths;
  LSM9DS1_CTRL_REG9_t ctrl_reg9;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_ACT_THS, 1, (uint8_t*)&act_ths);
  if(ret == 0)
  {
    act_ths.sleep_on_inact_en = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_ACT_THS, 1, (uint8_t*)&act_ths);
  }
  if(ret == 0)
  {
    ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG9, 1, (uint8_t*)&ctrl_reg9);
  }
  if(ret == 0){
    ctrl_reg9.sleep_g = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_CTRL_REG9, 1, (uint8_t*)&ctrl_reg9);
  }

  return ret;
}

/**
  * @brief  Gyroscope operating mode during inactivity.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Get the values of sleep_on_inact_en in reg ACT_THS.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Act_Mode_Get(uint8_t B_Addr, LSM9DS1_ACT_MODE_t *val)
{
  LSM9DS1_ACT_THS_t act_ths;
  LSM9DS1_CTRL_REG9_t ctrl_reg9;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_ACT_THS, 1, (uint8_t*)&act_ths);
  if(ret == 0)
  {
    ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG9, 1, (uint8_t*)&ctrl_reg9);
  }
  switch (act_ths.sleep_on_inact_en & ctrl_reg9.sleep_g)
  {
    case LSM9DS1_GYRO_POWER_DOWN:
      *val = LSM9DS1_GYRO_POWER_DOWN;
      break;
    case LSM9DS1_GYRO_SLEEP:
      *val = LSM9DS1_GYRO_SLEEP;
      break;
    default:
      *val = LSM9DS1_GYRO_POWER_DOWN;
      break;
  }

  return ret;
}

/**
  * @brief  Inactivity duration in number of sample.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores data to be write.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Act_Duration_Set(uint8_t B_Addr, uint8_t *buff)
{
  LSM9DS1_Error_et ret;
  ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_ACT_DUR, 1, buff);
  return ret;
}

/**
  * @brief  Inactivity duration in number of sample.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Act_Duration_Get(uint8_t B_Addr, uint8_t *buff)
{
  LSM9DS1_Error_et ret;
  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_ACT_DUR, 1, buff);
  return ret;
}

/**
  * @brief  Inactivity interrupt output signal.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Return an option of "LSM9DS1_INACT_t".(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Act_Src_Get(uint8_t B_Addr, LSM9DS1_INACT_t *val)
{
  LSM9DS1_STATUS_REG_t status_reg;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_STATUS_REG, 1, (uint8_t*)&status_reg);
  switch (status_reg.inact)
  {
    case LSM9DS1_ACTIVITY:
      *val = LSM9DS1_ACTIVITY;
      break;
    case LSM9DS1_INACTIVITY:
      *val = LSM9DS1_INACTIVITY;
      break;
    default:
      *val = LSM9DS1_ACTIVITY;
      break;
  }
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup     LSM9DS1_ Six_position_detection(6D/4D).
  * @brief        This section groups all the functions concerning six
  *               position detection (6D).
  * @{
  *
  */

/**
  * @brief  6D feature working mode.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "6d" in reg LSM9DS1.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_6d_Mode_Set(uint8_t B_Addr, LSM9DS1_6D_MODE_t val)
{
  LSM9DS1_INT_GEN_CFG_XL_t int_gen_cfg_xl;
  LSM9DS1_CTRL_REG4_t ctrl_reg4;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_GEN_CFG_XL, 1, (uint8_t*)&int_gen_cfg_xl);
  if(ret == 0)
  {
    int_gen_cfg_xl._6d = ((uint8_t)val & 0x01U);
    int_gen_cfg_xl.aoi_xl = ( ( (uint8_t)val & 0x02U ) >> 1 );
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_INT_GEN_CFG_XL, 1, (uint8_t*)&int_gen_cfg_xl);
  }
  if(ret == 0)
  {
    ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG4, 1, (uint8_t*)&ctrl_reg4);
  }
  if(ret == 0)
  {
    ctrl_reg4._4d_xl1 = ( ( (uint8_t)val & 0x04U ) >> 2 );
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_CTRL_REG4, 1, (uint8_t*)&ctrl_reg4);
  }
  return ret;
}

/**
  * @brief  6D feature working mode.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Get the values of 6d in reg INT_GEN_CFG_XL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_6d_Mode_Get(uint8_t B_Addr, LSM9DS1_6D_MODE_t *val)
{
  LSM9DS1_INT_GEN_CFG_XL_t int_gen_cfg_xl;
  LSM9DS1_CTRL_REG4_t ctrl_reg4;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_GEN_CFG_XL, 1, (uint8_t*)&int_gen_cfg_xl);
  if(ret == 0)
  {
    ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG4, 1, (uint8_t*)&ctrl_reg4);
  }
  switch ((ctrl_reg4._4d_xl1 << 2) | (int_gen_cfg_xl.aoi_xl << 1) | int_gen_cfg_xl.aoi_xl)
  {
    case LSM9DS1_POS_MOVE_RECO_DISABLE:
      *val = LSM9DS1_POS_MOVE_RECO_DISABLE;
      break;
    case LSM9DS1_6D_MOVE_RECO:
      *val = LSM9DS1_6D_MOVE_RECO;
      break;
    case LSM9DS1_4D_MOVE_RECO:
      *val = LSM9DS1_4D_MOVE_RECO;
      break;
    case LSM9DS1_6D_POS_RECO:
      *val = LSM9DS1_6D_POS_RECO;
      break;
    case LSM9DS1_4D_POS_RECO:
      *val = LSM9DS1_4D_POS_RECO;
      break;
    default:
      *val = LSM9DS1_POS_MOVE_RECO_DISABLE;
      break;
  }
  return ret;
}

/**
  * @brief  6D functionality axis interrupt threshold.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores data to be write.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_6d_Trshld_Set(uint8_t B_Addr, uint8_t *buff)
{
  LSM9DS1_Error_et ret;
  ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_INT_GEN_THS_X_XL, 3, buff);
  return ret;
}

/**
  * @brief  6D functionality axis interrupt threshold.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_6d_Trshld_Get(uint8_t B_Addr, uint8_t *buff)
{
  LSM9DS1_Error_et ret;
  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_GEN_THS_X_XL, 3, buff);
  return ret;
}

/**
  * @brief  .[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    .(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_6d_Src_Get(uint8_t B_Addr, LSM9DS1_6D_SRC_t *val)
{
  LSM9DS1_INT_GEN_SRC_XL_t int_gen_src_xl;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_INT_GEN_SRC_XL, 1, (uint8_t*)&int_gen_src_xl);
  val->xl_xl = int_gen_src_xl.xl_xl;
  val->xh_xl = int_gen_src_xl.xh_xl;
  val->yl_xl = int_gen_src_xl.yl_xl;
  val->yh_xl = int_gen_src_xl.yh_xl;
  val->zl_xl = int_gen_src_xl.zl_xl;
  val->zh_xl = int_gen_src_xl.zh_xl;
  val->ia_xl = int_gen_src_xl.ia_xl;

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup     LSM9DS1_Fifo
  * @brief        This section group all the functions concerning the
  *               fifo usage
  * @{
  *
  */

/**
  * @brief  Sensing chain FIFO stop values memorization at threshold
  *         level.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Change the values of stop_on_fth in reg CTRL_REG9.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Fifo_Stop_On_Wtm_Set(uint8_t B_Addr, uint8_t val)
{
  LSM9DS1_CTRL_REG9_t ctrl_reg9;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG9, 1, (uint8_t*)&ctrl_reg9);
  if(ret == 0)
  {
    ctrl_reg9.stop_on_fth = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_CTRL_REG9, 1, (uint8_t*)&ctrl_reg9);
  }
  return ret;
}

/**
  * @brief  Sensing chain FIFO stop values memorization at
  *         threshold level.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Get the values of stop_on_fth in reg CTRL_REG9.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Fifo_Stop_On_Wtm_Get(uint8_t B_Addr, uint8_t *val)
{
  LSM9DS1_CTRL_REG9_t ctrl_reg9;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG9, 1, (uint8_t*)&ctrl_reg9);
  *val = (uint8_t)ctrl_reg9.stop_on_fth;

  return ret;
}

/**
  * @brief  FIFO mode selection.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "fifo_en" in reg LSM9DS1.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Fifo_Mode_Set(uint8_t B_Addr, LSM9DS1_FIFO_MD_t val)
{
  LSM9DS1_CTRL_REG9_t ctrl_reg9;
  LSM9DS1_FIFO_CTRL_t fifo_ctrl;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG9, 1, (uint8_t*)&ctrl_reg9);
  if(ret == 0)
  {
    ctrl_reg9.fifo_en = ( ( (uint8_t)val & 0x10U ) >> 4);
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_CTRL_REG9, 1, (uint8_t*)&ctrl_reg9);
  }
  if(ret == 0)
  {
    ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_FIFO_CTRL, 1, (uint8_t*)&fifo_ctrl);
  }
  if(ret == 0)
  {
    fifo_ctrl.fmode = ( (uint8_t)val & 0x0FU );
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_FIFO_CTRL, 1, (uint8_t*)&fifo_ctrl);
  }
  return ret;
}

/**
  * @brief  FIFO mode selection.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Get the values of fifo_en in reg CTRL_REG9.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Fifo_Mode_Get(uint8_t B_Addr, LSM9DS1_FIFO_MD_t *val)
{
  LSM9DS1_CTRL_REG9_t ctrl_reg9;
  LSM9DS1_FIFO_CTRL_t fifo_ctrl;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG9, 1, (uint8_t*)&ctrl_reg9);
  if(ret == 0)
  {
    ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_FIFO_CTRL, 1, (uint8_t*)&fifo_ctrl);
  }
  switch ((ctrl_reg9.fifo_en << 4) | ctrl_reg9.fifo_en)
  {
    case LSM9DS1_FIFO_OFF:
      *val = LSM9DS1_FIFO_OFF;
      break;
    case LSM9DS1_BYPASS_MODE:
      *val = LSM9DS1_BYPASS_MODE;
      break;
    case LSM9DS1_FIFO_MODE:
      *val = LSM9DS1_FIFO_MODE;
      break;
    case LSM9DS1_STREAM_TO_FIFO_MODE:
      *val = LSM9DS1_STREAM_TO_FIFO_MODE;
      break;
    case LSM9DS1_BYPASS_TO_STREAM_MODE:
      *val = LSM9DS1_BYPASS_TO_STREAM_MODE;
      break;
    case LSM9DS1_STREAM_MODE:
      *val = LSM9DS1_STREAM_MODE;
      break;
    default:
      *val = LSM9DS1_FIFO_OFF;
      break;
  }

  return ret;
}

/**
  * @brief  Batching of temperature data.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Change the values of fifo_temp_en in reg CTRL_REG9.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Fifo_Temp_Batch_Set(uint8_t B_Addr, uint8_t val)
{
  LSM9DS1_CTRL_REG9_t ctrl_reg9;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG9, 1, (uint8_t*)&ctrl_reg9);
  if(ret == 0)
  {
    ctrl_reg9.fifo_temp_en = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_CTRL_REG9, 1, (uint8_t*)&ctrl_reg9);
  }
  return ret;
}

/**
  * @brief  Batching of temperature data.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Get the values of fifo_temp_en in reg CTRL_REG9.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Fifo_Temp_Batch_Get(uint8_t B_Addr, uint8_t *val)
{
  LSM9DS1_CTRL_REG9_t ctrl_reg9;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG9, 1, (uint8_t*)&ctrl_reg9);
  *val = (uint8_t)ctrl_reg9.fifo_temp_en;

  return ret;
}

/**
  * @brief  FIFO watermark level selection.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Change the values of fth in reg FIFO_CTRL.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Fifo_Watermark_Set(uint8_t B_Addr, uint8_t val)
{
  LSM9DS1_FIFO_CTRL_t fifo_ctrl;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_FIFO_CTRL, 1, (uint8_t*)&fifo_ctrl);
  if(ret == 0)
  {
    fifo_ctrl.fth = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_FIFO_CTRL, 1, (uint8_t*)&fifo_ctrl);
  }
  return ret;
}

/**
  * @brief  FIFO watermark level selection.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Get the values of fth in reg FIFO_CTRL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Fifo_Watermark_Get(uint8_t B_Addr, uint8_t *val)
{
  LSM9DS1_FIFO_CTRL_t fifo_ctrl;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_FIFO_CTRL, 1, (uint8_t*)&fifo_ctrl);
  *val = (uint8_t)fifo_ctrl.fth;

  return ret;
}

/**
  * @brief  FIFOfullstatus.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Iet the values of "fss" in reg FIFO_SRC.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Fifo_Full_Flag_Get(uint8_t B_Addr, uint8_t *val)
{
  LSM9DS1_FIFO_SRC_t fifo_src;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_FIFO_SRC, 1, (uint8_t*)&fifo_src);
  *val = fifo_src.fss;

  return ret;
}

/**
  * @brief  Number of unread words (16-bit axes) stored in FIFO.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Iet the values of "fss" in reg FIFO_SRC.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Fifo_Data_Level_Get(uint8_t B_Addr, uint8_t *val)
{
  LSM9DS1_FIFO_SRC_t fifo_src;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_FIFO_SRC, 1, (uint8_t*)&fifo_src);
  *val = fifo_src.fss;

  return ret;
}

/**
  * @brief  FIFO overrun status.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Iet the values of "ovrn" in reg FIFO_SRC.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Fifo_Ovr_Flag_Get(uint8_t B_Addr, uint8_t *val)
{
  LSM9DS1_FIFO_SRC_t fifo_src;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_FIFO_SRC, 1, (uint8_t*)&fifo_src);
  *val = fifo_src.ovrn;

  return ret;
}

/**
  * @brief  FIFO watermark status.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Iet the values of "fth" in reg FIFO_SRC.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Fifo_Wtm_Flag_Get(uint8_t B_Addr, uint8_t *val)
{
  LSM9DS1_FIFO_SRC_t fifo_src;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_FIFO_SRC, 1, (uint8_t*)&fifo_src);
  *val = fifo_src.fth;

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup     LSM9DS1_Self_test
  * @brief        This section groups all the functions that manage self
  *               test configuration
  * @{
  *
  */

/**
  * @brief  Enable/disable self-test mode for accelerometer.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Change the values of st_xl in reg CTRL_REG10.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Xl_Self_Test_Set(uint8_t B_Addr, uint8_t val)
{
  LSM9DS1_CTRL_REG10_t ctrl_reg10;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG10, 1, (uint8_t*)&ctrl_reg10);
  if(ret == 0)
  {
    ctrl_reg10.st_xl = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_CTRL_REG10, 1, (uint8_t*)&ctrl_reg10);
  }
  return ret;
}

/**
  * @brief  Enable/disable self-test mode for accelerometer.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Get the values of st_xl in reg CTRL_REG10.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Xl_Self_Test_Get(uint8_t B_Addr, uint8_t *val)
{
  LSM9DS1_CTRL_REG10_t ctrl_reg10;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG10, 1, (uint8_t*)&ctrl_reg10);
  *val = (uint8_t)ctrl_reg10.st_xl;

  return ret;
}

/**
  * @brief  Enable/disable self-test mode for gyroscope.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Change the values of st_g in reg CTRL_REG10.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Gy_Self_Test_Set(uint8_t B_Addr, uint8_t val)
{
  LSM9DS1_CTRL_REG10_t ctrl_reg10;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG10, 1, (uint8_t*)&ctrl_reg10);
  if(ret == 0)
  {
    ctrl_reg10.st_g = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_CTRL_REG10, 1, (uint8_t*)&ctrl_reg10);
  }
  return ret;
}

/**
  * @brief  Enable/disable self-test mode for gyroscope.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Get the values of st_g in reg CTRL_REG10.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Gy_Self_Test_Get(uint8_t B_Addr, uint8_t *val)
{
  LSM9DS1_CTRL_REG10_t ctrl_reg10;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG10, 1, (uint8_t*)&ctrl_reg10);
  *val = (uint8_t)ctrl_reg10.st_g;

  return ret;
}

/**
  * @brief  Enable/disable self-test mode for magnatic sensor.[set]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Change the values of st in reg CTRL_REG1_M.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Mag_Self_Test_Set(uint8_t B_Addr, uint8_t val)
{
  LSM9DS1_CTRL_REG1_M_t ctrl_reg1_m;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG1_M, 1, (uint8_t*)&ctrl_reg1_m);
  if(ret == 0)
  {
    ctrl_reg1_m.st = (uint8_t)val;
    ret = LSM9DS1_WriteReg(B_Addr, LSM9DS1_CTRL_REG1_M, 1, (uint8_t*)&ctrl_reg1_m);
  }
  return ret;
}

/**
  * @brief  Enable/disable self-test mode for magnatic sensor.[get]
  *
  * @param  B_Addr    Read / write interface definitions.(ptr)
  * @param  val    Get the values of st in reg CTRL_REG1_M.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
LSM9DS1_Error_et LSM9DS1_Mag_Self_Test_Get(uint8_t B_Addr, uint8_t *val)
{
  LSM9DS1_CTRL_REG1_M_t ctrl_reg1_m;
  LSM9DS1_Error_et ret;

  ret = LSM9DS1_ReadReg(B_Addr, LSM9DS1_CTRL_REG1_M, 1, (uint8_t*)&ctrl_reg1_m);
  *val = (uint8_t)ctrl_reg1_m.st;

  return ret;
}

/**
  * @}
  *
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
