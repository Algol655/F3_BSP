/**
******************************************************************************
* File Name          : ENS160.C
* Description        : This file contains the common defines of the application
******************************************************************************
* Created by Tommaso Sabatini 2022
******************************************************************************
*/

#include "platform/ENS160_Driver.h"
#include "compiler/compiler.h"

uint8_t BurnIn_Time_Complete=0;
uint8_t RunIn_Time_Complete=0;
uint8_t Baseline_Time_Complete=0;
uint8_t EBaseline_Time_Complete=0;

//These are the air quality values obtained from the sensor
uint16_t tVOC = 0;
uint16_t CO2 = 0;
uint8_t ENS160_AQI;

/** @defgroup LPS25HB_My_Private_Variables
* @{
*/
uint8_t ENS160_1[7] = 	{
							ENS160_OPMODE_VAL,
							ENS160_CONFIG_VAL,
							ENS160_COMMAND_VAL,
							ENS160_TEMP_IN_VAL_0,
							ENS160_TEMP_IN_VAL_1,
							ENS160_RH_IN_VAL_0,
							ENS160_RH_IN_VAL_1,
						};
uint8_t ENS160_2[8] = 	{
							ENS160_GPR_WRITE_0_VAL,
							ENS160_GPR_WRITE_1_VAL,
							ENS160_GPR_WRITE_2_VAL,
							ENS160_GPR_WRITE_3_VAL,
							ENS160_GPR_WRITE_4_VAL,
							ENS160_GPR_WRITE_5_VAL,
							ENS160_GPR_WRITE_6_VAL,
							ENS160_GPR_WRITE_7_VAL
						};

/**
* @}
*/

/**
 * @brief  Calculates eCO2 moving average
 */
void eCO2_MovingAverage(uint16_t *in, uint16_t *out, uint16_t length)
{
	uint32_t sum = 0;
	static uint32_t mem1[10] = {0};	//array dimension MUST be = length
	static uint32_t mem2[1] = {0};

	mem2[0] = (mem2[0] + 1) % length;

	mem1[mem2[0]] = *in;

	for (int32_t i = 0; i < length; i++)
	{
		sum += mem1[i];
	}

	*out = sum / length;
}

/**
 * @brief  Calculates eTVOC moving average
 */
void eTVOC_MovingAverage(uint16_t *in, uint16_t *out, uint16_t length)
{
	uint32_t sum = 0;
	static uint32_t mem1[10] = {0};	//array dimension MUST be = length
	static uint32_t mem2[1] = {0};

	mem2[0] = (mem2[0] + 1) % length;

	mem1[mem2[0]] = *in;

	for (int32_t i = 0; i < length; i++)
	{
		sum += mem1[i];
	}

	*out = sum / length;
}

/**
  * @brief  Read generic device register
  *
  * @param  B_Addr   read / write interface definitions(ptr)
  * @param  reg   register to read
  * @param  data  pointer to buffer that store the data read(ptr)
  * @param  len   number of consecutive register to read
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
ENS160_Error_et ENS160_ReadReg(uint8_t B_Addr, uint8_t RegAddr, uint16_t NumByteToRead, uint8_t *Data)
{

//if ( NumByteToRead > 1 ) RegAddr |= 0x80;
  if (I2C_ReadData(B_Addr, RegAddr, &Data[0], NumByteToRead))
    return ENS160_ERROR;
  else
    return ENS160_OK;
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
  */
ENS160_Error_et ENS160_WriteReg(uint8_t B_Addr, uint8_t RegAddr, uint16_t NumByteToWrite, uint8_t *Data)
{

//if ( NumByteToWrite > 1 ) RegAddr |= 0x80;
  if (I2C_WriteData(B_Addr, RegAddr, &Data[0], NumByteToWrite))
    return ENS160_ERROR;
  else
    return ENS160_OK;
}

/*******************************************************************************
* Function Name	: ENS160_WriteReg
* Description   : Generic Writing function in DMA mode. It must be fullfilled with either
*         		: I2C or SPI writing function
* Input       	: Register Address, Data to be written
* Output      	: None
* Return      	: Status [ENS160_ERROR, ENS160_OK]
*******************************************************************************/
ENS160_Error_et ENS160_WriteReg_DMA(uint8_t B_Addr, uint8_t RegAddr, uint16_t NumByteToWrite, uint8_t *Data)
{

//if ( NumByteToWrite > 1 ) RegAddr |= 0x80;
  if (I2C_WriteData_DMA(B_Addr, RegAddr, &Data[0], NumByteToWrite))
    return ENS160_ERROR;
  else
    return ENS160_OK;
}

/*******************************************************************************
* Function Name	: ENS160_Init
* Description   : Device initialization
* Input       	: Register Address, Data to be written
* Output      	: None
* Return      	: Status [ENS160_ERROR, ENS160_OK]
*******************************************************************************/
ENS160_Error_et ENS160_Init(uint8_t B_Addr, uint8_t RegAddr, uint8_t* RegValues, uint8_t Size)
{
	if (ENS160_WriteReg(B_Addr, RegAddr, Size, &RegValues[0]))
		return ENS160_ERROR;

	return ENS160_OK;
}

/**
  * @brief  DeviceWhoamI.[get]
  *
  * @param  B_Addr    ENS160 Base Address
  * @param  buff      Buffer that stores the data read.(ptr)
  * @retval           Interface status (MANDATORY: return 0 -> no Error).
  *
  */
ENS160_Error_et ENS160_Get_Dev(uint8_t B_Addr, uint8_t *buff)
{
	ENS160_Error_et ret = 0;

	uint16_t DevId;

	if (ENS160_ReadReg(B_Addr, ENS160_PART_ID, 2, buff))
		return ENS160_ERROR;

	DevId = (uint16_t)((buff[1] << 8) | buff[0]);

	if (DevId != ENS160_WHO_AM_I_VAL)
		return ENS160_ERROR;

	return ret;
}

/**
  * @brief  ENS160_Idle_Mode - Initialize idle mode
  *
  * @param  B_Addr    ENS160 Base Address
  * @retval           Interface status (MANDATORY: return 0 -> no Error).
  *
  */
ENS160_Error_et ENS160_Idle_Mode(uint8_t B_Addr)
{
	ENS160_Error_et ret = 0;

	if (ENS160_WriteReg(B_Addr, ENS160_COMMAND, 1, (uint8_t*)ENS160_COMMAND_NOP))
		return ENS160_ERROR;
	if (ENS160_WriteReg(B_Addr, ENS160_COMMAND, 1, (uint8_t*)ENS160_COMMAND_CLRGPR))
		return ENS160_ERROR;
	Sleep(10);

	if (ENS160_WriteReg(B_Addr, ENS160_OPMODE, 1, (uint8_t*)ENS160_OPMODE_IDLE))
		return ENS160_ERROR;
	Sleep(10);

	return ret;
}

/**
  * @brief  ENS160_Idle_Mode - Initialize Standard mode
  *
  * @param  B_Addr    ENS160 Base Address
  * @retval           Interface status (MANDATORY: return 0 -> no Error).
  *
  */
ENS160_Error_et ENS160_Std_Mode(uint8_t B_Addr)
{
	ENS160_Error_et ret = 0;
	uint8_t OpMode[1] = {0x02};

//	if (ENS160_WriteReg(B_Addr, ENS160_OPMODE, 1, (uint8_t*)ENS160_OPMODE_STD))
	if (ENS160_WriteReg(B_Addr, ENS160_OPMODE, 1, (uint8_t*)OpMode))
		return ENS160_ERROR;
	Sleep(20);

	return ret;
}

/**
  * @brief  ENS160_Get_FW_Ver - Read firmware revisions
  *
  * @param  B_Addr    ENS160 Base Address
  * @param  buff      Buffer that stores the data read.(ptr)
  * @retval           Interface status (MANDATORY: return 0 -> no Error).
  *
  */
ENS160_Error_et ENS160_Get_FW_Ver(uint8_t B_Addr, uint8_t *buff, bool Check_FW_Ver)
{
	ENS160_Error_et ret = 0;
	uint32_t FW_Vers = 0;

	if (ENS160_WriteReg(B_Addr, ENS160_COMMAND, 1, (uint8_t*)ENS160_COMMAND_GET_APPVER))
		return ENS160_ERROR;
	Sleep(20);
	if (ENS160_ReadReg(B_Addr, ENS160_GPR_READ_4, 3, buff))
		return ENS160_ERROR;

	FW_Vers = ((uint32_t)(buff[2] & 0xFF) << 16) | ((uint32_t)(buff[1] & 0xFF) << 8) | (uint32_t)(buff[0] & 0xFF);

	if (Check_FW_Ver)
	{
		if (FW_Vers != 0xC91EF)
			return ENS160_ERROR;
	}

	return ret;
}

/*
 * @brief //Given a temp and humidity, write this data to the ENS160 for better compensation
	 //This function expects the humidity and temp to come in as floats
 * @param  relativeHumidity HUMIDITY.
 * @param  temperature TEMPERATURE.
 * @retval None.
*/
ENS160_Error_et ENS160_SetEnvironmentalData(float relativeHumidity, float temperature)
{
	ENS160_Error_et ret = 0;

	uint8_t envData[4];
	uint16_t temp;
	uint16_t rH;

	temp = (uint16_t)((lrintf((temperature + 273.15f) * 64.0f)));
	rH = (uint16_t)((lrintf(relativeHumidity * 512.0f)));

	envData[0] = temp & 0xFF;
	envData[1] = (temp >> 8) & 0xFF;
	envData[2] = rH & 0xff;;
	envData[3] = (rH >> 8) & 0xFF;

	if(ENS160_WriteReg(ENS160_BADDR, ENS160_TEMP_IN, 4, &envData[0]))
		return ENS160_ERROR;

	return ret;
}

/*
  * @brief  Updates the total volatile organic compounds (TVOC) in parts per billion (PPB) and the CO2 value.
  * @param  Pointer to the buffer that stores the data read.
  * @retval Interface status (MANDATORY: return 0 -> no Error).
 */
ENS160_Error_et	ENS160_Get_Measurement(ENS160_MeasureTypeDef_st *Measurement_Value)
{
	ENS160_Error_et ret = 0;
	uint8_t data_rq[12];
	static uint16_t ECO2, ETVOC;

	if(ENS160_ReadReg(ENS160_BADDR, ENS160_DEVICE_STATUS, 12, &data_rq[0]))
		return ENS160_ERROR;

	Measurement_Value->Status = (uint8_t)data_rq[0];
	if(data_rq[0] & ENS160_DATA_STATUS_NEWDAT)		//Check for new environmental data availability
	{
		Measurement_Value->AQI_UBA = (uint8_t)data_rq[1];
	//	Measurement_Value->eTVOC = (uint16_t)((data_rq[3] << 8) | data_rq[2]);
	//	Measurement_Value->eCO2 = (uint16_t)(data_rq[5] << 8) | data_rq[4]);
		ETVOC = (uint16_t)((data_rq[3] << 8) | data_rq[2]);
		eTVOC_MovingAverage(&ETVOC, &(Measurement_Value->eTVOC), 10);
		ECO2 = (uint16_t)((data_rq[5] << 8) | data_rq[4]);
		eCO2_MovingAverage(&ECO2, &(Measurement_Value->eCO2), 10);
	}

	return ret;
}

/*
 * @brief	Read the raw resistance and baseline values
 * @param 	NONE.
 * @retval	Interface status (MANDATORY: return 0 -> no Error).
*/
ENS160_Error_et	ENS160_Get_Raw_Data(ENS160_MeasureTypeDef_st *Measurement_Value)
{
	ENS160_Error_et ret = 0;
	uint8_t misr;
	uint8_t data_rq[6];
	uint8_t func_rq[8];

	if(ENS160_ReadReg(ENS160_BADDR, ENS160_DEVICE_STATUS, 6, &data_rq[0]))
		return ENS160_ERROR;

	Measurement_Value->Status = (uint8_t)data_rq[0];

	if(data_rq[0] & ENS160_DATA_STATUS_NEWGPR)		//Check for new ENS160 function data availability
	{
		if(ENS160_ReadReg(ENS160_BADDR, ENS160_GPR_READ_0, 8, &func_rq[0]))
			return ENS160_ERROR;
		else									// Read raw resistance values
		{
			Measurement_Value->RawData_0 = (uint16_t)((func_rq[1] << 8) | func_rq[0]);
			Measurement_Value->RawData_1 = (uint16_t)((func_rq[3] << 8) | func_rq[2]);
			Measurement_Value->RawData_2 = (uint16_t)((func_rq[5] << 8) | func_rq[4]);
			Measurement_Value->RawData_3 = (uint16_t)((func_rq[7] << 8) | func_rq[6]);
		}

		if(ENS160_ReadReg(ENS160_BADDR, ENS160_DATA_BL, 8, &func_rq[0]))
			return ENS160_ERROR;
		else									// Read raw resistance values
		{
			Measurement_Value->BaseLine_0 = (uint16_t)((func_rq[1] << 8) | func_rq[0]);
			Measurement_Value->BaseLine_1 = (uint16_t)((func_rq[3] << 8) | func_rq[2]);
			Measurement_Value->BaseLine_2 = (uint16_t)((func_rq[5] << 8) | func_rq[4]);
			Measurement_Value->BaseLine_3 = (uint16_t)((func_rq[7] << 8) | func_rq[6]);
		}

		if (ENS160_ReadReg(ENS160_BADDR, ENS160_DATA_MISR, 1, &misr))
			return ENS160_ERROR;
		else
			Measurement_Value->Misr = misr;
	}

	return ret;
}

/*
 * @brief  softRest
 * @param  NONE.
 * @retval Interface status (MANDATORY: return 0 -> no Error).
*/
ENS160_Error_et	ENS160_SoftRST()
{
	ENS160_Error_et ret = 0;

	if(ENS160_WriteReg(ENS160_BADDR, ENS160_OPMODE, 1, (uint8_t*)ENS160_OPMODE_RESET))
		return ENS160_ERROR;

	Sleep(10);		// Wait to boot after reset

	return ret;
}

/*******************************************************************************
* Function Name	: MX_ENS160_Init
* Description   : ENS160 Global init
*         		: I2C writing function
* Input       	: None
* Output      	: None
* Return      	: Status [ENS160_ERROR, ENS160_OK]
*******************************************************************************/
ENS160_Error_et MX_ENS160_Init()
{
	uint8_t FW_Ver[3];
	uint8_t DevID[2];

    // Sends a reset to the ENS160. Returns false on I2C problems
	if (ENS160_SoftRST())
		return ENS160_ERROR;
    Sleep(100);

    // Reads the part ID and confirms valid sensor
	if (ENS160_Get_Dev(ENS160_BADDR, (uint8_t*)DevID))
		return ENS160_ERROR;

	// Initialize idle mode and confirms
	if (ENS160_Idle_Mode(ENS160_BADDR))
		return ENS160_ERROR;

	// Read firmware revisions
	if (ENS160_Get_FW_Ver(ENS160_BADDR, (uint8_t*)FW_Ver, false))
		return ENS160_ERROR;

	// Initialize Standard mode, set references temperature and humidity and confirms
	if (ENS160_Init(ENS160_BADDR, ENS160_OPMODE, &ENS160_1[0], 7))
		return ENS160_ERROR;

	// Initialize Standard mode and confirms
/*	if (ENS160_Std_Mode(ENS160_BADDR))
		return ENS160_ERROR; */

	return ENS160_OK;
}

