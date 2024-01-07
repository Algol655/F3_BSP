/**
******************************************************************************
* File Name          : CCS811.C
* Description        : This file contains the common defines of the application
******************************************************************************
* Created by Akash kapashia, 2017
* Modified by Tommaso Sabatini 2019
******************************************************************************
*/
#include "platform/CCS811_Driver.h"
#include "compiler/compiler.h"

uint8_t BurnIn_Time_Complete=0;
uint8_t RunIn_Time_Complete=0;
uint8_t Baseline_Time_Complete=0;
uint8_t EBaseline_Time_Complete=0;

//These are the air quality values obtained from the sensor
uint16_t tVOC = 0;
uint16_t CO2 = 0;
uint8_t mosetting = 0;
uint32_t ELBaseline_period = 0;
uint32_t ALBaseline_period = 0;
bool load_baseline = false;
bool baseline_loaded = false;
float relativeHumidity = 65.0, temperature = 25.0;

/** @defgroup LPS25HB_My_Private_Variables
* @{
*/
uint8_t CCS811_1[1] = 	{							//Added By Me!!!
							CCS811_MEAS_MODE_VAL
						};
uint8_t CCS811_2[1] = 	{							//Added By Me!!!
							CCS811_APP_START_VAL
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
	static uint32_t eCO2_mem1[10] = {0};	//array dimension MUST be = length
	static uint32_t eCO2_mem2[1] = {0};

	eCO2_mem2[0] = (eCO2_mem2[0] + 1) % length;

	eCO2_mem1[eCO2_mem2[0]] = *in;

	for (int32_t i = 0; i < length; i++)
	{
		sum += eCO2_mem1[i];
	}

	*out = sum / length;
}

/**
 * @brief  Calculates eTVOC moving average
 */
void eTVOC_MovingAverage(uint16_t *in, uint16_t *out, uint16_t length)
{
	uint32_t sum = 0;
	static uint32_t eTVOC_mem1[10] = {0};	//array dimension MUST be = length
	static uint32_t eTVOC_mem2[1] = {0};

	eTVOC_mem2[0] = (eTVOC_mem2[0] + 1) % length;

	eTVOC_mem1[eTVOC_mem2[0]] = *in;

	for (int32_t i = 0; i < length; i++)
	{
		sum += eTVOC_mem1[i];
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
CCS811_Error_et CCS811_ReadReg(uint8_t B_Addr, uint8_t RegAddr, uint16_t NumByteToRead, uint8_t *Data)
{

//if ( NumByteToRead > 1 ) RegAddr |= 0x80;
  if (I2C_ReadData(B_Addr, RegAddr, &Data[0], NumByteToRead))
    return CCS811_ERROR;
  else
    return CCS811_OK;
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
CCS811_Error_et CCS811_WriteReg(uint8_t B_Addr, uint8_t RegAddr, uint16_t NumByteToWrite, uint8_t *Data)
{

//if ( NumByteToWrite > 1 ) RegAddr |= 0x80;
  if (I2C_WriteData(B_Addr, RegAddr, &Data[0], NumByteToWrite))
    return CCS811_ERROR;
  else
    return CCS811_OK;
}

/*******************************************************************************
* Function Name	: CCS811_WriteReg
* Description   : Generic Writing function in DMA mode. It must be fullfilled with either
*         		: I2C or SPI writing function
* Input       	: Register Address, Data to be written
* Output      	: None
* Return      	: Status [CCS811_ERROR, CCS811_OK]
*******************************************************************************/
CCS811_Error_et CCS811_WriteReg_DMA(uint8_t B_Addr, uint8_t RegAddr, uint16_t NumByteToWrite, uint8_t *Data)
{

//if ( NumByteToWrite > 1 ) RegAddr |= 0x80;
  if (I2C_WriteData_DMA(B_Addr, RegAddr, &Data[0], NumByteToWrite))
    return CCS811_ERROR;
  else
    return CCS811_OK;
}

/*******************************************************************************
* Function Name	: CCS811_Init
* Description   : Device initialization
* Input       	: Register Address, Data to be written
* Output      	: None
* Return      	: Status [CCS811_ERROR, CCS811_OK]
*******************************************************************************/
CCS811_Error_et CCS811_Init(uint8_t B_Addr, uint8_t RegAddr, uint8_t* RegValues, uint8_t Size)
{
	if (CCS811_WriteReg(B_Addr, RegAddr, Size, &RegValues[0]))
		return CCS811_ERROR;

	return CCS811_OK;
}

/**
  * @brief  DeviceWhoamI.[get]
  *
  * @param  B_Addr    CCS811 Base Address
  * @param  buff      Buffer that stores the data read.(ptr)
  * @retval           Interface status (MANDATORY: return 0 -> no Error).
  *
  */
CCS811_Error_et CCS811_Dev_Id_Get(uint8_t B_Addr, uint8_t *buff)
{
	CCS811_Error_et ret;
	ret = CCS811_ReadReg(B_Addr, CCS811_HW_ID, 1, buff);

	return ret;
}

/**
  * @brief  Status.[get]
  *
  * @param  B_Addr    CCS811 Base Address
  * @param  buff      Buffer that stores the data read.(ptr)
  * @retval           Interface status (MANDATORY: return 0 -> no Error).
  *
  */
CCS811_Error_et CCS811_Status_Get(uint8_t B_Addr, uint8_t *buff)
{
  CCS811_Error_et ret;
  ret = CCS811_ReadReg(B_Addr, CCS811_STATUS, 1, buff);

  return ret;
}

/**
  * @brief  Error.[get]
  *
  * @param  B_Addr    CCS811 Base Address
  * @param  buff      Buffer that stores the data read.(ptr)
  * @retval           Interface status (MANDATORY: return 0 -> no Error).
  *
  */
CCS811_Error_et CCS811_Error_Get(uint8_t B_Addr, uint8_t *buff)
{
  CCS811_Error_et ret;
  ret = CCS811_ReadReg(B_Addr, CCS811_ERROR_ID, 1, buff);

  return ret;
}

/*
 * @brief  //Mode 0 = Idle
	    //Mode 1 = read every 1s
	    //Mode 2 = every 10s
	    //Mode 3 = every 60s
	    //Mode 4 = RAW mode.
 * @param  MODE.
 * @retval Interface status (MANDATORY: return 0 -> no Error).
*/
CCS811_Error_et setDriveMode(uint8_t B_Addr, uint8_t mode)
{
//	uint8_t  *modeset;
	uint8_t *modeset = (uint8_t *)malloc(sizeof(uint8_t));	//To avoid:
	*modeset = mosetting;				//"modeset" is used uninitialized in this function warning

	if (mode > 4) mode = 4;					//Error correction
	if (CCS811_ReadReg(B_Addr, CCS811_MEAS_MODE, 1, &mosetting))	//Read what's currently there
		return CCS811_ERROR;
	HAL_Delay(i2c_delay);
	mosetting &=~(7<<4);					//Clear DRIVE_MODE bits
	mosetting |= (mode << 4);				//Mask in mode

	if (CCS811_WriteReg(B_Addr, CCS811_MEAS_MODE, 1, &mosetting))
		return CCS811_ERROR;

	return CCS811_OK;
}

/*
  * @brief  Updates the total volatile organic compounds (TVOC) in parts per billion (PPB) and the CO2 value.
  * @param  Pointer to the buffer that stores the data read.
  * @retval Interface status (MANDATORY: return 0 -> no Error).
 */
CCS811_Error_et	CCS811_Get_Measurement(CCS811_MeasureTypeDef_st *Measurement_Value)
{
	CCS811_Error_et ret;
	uint8_t data_rq[8];
	static uint16_t ECO2, ETVOC;

	ret = CCS811_ReadReg(CCS811_BADDR, CCS811_ALG_RESULT_DATA, 8, &data_rq[0]);

	/*	TVOC value, in parts per billion (ppb)
	eC02 value, in parts per million (ppm) */
	ECO2 = (uint16_t)((data_rq[0] << 8) | data_rq[1]);
	eCO2_MovingAverage(&ECO2, &(Measurement_Value->eCO2), 10);
	ETVOC = (uint16_t)((data_rq[2] << 8) | data_rq[3]);
	eTVOC_MovingAverage(&ETVOC, &(Measurement_Value->eTVOC), 10);
//	Measurement_Value->eCO2 = ((uint8_t)data_rq[0] << 8) | data_rq[1];
//	Measurement_Value->eTVOC = ((uint8_t)data_rq[2] << 8) | data_rq[3];
	Measurement_Value->Status = (uint8_t)data_rq[4];
	Measurement_Value->ErrorID = (uint8_t)data_rq[5];
	Measurement_Value->RawData = (uint16_t)((data_rq[6] << 8) | data_rq[7]);

	return ret;
}

/*
 * @brief //Given a temp and humidity, write this data to the CCS811 for better compensation
	 //This function expects the humidity and temp to come in as floats
 * @param  relativeHumidity HUMIDITY.
 * @param  temperature TEMPERATURE.
 * @retval None.
*/
CCS811_Error_et CCS811_SetEnvironmentalData(float relativeHumidity, float temperature)
{
	CCS811_Error_et ret;
	int rH = relativeHumidity * 1000; //42.348 becomes 42348
	int temp = temperature * 1000; //23.2 becomes 23200

	uint8_t envData[4];

	//Split value into 7-bit integer and 9-bit fractional
	envData[0] = ((rH % 1000) / 100) > 7 ? (rH / 1000 + 1) << 1 : (rH / 1000) << 1;
	envData[1] = 0; //CCS811 only supports increments of 0.5 so bits 7-0 will always be zero
	if (((rH % 1000) / 100) > 2 && (((rH % 1000) / 100) < 8))
	{
		envData[0] |= 1; //Set 9th bit of fractional to indicate 0.5%
	}

	temp += 25000; //Add the 25C offset
	//Split value into 7-bit integer and 9-bit fractional
	envData[2] = ((temp % 1000) / 100) > 7 ? (temp / 1000 + 1) << 1 : (temp / 1000) << 1;
	envData[3] = 0;
	if (((temp % 1000) / 100) > 2 && (((temp % 1000) / 100) < 8))
	{
		envData[2] |= 1;  //Set 9th bit of fractional to indicate 0.5C
	}

/*	uint8_t env[6];		//Debug
	env[0]=CCS811_ENV_DATA;
	env[1]=envData[0];
	env[2]=envData[1];
	env[3]=envData[2];
	env[5]=envData[3]; */

	ret = CCS811_WriteReg(CCS811_BADDR, CCS811_ENV_DATA, 4, &envData[0]);

	return ret;
}

/*
 * @brief  softRest
 * @param  NONE.
 * @retval Interface status (MANDATORY: return 0 -> no Error).
*/
CCS811_Error_et	CCS811_SoftRST()
{
	CCS811_Error_et ret;
	uint8_t rstCMD[4] = {0x11,0xE5,0x72,0x8A};

	ret = CCS811_WriteReg(CCS811_BADDR, CCS811_SW_RESET, 4, &rstCMD[0]);

	return ret;
}

/*
 * @brief	Read the baseline value and saves into EEPROM
 *			Used for telling sensor what 'clean' air is
 *			You must put the sensor in clean air and record this value
 * @param 	NONE.
 * @retval	Interface status (MANDATORY: return 0 -> no Error).
*/
CCS811_Error_et	CCS811_Save_Baseline(bool SaveToFlash)
{
	CCS811_Error_et ret;
	uint8_t baseline[2];
	uint8_t b_offset;
	extern FLASH_DATA_ORG FlashDataOrg;
	extern uint32_t CCS811_VOC_Ro;

	ret = CCS811_ReadReg(CCS811_BADDR, CCS811_BASELINE, 2, &baseline[0]);
	uint8_t baselineMSB = baseline[0];
	uint8_t baselineLSB = baseline[1];
	uint16_t BaseLine = ((uint16_t)baselineMSB << 8) | baselineLSB;
	CCS811_VOC_Ro = (uint32_t)BaseLine;
	if (SaveToFlash)
	{
		FlashDataOrg.b_status.s0 = (uint32_t)BaseLine;	//Save in board database structure

		b_offset = FlashDataOrg.b_status.s0_offset;
		Write_Flash(BaseLine, b_offset);
	}
	
	return ret;
}

/*
 * @brief	Restore the baseline value
 *			Used for telling sensor what 'clean' air is
 *			You must put the sensor in clean air and record this value
 * @param  NONE.
 * @retval Interface status (MANDATORY: return 0 -> no Error).
 */
CCS811_Error_et	CCS811_Restore_Baseline(bool RestoreFromFlash)
{
	CCS811_Error_et ret;
	uint32_t  restore_Baseline;
	uint8_t res_bs[2];
	uint8_t b_offset;
	extern FLASH_DATA_ORG FlashDataOrg;

	if (RestoreFromFlash)
	{
		b_offset = FlashDataOrg.b_status.s0_offset;
		restore_Baseline = *((uint32_t*)(DATA_EEPROM_BASE + b_offset));
		FlashDataOrg.b_status.s0 = restore_Baseline;	//Read from board data structure
	} else
	{
		restore_Baseline = FlashDataOrg.b_status.s0;
	}
	res_bs[0]=restore_Baseline>>8;
	res_bs[1]=restore_Baseline&0x000000FF;
	ret = CCS811_WriteReg(CCS811_BADDR, CCS811_BASELINE, 2, &res_bs[0]);

	return ret;
}

/*******************************************************************************
* Function Name	: MX_CCS811_Init
* Description   : CCS811 Global init
*         		: I2C writing function
* Input       	: None
* Output      	: None
* Return      	: Status [CCS811_ERROR, CCS811_OK]
*******************************************************************************/
CCS811_Error_et MX_CCS811_Init()
{
	uint8_t DevID, Status, ErrorCode;
    uint8_t    lodata[1];
    lodata[0]= CCS811_APP_START;

    Store_CCS811_Baseline = false;

    HAL_GPIO_WritePin(CCS811_nWAKE_GPIO_Port, CCS811_nWAKE_Pin, GPIO_PIN_RESET);
    Sleep(100);

    // SW Reset
	if (CCS811_SoftRST())
		return CCS811_ERROR;
    Sleep(100);

    // Check for device-id
	if (CCS811_Dev_Id_Get(CCS811_BADDR, &DevID))
		return CCS811_ERROR;
	if (DevID != CCS811_WHO_AM_I_VAL)
		return CCS811_ERROR;

	// Check for APP_VALID
	if (CCS811_Status_Get(CCS811_BADDR, &Status))
		return CCS811_ERROR;
	if (Status & 0x10)				//If a valid FW present
	{
		// Start the CCS811 state transition from boot to application mode
		// A write with no data is required (!!)
		if (HAL_I2C_Master_Transmit(&hi2c1, CCS811_BADDR, &lodata[0], 1, 100))
			return CCS811_ERROR;
	    Sleep(50);
	} else return CCS811_ERROR;

	// Check for FW_MODE
	if (CCS811_Status_Get(CCS811_BADDR, &Status))
		return CCS811_ERROR;
	if (Status & 0x80)				//If FW is in application mode
	{
		// Set measurement mode and conditions register
		if (CCS811_Init(CCS811_BADDR, CCS811_MEAS_MODE, &CCS811_1[0], 1))
			return CCS811_ERROR;
	    Sleep(50);
//		CCS811_ReadReg(CCS811_BADDR, CCS811_MEAS_MODE, 1, &tmp);					//Test
	} else return CCS811_ERROR;

	// Check for device-error
	if (CCS811_Status_Get(CCS811_BADDR, &Status))
		return CCS811_ERROR;
	if (Status & 0x01)				//If error check error source
	{
		CCS811_Error_Get(CCS811_BADDR, &ErrorCode);
		return CCS811_ERROR;
	}

	//Set Environment Parameters for environmental compensation
	if (CCS811_SetEnvironmentalData(relativeHumidity, temperature))
		return CCS811_ERROR;

//	HAL_Delay(i2c_delay);
//	setDriveMode(CCS811_BADDR, 1);
//	CCS811_ReadReg(CCS811_BADDR, CCS811_MEAS_MODE, 1, &tmp);					//Test

	return CCS811_OK;
}
