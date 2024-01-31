/*! ----------------------------------------------------------------------------
 * @file	MEMS_app.c
 * @brief	My MEMS Application Programs Collection.
 *          T.B.D.
 *
 * @author  Tommaso Sabatini, 2019
 */

#include "application/MEMS_app.h"

const uint8_t DeviceName[4] ="S191";
const uint8_t HW_Version[4] ="1000";
const uint8_t SW_Version[4] ="2500";
const uint32_t Vendor_ID  = 0x2316F;
const uint32_t Prdct_Code = 10000324;
const uint32_t Rev_Number = 0;
const uint32_t Ser_Number = 1;
#if (PRESSURE_SENSOR_PRESENT==1)
	#if (CALC_ALTITUDE==1)
		static const double_t H_Correction = 15.0;
	#endif
	int32_t P_Correction = 0;			//In mbar
	uint16_t MSL = 0;					//Meters above sea level
#endif
#if (HUMIDITY_SENSOR_PRESENT==1)
	int32_t RH_Correction = 0;			//In %
#endif
#if ((HUMIDITY_SENSOR_PRESENT==1) || (PRESSURE_SENSOR_PRESENT==1))
	int16_t T_Correction = 0;			//In °C
#endif
#if (VOC_SENSOR_PRESENT==1)
#if (CCS811)
	uint32_t CCS811_VOC_Ro = 0;			//In ohm
	uint32_t CCS811_VOC_Ro_Stored = 0;
	bool CCS811_Save_Baseline_Reserved = false;
#endif
	uint8_t VOC_Correction = 0;			//In units*100
#endif
#if (GAS_SENSOR_MODULE_PRESENT==1)
	int8_t CH2O_Corr = 0;				//In mVolts/10: 1 = 10mV Correction
	int8_t O3_Corr = 0;					//In mVolts/10: 1 = 10mV Correction
	int8_t NO2_Corr = 0;				//In mVolts/10: 1 = 10mV Correction
	int8_t NH3_Corr = 0;				//In mVolts/10: 1 = 10mV Correction
	int8_t CO_Corr = 0;					//In mVolts/10: 1 = 10mV Correction
	int8_t SO2_Corr = 0;				//In mVolts/10: 1 = 10mV Correction
	int8_t C6H6_Corr = 0;				//In mVolts/10: 1 = 10mV Correction
	uint32_t SMD1001_CH2O_Vo = 0;		//In mVolts
	uint32_t MiCS_6814_CO_Ro = 0;		//In ohm
	uint32_t MiCS_6814_NH3_Ro = 0;		//In ohm
	uint32_t MiCS_6814_NO2_Ro = 0;		//In ohm
	uint32_t SMD1001_CH2O_Rf = 0;		//In ohm
	uint32_t MiCS_6814_CO_Rf = 0;		//In ohm
	uint32_t MiCS_6814_NH3_Rf = 0;		//In ohm
	uint32_t MiCS_6814_NO2_Rf = 0;		//In ohm
	float32_t SMD1001_CH2O_Vs;			//In Volt
	float32_t MiCS_6814_CO_Rs;			//In ohm
	float32_t MiCS_6814_NO2_Rs;			//In ohm
	float32_t MiCS_6814_NH3_Rs;			//In ohm
#endif
#if (GUI_SUPPORT==1)
#if (IMU_PRESENT==1)
	sDISPLAY_INFO display_info_list[] = {
	{INFO_TYPE_FLOAT,1,2,VAR_TYPE_FLOAT,0,"float|Environmental|Temperature|°C|Pressure|mBar||||||||||||",0},
	{INFO_TYPE_GRAPH,2,1,VAR_TYPE_FLOAT,8,"graph|Compass|Deg|1|19|Haeading||||||1",0},
	{INFO_TYPE_GRAPH,3,3,VAR_TYPE_FLOAT,12,"graph|Linear\nAcceleration|[g]|1|19|X Axis|Y Axis|Z Axis||||1",0},
	{INFO_TYPE_GRAPH,4,3,VAR_TYPE_FLOAT,24,"graph|Cal.\nAngular Rate|[dps]|1|19|X Axis|Y Axis|Z Axis||||1",0},
	{INFO_TYPE_GRAPH,5,3,VAR_TYPE_FLOAT,36,"graph|Cal.\nAcceleration|[g]|1|19|X axis|Y axis|Z azis||||1",0},
	{INFO_TYPE_GRAPH,6,3,VAR_TYPE_FLOAT,48,"graph|Cal.\nMagn.Field|[uT]|1|19|X Axis|Y Axis|Z Axis||||1",0},
	{INFO_TYPE_SCATTER,7,3,VAR_TYPE_FLOAT,48,"scatter|Magn.Scatter\nPlot|uT|1|Mag X|Mag Y|Mag Z|1",0},
	{INFO_TYPE_GRAPH,8,3,VAR_TYPE_FLOAT,60,"graph|Rotation\nVector|Deg|1|19|Yaw (Z)|Pitch (Y)|Roll (X)||||1",0},
	{INFO_TYPE_FUSION,9,4,VAR_TYPE_FLOAT,72,"fusion|Sensor\nFusion|Teapot",0},
	{INFO_TYPE_GRAPH,10,5,VAR_TYPE_FLOAT,88,"graph|Velocity|m/s|1|19|X Axis|Y Axis|Z Axis|XYZ Axis|XY Axis||1",0},
	{INFO_TYPE_INT32,11,4,VAR_TYPE_INT32,108,"int32|Calibr.Quality\nActivity|Accelerometer Calibr. Quality||Magnetometer Calibr. Quality||Gyroscope Calibr. Status||Activity Status|||||||||",0},
	{INFO_TYPE_AINT32,12,1,VAR_TYPE_INT32,108,"aint32|Restart\nCalib.|AlgoRst|||||||",0},
	{0,0,0,0,0,0,0}};
#else	//IMU_PRESENT==0
	sDISPLAY_INFO display_info_list[] = {
	{INFO_TYPE_GRAPH,1,5,VAR_TYPE_FLOAT,0,"graph|Environmental||150|19|Temperature (°C)|Dew Point (°C)|Heat Index (°C)|RH (%)|Pressure (mBar/100)||1",0},
	{INFO_TYPE_GRAPH,2,4,VAR_TYPE_INT32,20,"graph|Air Quality 1||8000|1|TVOC (ppb)|CO (mg/m3)|NO2 (ug/m3)|NH3 (ug/m3)|||1",0},
	{INFO_TYPE_GRAPH,3,4,VAR_TYPE_INT32,36,"graph|Air Quality 2||2000|1|CH2O (ug/m3)|O3 (ug/m3)|SO2 (ug/m3)|C6H6 (ug/m3)|||1",0},
	{INFO_TYPE_GRAPH,4,4,VAR_TYPE_INT32,52,"graph|Air Pollution||200|1|PM10 (ug/m3)|PM4 (ug/m3)|PM2.5 (ug/m3)|PM1 (ug/m3)|||1",0},
	{0,0,0,0,0,0,0}};
#endif	//IMU_PRESENT==0
/**
 * @brief  Display function block update
 */
void Display_Update(void *source, sDISPLAY_INFO *header)
{
	if (header->info_type == INFO_TYPE_FFT)
	{
#if (defined (USE_NUCLEO))
		UART_SendFFT((float*)source, header->variable_count / 2, header->info_index);
#elif (defined (USE_SENSORTILE))
		VCOM_send_FFT((float*)source, header->variable_count / 2, header->info_index);
#endif
	}
	else
	{
		if (header->variable_type == VAR_TYPE_BIT)
		{
			if (header->variable_count > 8)
			{
				uint16_t tmp = 0;
				uint8_t i;
				for (i=0;i<header->variable_count;i++)
				{
					if (((uint32_t*)(source))[i] != 0)
					{
						tmp += (1 <<  i);
					}
				}
				memcpy(&dataseq[STREAM_DATA + header->stream_position], &tmp, 2);
			}
			else
			{
				uint8_t tmp = 0;
				uint8_t i;
				for (i=0;i<header->variable_count;i++)
				{
					if (((uint32_t*)source)[i] != 0)
					{
						tmp += (1 <<  i);
					}
				}
				memcpy(&dataseq[STREAM_DATA + header->stream_position], &tmp, 1);
			}
		}
		else
		{
			memcpy(&dataseq[STREAM_DATA + header->stream_position], source, header->variable_count * 4);
		}
	}
}
#endif	//GUI_SUPPORT==1

void AB_Init(void)
{
	extern FLASH_DATA_ORG FlashDataOrg;
	extern uint8_t Version[];
	static bool FirstRowFull = false;

/*	T_Out = 0xFFFF; P0_Out = 0xFFFFFFFF; Hum_Out = 0xFFFF; T2_Out = 0xFFFF; T3_Out = 0xFFFF; P_Out = 0xFFFFFFFF;
	eq_TVOC = 0xFFFF; eq_CO2 = 0xFFFF; CO = 0xFFFF; CH2O = 0xFFFF; NO2 = 0xFFFF; NH3 = 0xFFFF; O3 = 0xFFFF; SO2 = 0xFFFF; C6H6 = 0xFFFF;
	eq_TVOC_1h_Mean = 0xFFFF; eq_CO2_1h_Mean = 0xFFFF; CO_8h_Mean = 0xFFFF; CH2O_8h_Mean = 0xFFFF; NO2_1h_Mean = 0xFFFF; NH3_8h_Mean = 0xFFFF;
	O3_1h_Mean = 0xFFFF; SO2_1h_Mean = 0xFFFF; C6H6_24h_Mean = 0xFFFF;
	MC_1p0 = 0xFFFF; MC_2p5 = 0xFFFF; MC_4p0 = 0xFFFF; MC_10p0 = 0xFFFF; NC_0p5 = 0xFFFF;
	MC_1p0_24h_Mean = 0xFFFF; MC_2p5_24h_Mean = 0xFFFF; MC_4p0_24h_Mean = 0xFFFF; MC_10p0_24h_Mean = 0xFFFF;
	Gas_AQI = 0xFF; AVG_Gas_AQI = 0xFF; AVG_PMx_AQI = 0xFF; */

	memset(ReadyDevices, 0, sizeof(ReadyDevices));
	Read_Flash(NULL, 0);					//Update board data from flash
	if ((FlashDataOrg.b_status.sf == 0xFFFFFFFF) || (FlashDataOrg.b_status.sf == 0x00))
	{
/*
 * The default MiCS6814 Ro values were calculated by averaging the minimum/maximum
 * values reported on page. 2 of the data sheet
 */
		FlashDataOrg.b_status.sb = 800000;	//MiCS6814 Ro CO default value
		FlashDataOrg.b_status.sc = 755000;	//MiCS6814 Ro NH3 default value
		FlashDataOrg.b_status.sd = 10400;	//MiCS6814 Ro NO2 default value
#if (GSB_HW_VER == 10)
		FlashDataOrg.b_status.sf = 806000;	//MiCS6814 Rf CO default value
		FlashDataOrg.b_status.s10 = 80600;	//MiCS6814 Rf NH3 default value
		FlashDataOrg.b_status.s11 = 6340;	//MiCS6814 Rf NO2 default value
#elif ((GSB_HW_VER == 20) || (GSB_HW_VER == 21))
		FlashDataOrg.b_status.sf = 27000;	//MiCS6814 Rf CO default value
		FlashDataOrg.b_status.s10 = 27000;	//MiCS6814 Rf NH3 default value
		FlashDataOrg.b_status.s11 = 2700;	//MiCS6814 Rf NO2 default value
		FlashDataOrg.b_status.s14 = 2500;	//SMD1001 Vo CH2O default value (2.5V)
		FlashDataOrg.b_status.s15 = 10000;	//SMD1001 Rf CH2O default value
#endif
	}
	Version[14] = HW_Version[0];
	Version[16] = HW_Version[1];
	Version[32] = SW_Version[0];
	Version[34] = SW_Version[1];
	Version[36] = SW_Version[2];
	Version[37] = SW_Version[3];
#if (PRESSURE_SENSOR_PRESENT==1)
	/* Pressure_Init(), Temperature_Init() */
	BIT_SET(SensorStatusReg,16);	//Set pressure sensor presence in SensorStatusRegister
	NumberOfDevices++;				//Increment number of sensors mounted
#if (LPS25HB)
	PMin = LPS25HB_UPPER_P_LIMIT;	//Initialize Min Max values variables
	PMax = LPS25HB_LOWER_P_LIMIT;
#if (HUMIDITY_SENSOR_PRESENT==0)
	TMin = LPS25HB_UPPER_T_LIMIT;
	TMax = LPS25HB_LOWER_T_LIMIT;
#endif
	LPS25HB_status = MX_LPS25HB_Init();		//Initialize (disabled) Sensor
	if (LPS25HB_status == (uint8_t)LPS25HB_OK)
	{
		strcat(ReadyDevices, "LPS25HB ");
		BIT_SET(SensorStatusReg,0);	//Set pressure sensor status in SensorStatusRegister
	}
#elif (LPS22HB)
	PMin = LPS22HB_UPPER_P_LIMIT;	//Initialize Min Max values variables
	PMax = LPS22HB_LOWER_P_LIMIT;
#if (HUMIDITY_SENSOR_PRESENT==0)
	TMin = LPS22HB_UPPER_T_LIMIT;
	TMax = LPS22HB_LOWER_T_LIMIT;
#endif
	LPS22HB_status = MX_LPS22HB_Init();		//Initialize (disabled) Sensor
	if (LPS22HB_status == (uint8_t)LPS22HB_OK)
	{
		strcat(ReadyDevices, "LPS22HB ");
		BIT_SET(SensorStatusReg,0);	//Set pressure sensor status in SensorStatusRegister
	}
#endif
    HAL_TIM_Base_Start_IT(&htim3);			//Start Timer3 after LPS52HB Init
    //Set the calibration values
	T_Correction = (int16_t)((FlashDataOrg.b_status.s3) & 0x0000FFFF);	//Add Temperature calibration to sensor value
	P_Correction = (int32_t)FlashDataOrg.b_status.s4;	//Add Pressure calibration to sensor value;
	MSL = (uint16_t)((FlashDataOrg.b_status.se) & 0x0000FFFF);
	if ((MSL == 0xFFFF) || (MSL == 0x0))	//If no value has been programmed then set with the default value
		MSL = 10;
#endif

#if (HUMIDITY_SENSOR_PRESENT==1)
	/* Humidity_Init(), Temperature_Init() */
	BIT_SET(SensorStatusReg,17);	//Set humidity sensor presence in SensorStatusRegister
	NumberOfDevices++;				//Increment number of sensors mounted
#if (HTS221)
	HMin = HTS221_UPPER_H_LIMIT;	//Initialize Min Max values variables
	HMax = HTS221_LOWER_H_LIMIT;
	TMin = HTS221_UPPER_T_LIMIT;
	TMax = HTS221_LOWER_T_LIMIT;
	HTS221_status = MX_HTS221_Init();		//Initialize (disabled) Sensor
	if (HTS221_status == (uint8_t)HTS221_OK)
	{
		strcat(ReadyDevices, "HTS221 ");
		BIT_SET(SensorStatusReg,1);	//Set humidity sensor status in SensorStatusRegister
	}
#elif (SHT4x)
	HMin = SHT4x_UPPER_H_LIMIT;	//Initialize Min Max values variables
	HMax = SHT4x_LOWER_H_LIMIT;
	TMin = SHT4x_UPPER_T_LIMIT;
	TMax = SHT4x_LOWER_T_LIMIT;
	SHT4x_status = MX_SHT4x_Init();		//Initialize (disabled) Sensor
	if (SHT4x_status == (uint8_t)SHT4x_OK)
	{
		strcat(ReadyDevices, "SHT4x ");
		BIT_SET(SensorStatusReg,1);	//Set humidity sensor status in SensorStatusRegister
	}
#endif
    HAL_TIM_Base_Start_IT(&htim3);			//Start Timer3 after HTS221 Init
    //Set the calibration values
	T_Correction = (int16_t)FlashDataOrg.b_status.s3;	//Add Temperature calibration to sensor value
	RH_Correction = (int32_t)FlashDataOrg.b_status.s5;	//Add Humidity calibration to sensor value;
#endif

#if (UVx_SENSOR_PRESENT==1)
	/* UVx_Init() */
	BIT_SET(SensorStatusReg,18);	//Set UVx sensor presence in SensorStatusRegister
	NumberOfDevices++;				//Increment number of sensors mounted
#if (VEML6075)
	VEML6075_status = MX_VEML6075_Init();	//Initialize (disabled) Sensor
	if (VEML6075_status == (uint8_t)VEML6075_OK)
	{
		strcat(ReadyDevices, "VEML6075 ");
		if ((strlen(ReadyDevices) > GLCD_ROW_MAX_LENGTH) && (!FirstRowFull))
		{
			strcat(ReadyDevices, "\n\r");
			FirstRowFull = true;
		}
		BIT_SET(SensorStatusReg,2);	//Set UVx sensor status in SensorStatusRegister
	}
#elif (LTR390UV)
	LTR390UV_status = MX_LTR390UV_Init();	//Initialize (disabled) Sensor
	if (LTR390UV_status == (uint8_t)LTR390UV_OK)
	{
		strcat(ReadyDevices, "LTR390UV ");
		if ((strlen(ReadyDevices) > GLCD_ROW_MAX_LENGTH) && (!FirstRowFull))
		{
			strcat(ReadyDevices, "\n\r");
			FirstRowFull = true;
		}
		BIT_SET(SensorStatusReg,2);	//Set UVx sensor status in SensorStatusRegister
	}
#endif	// VEML6075
    HAL_TIM_Base_Start_IT(&htim3);			//Start Timer3 after VEML6075 Init
#endif	// UVx_SENSOR_PRESENT

#if (ALS_SENSOR_PRESENT==1)
	/* ALS_Init() */
	BIT_SET(SensorStatusReg,23);	//Set ALS sensor presence in SensorStatusRegister
	NumberOfDevices++;				//Increment number of sensors mounted
	VEML7700_status = MX_VEML7700_Init();	//Initialize (disabled) Sensor
	if (VEML7700_status == (uint8_t)VEML7700_OK)
	{
		strcat(ReadyDevices, "VEML7700 ");
		if ((strlen(ReadyDevices) > GLCD_ROW_MAX_LENGTH) && (!FirstRowFull))
		{
			strcat(ReadyDevices, "\n\r");
			FirstRowFull = true;
		}
		BIT_SET(SensorStatusReg,7);	//Set ALS sensor status in SensorStatusRegister
	}
    HAL_TIM_Base_Start_IT(&htim3);			//Start Timer3 after VEML7700 Init
#endif

#if (VOC_SENSOR_PRESENT==1)
	/* tVOC_Init() CO2_Init() */
	BIT_SET(SensorStatusReg,19);	//Set VOC sensor presence in SensorStatusRegister
	NumberOfDevices++;				//Increment number of sensors mounted
#if (CCS811)
    CCS811_status = MX_CCS811_Init();		//Initialize (disabled) Sensor
	//Initialize CCS811 eTVOC, eCO2 Max values variables
	eq_TVOC_1h_MeanMax = CCS811_LOWER_TVOC_LIMIT;	//Initialize Min Max values variables
	eq_CO2_1h_MeanMax = CCS811_LOWER_CO2_LIMIT;
	if (CCS811_status == (uint8_t)CCS811_OK)
	{
		strcat(ReadyDevices, "CCS811 ");
		if ((strlen(ReadyDevices) > GLCD_ROW_MAX_LENGTH) && (!FirstRowFull))
		{
			strcat(ReadyDevices, "\n\r");
			FirstRowFull = true;
		}
		BIT_SET(SensorStatusReg,3);	//Set VOC sensor status in SensorStatusRegister
	}
	CCS811_VOC_Ro_Stored = FlashDataOrg.b_status.s0;
//	VOC_Correction = (uint8_t)(((FlashDataOrg.b_status.sx) & 0x000000FF) / 100);
//	if ((VOC_Correction == 0xFF) || (VOC_Correction == 0x0))
		VOC_Correction = 1;			//If no value has been programmed then set with the default value
#elif (ENS160)
    ENS160_status = MX_ENS160_Init();		//Initialize (disabled) Sensor
	//Initialize EN160 eTVOC, eCO2 Max values variables
	eq_TVOC_1h_MeanMax = ENS160_LOWER_TVOC_LIMIT;	//Initialize Min Max values variables
	eq_CO2_1h_MeanMax = ENS160_LOWER_CO2_LIMIT;
	if (ENS160_status == (uint8_t)ENS160_OK)
	{
		strcat(ReadyDevices, "ENS160 ");
		if ((strlen(ReadyDevices) > GLCD_ROW_MAX_LENGTH) && (!FirstRowFull))
		{
			strcat(ReadyDevices, "\n\r");
			FirstRowFull = true;
		}
		BIT_SET(SensorStatusReg,3);	//Set VOC sensor status in SensorStatusRegister
	}
	VOC_Correction = 1;
#endif	// CCS811
    HAL_TIM_Base_Start_IT(&htim3);			//Start Timer3 after CCS811 Init
#endif	// VOC_SENSOR_PRESENT

#if (PARTICULATE_SENSOR_PRESENT==1)
	/* Particulate_Matter_Init() */
	BIT_SET(SensorStatusReg,20);	//Set PM sensor presence in SensorStatusRegister
	NumberOfDevices++;				//Increment number of sensors mounted
	MC_2p5_24h_MeanMax = SPS30_LOWER_PM2P5_LIMIT;	//Initialize Min Max values variables
	MC_10p0_24h_MeanMax = SPS30_LOWER_PM10_LIMIT;
    SPS30_status = MX_SPS30_Init();			//Initialize (disabled) Sensor
	if (SPS30_status == (uint8_t)SPS30_OK)
	{
		strcat(ReadyDevices, "SPS30 ");
		if ((strlen(ReadyDevices) > GLCD_ROW_MAX_LENGTH) && (!FirstRowFull))
		{
			strcat(ReadyDevices, "\n\r");
			FirstRowFull = true;
		}
		BIT_SET(SensorStatusReg,4);	//Set PM sensor status in SensorStatusRegister
	}
    HAL_TIM_Base_Start_IT(&htim3);			//Start Timer3 after SPS30 Init
#endif

#if (GAS_SENSOR_MODULE_PRESENT==1)
	/* AnlgSensorUnit_Init() */
	BIT_SET(SensorStatusReg,21);	//Set Gas Sensor Module presence in SensorStatusRegister
	NumberOfDevices++;				//Increment number of sensors mounted
	//Initialize Gases sensors Max values variables
	CO_8h_MeanMax = ANLG_LOWER_CO_LIMIT;
	CH2O_8h_MeanMax = ANLG_LOWER_CH2O_LIMIT;
	NO2_1h_MeanMax = ANLG_LOWER_NO2_LIMIT;
	NH3_8h_MeanMax = ANLG_LOWER_NH3_LIMIT;
	#if (OUTDOOR_MODE)
		O3_1h_MeanMax = ANLG_LOWER_O3_LIMIT;
		SO2_1h_MeanMax = ANLG_LOWER_SO2_LIMIT;
		C6H6_24h_MeanMax = ANLG_LOWER_C6H6_LIMIT;
	#endif
	strcat(ReadyDevices, "GAS_MODULE ");
	if ((strlen(ReadyDevices) > GLCD_ROW_MAX_LENGTH) && (!FirstRowFull))
	{
		strcat(ReadyDevices, "\n\r");
		FirstRowFull = true;
	}
	BIT_SET(SensorStatusReg,5);		//Set Gas Sensor Module status in SensorStatusRegister
	#if (OUTDOOR_MODE)
		BIT_SET(SensorStatusReg,22);	//Set Gas Sensor Full Equipped Module presence in SensorStatusRegister
		BIT_SET(SensorStatusReg,6);		//Set Gas Sensor Full Equipped Module status in SensorStatusRegister
	#endif
    HAL_TIM_Base_Start_IT(&htim3);		//Start Timer3 after Analog Module Init
    //Set the calibration values
    CO_Corr = (int8_t)(FlashDataOrg.b_status.sa & 0x000000FF);
    CH2O_Corr = (int8_t)(FlashDataOrg.b_status.s9 & 0x000000FF);
	NO2_Corr = (int8_t)((FlashDataOrg.b_status.s9 >> 16) & 0x000000FF);
	NH3_Corr = (int8_t)((FlashDataOrg.b_status.s9 >> 24) & 0x000000FF);
	#if (OUTDOOR_MODE)
		O3_Corr = (int8_t)((FlashDataOrg.b_status.s9 >> 8) & 0x000000FF);
		SO2_Corr = (int8_t)((FlashDataOrg.b_status.sa >> 8) & 0x000000FF);
		C6H6_Corr = (int8_t)((FlashDataOrg.b_status.sa >> 16) & 0x000000FF);
	#endif //OUTDOOR_MODE
	MiCS_6814_CO_Ro = FlashDataOrg.b_status.sb;
	MiCS_6814_NH3_Ro = FlashDataOrg.b_status.sc;
	MiCS_6814_NO2_Ro = FlashDataOrg.b_status.sd;
	MiCS_6814_CO_Rf = FlashDataOrg.b_status.sf;
	MiCS_6814_NH3_Rf = FlashDataOrg.b_status.s10;
	MiCS_6814_NO2_Rf = FlashDataOrg.b_status.s11;
	#if ((GSB_HW_VER == 20) || (GSB_HW_VER == 21))
		SMD1001_CH2O_Vo = FlashDataOrg.b_status.s14;
		SMD1001_CH2O_Rf = FlashDataOrg.b_status.s15;
	#endif
#endif	//GAS_SENSOR_MODULE_PRESENT
//Define the number of pages to be displayed
//Page1: Enviromental Page values.
//Page2: Air Quality Page values
//Page3: Air Pollution Page values
//HR, ALS and UVx values are displayed in the same page of pressure value (Environmental Page);
//CH2O, O3, CO, NO2, NH3, SO2, C6H6 values are displayed in the same page VOC value (Air Quality Page)
//so the number of pages to be displayed does not take into account
//the presence of the humidity, UVx and Analog Module sensors
	NumberOfPages = ((HUMIDITY_SENSOR_PRESENT==1) ? (NumberOfDevices-1) : NumberOfDevices);
	NumberOfPages = ((ALS_SENSOR_PRESENT==1) ? (NumberOfPages-1) : NumberOfPages);
	NumberOfPages = ((UVx_SENSOR_PRESENT==1) ? (NumberOfPages-1) : NumberOfPages);
	NumberOfPages = ((GAS_SENSOR_MODULE_PRESENT==1) ? (NumberOfPages-1) : NumberOfPages);

#if (IMU_PRESENT==1)
	MagCalStatus = 0;
	MagCalRequest = 0;
	CalibrationMode = DYNAMIC_CALIBRATION;
	AC_TimeStamp = 0;
	avg_cnt = 0;

	/* Gyro_Init(), Aceelero_Init(), Mag_Init() */
	BIT_SET(SensorStatusReg,24);	//Set IMU presence in SensorStatusRegister
//	NumberOfDevices++;				//Increment number of sensors mounted
	LSM9DS1_status = MX_LSM9DS1_Init();		//Initialize (disabled) Sensors
	if (LSM9DS1_status == (uint8_t)LSM9DS1_OK)
	{
		strcat(ReadyDevices, "LSM9DS1 ");
		if ((strlen(ReadyDevices) > GLCD_ROW_MAX_LENGTH) && (!FirstRowFull))
		{
			strcat(ReadyDevices, "\n\r");
			FirstRowFull = true;
		}
		BIT_SET(SensorStatusReg,8);	//Set IMU status in SensorStatusRegister
	}
	/* MotionFX_Init() */
	Sleep(100);	//Required starting from the STM32Cube_FW_F4_V1.26.0. I don't know why!!
    MotionFX_manager_init();				//Sensor Fusion API initialization function
    MotionFX_manager_get_version(FX_lib_version, &FX_lib_version_len);	//Get FX library version

    /* MotionGC_Init(0.008, 0.15, 0.002, 1, 1.3, 15) */
    MotionGC_manager_init((float)GC_ALGO_FREQ);	//Gyroscope Calibr. API initialization function
    MotionGC_manager_get_version(GC_lib_version, &GC_lib_version_len);	//Get GC library version

    /* MotionAR_Init() */
    MotionAR_manager_init();				//Activity Recognition API initialization function
    MotionAR_manager_get_version(AR_lib_version, &AR_lib_version_len);	//Get AR library version

    /* MotionAC_Init(1, 0.2) */
    MotionAC_manager_init(MAC_ENABLE_LIB);	//Accelerometer Calibr. API initialization function
    MotionAC_manager_get_version(AC_lib_version, &AC_lib_version_len);	//Get AC library version

#if (GUI_SUPPORT==0)
    HAL_TIM_Base_Start_IT(&htim1);			//Start Timer1 after LSM9DS1 Init
#endif
    /* Initialize magnetometer calibration */
    MotionFX_manager_MagCal_start(MAG_CAL_ALGO_PERIOD);	//Enable mag.Calibr. MAG_CAL_ALGO_PERIOD defined in tim.h
    /* Test if calibration data are available */
    MotionFX_MagCal_getParams(&mag_cal_test);
   	/* If calibration data are available load HI coefficients */
   	if (mag_cal_test.cal_quality == MFX_MAGCALGOOD)
    {
    	ans_float = (mag_cal_test.hi_bias[0] * uT50_to_mGauss);
    	MagOffset.AXIS_X = (int32_t)ans_float;
    	ans_float = (mag_cal_test.hi_bias[1] * uT50_to_mGauss);
    	MagOffset.AXIS_Y = (int32_t)ans_float;
    	ans_float = (mag_cal_test.hi_bias[2] * uT50_to_mGauss);
    	MagOffset.AXIS_Z = (int32_t)ans_float;
    	MagCalStatus = 1;
    }
//	Sensor_Hub_Init(0, 100, 1, 1);
//	MotionFX_Init();
//	MotionGC_Init(0.008, 0.15, 0.002, 1, 1.3, 15);
//	MotionAC_Init(1, 0.2);
//	MotionMC_Init();
//	MotionAW_Init();
//	Temperature_Init();
//	Pressure_Init();
//	Constant_Int_2_out[0] = 1;		Activity Status
//	Constant_Float_2_out[0] = 20;
//	Constant_Float_3_out[0] = 21;
//	Constant_Float_4_out[0] = 22;
//	Constant_Float_5_out[0] = 23;
//	Constant_Float_6_out[0] = 24;
	Input_Value_Int_1_out[0] = 0;
//	Input_Value_Init(Input_Value_Int_1_out, &display_info_list[11]);
//	Message_Length = 124;
#else	//IMU_PRESENT==0
//	Sensor_Hub_Init(0, 100, 1, 1);
//	Temperature_Init();
//	Pressure_Init();
//	Humidity_Init();
	Constant_Float_1_out[0] = 100;
//	Constant_Int_1_out[0] = 243;	eTVOC
//	Constant_Int_2_out[0] = 3;		CO
//	Constant_Int_4_out[0] = 801;	NO2
//	Constant_Int_3_out[0] = 101;	NH3
//	Constant_Int_5_out[0] = 55;		CH2O
//	Constant_Int_8_out[0] = 51;		O3
//	Constant_Int_7_out[0] = 2;		SO2
//	Constant_Int_6_out[0] = 110;	C6H6
//	Constant_Int_9_out[0] = 10;		PM10
//	Constant_Int_12_out[0] = 1;		PM4.0
//	Constant_Int_11_out[0] = 4;		PM2.5
//	Constant_Int_10_out[0] = 2;		PM1.0
//	Message_Length = 68;
#endif
}

#if (GUI_SUPPORT==1)
#if (IMU_PRESENT==1)
void Mux_4_int(int32_t *in1, int32_t *in2, int32_t *in3, int32_t *in4, int32_t *out)
{
	out[0] = *in1;
	out[1] = *in2;
	out[2] = *in3;
	out[3] = *in4;
}

void Mux_2_float(float *in1, float *in2, float *out)
{
	out[0] = *in1;
	out[1] = *in2;
}

void AB_Handler(void)
{
	uint16_t Len = 132;
	uint16_t CountOut;

//	Sensor_Hub_Handler(&Sensor_Hub_2_out);
//	LinearAcceleration9X_GetData(Sensor_Hub_2_out, Linear_Acceleration_9X_2_data);
//	Heading9X_GetData(Sensor_Hub_2_out, Heading_9X_2_data);
//	Rotation9X_GetData(Sensor_Hub_2_out, Rotation_Vector_9X_2_data);
//	Quaternions9X_GetData(Sensor_Hub_2_out, Quaternions_9X_2_data);
//	GyroCal_GetData(Sensor_Hub_2_out, Input_Value_Int_1_out, Calibrated_Angular_Rate_dps_2_data);
//	AccCal_GetData(Sensor_Hub_2_out, Input_Value_Int_1_out, Calibrated_Acceleration_g_2_data, Calibrated_Acceleration_g_2_quality);
//	ActivityWrist_GetData(Sensor_Hub_2_out, Input_Value_Int_1_out, Activity_Index_2_data);
//	MagCal_GetData(Sensor_Hub_2_out, Input_Value_Int_1_out, Calibrated_Magnetic_Field_uT_2_data, Calibrated_Magnetic_Field_uT_2_quality);
//	Temperature_Sensor_GetData(Sensor_Hub_2_out, Temperature_C_2_data);
//	Pressure_Sensor_GetData(Sensor_Hub_2_out, Pressure_hPa_2_data);
	Mux_4_int(Calibrated_Acceleration_g_1_quality, Calibrated_Magnetic_Field_uT_1_quality, Calibrated_Gyroscope_g_1_status, Activity_Index_1_data, Mux_Int_1_out);
	Mux_2_float(Temperature_C_1_data, Pressure_hPa_1_data, Mux_Float_1_out);
	Display_Update(Mux_Float_1_out, &display_info_list[0]);
	Display_Update(Heading_9X_1_data, &display_info_list[1]);
	Display_Update(Linear_Acceleration_9X_1_data, &display_info_list[2]);
	Display_Update(data_in.gyro, &display_info_list[3]);
	Display_Update(data_in.acc, &display_info_list[4]);
	Display_Update(data_in.mag, &display_info_list[5]);
	Display_Update(data_in.mag, &display_info_list[6]);
	Display_Update(Rotation_Vector_9X_1_data, &display_info_list[7]);
	Display_Update(Quaternions_9X_1_data, &display_info_list[8]);
	Display_Update(Linear_Speed_9X_1_data, &display_info_list[9]);
	Display_Update(Mux_Int_1_out, &display_info_list[10]);
	Input_Value_Init(Input_Value_Int_1_out, &display_info_list[11]);

   	if (send_lcl_imu_data)	//Send IMU data to IMU_GUI only when sensors are enabled (Button Start in UnicleoGUI pressed)
   	{
   		Init_Streaming_Header(&dataseq[0]);

   		CHK_ComputeAndAdd(&dataseq[0], &Len, 0);
   		CountOut = ByteStuffCopy(&dataseq[0], &dataseq[0], &Len);
   		Message_Length = CountOut;
   	}
}
#else	//IMU_PRESENT==0
void Mux_5_float(float *in1, float *in2, float *in3, float *in4, float *in5, float *out)
{
	out[0] = *in1;
	out[1] = *in2;
	out[2] = *in3;
	out[3] = *in4;
	out[4] = *in5;
}

void Mux_4_int(int32_t *in1, int32_t *in2, int32_t *in3, int32_t *in4, int32_t *out)
{
	out[0] = *in1;
	out[1] = *in2;
	out[2] = *in3;
	out[3] = *in4;
}

void AB_Handler(void)
{
	uint16_t Len = 80;
	uint16_t CountOut;

//	Sensor_Hub_Handler(&Sensor_Hub_1_out);
//	Temperature_Sensor_GetData(Sensor_Hub_1_out, Temperature_C_1_data);
//	Temperature_Sensor_GetData(Sensor_Hub_1_out, Temperature_C_2_data);
//	Temperature_Sensor_GetData(Sensor_Hub_1_out, Temperature_C_3_data);
//	Pressure_Sensor_GetData(Sensor_Hub_1_out, Pressure_hPa_1_data);
//	Humidity_Sensor_GetData(Sensor_Hub_1_out, Humidity_percent_1_data);
	Divide_1_out[0] = Pressure_hPa_1_data[0] / Constant_Float_1_out[0];
	Mux_5_float(Temperature_C_1_data, Altitude_C_1_data, Temperature_C_2_data, Humidity_percent_1_data, Divide_1_out, Mux_Float_1_out);
	Mux_4_int(eTVOC_1_data, CO_data, NO2_data, NH3_data, Mux_Int_1_out);
	Mux_4_int(CH2O_data, O3_data, SO2_data, C6H6_data, Mux_Int_2_out);
	Mux_4_int(PM10_1_data, PM4p0_1_data, PM2p5_1_data, PM1p0_1_data, Mux_Int_3_out);
	Display_Update(Mux_Float_1_out, &display_info_list[0]);
	Display_Update(Mux_Int_1_out, &display_info_list[1]);
	Display_Update(Mux_Int_2_out, &display_info_list[2]);
	Display_Update(Mux_Int_3_out, &display_info_list[3]);

   	if (send_lcl_voc_data || send_lcl_uvx_data || send_lcl_als_data || send_lcl_hum_data || send_lcl_prs_data || send_lcl_pms_data)
   	{	//Send data to IMU_GUI only when sensors are enabled (Button Start in UnicleoGUI pressed)
   		Init_Streaming_Header(&dataseq[0]);

   		CHK_ComputeAndAdd(&dataseq[0], &Len, 0);
   		CountOut = ByteStuffCopy(&dataseq[0], &dataseq[0], &Len);
   		Message_Length = CountOut;
   	}
}
#endif	//IMU_PRESENT==0

int SC_Get_Config_String(uint8_t* Buff, uint8_t id)
{
	int i = 0;
	int j = 0;
	uint16_t Len;
	uint16_t CountOut;

	while (display_info_list[i].info_index != 0)
	{
		if (display_info_list[i].info_index == id)
		{
			for (j = 0; j < strlen(display_info_list[i].config_string); j++)
			{
				Buff[5 + j] = display_info_list[i].config_string[j];
			}
		}
		i++;
	}
	Len = 5 + j;
	Build_Reply_Header(Buff);
 	CHK_ComputeAndAdd(&Buff[0], &Len, 0);
	CountOut = ByteStuffCopy(&Buff[0], &Buff[0], &Len);
	memcpy(&dataseq[0], &Buff[0], CountOut);
	Message_Length = CountOut;

  return 0;
}

int SC_Get_Custom_Config(uint8_t* Buff)
{
	int i = 0;
	int j = 0;
	int conn_index;
	int index;
	uint16_t Len;
	uint16_t CountOut;

	sCONFIG_RECORD *cr;

	Buff[5] = 0; //number of records
	index = 1;
	i = 0;

	while (display_info_list[i].info_index != 0)
	{
		display_info_list[i].already_processed = 0;
		i++;
	}
	i = 0;
	while (display_info_list[i].info_index != 0)
	{
		if (display_info_list[i].info_type > INFO_TYPE_AFLOAT)	//output configurations only
		{
			if (display_info_list[i].already_processed == 0)
			{
				cr = (sCONFIG_RECORD*)&(Buff[5 + index]);
				display_info_list[i].already_processed = 1; 	//mark as already processed
				cr->var_count =  display_info_list[i].variable_count;
				cr->var_type = display_info_list[i].variable_type;
				conn_index = 0;
				cr->conn[conn_index++] = display_info_list[i].info_index;
				cr->conn_size = 1;
				Buff[5]++; 										//number of records
				if (display_info_list[i].info_type != INFO_TYPE_FFT)
				{
					j=0;
					while (display_info_list[j].info_index != 0)
					{
						if (display_info_list[j].info_type > INFO_TYPE_AFLOAT)		//output configurations only
						{
							if (display_info_list[j].already_processed == 0)
							{
								if (display_info_list[j].stream_position == display_info_list[i].stream_position &&
								display_info_list[j].info_type != INFO_TYPE_FFT)	//add other outputs with the same stream address
								{
									cr->conn[conn_index++] = display_info_list[j].info_index;
									cr->conn_size++;
									display_info_list[j].already_processed = 1;		//mark as already processed
								}
							}
						}
						j++;
					}
				}
				index += 3 + conn_index;  //add connections count
			}
		}
		i++;
	}
	//input configurations
	i = 0;
	while (display_info_list[i].info_index != 0)
	{
		if (display_info_list[i].info_type <= INFO_TYPE_AFLOAT) //input configurations only
		{
			if (display_info_list[i].already_processed == 0)
			{
				cr = (sCONFIG_RECORD*)&(Buff[5 + index]);
				display_info_list[i].already_processed = 1;  	//mark as already processed
				cr->var_count =  display_info_list[i].variable_count;
				cr->var_type = display_info_list[i].variable_type;
				cr->var_type += VAR_TYPE_INPUT;
				cr->conn[0] = display_info_list[i].info_index;
				cr->conn_size = 1;
				Buff[5]++; //number of records
				index += 4;
			}
		}
		i++;
	}
	Len = 5 + index;
	Build_Reply_Header(Buff);
	CHK_ComputeAndAdd(&Buff[0], &Len, 0);
	CountOut = ByteStuffCopy(&Buff[0], &Buff[0], &Len);
	memcpy(&dataseq[0], &Buff[0], CountOut);
	Message_Length = CountOut;

  return 0;
}

/**
  * @brief  Handle Sensors command
  * @param  Msg the pointer to the message to be handled
  * @param  custom_values the pointer to the custom values
  * @retval 1 if the message is correctly handled, 0 otherwise
  */
int Handle_Sensor_command(uint8_t* Buff)
{
	/* Commands */
	switch (Buff[3])
	{
		case SC_GET_CONFIG_STRING:
			return SC_Get_Config_String(Buff, Buff[4]);

		case SC_GET_CUSTOM_CONFIG:
			return SC_Get_Custom_Config(Buff);

		case SC_SET_CUSTOM_VALUES:
//			return SC_Set_Custom_Values(Buff);

		case SC_GET_CUSTOM_VALUES:
//			return SC_Get_Custom_Values(Buff);

		default:
			return 0;
	}
}
#endif	//GUI_SUPPORT==1

#if ((IMU_PRESENT==1) && (GUI_SUPPORT==0))
void AC_Handler(int* Len, LSM9DS1_MeasureTypeDef_st *IMU_Axes, uint8_t* Buff)
{
	double_t XAcceleration, YAcceleration, ZAcceleration;
	double_t LinXAcceleration, LinYAcceleration, LinZAcceleration;
	double_t XVelocity, YVelocity, ZVelocity;
	double_t XAngularRate, YAngularRate, ZAngularRate;
	double_t XMagneticField, YMagneticField, ZMagneticField;
	extern const double_t AccelConvFactor;

	//Convert IMU acceleration from mg to m/s^2
	XAcceleration = (double_t)IMU_Axes->Acc_Out.AXIS_X * AccelConvFactor;
	YAcceleration = (double_t)IMU_Axes->Acc_Out.AXIS_Y * AccelConvFactor;
	ZAcceleration = (double_t)IMU_Axes->Acc_Out.AXIS_Z * AccelConvFactor;
	//Convert IMU angular rate from md/s to d/s
	XAngularRate = (double_t)IMU_Axes->Gy_Out.AXIS_X * mdps_to_dps;
	YAngularRate = (double_t)IMU_Axes->Gy_Out.AXIS_Y * mdps_to_dps;
	ZAngularRate = (double_t)IMU_Axes->Gy_Out.AXIS_Z * mdps_to_dps;
	//convert IMU magnetic field from mGauss to uTesla
	XMagneticField = (double_t)IMU_Axes->Mag_Out.AXIS_X * mGauss_to_uT;
	YMagneticField = (double_t)IMU_Axes->Mag_Out.AXIS_Y * mGauss_to_uT;
	ZMagneticField = (double_t)IMU_Axes->Mag_Out.AXIS_Z * mGauss_to_uT;
	//Convert linear acceleration from g to m/s^2
	LinXAcceleration = (double_t)Linear_Acceleration_9X_1_data[0] * g_to_ms2;
	LinYAcceleration = (double_t)Linear_Acceleration_9X_1_data[1] * g_to_ms2;
	LinZAcceleration = (double_t)Linear_Acceleration_9X_1_data[2] * g_to_ms2;

	XVelocity = (double_t)Linear_Speed_9X_1_data[0];
	YVelocity = (double_t)Linear_Speed_9X_1_data[1];
	ZVelocity = (double_t)Linear_Speed_9X_1_data[2];

    if (send_lcl_imu_data)
    {
    	send_lcl_imu_data = false;
    	*Len += sprintf((char*)&Buff[*Len], "AccelX: %5.1f m/s%c - ", XAcceleration, 0xB2);
    	*Len += sprintf((char*)&Buff[*Len], "AccelY: %5.1f m/s%c - ", YAcceleration, 0xB2);
    	*Len += sprintf((char*)&Buff[*Len], "AccelZ: %5.1f m/s%c\r\n", ZAcceleration, 0xB2);

    	*Len += sprintf((char*)&Buff[*Len], "LAcceX: %5.1f m/s%c - ", LinXAcceleration, 0xB2);
    	*Len += sprintf((char*)&Buff[*Len], "LAcceY: %5.1f m/s%c - ", LinYAcceleration, 0xB2);
    	*Len += sprintf((char*)&Buff[*Len], "LAcceZ: %5.1f m/s%c\r\n", LinZAcceleration, 0xB2);

    	*Len += sprintf((char*)&Buff[*Len], "VelocX: %5.1f m/s  - ", XVelocity);
    	*Len += sprintf((char*)&Buff[*Len], "VelocY: %5.1f m/s  - ", YVelocity);
    	*Len += sprintf((char*)&Buff[*Len], "VelocZ: %5.1f m/s\r\n", ZVelocity);

    	*Len += sprintf((char*)&Buff[*Len], "Gyro.X: %5.1f %c/s  - ", XAngularRate, 0xB0);
    	*Len += sprintf((char*)&Buff[*Len], "Gyro.Y: %5.1f %c/s  - ", YAngularRate, 0xB0);
    	*Len += sprintf((char*)&Buff[*Len], "Gyro.Z: %5.1f %c/s\r\n", ZAngularRate, 0xB0);

    	*Len += sprintf((char*)&Buff[*Len], "Magn.X: %5.1f %cT   - ", XMagneticField, 0xB5);
    	*Len += sprintf((char*)&Buff[*Len], "Magn.Y: %5.1f %cT   - ", YMagneticField, 0xB5);
    	*Len += sprintf((char*)&Buff[*Len], "Magn.Z: %5.1f %cT\r\n\n", ZMagneticField, 0xB5);
    }
}
#endif	//(IMU_PRESENT==1) && (GUI_SUPPORT==0)

/**
 * @brief  Inputs function block update
 */
void Input_Value_Init(void *source, sDISPLAY_INFO *header)
{
  header->p_node = source;
}

#if (PRESSURE_SENSOR_PRESENT==1)
/**
 * @brief  Handles the LPS25HB PRESS, TEMP sensor data getting/sending.
 * @brief  Build the Buff array from the float (LSB first)
 * @brief  Fill the PRESS, TEMP parts of the Buff stream
 * @param  PressTemp: Pointer to PressTemp data structure
 * @param  Buff: Stream pointer
 * @retval None
 */
#if (LPS25HB)
void Pressure_Sensor_Handler(LPS25HB_MeasureTypeDef_st *PressTemp, uint8_t* Buff)
#elif (LPS22HB)
void Pressure_Sensor_Handler(LPS22HB_MeasureTypeDef_st *PressTemp, uint8_t* Buff)
#endif
{
//	const double_t R = 287.05;				//R is the general gas constant
//	const double_t g = 9.80665;				//g is the acceleration of gravity
#if (CALC_ALTITUDE==1)
	static const double_t K = 273.25;		//K is the conversion value °C to °K
	static const double_t P_Out0 = 1013.25;	//P_Out0 is pressure at sea level (in hPA)
	static const double_t R_su_g = 29.270953894;	//Division value of R/g
#endif

#if (HUMIDITY_SENSOR_PRESENT==0)
	T_Out = PressTemp->Tout + T_Correction;	//Use HTS221 if present to take T_Out
	TMin = (T_Out < TMin) ? T_Out : TMin;
	TMax = (T_Out > TMax) ? T_Out : TMax;
	Temperature = T_Out/10.0;				//only if the humidity sensor is not present
	temp_value = (float32_t)(Temperature);
#endif

	P_Out = PressTemp->Pout + P_Correction;
	P_OutDbl = (double_t)(P_Out/100.0);
	P0_OutDbl = P0((float32_t)P_OutDbl, (float32_t)MSL, (float32_t)Temperature);
	P0_Out = (int32_t)(lrintf((float32_t)(P0_OutDbl * 100.0)));
	PMin = (P0_Out < PMin) ? P0_Out : PMin;
	PMax = (P0_Out > PMax) ? P0_Out : PMax;
	Pressure = P0_OutDbl;
	press_value = (float32_t)(P_OutDbl);

#if (CALC_ALTITUDE==1)
	//Convert Pascal in meters. From AN4450: Hardware and software guidelines for use of LPS25H pressure. 1mB = 8.7m
	//Altitude = ((1-pow((P_OutDbl)/P_Out0,0.190284))*145366.45/3.280839895) + H_Correction;
	Altitude = (R_su_g)*(Temperature+K)*(log(P_Out0/(P_OutDbl))) + H_Correction;
#endif
#if (GUI_SUPPORT==1)
	#if (HUMIDITY_SENSOR_PRESENT==0)
		Temperature_C_1_data[0] = (float)Temperature;	//For UnicleoGUI
	#endif
	Pressure_hPa_1_data[0] = (float)Pressure;			//For UnicleoGUI
	#if (CALC_ALTITUDE==1)
		Altitude_C_1_data[0] = (float)Altitude;			//For UnicleoGUI
	#endif
#endif
#if (LPS25HB)
	(void)memcpy((void *)&Buff[7], PressTemp, sizeof(LPS25HB_MeasureTypeDef_st));
#elif (LPS22HB)
	(void)memcpy((void *)&Buff[7], PressTemp, sizeof(LPS22HB_MeasureTypeDef_st));
#endif
	/* sizeof(LPS25HB_MeasureTypeDef_st) == 8 */
}
#endif

#if (HUMIDITY_SENSOR_PRESENT==1)
/**
 * @brief  Handles the HTS221/SHT4x HUM, TEMP sensor data getting/sending.
 * @brief  Build the Buff array from the float (LSB first)
 * @brief  Fill the HUM parts of the Buff stream
 * @param  HumTemp: Pointer to HumTemp data structure
 * @param  Buff: Stream pointer
 * @retval None
 */
#if (HTS221)
void Humidity_Sensor_Handler(HTS221_MeasureTypeDef_st *HumTemp, uint8_t* Buff)
#elif (SHT4x)
void Humidity_Sensor_Handler(SHT4x_MeasureTypeDef_st *HumTemp, uint8_t* Buff)
#endif
{
#if (CALC_DEWPOINT==1)
	static const double_t c1 = 0.00135;
	static const double_t c2 = 0.35;
	static const double_t c3 = 84.0;
	static const double_t K = 273.25;	//K is the conversion value °C to °K
	static double_t val1, val2, val3;
#endif
	static uint16_t p_T_Out = 0; static double p_Temp = 0; static float32_t p_TempD = 0;
	static uint8_t p_Hum = 0; 	static uint16_t p_Hum_Out = 0;
	static float32_t p_temp_value = 0; static float32_t p_hum_value = 0;
	static float32_t TemperatureP = 0;

	//When the RH*10 reaches 800 (80%) then it applies the correction in proportion
	//to the 1000 - RH*10 difference. This prevents RH% from exceeding 100%
	if (HumTemp->Hout > 790)
		RH_Correction = (int32_t)(lrintf(((float32_t)((1000 - HumTemp->Hout) * ((RH_Correction)/200)))));
	T_Out = HumTemp->Tout + T_Correction;	//Use HTS221 if present. T_Out is used by BLE in app_bluenrg_2.c User_Process() function
	temp_value = (float)T_Out;

	Hum_Out = HumTemp->Hout + (uint16_t)RH_Correction;
	hum_value = (float)Hum_Out;
	temp_value = temp_value/10.0;
	hum_value = hum_value/10.0;

	//Temperature and humidity readings cannot be taken when the heater is activated
	if ((!ServiceTimer2.Start) && (!ServiceTimer4.Start))
	{
		p_Temp = Temperature = (double_t)temp_value;	//Use HTS221 temperature if present
		p_T_Out = T_Out;
		p_temp_value = temp_value;
		TMin = (T_Out < TMin) ? T_Out : TMin;
		TMax = (T_Out > TMax) ? T_Out : TMax;
		p_Hum = Humidity = (uint8_t)(lrintf(hum_value));
		p_Hum_Out = Hum_Out;
		p_hum_value = hum_value;
		HMin = (Hum_Out < HMin) ? Hum_Out : HMin;
		HMax = (Hum_Out > HMax) ? Hum_Out : HMax;
#if (CALC_DEWPOINT==1)
		val1 = (100-hum_value)/5;
		val2 = pow(((K+Temperature)/300), 2);
		val3 = c1*pow((hum_value-c3), 2);
		p_TempD = TemperatureD = (float)(Temperature - (val1*val2) - val3 + c2);
		T2_Out = (uint16_t)(lrintf((float)(p_TempD*10)));	//Use DewPoint as T2_Out
#endif
	} else	//Use last valid values
	{
		T_Out = p_T_Out;
		Temperature = p_Temp;
		temp_value = p_temp_value;
		Hum_Out = p_Hum_Out;
		Humidity = p_Hum;
		hum_value = p_hum_value;
#if (CALC_DEWPOINT==1)
		TemperatureD = p_TempD;
#endif
	}
	//Calculate the perceived temperature using the Heat Index (HI)
	if (temp_value > 26)	//The HI is valid only for temperatures above 26 °C
		TemperatureP = F2C(HI(C2F(temp_value), hum_value));
	else
		TemperatureP = temp_value;
	HI = TemperatureP;
	T3_Out = (uint16_t)(lrintf((float)(HI*10)));	//Use HI as T3_Out
	//Calculate the perceived temperature using the Summer Simmer Index (SSI)
	if (temp_value > 22)	//The SSI is valid only for temperatures above 22 °C
		TemperatureP = F2C(SSI(C2F(temp_value), hum_value));
	else
		TemperatureP = temp_value;
	SI = TemperatureP;
//	T2_Out = (uint16_t)(lrintf((float)(SI*10)));	//Use HI as T2_Out
	//Activates the heater if RH is greater than 99% and has not already been activated
	//and if more than one hour has passed since the last activation
	if ((Humidity > 99) && (!ServiceTimer3.Start))
	{
	#if (HTS221)
		HTS221_status = HTS221_Set_HeaterState(HTS221_BADDR, HTS221_ENABLE);
	#elif (SHT4x)
		// activate heater with xx mW for xx s including a high precision measurement just before deactivation
	#endif
		ServiceTimerStart(STim_2);
		ServiceTimerStart(STim_3);
		ServiceTimerStart(STim_4);
	}
	Weather_Forecast((float)Pressure, (float)Temperature, Humidity);
#if (GUI_SUPPORT==1)
	Temperature_C_1_data[0] = temp_value;	//For UnicleoGUI
	Temperature_C_2_data[0] = HI;			//For UnicleoGUI
	Humidity_percent_1_data[0] = hum_value;	//For UnicleoGUI
	#if (CALC_DEWPOINT==1)
		Altitude_C_1_data[0] = TemperatureD;//For UnicleoGUI
	#endif
#endif
#if (HTS221)
	(void)memcpy((void *)&Buff[15], HumTemp, sizeof(HTS221_MeasureTypeDef_st));
#elif (SHT4x)
	(void)memcpy((void *)&Buff[15], HumTemp, sizeof(SHT4x_MeasureTypeDef_st));
#endif
	/* sizeof(HTS221_MeasureTypeDef_st) == 4 */
}
#endif

#if (UVx_SENSOR_PRESENT==1)
/**
 * @brief  Handles the VEML6075 UVA UVB, UV Index sensor data getting/sending.
 * @brief  Build the Buff array from the float (LSB first)
 * @brief  Fill the UV lights parts of the Buff stream
 * @param  UVx: Pointer to UV data structure
 * @param  Buff: Stream pointer
 * @retval None
 */
#if (VEML6075)
void UVx_Sensor_Handler(VEML6075_MeasureTypeDef_st *UVx, uint8_t* Buff)
{
	UVa = (float32_t)(UVx->uva);
	UVb = (float32_t)(UVx->uvb);
#elif (LTR390UV)
void UVx_Sensor_Handler(LTR390UV_MeasureTypeDef_st *UVx, uint8_t* Buff)
{
	Lux = UVx->Lux;
#endif
	UV_Index = UVx->UVI;
#if (GUI_SUPPORT==1)
	UVA_1_data[0] = UVa;
	UVB_1_data[0] = UVb;
#endif
#if (VEML6075)
	(void)memcpy((void *)&Buff[19], UVx, sizeof(VEML6075_MeasureTypeDef_st));
#elif (LTR390UV)
	(void)memcpy((void *)&Buff[19], UVx, 12);
#endif
	/* sizeof(VEML6075_MeasureTypeDef_st) == 12 */
}
#endif

#if (VOC_SENSOR_PRESENT==1)
/**
 * @brief  Handles the CCS811 sensor data getting/sending.
 * @brief  Build the Buff array from the float (LSB first)
 * @brief  Calculate the hourly/daily average of the concentration values
 * @brief  Fill the eTVOC, eCO2 parts of the Buff stream
 * @param  voc: Pointer to eVOC_eCO2 data structure
 * @param  Buff: Stream pointer
 * @retval None
 */
#if (CCS811)
void VOC_Sensor_Handler(CCS811_MeasureTypeDef_st *voc, uint8_t* Buff)
#elif (ENS160)
void VOC_Sensor_Handler(ENS160_MeasureTypeDef_st *voc, uint8_t* Buff)
#endif
{
	static const uint32_t AverageWindow_1h = 720;	//3600/5: Number of readings in one hour
	static const uint32_t AverageWindow_5m = 60;	//300/5: Number of readings in 5 minutes
	static const uint32_t AverageWindow_1m = 12;	//60/5: Number of readings in 1 minutes
//	static const uint32_t AverageWindow_8h = 5760;	//3600*8/5: Number of readings in 8 hour
	static float32_t eTVOC_avg = 0; static float32_t eCO2_avg = 0;
	static float32_t eTVOC_avg5m = 0; static float32_t eCO2_avg5m = 0;
	static float32_t eTVOC_avg1m = 0; static float32_t eCO2_avg1m = 0;
#if (USE_BKUP_SRAM)
	static bool VOC_mean_init = true;
#endif
//	uint8_t save_status;

/* A preliminary moving average is performed to prevent very high transient eTVOC or eCO values
 * from distorting the long-term air quality calculation.
 * The moving average filter has two time constants: the first (used when the current value is
 * greater than the average value calculated up to then) is given by the value of the
 * "AverageWindow_5m" constant. The second (used when the present value is less than or equal
 * to the average value calculated up to then) is given by the value of the "AverageWindow_1m"constant.
 */
	eq_TVOC = voc->eTVOC * VOC_Correction;
	eTVOC_avg5m = approxMovingAverage(eTVOC_avg5m, (float32_t)eq_TVOC, AverageWindow_5m);
	eTVOC_avg1m = approxMovingAverage(eTVOC_avg1m, (float32_t)eq_TVOC, AverageWindow_1m);
	if (eTVOC_avg1m > eTVOC_avg5m)
		eq_TVOC = eTVOC_avg5m;
	else
		eq_TVOC = eTVOC_avg1m;

	eq_CO2 = voc->eCO2 * VOC_Correction;
	eCO2_avg5m = approxMovingAverage(eCO2_avg5m, (float32_t)eq_CO2, AverageWindow_5m);
	eCO2_avg1m = approxMovingAverage(eCO2_avg1m, (float32_t)eq_CO2, AverageWindow_1m);
	if ((float32_t)eq_CO2 > eCO2_avg5m)
		eq_CO2 = eCO2_avg5m;
	else
		eq_CO2 = eCO2_avg1m;

#if (GAS_SENSOR_MODULE_PRESENT==0)
	CO_Out = (uint32_t)(eq_CO2*100);	//Used by BLE in app_bluenrg_2.c User_Process() function
#endif
//Calculate mean values in an hour
#if (USE_BKUP_SRAM)						//Restores the average values if a watch-dog event has occurred
	if (VOC_mean_init)					//or following a short power-cycle (<1 min)
	{
		eTVOC_avg = (float32_t)eq_TVOC_1h_Mean;
		eCO2_avg = (float32_t)eq_CO2_1h_Mean;
		VOC_mean_init = false;
	}
#endif
	eTVOC_avg = approxMovingAverage(eTVOC_avg, (float32_t)eq_TVOC, AverageWindow_1h);
	voc->eTVOC_mean = (uint16_t)lrintf(eTVOC_avg);
	eCO2_avg = approxMovingAverage(eCO2_avg, (float32_t)eq_CO2, AverageWindow_1h);
	voc->eCO2_mean = (uint16_t)lrintf(eCO2_avg);

	eq_TVOC_1h_Mean = voc->eTVOC_mean;
	eq_TVOC_1h_MeanMax = (eq_TVOC_1h_Mean > eq_TVOC_1h_MeanMax) ? eq_TVOC_1h_Mean : eq_TVOC_1h_MeanMax;
	eq_CO2_1h_Mean = voc->eCO2_mean;
	eq_CO2_1h_MeanMax = (eq_CO2_1h_Mean > eq_CO2_1h_MeanMax) ? eq_CO2_1h_Mean : eq_CO2_1h_MeanMax;
#if (CCS811)
	if (Store_CCS811_Baseline)			//Perform a Baseline storage when button is pressed for more than 5s
	{									//or on a weekly basis. (Function HAL_RTCEx_RTCEventCallback in port.c)
		Store_CCS811_Baseline = false;
//		save_status = CCS811_Save_Baseline();
		CCS811_Save_Baseline(true);
	}
	if ((load_baseline) && (!baseline_loaded))
	{
		CCS811_Restore_Baseline(true);
		load_baseline = false;
		baseline_loaded = true;
	}
#endif	//CCS811
#if (GUI_SUPPORT==1)
	eTVOC_1_data[0] = (float)eq_TVOC;	//For UnicleoGUI
	eCO2_1_data[0] = (float)eq_TVOC;	//For UnicleoGUI
#endif
#if (CCS811)
	(void)memcpy((void *)&Buff[31], voc, sizeof(CCS811_MeasureTypeDef_st));
#elif (ENS160)
	(void)memcpy((void *)&Buff[31], voc, 12);
#endif
	/* sizeof(CCS811_MeasureTypeDef_st) == 12 */
}
#endif

#if (PARTICULATE_SENSOR_PRESENT==1)
/**
 * @brief  Handles the SPS30 Particulate Matter sensor data getting/sending.
 * @brief  Build the Buff array from the float (LSB first)
 * @brief  Calculate the hourly/daily average of the concentration values
 * @brief  Fill the Particulate parts of the Buff stream
 * @param  Particulate: Pointer to PMx data structure
 * @param  Buff: Stream pointer
 * @retval None
 */
void Particulate_Sensor_Handler(SPS30_MeasureTypeDef_st *Particulate, uint8_t* Buff)
{
	static const uint32_t AverageWindow_24h = 17280;	//3600*24/5: Number of readings in 24 hour
	static float32_t mc_1p0_avg = 0;
	static float32_t mc_2p5_avg = 0;
	static float32_t mc_4p0_avg = 0;
	static float32_t mc_10p0_avg = 0;
	static float32_t MC_1p0_f, MC_2p5_f, MC_4p0_f, MC_10p0_f;
#if (USE_BKUP_SRAM)
	static bool PMx_mean_init = true;
#endif

	MC_1p0 = (uint16_t)lrintf(MC_1p0_f);
	MC_2p5 = (uint16_t)lrintf(MC_2p5_f);
	MC_4p0 = (uint16_t)lrintf(MC_4p0_f);
	MC_10p0 = (uint16_t)lrintf(MC_10p0_f);

	NC_0p5 = (uint16_t)lrintf(Particulate->nc_0p5);
	NC_1p0 = (uint16_t)lrintf(Particulate->nc_1p0);
	NC_2p5 = (uint16_t)lrintf(Particulate->nc_2p5);
	NC_4p0 = (uint16_t)lrintf(Particulate->nc_4p0);
	NC_10p0 = (uint16_t)lrintf(Particulate->nc_10p0);

//	MC_1p0_f = (uint16_t)(Particulate->mc_1p0);
//	MC_2p5_f = (uint16_t)(Particulate->mc_2p5);
//	MC_4p0_f = (uint16_t)(Particulate->mc_4p0);
//	MC_10p0_f = (uint16_t)(Particulate->mc_10p0);
	if (NC_1p0 > 0)
		MC_1p0_f = PM1p0_CORRECTION(Particulate->mc_1p0);
	else
		MC_1p0_f = 0.0;

	if (NC_2p5 > 0)
		MC_2p5_f = PM2p5_CORRECTION(Particulate->mc_2p5);
	else
		MC_2p5_f = 0.0;

	if (NC_4p0 > 0)
		MC_4p0_f = PM4p0_CORRECTION(Particulate->mc_4p0);
	else
		MC_4p0_f =  0.0;

	if (NC_10p0)
		MC_10p0_f = PM10p0_CORRECTION(Particulate->mc_10p0);
	else
		MC_10p0_f = 0.0;

	TypicalParticleSize = Particulate->typical_particle_size;

	//Moving Average on AverageWindow_24h readings for Air quality values estimation.
	//PM1.0: ??; PM2.5: 25 μg/mc 24-hour mean; PM4.0: ??; PM10: 50 μg/mm 24-hour mean.
	//From World Health Organization Ambient (outdoor) air pollution Fact Sheet.
	//https://www.who.int/news-room/fact-sheets/detail/ambient-(outdoor)-air-quality-and-health
#if (USE_BKUP_SRAM)						//Restores the average values if a watch-dog event has occurred
	if (PMx_mean_init)					//or following a short power-cycle (<1 min)
	{
		mc_1p0_avg = (float32_t)MC_1p0_24h_Mean;
		mc_2p5_avg = (float32_t)MC_2p5_24h_Mean;
		mc_4p0_avg = (float32_t)MC_4p0_24h_Mean;
		mc_10p0_avg = (float32_t)MC_10p0_24h_Mean;
		PMx_mean_init = false;
	}
#endif
	mc_1p0_avg = approxMovingAverage(mc_1p0_avg, MC_1p0_f, AverageWindow_24h);
	Particulate->mc_1p0_mean = mc_1p0_avg;
	mc_2p5_avg = approxMovingAverage(mc_2p5_avg, MC_2p5_f, AverageWindow_24h);
	Particulate->mc_2p5_mean = mc_2p5_avg;
	mc_4p0_avg = approxMovingAverage(mc_4p0_avg, MC_4p0_f, AverageWindow_24h);
	Particulate->mc_4p0_mean = mc_4p0_avg;
	mc_10p0_avg = approxMovingAverage(mc_10p0_avg, MC_10p0_f, AverageWindow_24h);
	Particulate->mc_10p0_mean = mc_10p0_avg;

	MC_1p0_24h_Mean = (uint16_t)lrintf(Particulate->mc_1p0_mean);
	MC_2p5_24h_Mean = (uint16_t)lrintf(Particulate->mc_2p5_mean);
	MC_2p5_24h_MeanMax = (MC_2p5_24h_Mean > MC_2p5_24h_MeanMax) ? MC_2p5_24h_Mean : MC_2p5_24h_MeanMax;
	MC_4p0_24h_Mean = (uint16_t)lrintf(Particulate->mc_4p0_mean);
	MC_10p0_24h_Mean = (uint16_t)lrintf(Particulate->mc_10p0_mean);
	MC_10p0_24h_MeanMax = (MC_10p0_24h_Mean > MC_10p0_24h_MeanMax) ? MC_10p0_24h_Mean : MC_10p0_24h_MeanMax;
#if (GUI_SUPPORT==1)
	PM1p0_1_data[0] = (int32_t)MC_1p0;		//For UnicleoGUI
	PM1p0_2_data[0] = (int32_t)NC_1p0;		//For UnicleoGUI
	PM2p5_1_data[0] = (int32_t)MC_2p5;		//For UnicleoGUI
	PM2p5_2_data[0] = (int32_t)NC_2p5;		//For UnicleoGUI
	PM4p0_1_data[0] = (int32_t)MC_4p0;		//For UnicleoGUI
	PM4p0_2_data[0] = (int32_t)NC_4p0;		//For UnicleoGUI
	PM10_1_data[0] = (int32_t)MC_10p0;		//For UnicleoGUI
	PM10_2_data[0] = (int32_t)NC_10p0;		//For UnicleoGUI
#endif
	(void)memcpy((void *)&Buff[43], Particulate, sizeof(SPS30_MeasureTypeDef_st));
	/* sizeof(SPS30_MeasureTypeDef_st) == 56 */
}
#endif

#if (GAS_SENSOR_MODULE_PRESENT==1)
/**
 * @brief  Handles the analog gas sensor board data getting.
 * @brief  Build the Buff array from the float (LSB first)
 * @brief  Calculate the hourly/daily average of the concentration values
 * @brief  Fill the gases parts of the Buff stream
 * @param  anlg: Pointer to analog inputs data structure
 * @param  Buff: Stream pointer
 * @retval None
 */
void Gas_Sensor_Handler(ANLG_MeasureTypeDef_st *anlg, uint8_t* Buff)
{
	static const uint32_t AverageWindow_8h = 5760;	//3600*8/5: Number of readings in 8 hour
	static const uint32_t AverageWindow_1h = 720;	//3600*1/5: Number of readings in 1 hour
#if (OUTDOOR_MODE)
	static const uint32_t AverageWindow_24h = 17280;	//3600*24/5: Number of readings in 24 hour
#endif
	static float32_t co_8h_avg = 0;
	static float32_t ch2o_8h_avg = 0;
	static float32_t no2_1h_avg = 0;
	static float32_t nh3_8h_avg = 0;
#if (OUTDOOR_MODE)
	static float32_t o3_1h_avg = 0;
	static float32_t so2_1h_avg = 0;
	static float32_t c6h6_24h_avg = 0;
#endif
#if (USE_BKUP_SRAM)
	static bool Gases_mean_init = true;
#endif

	// Caution!!! Only for linear relationships between analog value and gas concentration
	// it is possible to apply the correction here!!
	// For non-linear relations the correction is applied in the "read_SMO_sensors ()" function
	CO = (uint16_t)lrintf(anlg->CO);
	CH2O = (uint16_t)lrintf(anlg->CH2O);
//	CH2O = (uint16_t)(lrintf(anlg->CH2O) + CH2O_Corr);
	NO2 = (uint16_t)lrintf(anlg->NO2);
	NH3 = (uint16_t)lrintf(anlg->NH3);
#if (OUTDOOR_MODE)
	O3 = (uint16_t)lrintf(anlg->O3);
	SO2 = (uint16_t)lrintf(anlg->SO2);
	C6H6 = (uint16_t)lrintf(anlg->C6H6);
//	O3 = (uint16_t)(lrintf(anlg->O3) + O3_Corr);
//	SO2 = (uint16_t)(lrintf(anlg->SO2) + SO2_Corr);
//	C6H6 = (uint16_t)(lrintf(anlg->C6H6) + C6H6_Corr);
#endif
#if (VOC_SENSOR_PRESENT==0)
	CO_Out = (uint32_t)(CO*100);	//Used by BLE in app_bluenrg_2.c User_Process() function
#endif

	//Moving Average on AverageWindow readings for Air quality values estimation.
	//From World Health Organization Ambient (outdoor) air pollution Fact Sheet.
	//https://www.who.int/news-room/fact-sheets/detail/ambient-(outdoor)-air-quality-and-health
#if (USE_BKUP_SRAM)					//Restores the average values if a watch-dog event has occurred
	if (Gases_mean_init)			//or following a short power-cycle (<1 min)
	{
		co_8h_avg = (float32_t)CO_8h_Mean;
		ch2o_8h_avg = (float32_t)CH2O_8h_Mean;
	#if (OUTDOOR_MODE)
		no2_1h_avg = (float32_t)NO2_1h_Mean;
		nh3_8h_avg = (float32_t)NH3_8h_Mean;
		o3_1h_avg = (float32_t)O3_1h_Mean;
		so2_1h_avg = (float32_t)SO2_1h_Mean;
		c6h6_24h_avg = (float32_t)C6H6_24h_Mean;
	#endif
		Gases_mean_init = false;
	}
#endif
	co_8h_avg = approxMovingAverage(co_8h_avg, anlg->CO, AverageWindow_8h);
	anlg->co_8h_mean = co_8h_avg;
	ch2o_8h_avg = approxMovingAverage(ch2o_8h_avg, anlg->CH2O, AverageWindow_8h);
	anlg->ch2o_8h_mean = ch2o_8h_avg;
	no2_1h_avg = approxMovingAverage(no2_1h_avg, anlg->NO2, AverageWindow_1h);
	anlg->no2_1h_mean = no2_1h_avg;
	nh3_8h_avg = approxMovingAverage(nh3_8h_avg, anlg->NH3, AverageWindow_8h);
	anlg->nh3_8h_mean = nh3_8h_avg;
#if (OUTDOOR_MODE)
	o3_1h_avg = approxMovingAverage(o3_1h_avg, anlg->O3, AverageWindow_1h);
	anlg->o3_1h_mean = o3_1h_avg;
	so2_1h_avg = approxMovingAverage(so2_1h_avg, anlg->SO2, AverageWindow_1h);
	anlg->so2_1h_mean = so2_1h_avg;
	c6h6_24h_avg = approxMovingAverage(c6h6_24h_avg, anlg->C6H6, AverageWindow_24h);
	anlg->c6h6_24h_mean = c6h6_24h_avg;
#endif

	CO_8h_Mean = (uint16_t)lrintf(anlg->co_8h_mean);
	CO_8h_MeanMax = (CO_8h_Mean > CO_8h_MeanMax) ? CO_8h_Mean : CO_8h_MeanMax;
	CH2O_8h_Mean = (uint16_t)lrintf(anlg->ch2o_8h_mean);
	CH2O_8h_MeanMax = (CH2O_8h_Mean > CH2O_8h_MeanMax) ? CH2O_8h_Mean : CH2O_8h_MeanMax;
	NO2_1h_Mean = (uint16_t)lrintf(anlg->no2_1h_mean);
	NO2_1h_MeanMax = (NO2_1h_Mean > NO2_1h_MeanMax) ? NO2_1h_Mean : NO2_1h_MeanMax;
	NH3_8h_Mean = (uint16_t)lrintf(anlg->nh3_8h_mean);
	NH3_8h_MeanMax = (NH3_8h_Mean > NH3_8h_MeanMax) ? NH3_8h_Mean : NH3_8h_MeanMax;
#if (OUTDOOR_MODE)
	O3_1h_Mean = (uint16_t)lrintf(anlg->o3_1h_mean);
	O3_1h_MeanMax = (O3_1h_Mean > O3_1h_MeanMax) ? O3_1h_Mean : O3_1h_MeanMax;
	SO2_1h_Mean = (uint16_t)lrintf(anlg->so2_1h_mean);
	SO2_1h_MeanMax = (SO2_1h_Mean > SO2_1h_MeanMax) ? SO2_1h_Mean : SO2_1h_MeanMax;
	C6H6_24h_Mean = (uint16_t)lrintf(anlg->c6h6_24h_mean);
	C6H6_24h_MeanMax = (C6H6_24h_Mean > C6H6_24h_MeanMax) ? C6H6_24h_Mean : C6H6_24h_MeanMax;
#endif

#if (GUI_SUPPORT==1)
	CO_data[0] = (int32_t)CO;			//For UnicleoGUI
	CH2O_data[0] = (int32_t)CH2O;		//For UnicleoGUI
	NO2_data[0] = (int32_t)NO2;			//For UnicleoGUI
	NH3_data[0] = (int32_t)NH3;			//For UnicleoGUI
	#if (OUTDOOR_MODE)
		O3_data[0] = (int32_t)O3;		//For UnicleoGUI
		SO2_data[0] = (int32_t)SO2;		//For UnicleoGUI
		C6H6_data[0] = (int32_t)C6H6;	//For UnicleoGUI
	#endif
#endif
	(void)memcpy((void *)&Buff[99], anlg, sizeof(ANLG_MeasureTypeDef_st));
	/* sizeof(ANLG_MeasureTypeDef_st) == 92 */
}
#endif

void Refresh_AQI(void)
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
	static AirQualityParameters_st AQ_Level;
#pragma GCC diagnostic pop

#if (GAS_SENSOR_MODULE_PRESENT==1)
#if (OUTDOOR_MODE)
	AQ_Level = AirQuality(eq_TVOC, eq_CO2, eq_TVOC_1h_Mean, eq_CO2_1h_Mean,
						  CH2O, CO, NO2, NH3, O3, SO2, C6H6, MC_10p0, MC_2p5,
						  CH2O_8h_Mean, CO_8h_Mean, NO2_1h_Mean, NH3_8h_Mean,
						  O3_1h_Mean, SO2_1h_Mean, C6H6_24h_Mean,
						  MC_10p0_24h_Mean, MC_2p5_24h_Mean);
#else	//OUTDOOR_MODE==0
	AQ_Level = AirQuality(eq_TVOC, eq_CO2, eq_TVOC_1h_Mean, eq_CO2_1h_Mean,
						  CH2O, CO, NO2, NH3, 0, 0, 0, MC_10p0, MC_2p5,
						  CH2O_8h_Mean, CO_8h_Mean, NO2_1h_Mean, NH3_8h_Mean, 0, 0, 0,
						  MC_10p0_24h_Mean, MC_2p5_24h_Mean);
#endif	//OUTDOOR_MODE
#else	//GAS_SENSOR_MODULE_PRESENT==0
	#if (PARTICULATE_SENSOR_PRESENT)
	AQ_Level = AirQuality(eq_TVOC, eq_CO2, eq_TVOC_1h_Mean, eq_CO2_1h_Mean,
						  0, 0, 0, 0, 0, 0, 0, MC_10p0, MC_2p5,
						  0, 0, 0, 0, 0, 0, 0,
						  MC_10p0_24h_Mean, MC_2p5_24h_Mean);
	#else
	AQ_Level = AirQuality(eq_TVOC, eq_CO2, eq_TVOC_1h_Mean, eq_CO2_1h_Mean,
						  0, 0, 0, 0, 0, 0, 0, 0, 0,
						  0, 0, 0, 0, 0, 0, 0,
						  0, 0);
	#endif
#endif //GAS_SENSOR_MODULE_PRESENT
}

#if (USE_BKUP_SRAM)
/*
 * @brief Every hour the average values of the sensors are stored in the processor's
 * 		  Static Ram Backup. In this way, after a "short" reset (not a power-cycle)
 * 		  the daily statistic will not be lost and will be immediately available
 * 		  for transmission.
 * @param  None
 * @retval None
 */
void Store_MeanValues_BackupRTC(void)
{
#if (VOC_SENSOR_PRESENT)
	//Store Air Quality Data. Averaged values
	HOST_TO_BKPR_LE_16(BakUpRTC_Data+6, eq_TVOC_1h_Mean);
	HOST_TO_BKPR_LE_16(BakUpRTC_Data+8, eq_CO2_1h_Mean);
	//Store Air Quality Data. Max values
	HOST_TO_BKPR_LE_16(BakUpRTC_Data+48, eq_TVOC_1h_Mean_Max);
	HOST_TO_BKPR_LE_16(BakUpRTC_Data+50, eq_CO2_1h_Mean_Max);
#endif
#if (GAS_SENSOR_MODULE_PRESENT)
	//Store Air Quality Data. Averaged values
	HOST_TO_BKPR_LE_16(BakUpRTC_Data+10, CO_8h_Mean);
	HOST_TO_BKPR_LE_16(BakUpRTC_Data+16, CH2O_8h_Mean);
	HOST_TO_BKPR_LE_16(BakUpRTC_Data+12, NO2_1h_Mean);
	HOST_TO_BKPR_LE_16(BakUpRTC_Data+14, NH3_8h_Mean);
	//Store Air Quality Data. Daily Max values
	HOST_TO_BKPR_LE_16(BakUpRTC_Data+52, CO_8h_Mean_Max);
	HOST_TO_BKPR_LE_16(BakUpRTC_Data+54, CH2O_8h_Mean_Max);
	HOST_TO_BKPR_LE_16(BakUpRTC_Data+56, NO2_1h_Mean_Max);
	HOST_TO_BKPR_LE_16(BakUpRTC_Data+58, NH3_8h_Mean_Max);
	#if (OUTDOOR_MODE)
		//Store Air Quality Data. Averaged values
		HOST_TO_BKPR_LE_16(BakUpRTC_Data+18, O3_1h_Mean);
		HOST_TO_BKPR_LE_16(BakUpRTC_Data+20, SO2_1h_Mean);
		HOST_TO_BKPR_LE_16(BakUpRTC_Data+22, C6H6_24h_Mean);
		//Store Air Quality Data. Daily Max values
		HOST_TO_BKPR_LE_16(BakUpRTC_Data+60, O3_1h_Mean_Max);
		HOST_TO_BKPR_LE_16(BakUpRTC_Data+62, SO2_1h_Mean_Max);
		HOST_TO_BKPR_LE_16(BakUpRTC_Data+64, C6H6_24h_Mean_Max);
	#endif
#endif	//GAS_SENSOR_MODULE_PRESENT
#if (PARTICULATE_SENSOR_PRESENT)
	//Store Air pollution Data. Averaged values
	HOST_TO_BKPR_LE_16(BakUpRTC_Data+24, MC_1p0_24h_Mean);
	HOST_TO_BKPR_LE_16(BakUpRTC_Data+26, MC_2p5_24h_Mean);
	HOST_TO_BKPR_LE_16(BakUpRTC_Data+28, MC_4p0_24h_Mean);
	HOST_TO_BKPR_LE_16(BakUpRTC_Data+30, MC_10p0_24h_Mean);
	//Store Air pollution Data. Daily Max values
	HOST_TO_BKPR_LE_16(BakUpRTC_Data+66, MC_2p5_24h_Mean_Max);
	HOST_TO_BKPR_LE_16(BakUpRTC_Data+68, MC_10p0_24h_Mean_Max);
#endif	//PARTICULATE_SENSOR_PRESENT
	//Store environmental data. Daily Min-Max values
#if (PRESSURE_SENSOR_PRESENT)
	HOST_TO_BKPR_LE_32(BakUpRTC_Data+32, P_Min);
	HOST_TO_BKPR_LE_32(BakUpRTC_Data+36, P_Max);
#endif
#if (HUMIDITY_SENSOR_PRESENT)
	HOST_TO_BKPR_LE_16(BakUpRTC_Data+44, H_Min);
	HOST_TO_BKPR_LE_16(BakUpRTC_Data+46, H_Max);
#endif
#if ((PRESSURE_SENSOR_PRESENT) || (HUMIDITY_SENSOR_PRESENT))
	HOST_TO_BKPR_LE_16(BakUpRTC_Data+40, T_Min);
	HOST_TO_BKPR_LE_16(BakUpRTC_Data+42, T_Max);
#endif
	//Store Global Status Register
	HOST_TO_BKPR_LE_32(BakUpRTC_Data+70, StatusReg);
	//Write in the backup register domain
	enable_backup_rtc();
	writeBkpRTC((uint8_t *)BakUpRTC_Data, sizeof(BakUpRTC_Data), 0);
	disable_backup_rtc();
}

/*
 * The average values of the sensors, written by the Store_MeanValues_BackupRTC()
 * function, are restored by this function
 * @param  None
 * @retval None
 */
void ReStore_MeanValues_BackupRTC(void)
{
#if (VOC_SENSOR_PRESENT)
	//Restore Air Quality Data. Averaged values
	memcpy(&eq_TVOC_1h_Mean, &BakUpRTC_Data[6], 2);
	memcpy(&eq_CO2_1h_Mean, &BakUpRTC_Data[8], 2);
	//Restore Air Quality Data. Daily Max values
	memcpy(&eq_TVOC_1h_Mean_Max, &BakUpRTC_Data[48], 2);
	memcpy(&eq_CO2_1h_Mean_Max, &BakUpRTC_Data[50], 2);
#endif
#if (GAS_SENSOR_MODULE_PRESENT)
	//Restore Air Quality Data. Averaged values
	memcpy(&CO_8h_Mean, &BakUpRTC_Data[10], 2);
	memcpy(&CH2O_8h_Mean, &BakUpRTC_Data[16], 2);
	memcpy(&NO2_1h_Mean, &BakUpRTC_Data[12], 2);
	memcpy(&NH3_8h_Mean, &BakUpRTC_Data[14], 2);
	//Restore Air Quality Data. Max values
	memcpy(&CO_8h_Mean_Max, &BakUpRTC_Data[52], 2);
	memcpy(&CH2O_8h_Mean_Max, &BakUpRTC_Data[54], 2);
	memcpy(&NO2_1h_Mean_Max, &BakUpRTC_Data[56], 2);
	memcpy(&NH3_8h_Mean_Max, &BakUpRTC_Data[58], 2);
	#if (OUTDOOR_MODE)
		//Restore Air Quality Data. Averaged values
		memcpy(&O3_1h_Mean, &BakUpRTC_Data[18], 2);
		memcpy(&SO2_1h_Mean, &BakUpRTC_Data[20], 2);
		memcpy(&C6H6_24h_Mean, &BakUpRTC_Data[22], 2);
		//Restore Air Quality Data. Daily Max values
		memcpy(&O3_1h_Mean_Max, &BakUpRTC_Data[60], 2);
		memcpy(&SO2_1h_Mean_Max, &BakUpRTC_Data[62], 2);
		memcpy(&C6H6_24h_Mean_Max, &BakUpRTC_Data[64], 2);
	#endif
#endif	//GAS_SENSOR_MODULE_PRESENT
#if (PARTICULATE_SENSOR_PRESENT)
	//Restore Air pollution Data. Averaged values
	memcpy(&MC_1p0_24h_Mean, &BakUpRTC_Data[24], 2);
	memcpy(&MC_2p5_24h_Mean, &BakUpRTC_Data[26], 2);
	memcpy(&MC_4p0_24h_Mean, &BakUpRTC_Data[28], 2);
	memcpy(&MC_10p0_24h_Mean, &BakUpRTC_Data[30], 2);
	//Restore Air pollution Data. Daily Max values
	memcpy(&MC_2p5_24h_Mean_Max, &BakUpRTC_Data[66], 2);
	memcpy(&MC_10p0_24h_Mean_Max, &BakUpRTC_Data[68], 2);
#endif	//PARTICULATE_SENSOR_PRESENT
	//Restore environmental Data. Daily Min-Max values
#if (PRESSURE_SENSOR_PRESENT)
	memcpy(&P_Min, &BakUpRTC_Data[32], 4);
	memcpy(&P_Max, &BakUpRTC_Data[36], 4);
#endif
#if (HUMIDITY_SENSOR_PRESENT)
	memcpy(&H_Min, &BakUpRTC_Data[44], 2);
	memcpy(&H_Max, &BakUpRTC_Data[46], 2);
#endif
#if ((PRESSURE_SENSOR_PRESENT) || (HUMIDITY_SENSOR_PRESENT))
	memcpy(&T_Min, &BakUpRTC_Data[40], 2);
	memcpy(&T_Max, &BakUpRTC_Data[42], 2);
#endif
	//Restore Global Status Register
	memcpy(&StatusReg, &BakUpRTC_Data[70], 4);
}
#endif	//USE_BKUP_SRAM

/*
 * This function stores the daily minimum and maximum values
 * detected by the sensors in the respective data structures.
 * @param  None
 * @retval None
 */
#if ((BLE_SUPPORT) && (BEACON_APP))
#if (CCS811)
void StoreMinMax(LPS25HB_MeasureTypeDef_st *PressTemp, HTS221_MeasureTypeDef_st *HumTemp, ANLG_MeasureTypeDef_st *Measurement_Value,
                 CCS811_MeasureTypeDef_st *voc, SPS30_MeasureTypeDef_st *Particulate)
#elif(ENS160)
void StoreMinMax(LPS25HB_MeasureTypeDef_st *PressTemp, HTS221_MeasureTypeDef_st *HumTemp, ANLG_MeasureTypeDef_st *Measurement_Value,
                 ENS160_MeasureTypeDef_st *voc, SPS30_MeasureTypeDef_st *Particulate)
#endif
{
#if (PRESSURE_SENSOR_PRESENT)
	PressTemp->Pout_DailyMax = P_Max = PMax;
	PressTemp->Pout_DailyMin = P_Min = PMin;
	//Re-Initialize LPS25HB Pressure Min Max values variables
	PMin = LPS25HB_UPPER_P_LIMIT;
	PMax = LPS25HB_LOWER_P_LIMIT;
	#if (HUMIDITY_SENSOR_PRESENT==0)
	PressTemp->Tout_DailyMax = T_Max = TMax;
	PressTemp->Tout_DailyMin = T_Min = TMin;
	//Re-Initialize LPS25HB Temperature Min Max values variables
	TMin = LPS25HB_UPPER_T_LIMIT;
	TMax = LPS25HB_LOWER_T_LIMIT;
	#endif
#endif
#if (HUMIDITY_SENSOR_PRESENT)
	HumTemp->Hout_DailyMax = H_Max = HMax;
	HumTemp->Hout_DailyMin = H_Min = HMin;
	//Re-Initialize HTS221 Humidity Min Max values variables
	HMin = HTS221_UPPER_H_LIMIT;
	HMax = HTS221_LOWER_H_LIMIT;
	HumTemp->Tout_DailyMax = T_Max = TMax;
	HumTemp->Tout_DailyMin = T_Min = TMin;
	//Re-Initialize HTS221 Temperature Min Max values variables
	TMin = HTS221_UPPER_T_LIMIT;
	TMax = HTS221_LOWER_T_LIMIT;
#endif
#if (VOC_SENSOR_PRESENT)
	voc->eTVOC_mean_DailyMax = eq_TVOC_1h_Mean_Max = eq_TVOC_1h_MeanMax;
	voc->eCO2_mean_DailyMax = eq_CO2_1h_Mean_Max = eq_CO2_1h_MeanMax;
	//Re-Initialize eTVOC, eCO2 Max values variables
	#if (CCS811)
		eq_TVOC_1h_MeanMax = CCS811_LOWER_TVOC_LIMIT;
		eq_CO2_1h_MeanMax = CCS811_LOWER_CO2_LIMIT;
	#elif (ENS160)
		eq_TVOC_1h_MeanMax = ENS160_LOWER_TVOC_LIMIT;
		eq_CO2_1h_MeanMax = ENS160_LOWER_CO2_LIMIT;
	#endif
#endif
#if (PARTICULATE_SENSOR_PRESENT)
	Particulate->mc_2p5_mean_DailyMax = MC_2p5_24h_Mean_Max = MC_2p5_24h_MeanMax;
	Particulate->mc_10p0_mean_DailyMax = MC_10p0_24h_Mean_Max = MC_10p0_24h_MeanMax;
	//Re-Initialize PM2.5, PM10 Max values variables
	MC_2p5_24h_MeanMax = SPS30_LOWER_PM2P5_LIMIT;
	MC_10p0_24h_MeanMax = SPS30_LOWER_PM10_LIMIT;
#endif
#if (GAS_SENSOR_MODULE_PRESENT)
	Measurement_Value->co_8h_mean_DailyMax = CO_8h_Mean_Max = CO_8h_MeanMax;
	Measurement_Value->ch2o_8h_mean_DailyMax = CH2O_8h_Mean_Max = CH2O_8h_MeanMax;
	Measurement_Value->no2_1h_mean_DailyMax = NO2_1h_Mean_Max = NO2_1h_MeanMax;
	Measurement_Value->nh3_8h_mean_DailyMax = NH3_8h_Mean_Max = NH3_8h_MeanMax;
	//Re-Initialize Gases Sensors Max values variables
	CO_8h_MeanMax = ANLG_LOWER_CO_LIMIT;
	CH2O_8h_MeanMax = ANLG_LOWER_CH2O_LIMIT;
	NO2_1h_MeanMax = ANLG_LOWER_NO2_LIMIT;
	NH3_8h_MeanMax = ANLG_LOWER_NH3_LIMIT;
	#if (OUTDOOR_MODE)
		Measurement_Value->o3_1h_mean_DailyMax = O3_1h_Mean_Max = O3_1h_MeanMax;
		Measurement_Value->so2_1h_mean_DailyMax = SO2_1h_Mean_Max = SO2_1h_MeanMax;
		Measurement_Value->c6h6_24h_mean_DailyMax = C6H6_24h_Mean_Max = C6H6_24h_MeanMax;
		//Re-Initialize Gases Sensors Max values variables
		O3_1h_MeanMax = ANLG_LOWER_O3_LIMIT;
		SO2_1h_MeanMax = ANLG_LOWER_SO2_LIMIT;
		C6H6_24h_MeanMax = ANLG_LOWER_C6H6_LIMIT;
	#endif
#endif	//GAS_SENSOR_MODULE_PRESENT
}
#endif	//((BLE_SUPPORT) && (BEACON_APP))

#if (IMU_PRESENT==1)
/**
 * @brief  Handles the LSM9DS1 ACC, GYR, MAG axes data getting/sending
 * @brief  Build the Buff array from the uint32_t (LSB first)
 * @brief  Fill the ACC, GYR, MAG parts of the Buff stream
 * @param  IMU_Axes: Axes pointer to axes data structure
 * @param  Buff: Stream pointer
 * @retval None
 */
void IMU_Handler(LSM9DS1_MeasureTypeDef_st *IMU_Axes, uint8_t* Buff)
{
	static float filterValue[MFX_NUM_AXES] = {0.0, 0.0, 0.0};
	static float scaleFactor = 0.1;		//Filter Coefficient: the lower the value, the greater the filter effect

#if (PRESSURE_SENSOR_PRESENT==0)
	float press_value, temp_value;
	const double_t n_samples = 200.00;
	int16_t Temp;

	T_OutRaw = IMU_Axes->TRaw_Out;			//
	T_Out = IMU_Axes->T_Out;				//Use and mean LSM9DS1 Temperature sensor values when
	//Mean LSM9DS1 IMU temperature			//LPS25HB Temperature sensor is not present
	Temp += T_Out;
	avg_cnt++;
	if (avg_cnt == n_samples)
	{
		avg_cnt = 0;
		Temperature = Temp/n_samples;
		Temp = 0;
	}

	press_value = (float)(Altitude);
	temp_value = (float)(Temperature);

	(void)memcpy((void *)&Buff[7], (void *)&press_value, sizeof(float));
	(void)memcpy((void *)&Buff[11], (void *)&temp_value, sizeof(float));
#endif
	/* Accelerometer_GetData */
	(void)memcpy((void *)&Buff[19], (void *)&IMU_Axes->Acc_Out.AXIS_X, 4);
	(void)memcpy((void *)&Buff[23], (void *)&IMU_Axes->Acc_Out.AXIS_Y, 4);
	(void)memcpy((void *)&Buff[27], (void *)&IMU_Axes->Acc_Out.AXIS_Z, 4);
	/* Filter giro raw values */
	filterValue[0] = filterValue[0] + scaleFactor * ((float)IMU_Axes->Gy_Out.AXIS_X - filterValue[0]);
	IMU_Axes->Gy_Out.AXIS_X = (int32_t)filterValue[0];
	filterValue[1] = filterValue[1] + scaleFactor * ((float)IMU_Axes->Gy_Out.AXIS_Y - filterValue[1]);
	IMU_Axes->Gy_Out.AXIS_Y = (int32_t)filterValue[1];
	filterValue[2] = filterValue[2] + scaleFactor * ((float)IMU_Axes->Gy_Out.AXIS_Z - filterValue[2]);
	IMU_Axes->Gy_Out.AXIS_Z = (int32_t)filterValue[2];
	/* Gyroscope_GetFilteredData */
	(void)memcpy((void *)&Buff[31], (void *)&IMU_Axes->Gy_Out.AXIS_X, 4);
	(void)memcpy((void *)&Buff[35], (void *)&IMU_Axes->Gy_Out.AXIS_Y, 4);
	(void)memcpy((void *)&Buff[39], (void *)&IMU_Axes->Gy_Out.AXIS_Z, 4);
	/* Magnetometer_GetData */
	(void)memcpy((void *)&Buff[43], (void *)&IMU_Axes->Mag_Out.AXIS_X, 4);
	(void)memcpy((void *)&Buff[47], (void *)&IMU_Axes->Mag_Out.AXIS_Y, 4);
	(void)memcpy((void *)&Buff[51], (void *)&IMU_Axes->Mag_Out.AXIS_Z, 4);
}

/**
 * @brief  Magnetometer calibration data handler (By MotionFX)
 * @param  Buff: Pointer of the Magnetometer calibration part of the stream
 * @retval None
 */
void MC_Data_Handler(LSM9DS1_MeasureTypeDef_st *IMU_Axes, uint8_t* Buff)
{
	static uint32_t MagTimeStamp = 0;

	if (MagCalRequest)		//Magnetometer calibration is performed at 25Hz rate
	{
		MagCalRequest = 0;
		mag_data_in.mag[0] = (float)IMU_Axes->Mag_Out.AXIS_X * mGauss_to_uT50;
		mag_data_in.mag[1] = (float)IMU_Axes->Mag_Out.AXIS_Y * mGauss_to_uT50;
		mag_data_in.mag[2] = (float)IMU_Axes->Mag_Out.AXIS_Z * mGauss_to_uT50;
		mag_data_in.time_stamp = (int)MagTimeStamp;
		MagTimeStamp += (uint32_t)MAG_CAL_ALGO_PERIOD;
		/* Run Magnetometer Calibration algorithm & Get the magnetometer compensation */
		MotionFX_manager_MagCal_run(&mag_data_in, &mag_data_out);
		/* Get Calibration quality */
		Calibrated_Magnetic_Field_uT_1_quality[0] = (int32_t)mag_data_out.cal_quality;
		/* Do offset & scale factor calibration */
	   	if (mag_data_out.cal_quality == MFX_MAGCALGOOD)
	   	{
	   		MagCalStatus = 1;
	   		ans_float = (mag_data_out.hi_bias[0] * uT50_to_mGauss);
	   		MagOffset.AXIS_X = (int32_t)ans_float;
	   		ans_float = (mag_data_out.hi_bias[1] * uT50_to_mGauss);
	   		MagOffset.AXIS_Y = (int32_t)ans_float;
	   		ans_float = (mag_data_out.hi_bias[2] * uT50_to_mGauss);
	   		MagOffset.AXIS_Z = (int32_t)ans_float;
			/* Disable magnetometer calibration */
	   		MotionFX_manager_MagCal_stop(MAG_CAL_ALGO_PERIOD);
	   	}
	}

	IMU_Axes->Mag_Out.AXIS_X = (int32_t)IMU_Axes->Mag_Out.AXIS_X - MagOffset.AXIS_X;
	IMU_Axes->Mag_Out.AXIS_Y = (int32_t)IMU_Axes->Mag_Out.AXIS_Y - MagOffset.AXIS_Y;
	IMU_Axes->Mag_Out.AXIS_Z = (int32_t)IMU_Axes->Mag_Out.AXIS_Z - MagOffset.AXIS_Z;
/*
  //Offset coefficients
  Serialize_s32(&Buff[55], (int32_t)acc_bias_to_mg(data_out.AccBias[0]), 4);
  Serialize_s32(&Buff[59], (int32_t)acc_bias_to_mg(data_out.AccBias[1]), 4);
  Serialize_s32(&Buff[63], (int32_t)acc_bias_to_mg(data_out.AccBias[2]), 4);

  //Scale factor coefficients
  FloatToArray(&Buff[67], data_out.SF_Matrix[0][0]);
  FloatToArray(&Buff[71], data_out.SF_Matrix[0][1]);
  FloatToArray(&Buff[75], data_out.SF_Matrix[0][2]);

  FloatToArray(&Buff[79], data_out.SF_Matrix[1][0]);
  FloatToArray(&Buff[83], data_out.SF_Matrix[1][1]);
  FloatToArray(&Buff[87], data_out.SF_Matrix[1][2]);

  FloatToArray(&Buff[91], data_out.SF_Matrix[2][0]);
  FloatToArray(&Buff[95], data_out.SF_Matrix[2][1]);
  FloatToArray(&Buff[99], data_out.SF_Matrix[2][2]);

  //Calibrated data
  Serialize_s32(&Buff[103], (int32_t) acc_comp.x, 4);
  Serialize_s32(&Buff[107], (int32_t) acc_comp.y, 4);
  Serialize_s32(&Buff[111], (int32_t) acc_comp.z, 4);

  //Calibration quality
  Serialize_s32(&Buff[115], (int32_t) data_out.CalQuality, 4); */
}

/**
 * @brief  Gyroscope calibration data handler
 * @param  Buff: Pointer of the Gyroscope calibration part of the stream
 * @retval None
 */
void GC_Data_Handler(LSM9DS1_MeasureTypeDef_st *IMU_Axes, uint8_t* Buff)
{
	MGC_input_t data_in;
	MGC_output_t data_out;
	int bias_update = 0;
//	SensorAxes_t GyrComp;

	if (GyroCalRequest)		//Gyroscope calibration is performed at 50Hz rate
	{
		GyroCalRequest = 0;
		data_in.Acc[0] = (float)IMU_Axes->Acc_Out.AXIS_X * mg_to_g;
		data_in.Acc[1] = (float)IMU_Axes->Acc_Out.AXIS_Y * mg_to_g;
		data_in.Acc[2] = (float)IMU_Axes->Acc_Out.AXIS_Z * mg_to_g;
		data_in.Gyro[0] = (float)IMU_Axes->Gy_Out.AXIS_X * mdps_to_dps;
		data_in.Gyro[1] = (float)IMU_Axes->Gy_Out.AXIS_Y * mdps_to_dps;
		data_in.Gyro[2] = (float)IMU_Axes->Gy_Out.AXIS_Z * mdps_to_dps;
		/* Run Gyroscope Calibration algorithm */
		MotionGC_manager_update(&data_in, &data_out, &bias_update);
		Calibrated_Gyroscope_g_1_status[0] = (int32_t) bias_update;
	}
	/* Do offset & scale factor calibration */
	MotionGC_manager_compensate(&IMU_Axes->Gy_Out, &IMU_Axes->Gy_Out);
	/* Update buffer */
	(void)memcpy((void *)&Buff[31], (void *)&IMU_Axes->Gy_Out.AXIS_X, 4);
	(void)memcpy((void *)&Buff[35], (void *)&IMU_Axes->Gy_Out.AXIS_Y, 4);
	(void)memcpy((void *)&Buff[39], (void *)&IMU_Axes->Gy_Out.AXIS_Z, 4);
/*
	//Offset coefficients
	Serialize_s32(&Buff[55], (int32_t)gyro_bias_to_mdps(data_out.GyroBiasX), 4);
	Serialize_s32(&Buff[59], (int32_t)gyro_bias_to_mdps(data_out.GyroBiasY), 4);
	Serialize_s32(&Buff[63], (int32_t)gyro_bias_to_mdps(data_out.GyroBiasZ), 4);

	//Calibrated data
	Serialize_s32(&Buff[67], (int32_t) GyrComp.x, 4);
	Serialize_s32(&Buff[71], (int32_t) GyrComp.y, 4);
	Serialize_s32(&Buff[75], (int32_t) GyrComp.z, 4);

	//Calibration quality
	Serialize_s32(&Buff[79], (int32_t) bias_update, 4); */
}

/**
 * @brief  Accelerometer calibration data handler
 * @param  Buff: Pointer of the Accelerometer calibration part of the stream
 * @retval None
 */
void AC_Data_Handler(LSM9DS1_MeasureTypeDef_st *IMU_Axes, uint8_t* Buff)
{
	uint8_t is_calibrated = 0;
	uint32_t time_stamp_uint32;
	MAC_input_t data_in;
	MAC_output_t data_out;
//  SensorAxes_t acc_comp;
	static MAC_calibration_mode_t prev_calibration_mode = DYNAMIC_CALIBRATION;

	if (CalibrationMode != prev_calibration_mode)
	{
		/* Reset library */
		MotionAC_manager_init(MAC_DISABLE_LIB);
		MotionAC_manager_init(MAC_ENABLE_LIB);
		/* Update knobs */
		pKnobs->Run6PointCal = (uint8_t)CalibrationMode;
		(void)MotionAC_SetKnobs(&Knobs);
		prev_calibration_mode = CalibrationMode;
	}

	if (AccCalRequest)		//Accelerometer calibration is performed at 50Hz rate
	{
		AccCalRequest = 0;
		/* Convert acceleration from [mg] to [g] */
		data_in.Acc[0] = (float)IMU_Axes->Acc_Out.AXIS_X * mg_to_g;
		data_in.Acc[1] = (float)IMU_Axes->Acc_Out.AXIS_Y * mg_to_g;
		data_in.Acc[2] = (float)IMU_Axes->Acc_Out.AXIS_Z * mg_to_g;
		time_stamp_uint32 = AC_TimeStamp * ReportInterval;
		data_in.TimeStamp = (int)time_stamp_uint32;
		/* Run Accelerometer Calibration algorithm */
		MotionAC_manager_update(&data_in, &is_calibrated);
		/* Get the accelerometer compensation */
		MotionAC_manager_get_params(&data_out);
		/* Get Calibration quality */
		Calibrated_Acceleration_g_1_quality[0] = (int32_t)data_out.CalQuality;
	}
	/* Do offset & scale factor calibration */
	MotionAC_manager_compensate(&IMU_Axes->Acc_Out, &IMU_Axes->Acc_Out);
	/* Update buffer */
	(void)memcpy((void *)&Buff[19], (void *)&IMU_Axes->Acc_Out.AXIS_X, 4);
	(void)memcpy((void *)&Buff[23], (void *)&IMU_Axes->Acc_Out.AXIS_Y, 4);
	(void)memcpy((void *)&Buff[27], (void *)&IMU_Axes->Acc_Out.AXIS_Z, 4);
/*
	//Offset coefficients
	Serialize_s32(&Msg->Data[55], (int32_t)acc_bias_to_mg(data_out.AccBias[0]), 4);
	Serialize_s32(&Msg->Data[59], (int32_t)acc_bias_to_mg(data_out.AccBias[1]), 4);
	Serialize_s32(&Msg->Data[63], (int32_t)acc_bias_to_mg(data_out.AccBias[2]), 4);

	//Scale factor coefficients
	FloatToArray(&Msg->Data[67], data_out.SF_Matrix[0][0]);
	FloatToArray(&Msg->Data[71], data_out.SF_Matrix[0][1]);
	FloatToArray(&Msg->Data[75], data_out.SF_Matrix[0][2]);

	FloatToArray(&Msg->Data[79], data_out.SF_Matrix[1][0]);
	FloatToArray(&Msg->Data[83], data_out.SF_Matrix[1][1]);
	FloatToArray(&Msg->Data[87], data_out.SF_Matrix[1][2]);

	FloatToArray(&Msg->Data[91], data_out.SF_Matrix[2][0]);
	FloatToArray(&Msg->Data[95], data_out.SF_Matrix[2][1]);
	FloatToArray(&Msg->Data[99], data_out.SF_Matrix[2][2]);

	//Calibrated data
	Serialize_s32(&Msg->Data[103], (int32_t) acc_comp.x, 4);
	Serialize_s32(&Msg->Data[107], (int32_t) acc_comp.y, 4);
	Serialize_s32(&Msg->Data[111], (int32_t) acc_comp.z, 4);

	//Calibration quality
	Serialize_s32(&Msg->Data[115], (int32_t) data_out.CalQuality, 4); */
}

/**
 * @brief  Activity Recognition data handler
 * @param  Msg the Activity Recognition data part of the stream
 * @retval None
 */
void AR_Data_Handler(LSM9DS1_MeasureTypeDef_st *IMU_Axes, uint8_t* Buff)
{
	MAR_input_t data_in = {.acc_x = 0.0f, .acc_y = 0.0f, .acc_z = 0.0f};
	static MAR_output_t ActivityCodeStored = MAR_NOACTIVITY;
	static MAR_output_t ActivityCode;
//	static uint32_t session_counter = 0;

	if (ActRecRequest)					//Activity Recognition is performed at 16Hz rate
	{
		ActRecRequest = 0;
		/* Convert acceleration from [mg] to [g] */
		data_in.acc_x = (float)IMU_Axes->Acc_Out.AXIS_X * mg_to_g;
		data_in.acc_y = (float)IMU_Axes->Acc_Out.AXIS_Y * mg_to_g;
		data_in.acc_z = (float)IMU_Axes->Acc_Out.AXIS_Z * mg_to_g;

		/* Run Activity Recognition algorithm */
		MotionAR_manager_run(&data_in, &ActivityCode, AR_TimeStamp);
		Activity_Index_1_data[0] = (uint8_t)ActivityCode;
		// Data changed
		if (ActivityCodeStored != ActivityCode)
		{
			// Store new data in buffer
//			RTC_DateTimeStamp(&DataByte[DataIndex].date_time);
//			DataByte[DataIndex].data_valid = 1;	// 0 if new session
//			DataByte[DataIndex].activity_type = (uint8_t)ActivityCode;
//			DataIndex++;
			ActivityCodeStored = ActivityCode;
		}

/*		if (ProgramState == GUI_MODE)
		{
			Serialize_s32(&Msg->Data[55], (int32_t)ActivityCode, 4);
		}
		else if (ProgramState == STANDALONE_MODE)
		{
			// New session
			if (SessionNew == 1U)
			{
				SessionNew = 0;
				session_counter = 0;
				DataIndex = 0;
				// Create new session record with only date and time
				RTC_DateTimeStamp(&DataByte[DataIndex].date_time);
				DataByte[DataIndex].data_valid = 0;	// 0 if new session
				DataByte[DataIndex].activity_type = (uint8_t)MAR_NOACTIVITY;
				DataIndex++;
				ActivityCodeStored = MAR_NOACTIVITY;
				return;
			}

			// Buffer full
			if (DataIndex >= DATABYTE_LEN)
			{
				// Data are stored in FLASH
				if (Datalog_SaveData2Mem(DataIndex) == 0U)
				{
					GuiModeRequest = 1;
				}
				DataIndex = 0;
			}

			// Data changed
			if (ActivityCodeStored != ActivityCode)
			{
				// Store new data in buffer
				RTC_DateTimeStamp(&DataByte[DataIndex].date_time);
				DataByte[DataIndex].data_valid = 1;	// 0 if new session
				DataByte[DataIndex].activity_type = (uint8_t)ActivityCode;
				DataIndex++;
				ActivityCodeStored = ActivityCode;
			}

			// Periodic data save
			if (session_counter >= SESSION_COUNTER_MAX)
			{
				// Data are stored in FLASH
				if (Datalog_SaveData2Mem(DataIndex) == 0U)
				{
					GuiModeRequest = 1;
				}
				session_counter = 0;
				DataIndex = 0;
			}
			else
			{
				session_counter++;
			}
		}
		else
		{
			Error_Handler();
		} */
	}
}

/**
 * @brief  Sensor Fusion data handler
 * @param  IMU_Axes: Axes pointer to axes data structure
 * @param  Buff: Stream pointer
 * @retval None
 */
void FX_Data_Handler(LSM9DS1_MeasureTypeDef_st *IMU_Axes, uint8_t* Buff)
{
//	MFX_input_t data_in;							//Declared in MEMS_app.h
	MFX_input_t *pdata_in = &data_in;
	MFX_output_t data_out;
	MFX_output_t *pdata_out = &data_out;

	data_in.gyro[0] = (float)IMU_Axes->Gy_Out.AXIS_X * mdps_to_dps;
	data_in.gyro[1] = (float)IMU_Axes->Gy_Out.AXIS_Y * mdps_to_dps;
	data_in.gyro[2] = (float)IMU_Axes->Gy_Out.AXIS_Z * mdps_to_dps;

	data_in.acc[0] = (float)IMU_Axes->Acc_Out.AXIS_X * mg_to_g;
	data_in.acc[1] = (float)IMU_Axes->Acc_Out.AXIS_Y * mg_to_g;
	data_in.acc[2] = (float)IMU_Axes->Acc_Out.AXIS_Z * mg_to_g;

	data_in.mag[0] = (float)IMU_Axes->Mag_Out.AXIS_X * mGauss_to_uT50;
	data_in.mag[1] = (float)IMU_Axes->Mag_Out.AXIS_Y * mGauss_to_uT50;
	data_in.mag[2] = (float)IMU_Axes->Mag_Out.AXIS_Z * mGauss_to_uT50;

	//Run Sensor Fusion algorithm
	MotionFX_manager_run(pdata_in, pdata_out, MOTIONFX_ENGINE_DELTATIME);

	(void)memcpy((void *)&Buff[55], (void *)pdata_out->quaternion, 4U * sizeof(float));
	(void)memcpy((void *)&Quaternions_9X_1_data[0], (void *)pdata_out->quaternion, 4U * sizeof(float));	//For UnicleoGUI
	(void)memcpy((void *)&Buff[71], (void *)pdata_out->rotation, 3U * sizeof(float));
	(void)memcpy((void *)&Rotation_Vector_9X_1_data[0], (void *)pdata_out->rotation, 3U * sizeof(float));//For UnicleoGUI
	(void)memcpy((void *)&Buff[83], (void *)pdata_out->gravity, 3U * sizeof(float));
	(void)memcpy((void *)&Buff[95], (void *)pdata_out->linear_acceleration, 3U * sizeof(float));
	(void)memcpy((void *)&Linear_Acceleration_9X_1_data[0], (void *)pdata_out->linear_acceleration, 3U * sizeof(float));	//For UnicleoGUI
	(void)memcpy((void *)&Buff[107], (void *)&pdata_out->heading, sizeof(float));
	(void)memcpy((void *)&Heading_9X_1_data[0], (void *)&pdata_out->heading, sizeof(float));				//For UnicleoGUI
	(void)memcpy((void *)&Buff[111], (void *)&pdata_out->headingErr, sizeof(float));
}

/**
 * @brief  Velocity estimation from linear acceleration
 * @param  LinAcc_0: pointer to previous acceleration sample (Linear_Acceleration_9X_0_data)
 * @param  LinAcc_1: pointer to current acceleration sample (Linear_Acceleration_9X_1_data)
 * @param  LinVel_0: pointer to previous velocity sample (Linear_Speed_9X_0_data)
 * @param  LinVel_1: pointer to current velocity sample (Linear_Speed_9X_1_data)
 * @param  CalGyroStatus: pointer to Calibrated_Gyroscope_g_1_status
 * 		   Used to recognize the IMU stationary status: velocity is set to 0
 * 		   when this parameter is 1.
 * @retval None
 */
void VE_Data_Handler(float *LinAcc_0, float *LinAcc_1, float *LinVel_0, float *LinVel_1, int32_t* CalGyroStatus)
{
	uint8_t stat;
//	double_t MKSLinAcc0X, MKSLinAcc0Y, MKSLinAcc0Z, MKSLinVel0X, MKSLinVel0Y, MKSLinVel0Z;
	double_t squared_LinVelX, squared_LinVelY, squared_LinVelZ, Esp;
//	double_t squared_LinAccX, squared_LinAccY, squared_LinAccZ, LinAccMod;

	stat = (uint8_t)*CalGyroStatus;
	Esp = (double_t)2.0;

/*	squared_LinAccX = pow((double_t)LinAcc_1[0], Esp);
	squared_LinAccY = pow((double_t)LinAcc_1[1], Esp);
	squared_LinAccZ = pow((double_t)LinAcc_1[2], Esp);
	LinAccMod = (float)(sqrt(squared_LinAccX + squared_LinAccY + squared_LinAccZ));	//x y z Lin.Acc module

	if (LinAccMod < 0.05)			//Force velocity to zero when LinAcc is under the threshold */
	if (stat)						//Force velocity to zero when there is no Gyro activity
	{
		LinVel_1[0] = 0.0f;
		LinVel_1[1] = 0.0f;
		LinVel_1[2] = 0.0f;
	}
	else
	{
		LinVel_1[0] = LinVel_0[0] + (float)(((LinAcc_0[0] * g_to_ms2) + (((LinAcc_1[0] * g_to_ms2) - (LinAcc_0[0] * g_to_ms2)) / 2.0)) * MOTIONFX_ENGINE_DELTATIME);
		LinVel_1[1] = LinVel_0[1] + (float)(((LinAcc_0[1] * g_to_ms2) + (((LinAcc_1[1] * g_to_ms2) - (LinAcc_0[1] * g_to_ms2)) / 2.0)) * MOTIONFX_ENGINE_DELTATIME);
		LinVel_1[2] = LinVel_0[2] + (float)(((LinAcc_0[2] * g_to_ms2) + (((LinAcc_1[2] * g_to_ms2) - (LinAcc_0[2] * g_to_ms2)) / 2.0)) * MOTIONFX_ENGINE_DELTATIME);
	}

	squared_LinVelX = pow((double_t)LinVel_1[0], Esp);
	squared_LinVelY = pow((double_t)LinVel_1[1], Esp);
	squared_LinVelZ = pow((double_t)LinVel_1[2], Esp);
	LinVel_1[3] = (float)(sqrt(squared_LinVelX + squared_LinVelY + squared_LinVelZ));	//x y z velocity module
	LinVel_1[4] = (float)(sqrt(squared_LinVelX + squared_LinVelY));						//x y velocity module

	(void)memcpy((void *)&LinVel_0[0], (void *)&LinVel_1[0], 5U * sizeof(float));		//Update old values
	(void)memcpy((void *)&LinAcc_0[0], (void *)&LinAcc_1[0], 5U * sizeof(float));
}

#endif	//IMU_PRESENT==0
