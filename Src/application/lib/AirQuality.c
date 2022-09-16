/**
 ******************************************************************************
 * @file    AirQuality.c
 * @brief   This file contains the air quality estimate functions
 *
 * @author  Tommaso Sabatini, 2020
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "application/AirQuality.h"

uint8_t CalcAQI(uint16_t C_low, uint16_t C_high, uint16_t C, uint8_t I_low, uint8_t I_high)
{
	uint8_t w_aqi;

	w_aqi = (uint8_t)lrintf((float32_t)(((I_high - I_low)/(C_high - C_low)) * (C - C_low) + I_low));

	return w_aqi;
}

AirQualityParameters_st AirQuality(uint16_t eq_TVOC, uint16_t eq_CO2, uint16_t eq_TVOC_1h_Mean, uint16_t eq_CO2_8h_Mean,
								   uint16_t CH2O, uint16_t CO, uint16_t NO2, uint16_t NH3,
								   uint16_t O3, uint16_t SO2, uint16_t C6H6,
								   uint16_t CH2O_8h_Mean, uint16_t CO_8h_Mean, uint16_t NO2_1h_Mean, uint16_t NH3_8h_Mean,
								   uint16_t O3_8h_Mean, uint16_t SO2_1h_Mean, uint16_t C6H6_24h_Mean,
								   uint16_t MC_10p0_24h_Mean, uint16_t MC_2p5_24h_Mean)
{
	const char AQ_Class[6][16]={{"1: EXCELLENT \r\n"},{"   2: FINE   \r\n"},{" 3: MODERATE \r\n"},{"   4: POOR   \r\n"},
						 	 	{"5: VERY POOR \r\n"},{"  6: SEVERE  \r\n"}};
	static AirQualityParameters_st AirQuality_Level =
								{.eTVOC_AQ=2, .eCO2_AQ=2, .PM10_AQ=2, .PM2p5_AQ=2, .CH2O_AQ=2, .CO_AQ=2, .NO2_AQ=2, .NH3_AQ=2,
								 .O3_AQ=2, .SO2_AQ=2, .C6H6_AQ= 2, .AvgGasAirQualityIndex=2,
								 .AvgPMxAirQualityIndex=2, .AvgGasAirQualityClass={}, .AvgPMxAirQualityClass={}};

	//Find the current eTVOC AirQuality level
	// eTVOC_Excellent = 65,	//ppb
	// eTVOC_Good = 220,		//ppb
	// eTVOC_Moderate = 660,	//ppb
	// eTVOC_Poor = 2200,		//ppb
	// eTVOC_Unhealty = 5500,	//ppb

	if (InRange(0, eTVOC_Exellent, eq_TVOC) == 1)
	{
//		AirQuality_Level.eTVOC_AQ = 0;
		AirQuality_Level.eTVOC_AQ = CalcAQI(0, eTVOC_Exellent, eq_TVOC, 0, 1);
	} else
	if (InRange(eTVOC_Exellent, eTVOC_Good, eq_TVOC) == 1)
	{
//		AirQuality_Level.eTVOC_AQ = 1;
		AirQuality_Level.eTVOC_AQ = CalcAQI(eTVOC_Exellent, eTVOC_Good, eq_TVOC, 1, 2);
	} else
	if (InRange(eTVOC_Good, eTVOC_Moderate, eq_TVOC) == 1)
	{
//		AirQuality_Level.eTVOC_AQ = 2;
		AirQuality_Level.eTVOC_AQ = CalcAQI(eTVOC_Good, eTVOC_Moderate, eq_TVOC, 2, 3);
	} else
	if (InRange(eTVOC_Moderate, eTVOC_Poor, eq_TVOC) == 1)
	{
//		AirQuality_Level.eTVOC_AQ = 3;
		AirQuality_Level.eTVOC_AQ = CalcAQI(eTVOC_Moderate, eTVOC_Poor, eq_TVOC, 3, 4);
	} else
	if (InRange(eTVOC_Poor, eTVOC_Unhealty, eq_TVOC) == 1)
	{
//		AirQuality_Level.eTVOC_AQ = 4;
		AirQuality_Level.eTVOC_AQ = CalcAQI(eTVOC_Poor, eTVOC_Unhealty, eq_TVOC, 4, 5);
	} else
	if (InRange(eTVOC_Poor, eTVOC_Unhealty, eq_TVOC) == 2)
	{
//		AirQuality_Level.eTVOC_AQ = 5;
		AirQuality_Level.eTVOC_AQ = CalcAQI(eTVOC_Poor, eTVOC_Unhealty,eq_TVOC,  5, 6);
	}

	//Find the current eCO2 AirQuality level
	// eCO2_FreshAir = 450,		//ppm
	// eCO2_Normal = 1832,		//ppm = 6.5*eTVOC_Good + 402
	// eCO2_Acceptable = 4692,	//ppm = 6.5*eTVOC_Moderate + 402
	// eCO2_Drowsiness = 14702,	//ppm = 6.5*eTVOC_Poor + 402
	// eCO2_Harmful = 36152,	//ppm = 6.5*eTVOC_Unhealty + 402
	if (InRange(0, eCO2_FreshAir, eq_CO2) == 1)
	{
//		AirQuality_Level.eCO2_AQ = 0;
		AirQuality_Level.eCO2_AQ = CalcAQI(0, eCO2_FreshAir, eq_CO2, 0, 1);
	} else
	if (InRange(eCO2_FreshAir, eCO2_Normal, eq_CO2) == 1)
	{
//		AirQuality_Level.eCO2_AQ = 1;
		AirQuality_Level.eCO2_AQ = CalcAQI(eCO2_FreshAir, eCO2_Normal, eq_CO2, 1, 2);
	} else
	if (InRange(eCO2_Normal, eCO2_Acceptable, eq_CO2) == 1)
	{
//		AirQuality_Level.eCO2_AQ = 2;
		AirQuality_Level.eCO2_AQ = CalcAQI(eCO2_Normal, eCO2_Acceptable, eq_CO2, 2, 3);
	} else
	if (InRange(eCO2_Acceptable, eCO2_Drowsiness, eq_CO2) == 1)
	{
//		AirQuality_Level.eCO2_AQ = 3;
		AirQuality_Level.eCO2_AQ = CalcAQI(eCO2_Acceptable, eCO2_Drowsiness, eq_CO2, 3, 4);
	} else
	if (InRange(eCO2_Drowsiness, eCO2_Harmful, eq_CO2) == 1)
	{
//		AirQuality_Level.eCO2_AQ = 4;
		AirQuality_Level.eCO2_AQ = CalcAQI(eCO2_Drowsiness, eCO2_Harmful, eq_CO2, 4, 5);
	} else
	if (InRange(eCO2_Drowsiness, eCO2_Harmful, eq_CO2) == 2)
	{
//		AirQuality_Level.eCO2_AQ = 5;
		AirQuality_Level.eCO2_AQ = CalcAQI(eCO2_Drowsiness, eCO2_Harmful, eq_CO2, 5, 6);
	}

	//Find the current CH2O AirQuality level
	// CH2O_Excellent = 60,	//ug/m3
	// CH2O_Fine = 120,		//ug/m3
	// CH2O_Moderate = 121,	//ug/m3
	// CH2O_Poor = 121,		//ug/m3
	// CH2O_VeryPoor = 121,	//ug/m3
	if (InRange(0, CH2O_Exellent, CH2O) == 1)
	{
//		AirQuality_Level.CH2O_AQ = 0;
		AirQuality_Level.CH2O_AQ = CalcAQI(0, CH2O_Exellent, CH2O, 0, 1);
	} else
	if (InRange(CH2O_Exellent, CH2O_Fine, CH2O) == 1)
	{
//		AirQuality_Level.CH2O_AQ = 1;
		AirQuality_Level.CH2O_AQ = CalcAQI(CH2O_Exellent, CH2O_Fine, CH2O, 1, 2);
	} else
	if (InRange(CH2O_Fine, CH2O_Moderate, CH2O) == 1)
	{
//		AirQuality_Level.CH2O_AQ = 2;
		AirQuality_Level.CH2O_AQ = CalcAQI(CH2O_Fine, CH2O_Moderate, CH2O, 2, 3);
	} else
	if (InRange(CH2O_Moderate, CH2O_Poor, CH2O) == 1)
	{
//		AirQuality_Level.CH2O_AQ = 3;
		AirQuality_Level.CH2O_AQ = CalcAQI(CH2O_Moderate, CH2O_Poor, CH2O, 3, 4);
	} else
	if (InRange(CH2O_Poor, CH2O_VeryPoor, CH2O) == 1)
	{
//		AirQuality_Level.CH2O_AQ = 4;
		AirQuality_Level.CH2O_AQ = CalcAQI(CH2O_Poor, CH2O_VeryPoor, CH2O, 4, 5);
	} else
	if (InRange(CH2O_Poor, CH2O_VeryPoor, CH2O) == 2)
	{
//		AirQuality_Level.CH2O_AQ = 5;
		AirQuality_Level.CH2O_AQ = CalcAQI(CH2O_Poor, CH2O_VeryPoor, CH2O, 5, 6);
	}

	//Find the current CO AirQuality level
	// CO_Excellent = 2,	//mg/m3
	// CO_Fine = 4,			//mg/m3
	// CO_Moderate = 7,		//mg/m3
	// CO_Poor = 10,		//mg/m3
	// CO_VeryPoor = 25,	//mg/m3
	if (InRange(0, CO_Exellent, CO) == 1)
	{
//		AirQuality_Level.CO_AQ = 0;
		AirQuality_Level.CO_AQ = CalcAQI(0, CO_Exellent, CO, 0, 1);
	} else
	if (InRange(CO_Exellent, CO_Fine, CO) == 1)
	{
//		AirQuality_Level.CO_AQ = 1;
		AirQuality_Level.CO_AQ = CalcAQI(CO_Exellent, CO_Fine, CO, 1, 2);
	} else
	if (InRange(CO_Fine, CO_Moderate, CO) == 1)
	{
//		AirQuality_Level.CO_AQ = 2;
		AirQuality_Level.CO_AQ = CalcAQI(CO_Fine, CO_Moderate, CO, 2, 3);
	} else
	if (InRange(CO_Moderate, CO_Poor, CO) == 1)
	{
//		AirQuality_Level.CO_AQ = 3;
		AirQuality_Level.CO_AQ = CalcAQI(CO_Moderate, CO_Poor, CO, 3, 4);
	} else
	if (InRange(CO_Poor, CO_VeryPoor, CO) == 1)
	{
//		AirQuality_Level.CO_AQ = 4;
		AirQuality_Level.CO_AQ = CalcAQI(CO_Poor, CO_VeryPoor, CO, 4, 5);
	} else
	if (InRange(CO_Poor, CO_VeryPoor, CO) == 2)
	{
//		AirQuality_Level.CO_AQ = 5;
		AirQuality_Level.CO_AQ = CalcAQI(CO_Poor, CO_VeryPoor, CO, 5, 6);
	}

	//Find the current NO2 AirQuality level
	// NO2_Excellent = 25,	//ug/m3
	// NO2_Fine = 40,		//ug/m3
	// NO2_Moderate = 100,	//ug/m3
	// NO2_Poor = 200,		//ug/m3
	// NO2_VeryPoor = 400,	//ug/m3
	if (InRange(0, NO2_Exellent, NO2) == 1)
	{
//		AirQuality_Level.NO2_AQ = 0;
		AirQuality_Level.NO2_AQ = CalcAQI(0, NO2_Exellent, NO2, 0, 1);
	} else
	if (InRange(NO2_Exellent, NO2_Fine, NO2) == 1)
	{
//		AirQuality_Level.NO2_AQ = 1;
		AirQuality_Level.NO2_AQ = CalcAQI(NO2_Exellent, NO2_Fine, NO2, 1, 2);
	} else
	if (InRange(NO2_Fine, NO2_Moderate, NO2) == 1)
	{
//		AirQuality_Level.NO2_AQ = 2;
		AirQuality_Level.NO2_AQ = CalcAQI(NO2_Fine, NO2_Moderate, NO2, 2, 3);
	} else
	if (InRange(NO2_Moderate, NO2_Poor, NO2) == 1)
	{
//		AirQuality_Level.NO2_AQ = 3;
		AirQuality_Level.NO2_AQ = CalcAQI(NO2_Moderate, NO2_Poor, NO2, 3, 4);
	} else
	if (InRange(NO2_Poor, NO2_VeryPoor, NO2) == 1)
	{
//		AirQuality_Level.NO2_AQ = 4;
		AirQuality_Level.NO2_AQ = CalcAQI(NO2_Poor, NO2_VeryPoor, NO2, 4, 5);
	} else
	if (InRange(NO2_Poor, NO2_VeryPoor, NO2) == 2)
	{
//		AirQuality_Level.NO2_AQ = 5;
		AirQuality_Level.NO2_AQ = CalcAQI(NO2_Poor, NO2_VeryPoor, NO2, 5, 6);
	}

	//Find the current NH3 AirQuality level
	// NH3_Excellent = 2,		//mg/m3
	// NH3_Fine = 5,			//mg/m3
	// NH3_Moderate = 9,		//mg/m3
	// NH3_Poor = 14,			//mg/m3
	// NH3_VeryPoor = 36,		//mg/m3
	if (InRange(0, NH3_Exellent, NH3) == 1)
	{
//		AirQuality_Level.NH3_AQ = 0;
		AirQuality_Level.NH3_AQ = CalcAQI(0, NH3_Exellent, NH3, 0, 1);
	} else
	if (InRange(NH3_Exellent, NH3_Fine, NH3) == 1)
	{
//		AirQuality_Level.NH3_AQ = 1;
		AirQuality_Level.NH3_AQ = CalcAQI(NH3_Exellent, NH3_Fine, NH3, 1, 2);
	} else
	if (InRange(NH3_Fine, NH3_Moderate, NH3) == 1)
	{
//		AirQuality_Level.NH3_AQ = 2;
		AirQuality_Level.NH3_AQ = CalcAQI(NH3_Fine, NH3_Moderate, NH3, 2, 3);
	} else
	if (InRange(NH3_Moderate, NH3_Poor, NH3) == 1)
	{
//		AirQuality_Level.NH3_AQ = 3;
		AirQuality_Level.NH3_AQ = CalcAQI(NH3_Moderate, NH3_Poor, NH3, 3, 4);
	} else
	if (InRange(NH3_Poor, NH3_VeryPoor, NH3) == 1)
	{
//		AirQuality_Level.NH3_AQ = 4;
		AirQuality_Level.NH3_AQ = CalcAQI(NH3_Poor, NH3_VeryPoor, NH3, 4, 5);
	} else
	if (InRange(NH3_Poor, NH3_VeryPoor, NH3) == 2)
	{
//		AirQuality_Level.NH3_AQ = 5;
		AirQuality_Level.NH3_AQ = CalcAQI(NH3_Poor, NH3_VeryPoor, NH3, 5, 6);
	}

	//Find the current O3 AirQuality level
	// O3_Excellent = 33,	//ug/m3
	// O3_Fine = 50,		//ug/m3
	// O3_Moderate = 80,	//ug/m3
	// O3_Poor = 120,		//ug/m3
	// O3_VeryPoor = 160	//ug/m3
	if (InRange(0, O3_Exellent, O3) == 1)
	{
//		AirQuality_Level.O3_AQ = 0;
		AirQuality_Level.O3_AQ = CalcAQI(0, O3_Exellent, O3, 0, 1);
	} else
	if (InRange(O3_Exellent, O3_Fine, O3) == 1)
	{
//		AirQuality_Level.O3_AQ = 1;
		AirQuality_Level.O3_AQ = CalcAQI(O3_Exellent, O3_Fine, O3, 1, 2);
	} else
	if (InRange(O3_Fine, O3_Moderate, O3) == 1)
	{
//		AirQuality_Level.O3_AQ = 2;
		AirQuality_Level.O3_AQ = CalcAQI(O3_Fine, O3_Moderate, O3, 2, 3);
	} else
	if (InRange(O3_Moderate, O3_Poor, O3) == 1)
	{
//		AirQuality_Level.O3_AQ = 3;
		AirQuality_Level.O3_AQ = CalcAQI(O3_Moderate, O3_Poor, O3, 3, 4);
	} else
	if (InRange(O3_Poor, O3_VeryPoor, O3) == 1)
	{
//		AirQuality_Level.O3_AQ = 4;
		AirQuality_Level.O3_AQ = CalcAQI(O3_Poor, O3_VeryPoor, O3, 4, 5);
	} else
	if (InRange(O3_Poor, O3_VeryPoor, O3) == 2)
	{
//		AirQuality_Level.O3_AQ = 5;
		AirQuality_Level.O3_AQ = CalcAQI(O3_Poor, O3_VeryPoor, O3, 5, 6);
	}

	//Find the current SO2 AirQuality level
	// SO2_Excellent = 25,		//ug/m3
	// SO2_Fine = 100,			//ug/m3
	// SO2_Moderate = 200,		//ug/m3
	// SO2_Poor = 350,			//ug/m3
	// SO2_VeryPoor = 500,		//ug/m3
	if (InRange(0, SO2_Exellent, SO2) == 1)
	{
//		AirQuality_Level.SO2_AQ = 0;
		AirQuality_Level.SO2_AQ = CalcAQI(0, SO2_Exellent, SO2, 0, 1);
	} else
	if (InRange(SO2_Exellent, SO2_Fine, SO2) == 1)
	{
//		AirQuality_Level.SO2_AQ = 1;
		AirQuality_Level.SO2_AQ = CalcAQI(SO2_Exellent, SO2_Fine, SO2, 1, 2);
	} else
	if (InRange(SO2_Fine, SO2_Moderate, SO2) == 1)
	{
//		AirQuality_Level.SO2_AQ = 2;
		AirQuality_Level.SO2_AQ = CalcAQI(SO2_Fine, SO2_Moderate, SO2, 2, 3);
	} else
	if (InRange(SO2_Moderate, SO2_Poor, SO2) == 1)
	{
//		AirQuality_Level.SO2_AQ = 3;
		AirQuality_Level.SO2_AQ = CalcAQI(SO2_Moderate, SO2_Poor, SO2, 3, 4);
	} else
	if (InRange(SO2_Poor, SO2_VeryPoor, SO2) == 1)
	{
//		AirQuality_Level.SO2_AQ = 4;
		AirQuality_Level.SO2_AQ = CalcAQI(SO2_Poor, SO2_VeryPoor, SO2, 4, 5);
	} else
	if (InRange(SO2_Poor, SO2_VeryPoor, SO2) == 2)
	{
//		AirQuality_Level.SO2_AQ = 5;
		AirQuality_Level.SO2_AQ = CalcAQI(SO2_Poor, SO2_VeryPoor, SO2, 5, 6);
	}

	//Find the current C6H6 AirQuality level
	// C6H6_Excellent = 1,		//ug/m3
	// C6H6_Fine = 1,			//ug/m3
	// C6H6_Moderate = 2,		//ug/m3
	// C6H6_Poor = 5,			//ug/m3
	// C6H6_VeryPoor = 6,		//ug/m3
	if (InRange(0, C6H6_Exellent, C6H6) == 1)
	{
//		AirQuality_Level.C6H6_AQ = 0;
		AirQuality_Level.C6H6_AQ = CalcAQI(0, C6H6_Exellent, C6H6, 0, 1);
	} else
	if (InRange(C6H6_Exellent, C6H6_Fine, C6H6) == 1)
	{
//		AirQuality_Level.C6H6_AQ = 1;
		AirQuality_Level.C6H6_AQ = CalcAQI(C6H6_Exellent, C6H6_Fine, C6H6, 1, 2);
	} else
	if (InRange(C6H6_Fine, C6H6_Moderate, C6H6) == 1)
	{
//		AirQuality_Level.C6H6_AQ = 2;
		AirQuality_Level.C6H6_AQ = CalcAQI(C6H6_Fine, C6H6_Moderate, C6H6, 2, 3);
	} else
	if (InRange(C6H6_Moderate, C6H6_Poor, C6H6) == 1)
	{
//		AirQuality_Level.C6H6_AQ = 3;
		AirQuality_Level.C6H6_AQ = CalcAQI(C6H6_Moderate, C6H6_Poor, C6H6, 3, 4);
	} else
	if (InRange(C6H6_Poor, C6H6_VeryPoor, C6H6) == 1)
	{
//		AirQuality_Level.C6H6_AQ = 4;
		AirQuality_Level.C6H6_AQ = CalcAQI(C6H6_Poor, C6H6_VeryPoor, C6H6, 4, 5);
	} else
	if (InRange(C6H6_Poor, C6H6_VeryPoor, C6H6) == 2)
	{
//		AirQuality_Level.C6H6_AQ = 5;
		AirQuality_Level.C6H6_AQ = CalcAQI(C6H6_Poor, C6H6_VeryPoor, C6H6, 5, 6);
	}

	//Find the overall air quality index due to current values of gaseous pollutants
	AirQuality_Level.GasAirQualityIndex =  (uint8_t)(max(9, (int32_t)AirQuality_Level.eTVOC_AQ, (int32_t)AirQuality_Level.eCO2_AQ,
														 (int32_t)AirQuality_Level.CH2O_AQ, (int32_t)AirQuality_Level.CO_AQ,
														 (int32_t)AirQuality_Level.NO2_AQ, (int32_t)AirQuality_Level.NH3_AQ,
														 (int32_t)AirQuality_Level.O3_AQ, (int32_t)AirQuality_Level.SO2_AQ,
														 (int32_t)AirQuality_Level.C6H6_AQ));
	strcpy(AirQuality_Level.GasAirQualityClass, AQ_Class[AirQuality_Level.GasAirQualityIndex]);
	Gas_AQI = AirQuality_Level.GasAirQualityIndex;	//Used in BLE Beacon packet

	//Find the 1h average eTVOC AirQuality level
	// eTVOC_Excellent = 65,	//ppb
	// eTVOC_Good = 220,		//ppb
	// eTVOC_Moderate = 660,	//ppb
	// eTVOC_Poor = 2200,		//ppb
	// eTVOC_Unhealty = 5500,	//ppb
	if (InRange(0, eTVOC_Exellent, eq_TVOC_1h_Mean) == 1)
	{
//		AirQuality_Level.eTVOC_AQ = 0;
		AirQuality_Level.eTVOC_AQ = CalcAQI(0, eTVOC_Exellent, eq_TVOC_1h_Mean, 0, 1);
	} else
	if (InRange(eTVOC_Exellent, eTVOC_Good, eq_TVOC_1h_Mean) == 1)
	{
//		AirQuality_Level.eTVOC_AQ = 1;
		AirQuality_Level.eTVOC_AQ = CalcAQI(eTVOC_Exellent, eTVOC_Good, eq_TVOC_1h_Mean, 1, 2);
	} else
	if (InRange(eTVOC_Good, eTVOC_Moderate, eq_TVOC_1h_Mean) == 1)
	{
//		AirQuality_Level.eTVOC_AQ = 2;
		AirQuality_Level.eTVOC_AQ = CalcAQI(eTVOC_Good, eTVOC_Moderate, eq_TVOC_1h_Mean, 2, 3);
	} else
	if (InRange(eTVOC_Moderate, eTVOC_Poor, eq_TVOC_1h_Mean) == 1)
	{
//		AirQuality_Level.eTVOC_AQ = 3;
		AirQuality_Level.eTVOC_AQ = CalcAQI(eTVOC_Moderate, eTVOC_Poor, eq_TVOC_1h_Mean, 3, 4);
	} else
	if (InRange(eTVOC_Poor, eTVOC_Unhealty, eq_TVOC_1h_Mean) == 1)
	{
//		AirQuality_Level.eTVOC_AQ = 4;
		AirQuality_Level.eTVOC_AQ = CalcAQI(eTVOC_Poor, eTVOC_Unhealty, eq_TVOC_1h_Mean, 4, 5);
	} else
	if (InRange(eTVOC_Poor, eTVOC_Unhealty, eq_TVOC_1h_Mean) == 2)
	{
//		AirQuality_Level.eTVOC_AQ = 5;
		AirQuality_Level.eTVOC_AQ = CalcAQI(eTVOC_Poor, eTVOC_Unhealty, eq_TVOC_1h_Mean, 5, 6);
	}

	//Find the 1h average eCO2 AirQuality level
	// eCO2_FreshAir = 450,		//ppm
	// eCO2_Normal = 1832,		//ppm = 6.5*eTVOC_Good + 402
	// eCO2_Acceptable = 4692,	//ppm = 6.5*eTVOC_Moderate + 402
	// eCO2_Drowsiness = 14702,	//ppm = 6.5*eTVOC_Poor + 402
	// eCO2_Harmful = 36152,	//ppm = 6.5*eTVOC_Unhealty + 402
	if (InRange(0, eCO2_FreshAir, eq_CO2_8h_Mean) == 1)
	{
//		AirQuality_Level.eCO2_AQ = 0;
		AirQuality_Level.eCO2_AQ = CalcAQI(0, eCO2_FreshAir, eq_CO2_8h_Mean, 0, 1);
	} else
	if (InRange(eCO2_FreshAir, eCO2_Normal, eq_CO2_8h_Mean) == 1)
	{
//		AirQuality_Level.eCO2_AQ = 1;
		AirQuality_Level.eCO2_AQ = CalcAQI(eCO2_FreshAir, eCO2_Normal, eq_CO2_8h_Mean, 1, 2);
	} else
	if (InRange(eCO2_Normal, eCO2_Acceptable, eq_CO2_8h_Mean) == 1)
	{
//		AirQuality_Level.eCO2_AQ = 2;
		AirQuality_Level.eCO2_AQ = CalcAQI(eCO2_Normal, eCO2_Acceptable, eq_CO2_8h_Mean, 2, 3);
	} else
	if (InRange(eCO2_Acceptable, eCO2_Drowsiness, eq_CO2_8h_Mean) == 1)
	{
//		AirQuality_Level.eCO2_AQ = 3;
		AirQuality_Level.eCO2_AQ = CalcAQI(eCO2_Acceptable, eCO2_Drowsiness, eq_CO2_8h_Mean, 3, 4);
	} else
	if (InRange(eCO2_Drowsiness, eCO2_Harmful, eq_CO2_8h_Mean) == 1)
	{
//		AirQuality_Level.eCO2_AQ = 4;
		AirQuality_Level.eCO2_AQ = CalcAQI(eCO2_Drowsiness, eCO2_Harmful, eq_CO2_8h_Mean, 4, 5);
	} else
	if (InRange(eCO2_Drowsiness, eCO2_Harmful, eq_CO2_8h_Mean) == 2)
	{
//		AirQuality_Level.eCO2_AQ = 5;
		AirQuality_Level.eCO2_AQ = CalcAQI(eCO2_Drowsiness, eCO2_Harmful, eq_CO2_8h_Mean, 5, 6);
	}

	//Find the 8h average CH2O AirQuality level
	// CH2O_Excellent = 60,	//ug/m3
	// CH2O_Fine = 120,		//ug/m3
	// CH2O_Moderate = 121,	//ug/m3
	// CH2O_Poor = 121,		//ug/m3
	// CH2O_VeryPoor = 121,	//ug/m3
	if (InRange(0, CH2O_Exellent, CH2O_8h_Mean) == 1)
	{
//		AirQuality_Level.CH2O_AQ = 0;
		AirQuality_Level.CH2O_AQ = CalcAQI(0, CH2O_Exellent, CH2O_8h_Mean, 0, 1);
	} else
	if (InRange(CH2O_Exellent, CH2O_Fine, CH2O_8h_Mean) == 1)
	{
//		AirQuality_Level.CH2O_AQ = 1;
		AirQuality_Level.CH2O_AQ = CalcAQI(CH2O_Exellent, CH2O_Fine, CH2O_8h_Mean, 1, 2);
	} else
	if (InRange(CH2O_Fine, CH2O_Moderate, CH2O_8h_Mean) == 1)
	{
//		AirQuality_Level.CH2O_AQ = 2;
		AirQuality_Level.CH2O_AQ = CalcAQI(CH2O_Fine, CH2O_Moderate, CH2O_8h_Mean, 2, 3);
	} else
	if (InRange(CH2O_Moderate, CH2O_Poor, CH2O_8h_Mean) == 1)
	{
//		AirQuality_Level.CH2O_AQ = 3;
		AirQuality_Level.CH2O_AQ = CalcAQI(CH2O_Moderate, CH2O_Poor, CH2O_8h_Mean, 3, 4);
	} else
	if (InRange(CH2O_Poor, CH2O_VeryPoor, CH2O_8h_Mean) == 1)
	{
//		AirQuality_Level.CH2O_AQ = 4;
		AirQuality_Level.CH2O_AQ = CalcAQI(CH2O_Poor, CH2O_VeryPoor, CH2O_8h_Mean, 4, 5);
	} else
	if (InRange(CH2O_Poor, CH2O_VeryPoor, CH2O_8h_Mean) == 2)
	{
//		AirQuality_Level.CH2O_AQ = 5;
		AirQuality_Level.CH2O_AQ = CalcAQI(CH2O_Poor, CH2O_VeryPoor, CH2O_8h_Mean, 5, 6);
	}

	//Find the 8h average CO AirQuality level
	// CO_Excellent = 2,	//mg/m3
	// CO_Fine = 4,			//mg/m3
	// CO_Moderate = 7,		//mg/m3
	// CO_Poor = 10,		//mg/m3
	// CO_VeryPoor = 25,	//mg/m3
	if (InRange(0, CO_Exellent, CO_8h_Mean) == 1)
	{
//		AirQuality_Level.CO_AQ = 0;
		AirQuality_Level.CO_AQ = CalcAQI(0, CO_Exellent, CO_8h_Mean, 0, 1);
	} else
	if (InRange(CO_Exellent, CO_Fine, CO_8h_Mean) == 1)
	{
//		AirQuality_Level.CO_AQ = 1;
		AirQuality_Level.CO_AQ = CalcAQI(CO_Exellent, CO_Fine, CO_8h_Mean, 1, 2);
	} else
	if (InRange(CO_Fine, CO_Moderate, CO_8h_Mean) == 1)
	{
//		AirQuality_Level.CO_AQ = 2;
		AirQuality_Level.CO_AQ = CalcAQI(CO_Fine, CO_Moderate, CO_8h_Mean, 2, 3);
	} else
	if (InRange(CO_Moderate, CO_Poor, CO_8h_Mean) == 1)
	{
//		AirQuality_Level.CO_AQ = 3;
		AirQuality_Level.CO_AQ = CalcAQI(CO_Moderate, CO_Poor, CO_8h_Mean, 3, 4);
	} else
	if (InRange(CO_Poor, CO_VeryPoor, CO_8h_Mean) == 1)
	{
//		AirQuality_Level.CO_AQ = 4;
		AirQuality_Level.CO_AQ = CalcAQI(CO_Poor, CO_VeryPoor, CO_8h_Mean, 4, 5);
	} else
	if (InRange(CO_Poor, CO_VeryPoor, CO_8h_Mean) == 2)
	{
//		AirQuality_Level.CO_AQ = 5;
		AirQuality_Level.CO_AQ = CalcAQI(CO_Poor, CO_VeryPoor, CO_8h_Mean, 5, 6);
	}

	//Find the 1h average NO2 AirQuality level
	// NO2_Excellent = 25,	//ug/m3
	// NO2_Fine = 40,		//ug/m3
	// NO2_Moderate = 100,	//ug/m3
	// NO2_Poor = 200,		//ug/m3
	// NO2_VeryPoor = 400,	//ug/m3
	if (InRange(0, NO2_Exellent, NO2_1h_Mean) == 1)
	{
//		AirQuality_Level.NO2_AQ = 0;
		AirQuality_Level.NO2_AQ = CalcAQI(0, NO2_Exellent, NO2_1h_Mean, 0, 1);
	} else
	if (InRange(NO2_Exellent, NO2_Fine, NO2_1h_Mean) == 1)
	{
//		AirQuality_Level.NO2_AQ = 1;
		AirQuality_Level.NO2_AQ = CalcAQI(NO2_Exellent, NO2_Fine, NO2_1h_Mean, 1, 2);
	} else
	if (InRange(NO2_Fine, NO2_Moderate, NO2_1h_Mean) == 1)
	{
//		AirQuality_Level.NO2_AQ = 2;
		AirQuality_Level.NO2_AQ = CalcAQI(NO2_Fine, NO2_Moderate, NO2_1h_Mean, 2, 3);
	} else
	if (InRange(NO2_Moderate, NO2_Poor, NO2_1h_Mean) == 1)
	{
//		AirQuality_Level.NO2_AQ = 3;
		AirQuality_Level.NO2_AQ = CalcAQI(NO2_Moderate, NO2_Poor, NO2_1h_Mean, 3, 4);
	} else
	if (InRange(NO2_Poor, NO2_VeryPoor, NO2_1h_Mean) == 1)
	{
//		AirQuality_Level.NO2_AQ = 4;
		AirQuality_Level.NO2_AQ = CalcAQI(NO2_Poor, NO2_VeryPoor, NO2_1h_Mean, 4, 5);
	} else
	if (InRange(NO2_Poor, NO2_VeryPoor, NO2_1h_Mean) == 2)
	{
//		AirQuality_Level.NO2_AQ = 5;
		AirQuality_Level.NO2_AQ = CalcAQI(NO2_Poor, NO2_VeryPoor, NO2_1h_Mean, 5, 6);
	}

	//Find the 8h average NH3 AirQuality level
	// NH3_Excellent = 2,		//mg/m3
	// NH3_Fine = 5,			//mg/m3
	// NH3_Moderate = 9,		//mg/m3
	// NH3_Poor = 14,			//mg/m3
	// NH3_VeryPoor = 36,		//mg/m3
	if (InRange(0, NH3_Exellent, NH3_8h_Mean) == 1)
	{
//		AirQuality_Level.NH3_AQ = 0;
		AirQuality_Level.NH3_AQ = CalcAQI(0, NH3_Exellent, NH3_8h_Mean, 0, 1);
	} else
	if (InRange(NH3_Exellent, NH3_Fine, NH3_8h_Mean) == 1)
	{
//		AirQuality_Level.NH3_AQ = 1;
		AirQuality_Level.NH3_AQ = CalcAQI(NH3_Exellent, NH3_Fine, NH3_8h_Mean, 1, 2);
	} else
	if (InRange(NH3_Fine, NH3_Moderate, NH3_8h_Mean) == 1)
	{
//		AirQuality_Level.NH3_AQ = 2;
		AirQuality_Level.NH3_AQ = CalcAQI(NH3_Fine, NH3_Moderate, NH3_8h_Mean, 2, 3);
	} else
	if (InRange(NH3_Moderate, NH3_Poor, NH3_8h_Mean) == 1)
	{
//		AirQuality_Level.NH3_AQ = 3;
		AirQuality_Level.NH3_AQ = CalcAQI(NH3_Moderate, NH3_Poor, NH3_8h_Mean, 3, 4);
	} else
	if (InRange(NH3_Poor, NH3_VeryPoor, NH3_8h_Mean) == 1)
	{
//		AirQuality_Level.NH3_AQ = 4;
		AirQuality_Level.NH3_AQ = CalcAQI(NH3_Poor, NH3_VeryPoor, NH3_8h_Mean, 4, 5);
	} else
	if (InRange(NH3_Poor, NH3_VeryPoor, NH3_8h_Mean) == 2)
	{
//		AirQuality_Level.NH3_AQ = 5;
		AirQuality_Level.NH3_AQ = CalcAQI(NH3_Poor, NH3_VeryPoor, NH3_8h_Mean, 5, 6);
	}

	//Find the 8h average O3 AirQuality level
	// O3_Excellent = 33,	//ug/m3
	// O3_Fine = 50,		//ug/m3
	// O3_Moderate = 80,	//ug/m3
	// O3_Poor = 120,		//ug/m3
	// O3_VeryPoor = 160	//ug/m3
	if (InRange(0, O3_Exellent, O3_8h_Mean) == 1)
	{
//		AirQuality_Level.O3_AQ = 0;
		AirQuality_Level.O3_AQ = CalcAQI(0, O3_Exellent, O3_8h_Mean, 0, 1);
	} else
	if (InRange(O3_Exellent, O3_Fine, O3_8h_Mean) == 1)
	{
//		AirQuality_Level.O3_AQ = 1;
		AirQuality_Level.O3_AQ = CalcAQI(O3_Exellent, O3_Fine, O3_8h_Mean, 1, 2);
	} else
	if (InRange(O3_Fine, O3_Moderate, O3_8h_Mean) == 1)
	{
//		AirQuality_Level.O3_AQ = 2;
		AirQuality_Level.O3_AQ = CalcAQI(O3_Fine, O3_Moderate, O3_8h_Mean, 2, 3);
	} else
	if (InRange(O3_Moderate, O3_Poor, O3_8h_Mean) == 1)
	{
//		AirQuality_Level.O3_AQ = 3;
		AirQuality_Level.O3_AQ = CalcAQI(O3_Moderate, O3_Poor, O3_8h_Mean, 3, 4);
	} else
	if (InRange(O3_Poor, O3_VeryPoor, O3_8h_Mean) == 1)
	{
//		AirQuality_Level.O3_AQ = 4;
		AirQuality_Level.O3_AQ = CalcAQI(O3_Poor, O3_VeryPoor, O3_8h_Mean, 4, 5);
	} else
	if (InRange(O3_Poor, O3_VeryPoor, O3_8h_Mean) == 2)
	{
//		AirQuality_Level.O3_AQ = 5;
		AirQuality_Level.O3_AQ = CalcAQI(O3_Poor, O3_VeryPoor, O3_8h_Mean, 5, 6);
	}

	//Find the 1h average SO2 AirQuality level
	// SO2_Excellent = 25,		//ug/m3
	// SO2_Fine = 100,			//ug/m3
	// SO2_Moderate = 200,		//ug/m3
	// SO2_Poor = 350,			//ug/m3
	// SO2_VeryPoor = 500,		//ug/m3
	if (InRange(0, SO2_Exellent, SO2_1h_Mean) == 1)
	{
//		AirQuality_Level.SO2_AQ = 0;
		AirQuality_Level.SO2_AQ = CalcAQI(0, SO2_Exellent, SO2_1h_Mean, 0, 1);
	} else
	if (InRange(SO2_Exellent, SO2_Fine, SO2_1h_Mean) == 1)
	{
//		AirQuality_Level.SO2_AQ = 1;
		AirQuality_Level.SO2_AQ = CalcAQI(SO2_Exellent, SO2_Fine, SO2_1h_Mean, 1, 2);
	} else
	if (InRange(SO2_Fine, SO2_Moderate, SO2_1h_Mean) == 1)
	{
//		AirQuality_Level.SO2_AQ = 2;
		AirQuality_Level.SO2_AQ = CalcAQI(SO2_Fine, SO2_Moderate, SO2_1h_Mean, 2, 3);
	} else
	if (InRange(SO2_Moderate, SO2_Poor, SO2_1h_Mean) == 1)
	{
//		AirQuality_Level.SO2_AQ = 3;
		AirQuality_Level.SO2_AQ = CalcAQI(SO2_Moderate, SO2_Poor, SO2_1h_Mean, 3, 4);
	} else
	if (InRange(SO2_Poor, SO2_VeryPoor, SO2_1h_Mean) == 1)
	{
//		AirQuality_Level.SO2_AQ = 4;
		AirQuality_Level.SO2_AQ = CalcAQI(SO2_Poor, SO2_VeryPoor, SO2_1h_Mean, 4, 5);
	} else
	if (InRange(SO2_Poor, SO2_VeryPoor, SO2_1h_Mean) == 2)
	{
//		AirQuality_Level.SO2_AQ = 5;
		AirQuality_Level.SO2_AQ = CalcAQI(SO2_Poor, SO2_VeryPoor, SO2_1h_Mean, 5, 6);
	}

	//Find the 24h average C6H6 AirQuality level
	// C6H6_Excellent = 1,		//ug/m3
	// C6H6_Fine = 1,			//ug/m3
	// C6H6_Moderate = 2,		//ug/m3
	// C6H6_Poor = 5,			//ug/m3
	// C6H6_VeryPoor = 6,		//ug/m3
	if (InRange(0, C6H6_Exellent, C6H6_24h_Mean) == 1)
	{
//		AirQuality_Level.C6H6_AQ = 0;
		AirQuality_Level.C6H6_AQ = CalcAQI(0, C6H6_Exellent, C6H6_24h_Mean, 0, 1);
	} else
	if (InRange(C6H6_Exellent, C6H6_Fine, C6H6_24h_Mean) == 1)
	{
//		AirQuality_Level.C6H6_AQ = 1;
		AirQuality_Level.C6H6_AQ = CalcAQI(C6H6_Exellent, C6H6_Fine, C6H6_24h_Mean, 1, 2);
	} else
	if (InRange(C6H6_Fine, C6H6_Moderate, C6H6_24h_Mean) == 1)
	{
//		AirQuality_Level.C6H6_AQ = 2;
		AirQuality_Level.C6H6_AQ = CalcAQI(C6H6_Fine, C6H6_Moderate, C6H6_24h_Mean, 2, 3);
	} else
	if (InRange(C6H6_Moderate, C6H6_Poor, C6H6_24h_Mean) == 1)
	{
//		AirQuality_Level.C6H6_AQ = 3;
		AirQuality_Level.C6H6_AQ = CalcAQI(C6H6_Moderate, C6H6_Poor, C6H6_24h_Mean, 3, 4);
	} else
	if (InRange(C6H6_Poor, C6H6_VeryPoor, C6H6_24h_Mean) == 1)
	{
//		AirQuality_Level.C6H6_AQ = 4;
		AirQuality_Level.C6H6_AQ = CalcAQI(C6H6_Poor, C6H6_VeryPoor, C6H6_24h_Mean, 4, 5);
	} else
	if (InRange(C6H6_Poor, C6H6_VeryPoor, C6H6_24h_Mean) == 2)
	{
//		AirQuality_Level.C6H6_AQ = 5;
		AirQuality_Level.C6H6_AQ = CalcAQI(C6H6_Poor, C6H6_VeryPoor, C6H6_24h_Mean, 5, 6);
	}

	//Find the overall air quality index due to average values of gaseous pollutants
	AirQuality_Level.AvgGasAirQualityIndex =  (uint8_t)(max(9, (int32_t)AirQuality_Level.eTVOC_AQ, (int32_t)AirQuality_Level.eCO2_AQ,
														 	(int32_t)AirQuality_Level.CH2O_AQ, (int32_t)AirQuality_Level.CO_AQ,
															(int32_t)AirQuality_Level.NO2_AQ, (int32_t)AirQuality_Level.NH3_AQ,
															(int32_t)AirQuality_Level.O3_AQ, (int32_t)AirQuality_Level.SO2_AQ,
															(int32_t)AirQuality_Level.C6H6_AQ));
	strcpy(AirQuality_Level.AvgGasAirQualityClass, AQ_Class[AirQuality_Level.AvgGasAirQualityIndex]);

	if (AirQuality_Level.AvgGasAirQualityIndex <= AirQuality_Level.GasAirQualityIndex)	//The BLE Tx AQI is always the lower between..
		AVG_Gas_AQI =  AirQuality_Level.AvgGasAirQualityIndex;	//..the average...
	else
		AVG_Gas_AQI = AirQuality_Level.GasAirQualityIndex;	//...and the instantaneous one. AVG_Gas_AQI Used in BLE Beacon packet

	//Find the 24h average PM10 AirQuality level
	// PM10_Excellent = 20,	//ug/m3
	// PM10_Fine = 35,		//ug/m3
	// PM10_Moderate = 50,	//ug/m3
	// PM10_Poor = 100,		//ug/m3
	// PM10_VeryPoor = 150,	//ug/m3
	if (InRange(0, PM10_Exellent, MC_10p0_24h_Mean) == 1)
	{
		AirQuality_Level.PM10_AQ = 0;
	} else
	if (InRange(PM10_Exellent, PM10_Fine, MC_10p0_24h_Mean) == 1)
	{
		AirQuality_Level.PM10_AQ = 1;
	} else
	if (InRange(PM10_Fine, PM10_Moderate, MC_10p0_24h_Mean) == 1)
	{
		AirQuality_Level.PM10_AQ = 2;
	} else
	if (InRange(PM10_Moderate, PM10_Poor, MC_10p0_24h_Mean) == 1)
	{
		AirQuality_Level.PM10_AQ = 3;
	} else
	if (InRange(PM10_Poor, PM10_VeryPoor, MC_10p0_24h_Mean) == 1)
	{
		AirQuality_Level.PM10_AQ = 4;
	} else
	if (InRange(PM10_Poor, PM10_VeryPoor, MC_10p0_24h_Mean) == 2)
	{
		AirQuality_Level.PM10_AQ = 5;
	}

	//Find the 24h average PM2.5 AirQuality level
	// PM2p5_Exellent = 10,	//ug/m3
	// PM2p5_Fine = 20,		//ug/m3
	// PM2p5_Moderate = 25,	//ug/m3
	// PM2p5_Poor = 50,		//ug/m3
	// PM2p5_VeryPoor = 110	//ug/m3
	if (InRange(0, PM2p5_Exellent, MC_2p5_24h_Mean) == 1)
	{
		AirQuality_Level.PM2p5_AQ = 0;
	} else
	if (InRange(PM2p5_Exellent, PM2p5_Fine, MC_2p5_24h_Mean) == 1)
	{
		AirQuality_Level.PM2p5_AQ = 1;
	} else
	if (InRange(PM2p5_Fine, PM2p5_Moderate, MC_2p5_24h_Mean) == 1)
	{
		AirQuality_Level.PM2p5_AQ = 2;
	} else
	if (InRange(PM2p5_Moderate, PM2p5_Poor, MC_2p5_24h_Mean) == 1)
	{
		AirQuality_Level.PM2p5_AQ = 3;
	} else
	if (InRange(PM2p5_Poor, PM2p5_VeryPoor, MC_2p5_24h_Mean) == 1)
	{
		AirQuality_Level.PM2p5_AQ = 4;
	} else
	if (InRange(PM2p5_Poor, PM2p5_VeryPoor, MC_2p5_24h_Mean) == 2)
	{
		AirQuality_Level.PM2p5_AQ = 5;
	}

	//Find the overall air quality index due to 1h average of PMx pollutants
	if (AirQuality_Level.PM10_AQ < AirQuality_Level.PM2p5_AQ)
	{
		AirQuality_Level.AvgPMxAirQualityIndex = AirQuality_Level.PM2p5_AQ;
	}
	else
	{
		AirQuality_Level.AvgPMxAirQualityIndex = AirQuality_Level.PM10_AQ;
	}
	strcpy(AirQuality_Level.AvgPMxAirQualityClass, AQ_Class[AirQuality_Level.AvgPMxAirQualityIndex]);
	AVG_PMx_AQI = AirQuality_Level.AvgPMxAirQualityIndex;	//Used in BLE Beacon packet

	if((AirQuality_Level.GasAirQualityIndex > 3) || (AnlgOvflStatusReg))
	{
		RiskReport = true;
		BIT_SET(Gas_AQI,7);			//Used in BLE Beacon packet
		BIT_SET(AVG_Gas_AQI,7);		//Used in BLE Beacon packet
	}
	else
	{
		RiskReport = false;
		BIT_CLEAR(Gas_AQI,7);		//Used in BLE Beacon packet
		BIT_CLEAR(AVG_Gas_AQI,7);	//Used in BLE Beacon packet
	}

	return AirQuality_Level;
}
