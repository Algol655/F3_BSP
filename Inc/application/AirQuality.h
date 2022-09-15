/**
 ******************************************************************************
 * @file    AirQuality.c
 * @brief   This file contains the air quality estimate headers
 *
 * @author  Tommaso Sabatini, 2020
 ******************************************************************************
 */

#ifndef APPLICATION_AIRQUALITY_H_
#define APPLICATION_AIRQUALITY_H_

#include "platform/port.h"

typedef struct
{
	// Air Quality Index of single gaseous pollutants
	uint8_t eTVOC_AQ;
	uint8_t eCO2_AQ;
	uint8_t CH2O_AQ;
	uint8_t CO_AQ;
	uint8_t NO2_AQ;
	uint8_t NH3_AQ;
	uint8_t O3_AQ;
	uint8_t SO2_AQ;
	uint8_t C6H6_AQ;
	// Air Quality Index of single PMx pollutants
	uint8_t PM10_AQ;
	uint8_t PM2p5_AQ;
	// Overall Air Quality Index from gaseous pollutants
	uint8_t GasAirQualityIndex;		//GAS Air quality index calculated on current values
	uint8_t AvgGasAirQualityIndex;	//GAS Air quality index calculated on 1h average values
	// Overall Air Quality Index from PMx pollutants
	uint8_t PMxAirQualityIndex;		//PMx Air quality index calculated on current values
	uint8_t AvgPMxAirQualityIndex;	//PMx Air quality index calculated on 1h average values
	// AQI Classification
	char GasAirQualityClass[16];	//Current GAS Air quality classification
	char AvgGasAirQualityClass[16];	//1 hour average GAS Air quality classification
	char PMxAirQualityClass[16];	//Current PMx Air quality classification
	char AvgPMxAirQualityClass[16];	//1 hour average PMx Air quality classification
} AirQualityParameters_st;

typedef enum
{
	eTVOC_Exellent = 65,	//ppb
	eTVOC_Good = 220,		//ppb
	eTVOC_Moderate = 660,	//ppb
	eTVOC_Poor = 2200,		//ppb
	eTVOC_Unhealty = 5500,	//ppb
	//
	eCO2_FreshAir = 450,	//ppm
	eCO2_Normal = 1832,		//ppm = 6.5*eTVOC_Good + 402
	eCO2_Acceptable = 4692,	//ppm = 6.5*eTVOC_Moderate + 402
	eCO2_Drowsiness = 14702,//ppm = 6.5*eTVOC_Poor + 402
	eCO2_Harmful = 36152,	//ppm = 6.5*eTVOC_Unhealty + 402
	//
	CH2O_Exellent = 20,		//ug/m3
	CH2O_Fine = 40,			//ug/m3
	CH2O_Moderate = 50,		//ug/m3
	CH2O_Poor = 120,		//ug/m3
	CH2O_VeryPoor = 124,	//ug/m3
	//
	CO_Exellent = 2,		//mg/m3
	CO_Fine = 4,			//mg/m3
	CO_Moderate = 7,		//mg/m3
	CO_Poor = 10,			//mg/m3
	CO_VeryPoor = 25,		//mg/m3
	//
	NO2_Exellent = 25,		//ug/m3
	NO2_Fine = 40,			//ug/m3
	NO2_Moderate = 100,		//ug/m3
	NO2_Poor = 200,			//ug/m3
	NO2_VeryPoor = 400,		//ug/m3
	//
	NH3_Exellent = 200,		//ug/m3
	NH3_Fine = 400,			//ug/m3
	NH3_Moderate = 800,		//ug/m3
	NH3_Poor = 1200,		//ug/m3
	NH3_VeryPoor = 1800,	//ug/m3
	//
	O3_Exellent = 33,		//ug/m3
	O3_Fine = 50,			//ug/m3
	O3_Moderate = 80,		//ug/m3
	O3_Poor = 120,			//ug/m3
	O3_VeryPoor = 160,		//ug/m3
	//
	SO2_Exellent = 25,		//ug/m3
	SO2_Fine = 100,			//ug/m3
	SO2_Moderate = 200,		//ug/m3
	SO2_Poor = 350,			//ug/m3
	SO2_VeryPoor = 500,		//ug/m3
	//
	C6H6_Exellent = 1,		//ug/m3
	C6H6_Fine = 1,			//ug/m3
	C6H6_Moderate = 2,		//ug/m3
	C6H6_Poor = 5,			//ug/m3
	C6H6_VeryPoor = 6,		//ug/m3
	//
	PM10_Exellent = 20,		//ug/m3
	PM10_Fine = 35,			//ug/m3
	PM10_Moderate = 50,		//ug/m3
	PM10_Poor = 100,		//ug/m3
	PM10_VeryPoor = 150,	//ug/m3
	//
	PM2p5_Exellent = 10,	//ug/m3
	PM2p5_Fine = 20,		//ug/m3
	PM2p5_Moderate = 25,	//ug/m3
	PM2p5_Poor = 50,		//ug/m3
	PM2p5_VeryPoor = 110	//ug/m3
} AQ_LimitsParameters_st;

AQ_LimitsParameters_st AQ_Limits;

//Exported Functions
AirQualityParameters_st AirQuality(uint16_t eq_TVOC, uint16_t eq_CO2, uint16_t eq_TVOC_1h_Mean, uint16_t eq_CO2_8h_Mean,
								   uint16_t CH2O, uint16_t CO, uint16_t NO2, uint16_t NH3,
								   uint16_t O3, uint16_t SO2, uint16_t C6H6,
								   uint16_t CH2O_8h_Mean, uint16_t CO_8h_Mean, uint16_t NO2_1h_Mean, uint16_t NH3_8h_Mean,
								   uint16_t O3_8h_Mean, uint16_t SO2_1h_Mean, uint16_t C6H6_24h_Mean,
								   uint16_t MC_10p0_24h_Mean, uint16_t MC_2p5_24h_Mean);
#endif /* APPLICATION_AIRQUALITY_H_ */
