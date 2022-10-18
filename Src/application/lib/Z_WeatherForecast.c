/**
 ******************************************************************************
 * @file    Weather_Forecast.c
 * @brief   This file contains the Zambretti weather forecast functions
 *
 * @author  Tommaso Sabatini, 2021
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "application/Z_WeatherForecast.h"

uint8_t Weather_Forecast(float PressVal, float TempVal, uint8_t HumVal)
{
	static Z_WheatherParameters_st Wheather_Values =
					{.PreviousPressVal = 1023.0, .CurrentPressVal=1023.0, .WeatherF_DeltaPressVal=0.0,
			         .PreviousTempVal=25.0, .CurrentTempVal=25.0, .DeltaTempVal=0.0, .PreviousHumVal=50,
					 .CurrentHumVal=50, .DeltaHumVal=0, .DeltaTime=0,
					 .ForecastEstimate=0, .PrevForecastEstimate=0, .Z_ForecastEstimate=0};
	static int16_t WeatherF_DeltaP = 0;
	static uint16_t CountTick = 0;
	static uint8_t Z_Value = 1;
	static int8_t SeasonAdjust = 0;
	static int8_t WindDirAdjust = 0;
	const uint8_t Z_Symbols[32] = {SunnyDWN,SunnyDWN,CloudyDWN,CloudyDWN,CloudyDWN,RainyDWN,RainyDWN,RainyDWN,RainyDWN,
				SunnySTDY,SunnySTDY,CloudySTDY,CloudySTDY,CloudySTDY,RainySTDY,RainySTDY,RainySTDY,RainySTDY,StormySTDY,
				SunnyUP,SunnyUP,SunnyUP,CloudyUP,CloudyUP,CloudyUP,CloudyUP,CloudyUP,CloudyUP,RainyUP,RainyUP,StormyUP,StormyUP};
	extern FLASH_DATA_ORG FlashDataOrg;
	static uint8_t Month;
//	static uint8_t delta = 0;	//Only for debug purpose!

//	PressVal += delta;			//Only for debug purpose!
//	Month = (uint8_t)((FlashDataOrg.b_date) & 0x000000FF);	//Only for debug purpose!
	if (CountTick == 0)
	{	//The counter is zero after a reset or after a power cycle, so we have (and we can)
		//to restore here the weather values used by the WeatherForecast function
		//Use the last Weather status after a reset or after a power cycle
		WeatherF_DeltaP = (int16_t)(FlashDataOrg.b_status.s6 & 0x0000FFFF);
		forecast = Wheather_Values.ForecastEstimate = (uint8_t)((FlashDataOrg.b_status.s6 >> 16) & 0x000000FF);
		Z_forecast = Wheather_Values.Z_ForecastEstimate = (uint8_t)((FlashDataOrg.b_status.s6 >> 24) & 0x000000FF);
		CountTick++;	//So we enter this branch only once
	} else
	if (++CountTick > DELTA_T)
	{
//		delta++;				//Only for debug purpose!
		Wheather_Values.PreviousPressVal = Wheather_Values.CurrentPressVal;
		Wheather_Values.CurrentPressVal = PressVal;
		Wheather_Values.WeatherF_DeltaPressVal = Wheather_Values.PreviousPressVal - Wheather_Values.CurrentPressVal;
		Wheather_Values.PreviousTempVal = Wheather_Values.CurrentTempVal;
		Wheather_Values.CurrentTempVal = TempVal;
		Wheather_Values.DeltaTempVal = Wheather_Values.DeltaTempVal - Wheather_Values.PreviousTempVal;
		Wheather_Values.PreviousHumVal = Wheather_Values.CurrentHumVal;
		Wheather_Values.CurrentHumVal = HumVal;
		Wheather_Values.DeltaHumVal = Wheather_Values.CurrentHumVal - Wheather_Values.PreviousHumVal;
		CountTick = 1;
		WeatherF_DeltaP = (int16_t)lrintf(Wheather_Values.WeatherF_DeltaPressVal);
		Month = (uint8_t)((FlashDataOrg.b_date) & 0x000000FF);
		if (((Month >= 1) && (Month < 4)) || (Month = 12))	//If Winter season
		{
			SeasonAdjust = 1;
		} else
		if ((Month > 5) && (Month < 10))	//If Summer season
		{
			SeasonAdjust = -1;
		} else
		{
			SeasonAdjust = 0;
		}

		Wheather_Values.PrevForecastEstimate = Wheather_Values.ForecastEstimate;

		if ((WeatherF_DeltaP > 0) && (WeatherF_DeltaP*10 >= DropFallingPressure))
		{	//Barometric Pressure decreasing
			Z_Value = (uint8_t)(lrintf(Z_FALLING(Wheather_Values.CurrentPressVal)) + SeasonAdjust + WindDirAdjust);
		} else	//Barometric Pressure increasing
		if  ((WeatherF_DeltaP < 0) && ((abs(WeatherF_DeltaP))*10 >= DropRisingPressure))
		{
			Z_Value = (uint8_t)(lrintf(Z_RISING(Wheather_Values.CurrentPressVal)) + SeasonAdjust + WindDirAdjust);
		} else	//Steady Barometric pressure
		{
			Z_Value = (uint8_t)(lrintf(Z_STEADY(Wheather_Values.CurrentPressVal)) + SeasonAdjust + WindDirAdjust);
		}
		if (Z_Value == 0)
			Z_Value = 1;
		if (Z_Value > 32)
			Z_Value = 32;
		Z_forecast = Wheather_Values.Z_ForecastEstimate = Z_Value;
		forecast = Wheather_Values.ForecastEstimate = Z_Symbols[Z_Value-1];
		//Stores the weather values in the board data structure.
		//They will be flashed within one hour, when the timeout expires in the
		//HAL_RTCEx_RTCEventCallback function in port.c
		FlashDataOrg.b_status.s6 = ((uint32_t)(Wheather_Values.Z_ForecastEstimate & 0xFF) << 24) |
								   ((uint32_t)(Wheather_Values.ForecastEstimate & 0xFF) << 16) |
								   ((uint32_t)(WeatherF_DeltaP & 0xFFFF));

	}

	return Wheather_Values.ForecastEstimate;
}
