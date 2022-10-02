/**
 ******************************************************************************
 * @file    Weather_Forecast.h
 * @brief   This file contains the Zambretti weather forecast functions headers
 *
 * @author  Tommaso Sabatini, 2021
 ******************************************************************************
 */

#ifndef WHEATHER_FORECAST_H_
#define WHEATHER_FORECAST_H_

#include "platform/port.h"
#define DELTA_T		(uint16_t)3*3600/5	//3 hours between reads. 5s tick timer
//#define DELTA_T	(uint16_t)1			//Only for debug purpose!
#define Z_FALLING(p)	-0.12*(p) + 127	//P range: 985..1050 mBar
#define Z_STEADY(p)		-0.13*(p) + 144	//P range: 960..1033 mBar
#define Z_RISING(p)		-0.16*(p) + 185	//P range: 947..1030 mBar

typedef struct
{
	float PreviousPressVal;
	float CurrentPressVal;
	float WeatherF_DeltaPressVal;
	float PreviousTempVal;
	float CurrentTempVal;
	float DeltaTempVal;
	uint8_t PreviousHumVal;
	uint8_t CurrentHumVal;
	uint8_t DeltaHumVal;
	uint8_t DeltaTime;
	uint8_t ForecastEstimate;		//This value will be used on the local display
	uint8_t PrevForecastEstimate;
	uint8_t Z_ForecastEstimate;		//This value will be used on the remote display (BLE, LORA, etc.)
} Z_WheatherParameters_st;

typedef enum
{
	FallingLowPressureLimit = 985,
	FallingHighPressureLimit = 1050,
	SteadyLowPressureLimit = 960,
	SteadyHighPressureLimit = 1033,
	RisingLowPressureLimit = 947,
	RisingHighPressureLimit = 1030,
	DropFallingPressure = 16,		//Delta mBar * 10
	DropRisingPressure = 16,		//Delta mBar * 10
	SunnyUP = 0x00,					//bit 7 = 0 Pressure increasing;
	CloudyUP = 0x01,
	RainyUP = 0x02,
	StormyUP = 0x03,
	SunnySTDY = 0x40,				//bit 6 = 1 Pressure steady;
	CloudySTDY = 0x41,
	RainySTDY = 0x42,
	StormySTDY = 0x43,
	SunnyDWN = 0x80,				//bit 7 = 1 Pressure decreasing;
	CloudyDWN = 0x81,
	RainyDWN = 0x82,
	StormyDWN = 0x83
} Z_WF_Parameters_st;

Z_WF_Parameters_st P_Limits;

typedef enum
{
	/* Pressure Falling: Z = 130 – 0.123p */
	Settled_Fine_Falling = 0x01,					//SunnyDWN
	Fine_Weather_Falling = 0x02,					//SunnyDWN
	Fine_Becoming_Less_Settled = 0x03,				//CloudyDWN
	Fairly_Fine_Showery_Later = 0x04,				//CloudyDWN
	Showery_Becoming_More_Unsettled = 0x05,			//CloudyDWN
	Unsettled_Rain_Later = 0x06,					//RainyDWN
	Rain_at_Times_Worse_Later = 0x07,				//RainyDWN
	Rain_at_Times_Becoming_Very_Unsettled = 0x08,	//RainyDWN
	Very_Unsettled_Rain__Falling = 0x09,			//RainyDWN
	/* Pressure Steady: Z = 147 – 0.133p */
	Settled_Fine_Steady = 0x0A,						//SunnySTDY
	Fine_Weather_Steady = 0x0B,						//SunnySTDY
	Fine_Possibly_Showers = 0x0C,					//CloudySTDY
	Fairly_Fine_Showers_Likely = 0x0D,				//CloudySTDY
	Showery_Bright_Intervals = 0x0E,				//CloudySTDY
	Changeable_Some_Rain = 0x0F,					//RainySTDY
	Unsettled_Rain_at_Times = 0x10,					//RainySTDY
	Rain_at_Frequent_Intervals = 0x11,				//RainySTDY
	Very_Unsettled_Rain_Steady = 0x12,				//RainySTDY
	Stormy_Much_Rain_Steady = 0x13,					//StormySTDY
	/*Pressure Rising: Z = 179 – 0.156p */
	Settled_Fine_Rising = 0x14,						//SunnyUP
	Fine_Weather_Rising = 0x15,						//SunnyUP
	Becoming_Fine = 0x16,							//SunnyUP
	Fairly_Fine_Improving = 0x17,					//CloudyUP
	Fairly_Fine_Possibly_Showers_Early = 0x18,		//CloudyUP
	Showery_Early_Improving = 0x19,					//CloudyUP
	Changeable_Mending = 0x1A,						//CloudyUP
	Rather_Unsettled_Clearing_Later = 0x1B,			//CloudyUP
	Unsettled_Probably_Improving = 0x1C,			//CloudyUP
	Unsettled_Short_Fine_Intervals = 0x1D,			//RainyUP
	Very_Unsettled_Finer_at_Times = 0x1E,			//RainyUP
	Stormy_Possibly_Improving = 0x1F,				//StormyUP
	Stormy_Much_Rain_Rising = 0x20					//StormyUP
} Z_WF_Values_st;

//Exported Functions
uint8_t Weather_Forecast(float PressVal, float TempVal, uint8_t HumVal);

#endif /* WHEATHER_FORECAST_H_ */

