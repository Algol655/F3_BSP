/*
 * ANLG_Driver.h
 *
 *  Created on: 10 apr 2021
 *      Author: Tommaso Sabatini
 */

#ifndef PLATFORM_ANLG_DRIVER_H_
#define PLATFORM_ANLG_DRIVER_H_

// User defines starts here
/**
* @}brief Gases Sensors Board HW Version definess.
*/
//#define GSB_HW_VER			(1U)		//Gas Sensor Board HW Version 0.1
#define GSB_HW_VER				(10U)		//Gas Sensor Board HW Version 1.0
//#define GSB_HW_VER			(20U)		//Gas Sensor Board HW Version 2.0
//#define GSB_HW_VER			(21U)		//Gas Sensor Board HW Version 2.1

#define USE_SMD1001_MODIFIED_CURVE	(2U)
#if (USE_SMD1001_MODIFIED_CURVE == 1)
	//SMD1001_CH2O sensor expanded Vs/Vo-ppm relationship: y(ppm) = -2.45723e-4(Vs/Vo)^3 + 8.29257e-3(Vs/Vo)^2 - -8.83605e-4(Vs/Vo) - 6.27613e-4 (10x modified curve following field trials)
	#define SMD1001_CH2O(x)	((-2.45723e-4F * pow(x, 3.0F)) + (8.29257e-3F * pow(x, 2.0F)) + (-8.83605e-4F * x) - 6.27613e-4F)
#elif (USE_SMD1001_MODIFIED_CURVE == 2)
	//SMD1001_CH2O sensor expanded Vs/Vo-ppm relationship: y(ppm) = -3.07154e-5(Vs/Vo)^3 + 1.9810e-3(Vs/Vo)^2 + 3.61234e-3(Vs/Vo) + 9.73013e-4 (20x modified curve following field trials)
	#define SMD1001_CH2O(x)	((-3.07154e-5F * pow(x, 3.0F)) + (1.9810e-3F * pow(x, 2.0F)) + (3.61234e-3F * x) + 9.73013e-4F)
#else
	//SMD1001_CH2O sensor Vs/Vo-ppm relationship: y(ppm) = -2.45723e-1(Vs/Vo)^3 + 1,49271(Vs/Vo)^2 - 2,09865(Vs/Vo) + 8.58155e-1 (See sensor data sheet)
	#define SMD1001_CH2O(x)	((-2.45723e-1F * pow(x, 3.0F)) + (1.49271F * pow(x, 2.0F)) + (-2.09861F * x) + 8.58155e-1F)
#endif
//SMD1001_CH2O sensor temperature compensation: y((Vs/Vo)/(Vs/Vo at 20°C)) = -0,00000456232(T)^3 + 0,000345486(T)^2 - 0,016034(T) + 1,22874 (see data sheet)
#define SMD1001_CH2O_TC(x)	((-4.56232e-6F * pow(x, 3.0F)) + (3.45486e-4F * pow(x, 2.0F)) + (-1.6034e-2F * x) + 1.22874)
//SMD1001_CH2O sensor humidity compensation: y((Vs/Vo)/(Vs/Vo at 60%RH)) = 0,000022082(RH)^2 - 0,00732913(RH) + 1,36025 (see data sheet)
#define SMD1001_CH2O_RHC(x)	((2.2082e-5F * pow(x, 2.0F)) + (-7.32913e-3F * x) + 1.36025)

#define ZE08_CH2O(x)	((3.125F * x) - 1.25F)	//y(ppm) = 3.125(Vadc) - 1.25
#define CH2O_MOL_WEIGHT	30.026F //Formaldehyde Molecular weight, g/mol
#define CH2O_ppm2ugm3(x)	((CH2O_MOL_WEIGHT * x * 1000.0F)/24.45F)	//ppm to ug/m3 CH2O conversion
#define ZE08_CH2O_TC1(x)	((0.0225F * x) + 0.7F)		//T < 0°C ZE08_CH2O sensor temperature compensation
#define ZE08_CH2O_TC2(x)	((0.012F * x) + 0.7F)		//T < 50°C, T > 0°C ZE08_CH2O sensor temperature compensation
#define ZE08_CH2O_RESOLUTION	0.01F		//The ZEH8 CH2O sensor resolution is 0.01ppm

#define ZE25_O3(x)		((6.2375F * x) - 2.475F)	//y(ppm) = 6.25(Vadc) - 2.5
#define O3_MOL_WEIGHT	48.0F	//Ozone Molecular weight, g/mol
#define O3_ppm2ugm3(x)	((O3_MOL_WEIGHT * x * 1000.0F)/24.45F)	//ppm to ug/m3 O3 conversion
#define ZE25_O3_TC1(x)	((0.012F * x) + 0.94F)		//T > -20°C, T < 5°C ZE25_O3 sensor temperature compensation
#define ZE25_O3_TC2(x)	((0.0F * x) + 1.0F)			//T > 5°C, T < 20°C ZE25_O3 sensor temperature compensation
#define ZE25_O3_TC3(x)	((-0.0025F * x) + 1.05F)	//T > 20°C ZE25_O3 sensor temperature compensation
#define ZE25_O3_RESOLUTION	0.01F		//The ZE25 O3 sensor data sheet resolution is 0.02ppm, but the circuit implemented
										//in Sensus191 extends the resolution to 0.01ppm.
#if (GSB_HW_VER == 10)
	#define SO2_RGAIN		392000.0F
#elif (GSB_HW_VER >= 20)
	#define SO2_RGAIN		(47000.0F * 10.0F)
#endif
#define ME4_SO2_SENSITIVITY	8e-7F	// 0.8uA/ppm
#define ME4_SO2(x)		(x / (ME4_SO2_SENSITIVITY * SO2_RGAIN))	//y(ppm) = x(Vadc) / (ME4_SO2_Sensitivity * SO2_RGAIN)
#define SO2_MOL_WEIGHT	64.06F	//Sulfur Dioxide Molecular weight, g/mol
#define SO2_ppm2ugm3(x)	((SO2_MOL_WEIGHT * x * 1000.0F)/24.45F)	//ppm to ug/m3 SO2 conversion
#define ME4_SO2_TC(x)	((-0.000023163F * pow(x, 2.0F)) + (0.0052F * x) + 0.9159F)	//ME4_SO2 sensor temperature compensation
#define ME4_SO2_ZC1(x)	((0.00029023F * x) + 0.019987F)			//ME4_SO2 sensor zero output temperature compensation <= 0°C
#define ME4_SO2_ZC2(x)	((-0.0010939F * x) + 0.021877F)			//ME4_SO2 sensor zero output temperature compensation from 0°C to 20°C
#define ME4_SO2_ZC3(x)	((0.0019812F * x) - 0.039623F)			//ME4_SO2 sensor zero output temperature compensation from 20°C to 40°C
#define ME4_SO2_ZC4(x)	((0.017042F * x) - 0.6423F)				//ME4_SO2 sensor zero output temperature compensation > 40°C
#define ME4_SO2_RESOLUTION	0.01F		//The ME4_SO2 sensor data sheet resolution is 0.1ppm, but the circuit implemented
										//in Sensus191 extends the resolution to 0.01ppm.
#if (GSB_HW_VER == 10)
	#define SO2_RGAIN		392000.0F
#elif (GSB_HW_VER >= 20)
	#define NO2_RGAIN		(47000.0F * 10.0F)
#endif
#define ME4_NO2_SENSITIVITY	12e-7F	// 12uA/ppm
#define ME4_NO2(x)		(x / (ME4_NO2_SENSITIVITY * NO2_RGAIN))	//y(ppm) = x(Vadc) / (ME4_NO2_Sensitivity * NO2_RGAIN)
#define NO2_MOL_WEIGHT	46.01F	//Nitrogen Dioxide Molecular weight, g/mol
#define NO2_ppm2ugm3(x)	((NO2_MOL_WEIGHT * x * 1000.0F)/24.45F)	//ppm to ug/m3 SO2 conversion
#define ME4_NO2_TC(x)	((0.0045649F * x) + 0.90993F)			//ME4_NO2 sensor temperature compensation
//#define ME4_NO2_ZC1(x)	((-0.00061106F * x) + 0.011839F)		//ME4_NO2 sensor zero output temperature compensation <= 0°C
//#define ME4_NO2_ZC2(x)	((0.0032845F * x) + 0.011839F)			//ME4_NO2 sensor zero output temperature compensation from 0°C to 20°C
//#define ME4_NO2_ZC3(x)	((-0.0017759F * x) + 0.11305F)			//ME4_NO2 sensor zero output temperature compensation from 20°C to 40°C
//#define ME4_NO2_ZC4(x)	((0.022075F * x) - 0.84098F)			//ME4_NO2 sensor zero output temperature compensation > 40°C
#define ME4_NO2_ZC1(x)	0.0										//ME4_NO2 sensor zero output temperature compensation <= 0°C
#define ME4_NO2_ZC2(x)	0.0										//ME4_NO2 sensor zero output temperature compensation from 0°C to 20°C
#define ME4_NO2_ZC3(x)	0.0										//ME4_NO2 sensor zero output temperature compensation from 20°C to 40°C
#define ME4_NO2_ZC4(x)	((0.018522F * x) - 0.74092F)			//ME4_NO2 sensor zero output temperature compensation > 40°C
#define ME4_NO2_RESOLUTION	0.01F		//The ME4_NO2 sensor data sheet resolution is 0.1ppm, the circuit implemented
										//in Sensus191 extends the resolution to 0.01ppm
#if (GSB_HW_VER == 10)
	#define C6H6_RGAIN		1e8F
#elif (GSB_HW_VER >= 20)
	#define C6H6_RGAIN		(47000.0F * 10.0F)
#endif
#define ME4_C6H6_SENSITIVITY	3e-10F	// 0.3nA/ppm
#define ME4_C6H6(x)		(x / (ME4_C6H6_SENSITIVITY * C6H6_RGAIN))	//y(ppm) = x(Vadc) / (ME4_C6H6_Sensitivity * C6H6_RGAIN)
#define C6H6_MOL_WEIGHT	78.11F	//Benzene Molecular weight, g/mol
#define C6H6_ppm2ugm3(x)	((C6H6_MOL_WEIGHT * x * 1000.0F)/24.45F)	//ppm to ug/m3 C6H6 conversion
#define ME4_C6H6_TC(x)	((0.0048F * x) + 0.88F)		//ME4_C6H6 sensor temperature compensation

#define CO_RGAIN		(47000.0F * 10.0F)
#define ME4_CO_SENSITIVITY	8e-8F	// 0.08 uA/ppm
#define ME4_CO(x)		(x / (ME4_CO_SENSITIVITY * CO_RGAIN))	//y(ppm) = x(Vadc) / (ME4_CO_Sensitivity * CO_RGAIN)
#define CO_MOL_WEIGHT	28.01F	//Carbon monoxide Molecular weight, g/mol
#define CO_ppm2mgm3(x)	((CO_MOL_WEIGHT * x)/24.45F)			//ppm to mg/m3 CO conversion
#define ME4_CO_TC(x)	((0.0074681F * x) + 0.84955F)			//ME4_CO sensor temperature compensation.
#define ME4_CO_ZC1(x)	((0.00025446F * x) + 0.015264F)			//ME4_CO sensor zero output temperature compensation <= 0°C
/*
 * It seems that the zero output temperature compensation curve reported in the datasheet does not correspond to the behavior
 * of the real sensor.
 * A better approximation of the real behavior is obtained by extending the validity of the ME4_CO_ZC2 correction up to 40°C
 * and starting the validity of the ME4_CO_ZC3 correction at 40°C.
 */
#define ME4_CO_ZC2(x)	((0.00036926F * x) + 0.020353F)			//ME4_CO sensor zero output temperature compensation from 0°C to 20°C (to 40°C)
#define ME4_CO_ZC3(x)	((0.010933F * x) - 0.18837F)			//ME4_CO sensor zero output temperature compensation from > 20°C (> 40°C)
#define ME4_CO_RESOLUTION	1.0F		//The ME4_CO sensor data sheet resolution is 1ppm

/*
 * Correction Model for Metal Oxide Sensor Drift Caused by Ambient Temperature and Humidity
 * (MDPI Sensor Article: https://www.mdpi.com/1424-8220/22/9/3301)
 * Modified by me to make Rs*=Rs at test temperature and humidity. (T=23°C, RH=50%, MiCS-6814 Data Sheet, page 2)
 */
//#define MiCS_6814_TC(Rs,t,h) 	(10.2243F + (0.7495F * Rs) - (0.1953F * t) + (0.0672F * h) + (0.0016F * Rs * t) + (0.0081F * Rs * h))
#define MiCS_6814_TC(Rs,t,h) 	(10.2243F + (0.5582F * Rs) - (0.1953F * t) + (0.0672F * h) + (0.0016F * Rs * t) + (0.0081F * Rs * h))

//#define MiCS_6814_NO2_TC(Rs,t,h) 	(10.2243F + (0.7495F * Rs) - (0.1953F * t) + (0.0672F * h) + (0.0016F * Rs * t) + (0.0081F * Rs * h))
//#define MiCS_6814_NO2(x)	((0.1499F * x) + 0.0042F)			//y(ppm) = 0.1499(Rs/Ro) + 0.0042
#define MiCS_6814_NO2(x)	(0.1459F * (pow(x, 1.007F)))		//y(ppm) = 0.1459((Rs/Ro)*E(1.007))

//#define MiCS_6814_NH3_TC(Rs,t,h)	(10.2243F + (0.7495F * Rs) - (0.1953F * t) + (0.0672F * h) + (0.0016F * Rs * t) + (0.0081F * Rs * h))
//#define MiCS_6814_NH3(x)	(0.5908F * (pow(x, -1.918F)))		//y(ppm) = 0.5908((Rs/Ro)*E(-1.918))
#define MiCS_6814_NH3(x)	(0.6803F * (pow(x, -1.67F)))		//y(ppm) = 0.6803((Rs/Ro)*E(-1.67))
#define NH3_MOL_WEIGHT	17.03F	//Ammonia Molecular weight, g/mol
#define NH3_ppm2ugm3(x)	((NH3_MOL_WEIGHT * x * 1000.0F)/24.45F)	//ppm to ug/m3 NH3 conversion

//#define MiCS_6814_CO_TC(Rs,t,h)		(10.2243F + (0.7495F * Rs) - (0.1953F * t) + (0.0672F * h) + (0.0016F * Rs * t) + (0.0081F * Rs * h))
//#define MiCS_6814_CO(x)	(4.4922F * (pow(x, -1.182F)))		//y(ppm) = 4.4922((Rs/Ro)*E(-1.1182))
#define MiCS_6814_CO(x)	(4.385F * (pow(x, -1.179F)))			//y(ppm) = 4.385((Rs/Ro)*E(-1.179))

#define OVFL_TIMEOUT (100)		//Time that an analog channel can overflow before being notified
#define SMO_SENSOR_TC (1)		//If 1 the SMO sensors temperature/humidity compensation are calculated
#define ZE_SENSOR_TC (0)		//If 1 the EC-ZE sensors temperature/humidity compensation are calculated
#define EC_SENSOR_TC (1)		//If 1 the EC-ME sensors temperature/humidity compensation are calculated
#define NO2_FROM_EC	 (OUTDOOR_MODE && (GSB_HW_VER==20 || GSB_HW_VER==21))	//If 1 the nitrogen dioxide concentration is calculated from
								//the value detected by the Electrochemical sensor, instead of the Metal Oxide sensor.
#define CH2O_FROM_EC ((OUTDOOR_MODE && (GSB_HW_VER==20 || GSB_HW_VER==21)) || GSB_HW_VER==10)	//If 1 the formaldehyde concentration is
								//calculated from the value detected by the Electrochemical sensor, instead of the Metal Oxide sensor.
#define CO_FROM_EC	 (OUTDOOR_MODE && (GSB_HW_VER==20 || GSB_HW_VER==21))	//If 1 the carbon monoxide concentration is calculated from
								//the value detected by the Electrochemical sensor, instead of the Metal Oxide sensor.
//#define CO_FROM_EC	(0)		//Only for test!!!

/**
* @}brief Analog Board Gases Sensors Min Max Init Values.
*/
#define ANLG_UPPER_CO_LIMIT		(25*100)	//100 times the "CO_VeryPoor" threshold
#define ANLG_LOWER_CO_LIMIT		(0)
#define ANLG_UPPER_CH2O_LIMIT	(124*100)	//100 times the "CH2O_VeryPoor" threshold
#define ANLG_LOWER_CH2O_LIMIT	(0)
#define ANLG_UPPER_NO2_LIMIT	(400*100)	//100 times the "NO2_VeryPoor" threshold
#define ANLG_LOWER_NO2_LIMIT	(0)
#define ANLG_UPPER_NH3_LIMIT	(124*100)	//100 times the "NH3_VeryPoor" threshold
#define ANLG_LOWER_NH3_LIMIT	(0)
#define ANLG_UPPER_O3_LIMIT		(240*100)	//100 times the "O3_VeryPoor" threshold
#define ANLG_LOWER_O3_LIMIT		(0)
#define ANLG_UPPER_SO2_LIMIT	(500*100)	//100 times the "SO2_VeryPoor" threshold
#define ANLG_LOWER_SO2_LIMIT	(0)
#define ANLG_UPPER_C6H6_LIMIT	(6*100)		//100 times the "C6H6_VeryPoor" threshold
#define ANLG_LOWER_C6H6_LIMIT	(0)
// User defines ends here

// User include starts here
#include "port.h"
#include "adc.h"
// User include ends here

uint8_t ain1_values[num_anlg_mux_in], prev_ain1_values[num_anlg_mux_in];
uint8_t ain2_values[num_anlg_mux_in], prev_ain2_values[num_anlg_mux_in];
uint16_t adc_values[num_ad_chs];
float32_t converted_values[num_ad_chs], converted_value;
float32_t mux1_inputs[num_anlg_mux_in], mux2_inputs[num_anlg_mux_in];
float32_t CPU_Temp, Vsense, Vrsense;
uint8_t dec_values[num_anlg_mux_in][64];
uint8_t ovfl_1[num_anlg_mux_in][8], ovfl_2[num_anlg_mux_in][8];

typedef enum
{
	ANLG_OK	 = 0x00,
	ANLG_ERROR = 0x01
} ANLG_Error_et;

//Analog mux 1 -> 0..15 Inputs
typedef enum ain1_functions
{
	AIN_00_1_CH2O = 0,	//From Winsen ZE08-CH2O Formaldehyde Sensor
	AIN_01_1_O3,		//From Winsen ZE25-O3 Ozone Sensor
	AIN_02_1_NO2,		//From MICS_6814 Nitrogen Dioxide Sensor
	AIN_03_1_NH3,		//From MICS_6814 Ammonia Sensor
	AIN_04_1_CO,		//From MICS_6814 Carbon Monoxide Sensor
	AIN_05_1_SO2,		//From Winsen ME4-SO2 Sulfur Dioxide Sensor
	AIN_06_1_C6H6,		//From Winsen ME4-C6H6 Benzene Sensor
	AIN_07_1_,
	AIN_08_1_,
	AIN_09_1_,
	AIN_10_1_,
	AIN_11_1_,
	AIN_12_1_,
	AIN_13_1_,
	AIN_14_1_,
	AIN_15_1_
} AIN1_FUNCTION;

AIN1_FUNCTION Ain1_Function;

//Analog mux 2 -> 0..15 Inputs
typedef enum ain2_functions
{
	AIN_00_2_ = 0,
	AIN_01_2_,
	AIN_02_2_,
	AIN_03_2_,
	AIN_04_2_,
	AIN_05_2_,
	AIN_06_2_,
	AIN_07_2_,
	AIN_08_2_,
	AIN_09_2_,
	AIN_10_2_,
	AIN_11_2_,
	AIN_12_2_,
	AIN_13_2_,
	AIN_14_2_,
	AIN_15_2_
} AIN2_FUNCTION;

AIN2_FUNCTION Ain2_Function;

typedef struct
{
	float32_t CO;
	float32_t NO2;
	float32_t NH3;
	float32_t CH2O;
	float32_t O3;
	float32_t SO2;
	float32_t C6H6;
	float32_t AIN8;
	float32_t AIN9;
	float32_t AIN10;
	float32_t AIN11;
	float32_t AIN12;
	float32_t AIN13;
	float32_t AIN14;
	float32_t AIN15;
	float32_t AIN16;
	float32_t co_8h_mean;
	float32_t no2_1h_mean;
	float32_t nh3_8h_mean;
	float32_t ch2o_8h_mean;
	float32_t o3_1h_mean;
	float32_t so2_1h_mean;
	float32_t c6h6_24h_mean;
	uint16_t co_8h_mean_DailyMax;
	uint16_t no2_1h_mean_DailyMax;
	uint16_t nh3_8h_mean_DailyMax;
	uint16_t ch2o_8h_mean_DailyMax;
	uint16_t o3_1h_mean_DailyMax;
	uint16_t so2_1h_mean_DailyMax;
	uint16_t c6h6_24h_mean_DailyMax;
} ANLG_MeasureTypeDef_st;

extern const uint8_t CDC_delay;
extern const uint8_t UART_delay;
extern uint8_t L50_menu_items_row7a[];
extern uint8_t L50_menu_items_row7b[];
extern uint8_t L50_menu_items_row4a[];
extern uint8_t L50_menu_items_row4b[];
extern uint8_t L50_menu_items_C14_2[];
extern uint8_t L50_menu_items_C15_2[];

/** @defgroup Function_Prototypes
* @{
*/
void read_analogs(void);
ANLG_Error_et ANLG_Get_Measurement(ANLG_MeasureTypeDef_st *Measurement_Value);

#endif /* PLATFORM_ANLG_DRIVER_H_ */
