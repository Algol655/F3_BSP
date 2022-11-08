/*
 * ANLG_Driver.h
 *
 *  Created on: 10 apr 2021
 *      Author: Tommaso Sabatini
 */

#ifndef PLATFORM_ANLG_DRIVER_H_
#define PLATFORM_ANLG_DRIVER_H_

// User defines starts here
#define ZE08_CH2O(x)	((3.125F * x) - 1.25F)	//y(ppm) = 3.125(Vadc) - 1.25
//#define ZE08_CH2O(x)	(((5.0F/124.0F) * x) - 1.25F)	//y(ppm) = 0.04F(Vadc_bin) - 1.25
#define CH2O_MOL_WEIGHT	30.026F //Formaldehyde Molecular weight, g/mol
#define CH2O_ppm2ugm3(x)	((CH2O_MOL_WEIGHT * x * 1000.0F)/24.45F)	//ppm to ug/m3 CH2O conversion

#define ZE25_O3(x)		((6.25F * x) - 2.5F)	//y(ppm) = 6.25(Vadc) - 2.5
//#define ZE25_O3(x)	(((5.0F/124.0F) * x) - 1.25F)	//y(ppm) = 0.04F(Vadc_bin) - 1.25
#define O3_MOL_WEIGHT	48.0F	//Ozone Molecular weight, g/mol
#define O3_ppm2ugm3(x)	((O3_MOL_WEIGHT * x * 1000.0F)/24.45F)	//ppm to ug/m3 O3 conversion

#define SO2_RGAIN		392000.0F
#define ME4_SO2_SENSITIVITY	8e-7	// 0.8uA/ppm
#define ME4_SO2(x)		(x / (ME4_SO2_SENSITIVITY * SO2_RGAIN))	//y(ppm) = x(Vadc) / (ME4_SO2_Sensitivity * SO2_RGAIN)
#define SO2_MOL_WEIGHT	64.6F	//Sulfur Dioxide Molecular weight, g/mol
#define SO2_ppm2ugm3(x)	((SO2_MOL_WEIGHT * x * 1000.0F)/24.45F)	//ppm to ug/m3 SO2 conversion

#define C6H6_RGAIN		1e8
#define ME4_C6H6_SENSITIVITY	3e-10	// 0.3nA/ppm
#define ME4_C6H6(x)		(x / (ME4_C6H6_SENSITIVITY * C6H6_RGAIN))	//y(ppm) = x(Vadc) / (ME4_C6H6_Sensitivity * C6H6_RGAIN)
#define C6H6_MOL_WEIGHT	78.11F	//Benzene Molecular weight, g/mol
#define C6H6_ppm2ugm3(x)	((C6H6_MOL_WEIGHT * x * 1000.0F)/24.45F)	//ppm to ug/m3 C6H6 conversion

#define CO_RGAIN		47000.0F
#define ME4_CO_SENSITIVITY	8e-8	// 0.08 uA/ppm
#define ME4_CO(x)		(x / (ME4_CO_SENSITIVITY * CO_RGAIN))	//y(ppm) = x(Vadc) / (ME4_CO_Sensitivity * CO_RGAIN)
//#define CO_MOL_WEIGHT	24.01F	//Carbon monoxide Molecular weight, g/mol
//#define CO_ppm2ugm3(x)	((CO_MOL_WEIGHT * x)/24.45F)	//ppm to ug/m3 CO conversion

#define MiCS_6814_NO2(x)	((0.1499F * x) + 0.0042F)	//y(ppm) = 0.1499(Rs/Ro) + 0.0042
#define NO2_MOL_WEIGHT	46.01F	//Nitrogen Dioxide Molecular weight, g/mol
#define NO2_ppm2ugm3(x)	((NO2_MOL_WEIGHT * x * 1000.0F)/24.45F)	//ppm to ug/m3 NO2 conversion

#define MiCS_6814_NH3(x)	(0.5908F * (pow(x, -1.918F)))	//y(ppm) = 0.5908(x*E(-1.918))
#define NH3_MOL_WEIGHT	17.03F	//Ammonia Molecular weight, g/mol
#define NH3_ppm2ugm3(x)	((NH3_MOL_WEIGHT * x * 1000.0F)/24.45F)	//ppm to ug/m3 NH3 conversion

#define MiCS_6814_CO(x)	(4.4922F * (pow(x, -1.182F)))	//y(ppm) = 4.4922(x*E(-1.1182))
#define CO_MOL_WEIGHT	28.01F	//Carbon monoxide Molecular weight, g/mol
#define CO_ppm2ugm3(x)	((CO_MOL_WEIGHT * x)/24.45F)	//ppm to mg/m3 CO conversion

#define OVFL_TIMEOUT (10000U)	//Time that an analog channel can overflow before being notified
// User defines ends here
// User include starts here
#include "port.h"
#include "adc.h"
// User include ends here

uint8_t ain1_values[num_anlg_mux_in], prev_ain1_values[num_anlg_mux_in];
uint8_t ain2_values[num_anlg_mux_in], prev_ain2_values[num_anlg_mux_in];
uint16_t adc_values[num_ad_chs];
float converted_values[num_ad_chs], converted_value;
float mux1_inputs[num_anlg_mux_in], mux2_inputs[num_anlg_mux_in];
float CPU_Temp, Vsense, Vrsense;
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
} ANLG_MeasureTypeDef_st;

extern const uint8_t CDC_delay;
extern uint8_t L50_menu_items_row7a[];
extern uint8_t L50_menu_items_row7b[];
extern uint8_t L50_menu_items_row4a[];
extern uint8_t L50_menu_items_row4b[];

/** @defgroup Function_Prototypes
* @{
*/
void read_analogs(void);
ANLG_Error_et ANLG_Get_Measurement(ANLG_MeasureTypeDef_st *Measurement_Value);

#endif /* PLATFORM_ANLG_DRIVER_H_ */
