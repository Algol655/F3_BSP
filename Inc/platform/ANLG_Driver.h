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
#define GSB_HW_VER				(10U)		//Gas Sensor Board HW Version 1.0
//#define GSB_HW_VER			(20U)		//Gas Sensor Board HW Version 2.0
//#define GSB_HW_VER			(21U)		//Gas Sensor Board HW Version 2.1

#define USE_CROSS_SENS_MATR			(0U)	//1: The concentration values ​​take into account the cross-interference values ​​of the sensors.
#if (OUTDOOR_MODE)
	#define CROSS_SENS_MATR_DIM		(7U)	//The square matrix dimensions
#else
	#define CROSS_SENS_MATR_DIM		(4U)	//The square matrix dimensions
#endif
#define USE_PPM_IN_CROSS_SENS_MATR	(0U)	//1: Use PPM; 0: Use ug/m3

/*
 * IDM Technology Inc. SMD1001 Formaldehyde Gas Sensor
 */
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

/*
 * Winsen Electronics Technology. ZE08-CH2O Electrochemical Formaldehyde Detection Module
 */
#define ZE08_CH2O(x)	((3.125F * x) - 1.25F)	//y(ppm) = 3.125(Vadc) - 1.25
#define CH2O_MOL_WEIGHT	30.026F //Formaldehyde Molecular weight, g/mol
#define CH2O_ppm2ugm3(x)	((CH2O_MOL_WEIGHT * x * 1000.0F)/24.45F)	//ppm to ug/m3 CH2O conversion
#define ZE08_CH2O_TC1(x)	((0.0225F * x) + 0.7F)		//T < 0°C ZE08_CH2O sensor temperature compensation
#define ZE08_CH2O_TC2(x)	((0.012F * x) + 0.7F)		//T < 50°C, T > 0°C ZE08_CH2O sensor temperature compensation
#define ZE08_CH2O_RESOLUTION	0.01F			//The ZEH8 CH2O sensor resolution is 0.01ppm

/*
 * Winsen Electronics Technology. ZE25-O3 Electrochemical Ozone Detection Module
 */
#define ZE25_O3(x)		((6.25F * x) - 2.5F)		//y(ppm) = 6.25(Vadc) - 2.5
#define O3_MOL_WEIGHT	48.0F	//Ozone Molecular weight, g/mol
#define O3_ppm2ugm3(x)	((O3_MOL_WEIGHT * x * 1000.0F)/24.45F)	//ppm to ug/m3 O3 conversion
#define ZE25_O3_TC1(x)	((0.012F * x) + 0.94F)		//T > -20°C, T < 5°C ZE25_O3 sensor temperature compensation
#define ZE25_O3_TC2(x)	((0.0F * x) + 1.0F)			//T > 5°C, T < 20°C ZE25_O3 sensor temperature compensation
#define ZE25_O3_TC3(x)	((-0.0025F * x) + 1.05F)	//T > 20°C ZE25_O3 sensor temperature compensation
#define ZE25_O3_RESOLUTION	0.01F			//The ZE25 O3 sensor data sheet resolution is 0.02ppm, but the
											//implemented sensor driver allows a good approximation up to 0.01ppm
/*
 * Winsen Electronics Technology. ME4-SO2 Electrochemical Sulfur Dioxide Gas Sensor
 */
#if (GSB_HW_VER == 10)
	#define SO2_RGAIN		392000.0F
#elif (GSB_HW_VER >= 20)
	#define SO2_RGAIN		(47000.0F * 10.0F)
#endif
#define ME4_SO2_SENSITIVITY	8e-7F	// 0.8uA/ppm
#define ME4_SO2(x)		(x / (ME4_SO2_SENSITIVITY * SO2_RGAIN))	//y(ppm) = x(Vadc) / (ME4_SO2_Sensitivity * SO2_RGAIN)
#define SO2_MOL_WEIGHT	64.06F	//Sulfur Dioxide Molecular weight, g/mol
#define SO2_ppm2ugm3(x)	((SO2_MOL_WEIGHT * x * 1000.0F)/24.45F)	//ppm to ug/m3 SO2 conversion
#define ME4_SO2_TC(x)	((-2.29648e-5F * pow(x, 2.0F)) + (5.24448e-3F * x) + 9.1586e-1F)	//ME4_SO2 sensor temperature compensation
//#define ME4_SO2_ZC1(x)	((5.80471e-4F * x) + 2.19299e-2F)		//ME4_SO2 sensor zero output temperature compensation from -20°C to 0°C
//#define ME4_SO2_ZC2(x)	((-1.09386e-3F * x) + 2.18772e-2F)		//ME4_SO2 sensor zero output temperature compensation from 0°C to 20°C
//#define ME4_SO2_ZC3(x)	((1.98117e-3F * x) - 3.96234e-2F)		//ME4_SO2 sensor zero output temperature compensation from 20°C to 40°C
//#define ME4_SO2_ZC4(x)	((1.70422e-2F * x) - 6.42303e-1F)		//ME4_SO2 sensor zero output temperature compensation > 40°C
/*
 * It seems that the zero output temperature compensation curve reported in the datasheet does not correspond to the behavior
 * of the real sensor.
 * A better approximation of the real behavior is obtained by using only the correction for temperatures > 40°C
 * and zeroing the correction for all other temperature ranges.
 */
#define ME4_SO2_ZC1(x)	0.0F									//ME4_SO2 sensor zero output temperature compensation from -20°C to 0°C
#define ME4_SO2_ZC2(x)	0.0F									//ME4_SO2 sensor zero output temperature compensation from 0°C to 20°C
#define ME4_SO2_ZC3(x)	0.0F									//ME4_SO2 sensor zero output temperature compensation from 20°C to 40°C
#define ME4_SO2_ZC4(x)	((1.70422e-2F * x) - 6.42303e-1F)		//ME4_SO2 sensor zero output temperature compensation > 40°C
#define ME4_SO2_RESOLUTION	0.01F			//The ME4_SO2 sensor data sheet resolution is 0.1ppm, but the implemented
											//sensor circuit and driver allows a good approximation down to 0.01ppm
/*
 * Winsen Electronics Technology. ME4-NO2 Electrochemical Nitrogen Dioxide Gas Sensor
 */
#if (GSB_HW_VER == 10)
	#define NO2_RGAIN		392000.0F
#elif (GSB_HW_VER >= 20)
	#define NO2_RGAIN		(47000.0F * 10.0F)
#endif
#define ME4_NO2_SENSITIVITY	12e-7F	// 12uA/ppm
#define ME4_NO2(x)		(x / (ME4_NO2_SENSITIVITY * NO2_RGAIN))	//y(ppm) = x(Vadc) / (ME4_NO2_Sensitivity * NO2_RGAIN)
#define NO2_MOL_WEIGHT	46.01F	//Nitrogen Dioxide Molecular weight, g/mol
#define NO2_ppm2ugm3(x)	((NO2_MOL_WEIGHT * x * 1000.0F)/24.45F)	//ppm to ug/m3 SO2 conversion
#define ME4_NO2_TC(x)	((2.77226e-5F * pow(x, 2.0F)) + (3.73631e-3F * x) + 8.97257e-1F)	//ME4_NO2 sensor temperature compensation
//#define ME4_NO2_ZC1(x)	((-6.11064e-4F * x) + 1.18394e-2F)		//ME4_NO2 sensor zero output temperature compensation <= 0°C
//#define ME4_NO2_ZC2(x)	((3.28447e-3F * x) + 1.18394e-2F)		//ME4_NO2 sensor zero output temperature compensation from 0°C to 20°C
//#define ME4_NO2_ZC3(x)	((-1.77591e-3F * x) + 1.13047e-1F)		//ME4_NO2 sensor zero output temperature compensation from 20°C to 40°C
//#define ME4_NO2_ZC4(x)	((2.20747e-2F * x) - 8.40977e-1F)		//ME4_NO2 sensor zero output temperature compensation > 40°C
/*
 * It seems that the zero output temperature compensation curve reported in the datasheet does not correspond to the behavior
 * of the real sensor.
 * A better approximation of the real behavior is obtained by using only the correction for temperatures > 40°C
 * and zeroing the correction for all other temperature ranges.
 */
#define ME4_NO2_ZC1(x)	0.0											//ME4_NO2 sensor zero output temperature compensation <= 0°C
#define ME4_NO2_ZC2(x)	0.0											//ME4_NO2 sensor zero output temperature compensation from 0°C to 20°C
#define ME4_NO2_ZC3(x)	0.0											//ME4_NO2 sensor zero output temperature compensation from 20°C to 40°C
#define ME4_NO2_ZC4(x)	((1.85229e-2F * x) - 7.40915e-1F)			//ME4_NO2 sensor zero output temperature compensation > 40°C
#define ME4_NO2_RESOLUTION	0.01F			//The ME4_NO2 sensor data sheet resolution is 0.1ppm, but the implemented
											//sensor circuit and driver allows a good approximation down to 0.01ppm
/*
 * Winsen Electronics Technology. ME4-C6H6 Electrochemical Benzene Gas Sensor
 */
#if (GSB_HW_VER == 10)
	#define C6H6_RGAIN		1e8F
#elif (GSB_HW_VER >= 20)
	#define C6H6_RGAIN		(47000.0F * 10.0F)
#endif
#define ME4_C6H6_SENSITIVITY	3e-10F	// 0.3nA/ppm
#define ME4_C6H6(x)		(x / (ME4_C6H6_SENSITIVITY * C6H6_RGAIN))		//y(ppm) = x(Vadc) / (ME4_C6H6_Sensitivity * C6H6_RGAIN)
#define C6H6_MOL_WEIGHT	78.11F	//Benzene Molecular weight, g/mol
#define C6H6_ppm2ugm3(x)	((C6H6_MOL_WEIGHT * x * 1000.0F)/24.45F)	//ppm to ug/m3 C6H6 conversion
#define ME4_C6H6_TC(x)	((0.0048F * x) + 0.88F)							//ME4_C6H6 sensor temperature compensation

/*
 * Winsen Electronics Technology. ME4-CO Electrochemical Carbon Monoxide Gas Sensor
 */
#define CO_RGAIN		(47000.0F * 10.0F)
#define ME4_CO_SENSITIVITY	8e-8F	// 0.08 uA/ppm
#define ME4_CO(x)		(x / (ME4_CO_SENSITIVITY * CO_RGAIN))	//y(ppm) = x(Vadc) / (ME4_CO_Sensitivity * CO_RGAIN)
#define CO_MOL_WEIGHT	28.01F	//Carbon monoxide Molecular weight, g/mol
#define CO_ppm2mgm3(x)	((CO_MOL_WEIGHT * x)/24.45F)			//ppm to mg/m3 CO conversion
#define ME4_CO_TC(x)	((-2.00002e-5F * pow(x, 2.0F)) + (7.86821e-3F * x) + 8.70896e-1F)	//ME4_CO sensor temperature compensation.
#define ME4_CO_ZC1(x)	((1.01785e-3F * x) + 2.03730e-2F)			//ME4_CO sensor zero output temperature compensation from -20°C to 0°C
/*
 * It seems that the zero output temperature compensation curve reported in the datasheet does not correspond to the behavior
 * of the real sensor.
 * A better approximation of the real behavior is obtained by extending the validity of the ME4_CO_ZC2 correction up to 40°C
 * and starting the validity of the ME4_CO_ZC3 correction at 40°C.
 */
#define ME4_CO_ZC2(x)	((2.69255e-4F * x) + 2.03526e-2F)		//ME4_CO sensor zero output temperature compensation from 0°C to 20°C (to 40°C)
#define ME4_CO_ZC3(x)	((1.09334e-2F * x) - 1.88370e-1F)		//ME4_CO sensor zero output temperature compensation from > 20°C (> 40°C)
#define ME4_CO_RESOLUTION	0.1F			//The ME4_CO sensor data sheet resolution is 1ppm, but the implemented
											//sensor circuit and driver allows a good approximation down to 0.1ppm

/*
 * SGX Sensortech. MiCS-6814 Metal Oxide Semiconductor CO, NO2, NH3 sensors in one package.
 */
#define USE_T_RH_CORRECTION_MODEL	(1U)
//#define MiCS_6814_NO2(x)	(0.1459F * (pow(x, 1.007F)))			//y(ppm) = 0.1459((Rs/Ro)*E(1.007)). With coefficients found online
#define MiCS_6814_NO2(x)	(1.59e-1F * (pow(x, 9.93177e-1F)))		//y(ppm) = 0,1590((Rs/Ro)*E(0,993177)). With coefficients calculated by me

//#define MiCS_6814_NH3(x)	(0.6803F * (pow(x, -1.67F)))			//y(ppm) = 0.6803((Rs/Ro)*E(-1.67)). With coefficients found online
#define MiCS_6814_NH3(x)	(6.25969e-1F * (pow(x, -1.23896F)))		//y(ppm) = 0.625969((Rs/Ro)*E(-1.23896)). With coefficients calculated by me
#define NH3_MOL_WEIGHT	17.03F										//Ammonia Molecular weight, g/mol
#define NH3_ppm2ugm3(x)	((NH3_MOL_WEIGHT * x * 1000.0F)/24.45F)		//ppm to ug/m3 NH3 conversion

//#define MiCS_6814_CO(x)	(4.385F * (pow(x, -1.179F)))			//y(ppm) = 4.385((Rs/Ro)*E(-1.179)). With coefficients found online
#define MiCS_6814_CO(x)	(4.39874F * (pow(x, -1.18430F)))			//y(ppm) = 4.39874((Rs/Ro)*E(-1.1843)). With coefficients calculated by me
#if (USE_T_RH_CORRECTION_MODEL == 1)
	/*
	 * Correction Model for Metal Oxide Sensor Drift Caused by Ambient Temperature and Humidity
	 * (MDPI Sensor Article: https://www.mdpi.com/1424-8220/22/9/3301)
	 * Modified by me to make Rs*=Rs at test temperature and humidity. (T=23°C, RH=50%, MiCS-6814 Data Sheet, page 2)
	 */
	//#define MiCS_6814_TC_XX(Rs,t,h) 	(10.2243F + (0.7495F * Rs) - (0.1953F * t) + (0.0672F * h) + (0.0016F * Rs * t) + (0.0081F * Rs * h))
	#define MiCS_6814_TC_NO2(Rs,t,h) 	(10.2243F + (0.5582F * Rs) - (0.1953F * t) + (0.0672F * h) + (0.0016F * Rs * t) + (0.0081F * Rs * h))
	#define MiCS_6814_TC_NH3(Rs,t,h) 	(10.2243F + (0.5582F * Rs) - (0.1953F * t) + (0.0672F * h) + (0.0016F * Rs * t) + (0.0081F * Rs * h))
//	#define MiCS_6814_TC_CO(Rs,t,h) 	(-7.4941F - (0.6914F * Rs) - (0.0531F * t) + (0.2860F * h) + (0.0484F * Rs * t) + (0.0071F * Rs * h))
	#define MiCS_6814_TC_CO(Rs,t,h) 	(-7.4941F - (0.46823F * Rs) - (0.0531F * t) + (0.2860F * h) + (0.0484F * Rs * t) + (0.0071F * Rs * h))
#else
	/*
	 * Correction Model for Metal Oxide Sensor Drift Caused by Ambient Temperature and Humidity
	 * Correction factor based on typical MOX behavior (T_ref = 23C, RH_ref = 50%)
	 * Reducing gases (RED (CO), NH3) experience an increase in resistance as temperature increases.
	 * Reducing gases (RED (CO), NH3) experience an increase in resistance as humidity increases.
	 * Oxidizing gases (OX (NO2) experience a decrease in resistance as temperature increases.
	 * Oxidizing gases (OX (NO2) experience an increase in resistance as humidity increases.
	 *
	 * IOP Conference Series: Materials Science and Engineering:
	 * Effect of Environmental Temperature and Humidity on Different Metal Oxide Gas Sensors at Various Gas Concentration Levels.
	 *
	 * Alfa, Beta values From Google AI)
	 */
	#define ALFA_NO2	(1.2e-3F)
	#define BETA_NO2	(-1.5e-3F)
	#define ALFA_CO		(-4.5e-3F)
	#define BETA_CO		(-2.8e-3F)
	#define ALFA_NH3	(-3.2e-3F)
	#define BETA_NH3	(-2.2e-3F)
	#define T0			(23.0F)
	#define H0			(50.0F)
	#define MiCS_6814_TC_NO2(Rs,t,h) 	(Rs * (1 + ALFA_NO2 * (t - T0) + BETA_NO2 *(h - H0)))
	#define MiCS_6814_TC_NH3(Rs,t,h) 	(Rs * (1 + ALFA_NH3 * (t - T0) + BETA_NH3 *(h - H0)))
	#define MiCS_6814_TC_CO(Rs,t,h) 	(Rs * (1 + ALFA_CO * (t - T0) + BETA_CO *(h - H0)))
#endif

#if (USE_CROSS_SENS_MATR)
/*
 * Parameters for defining the cross-sensitivity matrix (also called calibration matrix)
 */
	#if ((OUTDOOR_MODE) && (GSB_HW_VER > 10))
		#if (USE_PPM_IN_CROSS_SENS_MATR)
			#define KCH2O_CH2O	(1.0F)			//ZE08 Sensor Cross interference with CH2O
			#define KCH2O_O3	(0.0F)			//ZE08 Sensor Cross interference with O3
			#define KCH2O_SO2	(0.0F)			//ZE08 Sensor Cross interference with SO2
			#define KCH2O_NO2	(0.0F)			//ZE08 Sensor Cross interference with NO2
			#define KCH2O_CO	(0.64F/200.0F)	//ZE08 Sensor Cross interference with CO gas
			#define KCH2O_C6H6	(0.1F/10.0F)	//ZE08 Sensor Cross interference with C6H6
			#define KCH2O_NH3	(0.0F)			//ZE08 Sensor Cross interference with NH3

			#define KO3_CH2O	(0.0F)			//ZE25 Sensor Cross interference with CH2O
			#define KO3_O3		(1.0F)			//ZE25 Sensor Cross interference with O3
			#define KO3_SO2		(0.0F/5.0F)		//ZE25 Sensor Cross interference with SO2
			#define KO3_NO2		(2.0F/5.0F)		//ZE25 Sensor Cross interference with NO2
			#define KO3_CO		(0.0F/300.0F)	//ZE25 Sensor Cross interference with CO
			#define KO3_C6H6	(0.0F)			//ZE25 Sensor Cross interference with C6H6
			#define KO3_NH3		(0.0F)			//ZE25 Sensor Cross interference with NH3

			#define KSO2_CH2O	(18.0F/10.0F)	//ME4-SO2 Sensor Cross interference with CH2O
			#define KSO2_O3		(0.0F)			//ME4-SO2 Sensor Cross interference with O3
			#define KSO2_SO2	(1.0F)			//ME4-SO2 Sensor Cross interference with SO2
			#define KSO2_NO2	(-4.6F/5.0F)	//ME4-SO2 Sensor Cross interference with NO2
			#define KSO2_CO		(2.5F/200.0F)	//ME4-SO2 Sensor Cross interference with CO
			#define KSO2_C6H6	(0.0F/100.0F)	//ME4-SO2 Sensor Cross interference with C6H6
			#define KSO2_NH3	(0.0F)			//ME4-SO2 Sensor Cross interference with NH3

			#define KNO2_CH2O	(0.0F)			//ME4-NO2 Sensor Cross interference with CH2O
			#define KNO2_O3		(0.0F)			//ME4-NO2 Sensor Cross interference with O3
			#define KNO2_SO2	(-0.1F/5.0F)	//ME4-NO2 Sensor Cross interference with SO2
			#define KNO2_NO2	(1.0F)			//ME4-NO2 Sensor Cross interference with NO2
			#define KNO2_CO		(0.0F/300.0F)	//ME4-NO2 Sensor Cross interference with CO
			#define KNO2_C6H6	(0.0F)			//ME4-NO2 Sensor Cross interference with C6H6
			#define KNO2_NH3	(2.0F/20.0F)	//ME4-NO2 Sensor Cross interference with NH3

			#define KCO_CH2O	(0.0F)			//ME4-CO Sensor Cross interference with CH2O
			#define KCO_O3		(0.0F)			//ME4-CO Sensor Cross interference with O3
			#define KCO_SO2		(0.0F/20.0F)	//ME4-CO Sensor Cross interference with SO2
			#define KCO_NO2		(2.0F/5.0F)		//ME4-CO Sensor Cross interference with NO2
			#define KCO_CO		(1.0F)			//ME4-CO Sensor Cross interference with CO
			#define KCO_C6H6	(0.0F)			//ME4-CO Sensor Cross interference with C6H6
			#define KCO_NH3		(0.0F)			//ME4-CO Sensor Cross interference with NH3

			#define KC6H6_CH2O	(0.0F)			//ME4-C6H6 Sensor Cross interference with CH2O
			#define KC6H6_O3	(0.0F)			//ME4-C6H6 Sensor Cross interference with O3
			#define KC6H6_SO2	(25.0F/20.0F)	//ME4-C6H6 Sensor Cross interference with SO2
			#define KC6H6_NO2	(0.0F)			//ME4-C6H6 Sensor Cross interference with NO2
			#define KC6H6_CO	(221.0F/200.0F)	//ME4-C6H6 Sensor Cross interference with CO
			#define KC6H6_C6H6	(1.0F)			//ME4-C6H6 Sensor Cross interference with C6H6
			#define KC6H6_NH3	(0.0F)			//ME4-C6H6 Sensor Cross interference with NH3

			#define KNH3_CH2O	(0.0F)			//ME4-NH3 Sensor Cross interference with CH2O
			#define KNH3_O3		(0.0F)			//ME4-NH3 Sensor Cross interference with O3
			#define KNH3_SO2	(0.0F)			//ME4-NH3 Sensor Cross interference with SO2
			#define KNH3_NO2	(0.0F)			//ME4-NH3 Sensor Cross interference with NO2
			#define KNH3_CO		(0.0F)			//ME4-NH3 Sensor Cross interference with CO
			#define KNH3_C6H6	(0.0F)			//ME4-NH3 Sensor Cross interference with C6H6
			#define KNH3_NH3	(1.0F)			//ME4-NH3 Sensor Cross interference with NH3
		#else
			#define KCH2O_CH2O	(1.0F)												//ZE08 Sensor Cross interference with CH2O
			#define KCH2O_O3	((0.0F) * (CH2O_MOL_WEIGHT/O3_MOL_WEIGHT))			//ZE08 Sensor Cross interference with O3
			#define KCH2O_SO2	((0.0F) * (CH2O_MOL_WEIGHT/SO2_MOL_WEIGHT))			//ZE08 Sensor Cross interference with SO2
			#define KCH2O_NO2	((0.0F) * (CH2O_MOL_WEIGHT/NO2_MOL_WEIGHT))			//ZE08 Sensor Cross interference with NO2
			#define KCH2O_CO	((0.64F/200.0F) * (CH2O_MOL_WEIGHT/CO_MOL_WEIGHT))	//ZE08 Sensor Cross interference with CO gas
			#define KCH2O_C6H6	((0.1F/10.0F) * (CH2O_MOL_WEIGHT/C6H6_MOL_WEIGHT))	//ZE08 Sensor Cross interference with C6H6
			#define KCH2O_NH3	((0.0F) * (CH2O_MOL_WEIGHT/NH3_MOL_WEIGHT))			//ZE08 Sensor Cross interference with NH3

			#define KO3_CH2O	((0.0F) * (O3_MOL_WEIGHT/CH2O_MOL_WEIGHT))			//ZE25 Sensor Cross interference with CH2O
			#define KO3_O3		(1.0F)												//ZE25 Sensor Cross interference with O3
			#define KO3_SO2		((0.0F/5.0F) * (O3_MOL_WEIGHT/SO2_MOL_WEIGHT))		//ZE25 Sensor Cross interference with SO2
			#define KO3_NO2		((2.0F/5.0F) * (O3_MOL_WEIGHT/NO2_MOL_WEIGHT))		//ZE25 Sensor Cross interference with NO2
			#define KO3_CO		((0.0F/300.0F) * (O3_MOL_WEIGHT/CO_MOL_WEIGHT))		//ZE25 Sensor Cross interference with CO
			#define KO3_C6H6	((0.0F) * (O3_MOL_WEIGHT/C6H6_MOL_WEIGHT))			//ZE25 Sensor Cross interference with C6H6
			#define KO3_NH3		((0.0F) * (O3_MOL_WEIGHT/NH3_MOL_WEIGHT))			//ZE25 Sensor Cross interference with NH3

			#define KSO2_CH2O	((18.0F/10.0F) * (SO2_MOL_WEIGHT/CH2O_MOL_WEIGHT))	//ME4-SO2 Sensor Cross interference with CH2O
			#define KSO2_O3		((0.0F) * (SO2_MOL_WEIGHT/O3_MOL_WEIGHT))			//ME4-SO2 Sensor Cross interference with O3
			#define KSO2_SO2	(1.0F)												//ME4-SO2 Sensor Cross interference with SO2
			#define KSO2_NO2	((-4.6F/5.0F) * (SO2_MOL_WEIGHT/NO2_MOL_WEIGHT))	//ME4-SO2 Sensor Cross interference with NO2
			#define KSO2_CO		((2.5F/200.0F) * (SO2_MOL_WEIGHT/CO_MOL_WEIGHT))	//ME4-SO2 Sensor Cross interference with CO
			#define KSO2_C6H6	((0.0F/100.0F) * (SO2_MOL_WEIGHT/C6H6_MOL_WEIGHT))	//ME4-SO2 Sensor Cross interference with C6H6
			#define KSO2_NH3	((0.0F) * (SO2_MOL_WEIGHT/NH3_MOL_WEIGHT))			//ME4-SO2 Sensor Cross interference with NH3

			#define KNO2_CH2O	((0.0F) * (NO2_MOL_WEIGHT/CH2O_MOL_WEIGHT))			//ME4-NO2 Sensor Cross interference with CH2O
			#define KNO2_O3		((0.0F) * (NO2_MOL_WEIGHT/O3_MOL_WEIGHT))			//ME4-NO2 Sensor Cross interference with O3
			#define KNO2_SO2	((-0.1F/5.0F) * (NO2_MOL_WEIGHT/SO2_MOL_WEIGHT))	//ME4-NO2 Sensor Cross interference with SO2
			#define KNO2_NO2	(1.0F)												//ME4-NO2 Sensor Cross interference with NO2
			#define KNO2_CO		((0.0F/300.0F) * (NO2_MOL_WEIGHT/CO_MOL_WEIGHT))	//ME4-NO2 Sensor Cross interference with CO
			#define KNO2_C6H6	((0.0F) * (NO2_MOL_WEIGHT/C6H6_MOL_WEIGHT))			//ME4-NO2 Sensor Cross interference with C6H6
			#define KNO2_NH3	((2.0F/20.0F) * (NO2_MOL_WEIGHT/NH3_MOL_WEIGHT))	//ME4-NO2 Sensor Cross interference with NH3

			#define KCO_CH2O	((0.0F)	* (CO_MOL_WEIGHT/CH2O_MOL_WEIGHT))			//ME4-CO Sensor Cross interference with CH2O
			#define KCO_O3		((0.0F)	* (CO_MOL_WEIGHT/O3_MOL_WEIGHT))			//ME4-CO Sensor Cross interference with O3
			#define KCO_SO2		((0.0F/20.0F) * (CO_MOL_WEIGHT/SO2_MOL_WEIGHT))		//ME4-CO Sensor Cross interference with SO2
			#define KCO_NO2		((2.0F/5.0F) * (CO_MOL_WEIGHT/NO2_MOL_WEIGHT))		//ME4-CO Sensor Cross interference with NO2
			#define KCO_CO		(1.0F)												//ME4-CO Sensor Cross interference with CO
			#define KCO_C6H6	((0.0F)	* (CO_MOL_WEIGHT/C6H6_MOL_WEIGHT))			//ME4-CO Sensor Cross interference with C6H6
			#define KCO_NH3		((0.0F)	* (CO_MOL_WEIGHT/NH3_MOL_WEIGHT))			//ME4-CO Sensor Cross interference with NH3

			#define KC6H6_CH2O	((0.0F) * (C6H6_MOL_WEIGHT/CH2O_MOL_WEIGHT))		//ME4-C6H6 Sensor Cross interference with CH2O
			#define KC6H6_O3	((0.0F) * (C6H6_MOL_WEIGHT/O3_MOL_WEIGHT))			//ME4-C6H6 Sensor Cross interference with O3
			#define KC6H6_SO2	((25.0F/20.0F) * (C6H6_MOL_WEIGHT/SO2_MOL_WEIGHT))	//ME4-C6H6 Sensor Cross interference with SO2
			#define KC6H6_NO2	((0.0F) * (C6H6_MOL_WEIGHT/NO2_MOL_WEIGHT))			//ME4-C6H6 Sensor Cross interference with NO2
			#define KC6H6_CO	((221.0F/200.0F) * (C6H6_MOL_WEIGHT/CO_MOL_WEIGHT))	//ME4-C6H6 Sensor Cross interference with CO
			#define KC6H6_C6H6	(1.0F)												//ME4-C6H6 Sensor Cross interference with C6H6
			#define KC6H6_NH3	((0.0F) * (C6H6_MOL_WEIGHT/NH3_MOL_WEIGHT))			//ME4-C6H6 Sensor Cross interference with NH3

			#define KNH3_CH2O	((0.0F)	* (NH3_MOL_WEIGHT/CH2O_MOL_WEIGHT))			//ME4-NH3 Sensor Cross interference with CH2O
			#define KNH3_O3		((0.0F)	* (NH3_MOL_WEIGHT/O3_MOL_WEIGHT))			//ME4-NH3 Sensor Cross interference with O3
			#define KNH3_SO2	((0.0F)	* (NH3_MOL_WEIGHT/SO2_MOL_WEIGHT))			//ME4-NH3 Sensor Cross interference with SO2
			#define KNH3_NO2	((0.0F)	* (NH3_MOL_WEIGHT/NO2_MOL_WEIGHT))			//ME4-NH3 Sensor Cross interference with NO2
			#define KNH3_CO		((0.0F)	* (NH3_MOL_WEIGHT/CO_MOL_WEIGHT))			//ME4-NH3 Sensor Cross interference with CO
			#define KNH3_C6H6	((0.0F)	* (NH3_MOL_WEIGHT/C6H6_MOL_WEIGHT))			//ME4-NH3 Sensor Cross interference with C6H6
			#define KNH3_NH3	(1.0F)												//ME4-NH3 Sensor Cross interference with NH3
		#endif	// USE_PPM_IN_CROSS_SENS_MATR
		//Creating the coefficient matrix
		#define CROSS_INTERFERENCE_COEFF_MATRIX { 	\
			KCH2O_CH2O, KCH2O_O3, KCH2O_SO2, KCH2O_NO2, KCH2O_CO, KCH2O_C6H6, KCH2O_NH3,\
			KO3_CH2O,   KO3_O3,   KO3_SO2,   KO3_NO2,   KO3_CO,   KO3_C6H6,   KO3_NH3,	\
			KSO2_CH2O,  KSO2_O3,  KSO2_SO2,  KSO2_NO2,  KSO2_CO,  KSO2_C6H6,  KSO2_NH3,	\
			KNO2_CH2O,  KNO2_O3,  KNO2_SO2,  KNO2_NO2,  KNO2_CO,  KNO2_C6H6,  KNO2_NH3,	\
			KCO_CH2O,   KCO_O3,   KCO_SO2,   KCO_NO2,   KCO_CO,   KCO_C6H6,   KCO_NH3,	\
			KC6H6_CH2O, KC6H6_O3, KC6H6_SO2, KC6H6_NO2, KC6H6_CO, KC6H6_C6H6, KC6H6_NH3,\
			KNH3_CH2O,  KNH3_O3,  KNH3_SO2,  KNH3_NO2,  KNH3_CO,  KNH3_C6H6,  KNH3_NH3	\
		}
	#else	//Using indoor gas sensors board
		#if (USE_PPM_IN_CROSS_SENS_MATR)
			#define KCH2O_CH2O	(1.0F)			//ZE08 Sensor Cross interference with CH2O
			#define KCH2O_NO2	(0.0F)			//ZE08 Sensor Cross interference with NO2
			#define KCH2O_CO	(0.0F)			//ZE08 Sensor Cross interference with CO gas
			#define KCH2O_NH3	(0.0F)			//ZE08 Sensor Cross interference with NH3

			#define KNO2_CH2O	(0.0F)			//ME4-NO2 Sensor Cross interference with CH2O
			#define KNO2_NO2	(1.0F)			//ME4-NO2 Sensor Cross interference with NO2
			#define KNO2_CO		(0.0F)			//ME4-NO2 Sensor Cross interference with CO
			#define KNO2_NH3	(0.0F)			//ME4-NO2 Sensor Cross interference with NH3

			#define KCO_CH2O	(0.0F)			//ME4-CO Sensor Cross interference with CH2O
			#define KCO_NO2		(0.0F)			//ME4-CO Sensor Cross interference with NO2
			#define KCO_CO		(1.0F)			//ME4-CO Sensor Cross interference with CO
			#define KCO_NH3		(0.0F)			//ME4-CO Sensor Cross interference with NH3

			#define KNH3_CH2O	(0.0F)			//ME4-NH3 Sensor Cross interference with CH2O
			#define KNH3_NO2	(0.0F)			//ME4-NH3 Sensor Cross interference with NO2
			#define KNH3_CO		(0.0F)			//ME4-NH3 Sensor Cross interference with CO
			#define KNH3_NH3	(1.0F)			//ME4-NH3 Sensor Cross interference with NH3
		#else
			#define KCH2O_CH2O	(1.0F)												//ZE08 Sensor Cross interference with CH2O
			#define KCH2O_NO2	((0.0F) * (CH2O_MOL_WEIGHT/NO2_MOL_WEIGHT))			//ZE08 Sensor Cross interference with NO2
			#define KCH2O_CO	((0.0F) * (CH2O_MOL_WEIGHT/CO_MOL_WEIGHT))			//ZE08 Sensor Cross interference with CO gas
			#define KCH2O_NH3	((0.0F) * (CH2O_MOL_WEIGHT/NH3_MOL_WEIGHT))			//ZE08 Sensor Cross interference with NH3

			#define KNO2_CH2O	((0.0F) * (NO2_MOL_WEIGHT/CH2O_MOL_WEIGHT))			//ME4-NO2 Sensor Cross interference with CH2O
			#define KNO2_NO2	(1.0F)												//ME4-NO2 Sensor Cross interference with NO2
			#define KNO2_CO		((0.0F) * (NO2_MOL_WEIGHT/CO_MOL_WEIGHT))			//ME4-NO2 Sensor Cross interference with CO
			#define KNO2_NH3	((0.0F) * (NO2_MOL_WEIGHT/NH3_MOL_WEIGHT))			//ME4-NO2 Sensor Cross interference with NH3

			#define KCO_CH2O	((0.0F)	* (CO_MOL_WEIGHT/CH2O_MOL_WEIGHT))			//ME4-CO Sensor Cross interference with CH2O
			#define KCO_NO2		((0.0F) * (CO_MOL_WEIGHT/NO2_MOL_WEIGHT))			//ME4-CO Sensor Cross interference with NO2
			#define KCO_CO		(1.0F)												//ME4-CO Sensor Cross interference with CO
			#define KCO_C6H6	((0.0F)	* (CO_MOL_WEIGHT/C6H6_MOL_WEIGHT))			//ME4-CO Sensor Cross interference with C6H6
			#define KCO_NH3		((0.0F)	* (CO_MOL_WEIGHT/NH3_MOL_WEIGHT))			//ME4-CO Sensor Cross interference with NH3

			#define KNH3_CH2O	((0.0F)	* (NH3_MOL_WEIGHT/CH2O_MOL_WEIGHT))			//ME4-NH3 Sensor Cross interference with CH2O
			#define KNH3_NO2	((0.0F)	* (NH3_MOL_WEIGHT/NO2_MOL_WEIGHT))			//ME4-NH3 Sensor Cross interference with NO2
			#define KNH3_CO		((0.0F)	* (NH3_MOL_WEIGHT/CO_MOL_WEIGHT))			//ME4-NH3 Sensor Cross interference with CO
			#define KNH3_NH3	(1.0F)												//ME4-NH3 Sensor Cross interference with NH3
		#endif	// USE_PPM_IN_CROSS_SENS_MATR
		//Creating the coefficient matrix
		#define CROSS_INTERFERENCE_COEFF_MATRIX { 	\
			KCH2O_CH2O, KCH2O_NO2, KCH2O_CO, KCH2O_NH3,	\
			KNO2_CH2O,  KNO2_NO2,  KNO2_CO,  KNO2_NH3,	\
			KCO_CH2O,   KCO_NO2,   KCO_CO,   KCO_NH3,	\
			KNH3_CH2O,  KNH3_NO2,  KNH3_CO,  KNH3_NH3	\
		}
	#endif	// (OUTDOOR_MODE) && (GSB_HW_VER > 10)
#endif	// USE_CROSS_SENS_MATR

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
#include "platform/port.h"
//#include "adc.h"
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
