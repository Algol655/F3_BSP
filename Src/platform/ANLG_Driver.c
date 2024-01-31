/*
 * ANLG_Driver.c
 *
 *  Created on: 10 apr 2021
 *      Author: Tommaso Sabatini
 */

#include "platform/ANLG_Driver.h"
#include "compiler/compiler.h"

float32_t filter1_Value[16] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
const float32_t scale1_Factor[16] = {0.01, 0.01, 0.01, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05};
float32_t filter2_Value[16] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
const float32_t scale2_Factor[16] = {0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05};
const float32_t R1 = 1500.0; const float32_t R2 = 56000.0; float32_t Vref = 3.32;
//float R1 = 9108.0; float R2 = 47000.0; float Vref = 3.3;
const uint16_t ADC_RESOLUTION = 4096;	//ADC_RESOLUTION is 4096 for 12 bit, 1024 for 10 bit and 256 for 8 bit
uint8_t ain1_buf[16], ain2_buf[16];
uint8_t offset = 0x00;
float32_t ppm_CH2O, ppm_O3, ppm_NO2, ppm_NH3, ppm_CO, ppm_SO2, ppm_C6H6;

/* @fn 		Ain1_Check()
 * @brief 	Check if there was an event from an analog input and manage it
 * @return	null
 * */
void Ain1_Check(uint8_t* b, uint8_t* a, AIN1_FUNCTION An_Func)
{
	uint16_t mask;

	switch(An_Func)
	{
		case AIN_00_1_CH2O:
		{
			mask = *a;
			if (mask < 31)	//The minimum output value from the Winsen ZE08-CH2O formaldehyde
				mask = 31;	//sensor is 0.4V for 0 ppm (31-> 1F on the ADC 8 most significant bits.
			b[0] = mask;	//So don't accept negative values.
		}
		break;
		case AIN_01_1_O3:
		{
			mask = *a;
			if (mask < 31)	//The minimum output value from the Winsen ZE25-O3 ozone
				mask = 31;	//sensor is 0.4V for 0 ppm (31-> 1F on the ADC 8 most significant bits.
			b[1] = mask;	//So don't accept negative values.
		}
		break;
		case AIN_02_1_NO2:
		{
			mask = *a;
			b[2] = mask;
		}
		break;
		case AIN_03_1_NH3:
		{
			mask = *a;
			b[3] = mask;
		}
		break;
		case AIN_04_1_CO:
		{
			mask = *a;
			b[4] = mask;
		}
		break;
		case AIN_05_1_SO2:
		{
			mask = *a;
			b[5] = mask;
		}
		break;
		case AIN_06_1_C6H6:
		{
			mask = *a;
			b[6] = mask;
		}
		break;
		case AIN_07_1_:
		{
			mask = *a;
			b[7] = mask;
		}
		break;
		case AIN_08_1_:
		{
			mask = *a;
			b[8] = mask;
		}
		break;
		case AIN_09_1_:
		{
			mask = *a;
			b[9] = mask;
		}
		break;
		case AIN_10_1_:
		{
			mask = *a;
			b[10] = mask;
		}
		break;
		case AIN_11_1_:
		{
			mask = *a;
			b[11] = mask;
		}
		break;
		case AIN_12_1_:
		{
			mask = *a;
			b[12] = mask;
		}
		break;
		case AIN_13_1_:
		{
			mask = *a;
			b[13] = mask;
		}
		break;
		case AIN_14_1_:
		{
			mask = *a;
			b[14] = mask;
		}
		break;
		case AIN_15_1_:
		{
			mask = *a;
/*			if (mask > 116)						//If the read value corresponds to a voltage > 1.5V then
			{									//the Indoor Gas Sensor Board is not installed or is faulty, so..
				BIT_CLEAR(SensorStatusReg,21);	//Clear Gas Sensor Module presence in SensorStatusRegister
				BIT_CLEAR(SensorStatusReg,22);	//Clear Gas Sensor Full Equipped Module presence in SensorStatusRegister
				BIT_CLEAR(SensorStatusReg,5);	//Clear Gas Sensor Module status in SensorStatusRegister
				BIT_CLEAR(SensorStatusReg,6);	//Clear Gas Sensor Full Equipped Module status in SensorStatusRegister
			} else
			{
				BIT_SET(SensorStatusReg,21);	//Set Gas Sensor Module presence in SensorStatusRegister
				BIT_SET(SensorStatusReg,22);	//Set Gas Sensor Full Equipped Module presence in SensorStatusRegister
				BIT_SET(SensorStatusReg,5);		//Set Gas Sensor Module status in SensorStatusRegister
				BIT_SET(SensorStatusReg,6);		//Set Gas Sensor Full Equipped Module status in SensorStatusRegister
			} */
			b[15] = mask;
		}
		break;
		default:
			break;
	}
}

/* @fn 		Ain2_Check()
 * @brief 	Check if there was an event from an analog input and manage it
 * @return	null
 * */
void Ain2_Check(uint8_t* b, uint8_t* a, AIN2_FUNCTION An_Func)
{
	static uint32_t ErrorTimeOut = 0;
	static uint8_t ADCRestartCounter = 0;
	static bool ErrorTimerStarted = false;
	uint16_t mask;

	switch(An_Func)
	{
		case AIN_00_2_:
		{
			mask = *a;
			b[0] = mask;
		}
		break;
		case AIN_01_2_:
		{
			mask = *a;
			b[1] = mask;
		}
		break;
		case AIN_02_2_:
		{
			mask = *a;
			b[2] = mask;
		}
		break;
		case AIN_03_2_:
		{
			mask = *a;
			b[3] = mask;
		}
		break;
		case AIN_04_2_:
		{
			mask = *a;
			b[4] = mask;
		}
		break;
		case AIN_05_2_:
		{
			mask = *a;
			b[5] = mask;
		}
		break;
		case AIN_06_2_:
		{
			mask = *a;
			b[6] = mask;
		}
		break;
		case AIN_07_2_:
		{
			mask = *a;
			b[7] = mask;
		}
		break;
		case AIN_08_2_:
		{
			mask = *a;
			b[8] = mask;
		}
		break;
		case AIN_09_2_:
		{
			mask = *a;
			b[9] = mask;
		}
		break;
		case AIN_10_2_:
		{
			mask = *a;
			b[10] = mask;
		}
		break;
		case AIN_11_2_:
		{
			mask = *a;
			b[11] = mask;
		}
		break;
		case AIN_12_2_:
		{
			mask = *a;
			b[12] = mask;
		}
		break;
		case AIN_13_2_:
		{
			mask = *a;
			b[13] = mask;
		}
		break;
		/*
		 * This analog input is used to monitor the correct operation of the AD Converter.
		 * It is wired to ground by the Gas Sensor Board, so when the AD Converter reads
		 * a value greater than zero then it means that the device is functioning abnormally
		 * and requires a restart.
		 */
		case AIN_14_2_:
		{
			mask = *a;
			if (mask > 0)
			{
				if (!(ErrorTimerStarted))
				{
					ErrorTimeOut = HAL_GetTick();
					ErrorTimerStarted = true;
				}
				if ((HAL_GetTick() - ErrorTimeOut) >= OVFL_TIMEOUT)
				{
					ErrorTimerStarted = false;
					b[6] = 0;
					memset(&mux1_inputs[0], 0, sizeof(mux1_inputs));
					HAL_ADC_DeInit(&hadc1);
					MX_ADC1_Init();
					ADC_Config(&hadc1);
					//Increments the second nibble of the ADC restart event counter modulo 16...
					ADCRestartCounter++; ADCRestartCounter = (ADCRestartCounter << 4) & 0xF0;
					//...and copies the value to the status register second nibble
					StatusReg &= 0xFFFFFF0F; StatusReg |= (uint32_t)(ADCRestartCounter);
				}
			} else
			{
				ErrorTimerStarted = false;
				b[14] = mask;
			}
		}
		break;
		case AIN_15_2_:
		{
			mask = *a;
			if (mask > 116)						//If the read value corresponds to a voltage > 1.5V then
			{									//the Gas Sensor Board is not installed or is faulty, so..
				BIT_CLEAR(SensorStatusReg,21);	//Clear Gas Sensor Module presence in SensorStatusRegister
				BIT_CLEAR(SensorStatusReg,22);	//Clear Gas Sensor Full Equipped Module presence in SensorStatusRegister
				BIT_CLEAR(SensorStatusReg,5);	//Clear Gas Sensor Module status in SensorStatusRegister
				BIT_CLEAR(SensorStatusReg,6);	//Clear Gas Sensor Full Equipped Module status in SensorStatusRegister
			} else
			{
				BIT_SET(SensorStatusReg,21);	//Set Gas Sensor Module presence in SensorStatusRegister
				BIT_SET(SensorStatusReg,22);	//Set Gas Sensor Full Equipped Module presence in SensorStatusRegister
				BIT_SET(SensorStatusReg,5);		//Set Gas Sensor Module status in SensorStatusRegister
				BIT_SET(SensorStatusReg,6);		//Set Gas Sensor Full Equipped Module status in SensorStatusRegister
			}
			b[15] = mask;
		}
		break;
		default:
			break;
	}
}

void read_analogs(void)
{
	extern void DisplayAnalogValues(void);
	uint8_t i, j, k, n;
	const uint8_t mask1_shift = 4;
	uint16_t mask1 = 0xFFC;	//4092: Filter for 3mV sensitivity
//	uint16_t mask1 = 0xFF8;	//4088: Filter for 7mV sensitivity
//	uint16_t mask1 = 0xFF0;	//4080: Filter for 13mV sensitivity
//	uint16_t mask1 = 0xF80;	//3968: Filter for 100mV sensitivity
	uint16_t mask2 = 0xFF0;	//4080: Filter for 13mV sensitivity
//	uint16_t mask2 = 0xF80;	//3968: Filter for 100mV sensitivity
//	uint16_t mask2 = 0xFC0;	//4032: Filter for 52mV sensitivity
//	uint16_t mask3 = 0xFE0;	//4064: Filter for 26mV sensitivity
	const uint8_t mask2_shift = 4;
	static uint32_t Ovfl_Time0 = 0;
	static uint32_t Ovfl_Time1 = 0;

	conversion_ended = false;
	for (i = 0 ; i < num_anlg_mux_in; i++)
	{
		conversion_ended = false;
		//Select mux channel number i
		HAL_GPIO_WritePin(SEL_0_GPIO_Port, SEL_0_Pin, BIT_CHECK(i,0));
		HAL_GPIO_WritePin(SEL_1_GPIO_Port, SEL_1_Pin, BIT_CHECK(i,1));
		HAL_GPIO_WritePin(SEL_2_GPIO_Port, SEL_2_Pin, BIT_CHECK(i,2));
		HAL_GPIO_WritePin(SEL_3_GPIO_Port, SEL_3_Pin, BIT_CHECK(i,3));
		//Wait for signal stabilization
		Sleep(1);
		//Start Conversion
		if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADCxConvertedValue, num_ad_chs) != HAL_OK)
		{
			//Start Conversion Error
			Error_Handler();
		}
	    while (!conversion_ended)
		{
	    	usleep(1);
		}
		if (adc_values[0] < 0xFFF)	//Discard overflowed values
		{
			adc_values[0] -= offset;
			if (adc_values[0] > 0xFFF)
				adc_values[0] = 0x00;
//			memset(&ovfl_1[i][0], 0x00, 8);
			Ovfl_Time0 = HAL_GetTick();
			BIT_CLEAR(AnlgOvflStatusReg, i);
			//Get and filter the ADC1 i-th analog input
			mux1_inputs[i] = (float32_t)(((((adc_values[0] & mask1)) * ((R1 + R2) / R2) * Vref)) / ADC_RESOLUTION);
			if ((BLE_DataReady) && !(Test_Mode))
			{
				filter1_Value[i] = filter1_Value[i] + scale1_Factor[i] * (mux1_inputs[i] - filter1_Value[i]);
				mux1_inputs[i] = filter1_Value[i];
			} else		//This eliminates the initial transient of the filter
			{
				filter1_Value[i] = filter1_Value[i] + 1.0 * (mux1_inputs[i] - filter1_Value[i]);
				mux1_inputs[i] = filter1_Value[i];
			}

			ain1_values[i] = ((adc_values[0] & mask1) >> mask1_shift);	//Only the ADC msb 11..5 are used
			//Check for ain1 variations
//			if ((ain1_values[i] != prev_ain1_values[i]) || (service_timer0_expired))	//Uncomment to check only if analog input changes
//			{
				if (!Test_Mode)
				{
					Ain1_Check(&ain1_buf[0], &ain1_values[i], i);
//					service_timer0_expired = true;					//Uncomment to send message on analog input change
				}
//			}
			memcpy((void*)&prev_ain1_values, (void*)&ain1_values, sizeof(ain1_values));	//Save ADC Channel 1 values
		} else
		{
			if ((HAL_GetTick() - Ovfl_Time0) > OVFL_TIMEOUT)
				BIT_SET(AnlgOvflStatusReg, i);
//			strcpy((char*)&ovfl_1[i][0], "OVFL");
		}

		if (adc_values[1] < 0xFFF)	//Discard overflowed values
		{
			adc_values[1] -= offset;
			if (adc_values[1] > 0xFFF)
				adc_values[1] = 0x00;
//			memset(&ovfl_2[i][0], 0x00, 8);
			Ovfl_Time1 = HAL_GetTick();
			BIT_CLEAR(AnlgOvflStatusReg, i<<16);
			//Get and filter the ADC2 i-th analog input
			mux2_inputs[i] = (float32_t)(((((adc_values[1] & mask2)) * ((R1 + R2) / R2) * Vref)) / ADC_RESOLUTION);
			if ((BLE_DataReady) && !(Test_Mode))
			{
				filter2_Value[i] = filter2_Value[i] + scale2_Factor[i] * (mux2_inputs[i] - filter2_Value[i]);
				mux2_inputs[i] = filter2_Value[i];
			} else		//This eliminates the initial transient of the filter
			{
				filter2_Value[i] = filter2_Value[i] + 1.0 * (mux2_inputs[i] - filter2_Value[i]);
				mux2_inputs[i] = filter2_Value[i];
			}

			ain2_values[i] = ((adc_values[1] & mask2) >>  mask2_shift);	//Only the ADC msb 11..5 are used
			//Check for ain2 variations
//			if (ain2_values[i] != prev_ain2_values[i])	//Uncomment to check only if current value changes
//			{
				if (!Test_Mode)
				{
					Ain2_Check(&ain2_buf[0], &ain2_values[i], i);
//					service_timer0_expired = true;					//Uncomment to send message on current change
				}
//			}
			memcpy((void*)&prev_ain2_values, (void*)&ain2_values, sizeof(ain2_values));	//Save ADC Channel 2 values
		} else
		{
			if ((HAL_GetTick() - Ovfl_Time1) > OVFL_TIMEOUT)
				BIT_SET(AnlgOvflStatusReg, i<<16);
//			strcpy((char*)&ovfl_2[i][0], "OVFL");
		}
	}

	AnlgOvflStatusReg &= mux_channels_enabled;	//Clear the two multiplexers unused channels overflow events
	//Read CPU Temperature
	Vsense = (float32_t)((((adc_values[2] & 0xFF0) * ADC_REFERENCE_VOLTAGE_MV) / ADC_MAX_OUTPUT_VALUE));
	CPU_Temp = (float32_t)(((TEMP_SENSOR_VOLTAGE_MV_AT_25 - Vsense) / TEMP_SENSOR_AVG_SLOPE_MV_PER_CELSIUS) + 25.0);
	//Vrsense = (float)(adc_values[2] & mask3);				//Read Internal Temperature (Row Data)
	//If in test-mode then display values.
	if (Test_Mode)
	{
		for (k = 0 ; k < num_anlg_mux_in; k++)
		{
			n = sprintf((char*)&dec_values[k][0], "Canale %u ...: %4.3f Volts", k, mux1_inputs[k]);
//			n = sprintf((char*)&dec_values[k][0], "Canale %u ...: %4.3f Volts", k, mux2_inputs[k]);
			for (j = 0 ; j < 5; j++)
			{	 if (k > 7)
				{
					if (k < 10)
					{
						L50_menu_items_row7b[1+j+(6*(k-8))] = dec_values[k][14+j];
					} else
					{
						L50_menu_items_row7b[1+j+(6*(k-8))] = dec_values[k][15+j];
					}
				} else
				{
					L50_menu_items_row7a[2+j+(6*k)] = dec_values[k][14+j];
				}
			}
		}
		if (n)
		{
			DisplayAnalogValues();
		}
	}
}

/*
 * @fn      read_ZE_sensors()
 * @brief   Read the voltage values from the Winsen ZE series Electrochemical Detection Modules
 * 			and converts them to the corresponding value in ppm
 * @return	null
**/
void read_ZE_sensors(void)
{
#if (CH2O_FROM_EC)
	extern int8_t CH2O_Corr;	//In mVolts: 1 = 1mV Correction
#endif
	extern int8_t O3_Corr;		//In mVolts: 1 = 1mV Correction

#if (ZE_SENSOR_TC)
	extern double_t Temperature;
	float32_t ZE08_TC, ZE25_TC, TOut;

	TOut = (float32_t)Temperature;
	ZE08_TC = ZE08_CH2O_TC2(TOut);	//T >= 20°C ZE08 sensor Temperature Compensation
	ZE25_TC = ZE25_O3_TC3(TOut);	//T >= 20°C ZE25 sensor Temperature Compensation

	if (TOut < 0.0)
	{
		ZE08_TC = ZE08_CH2O_TC1(TOut);	//T < 0°C ZE08 sensor Temperature Compensation
		ZE25_TC = ZE25_O3_TC1(TOut);	//T < 0°C ZE25 sensor Temperature Compensation
	} else
	if ((TOut >= 0.0) && (TOut < 5.0))
	{
		ZE25_TC = ZE25_O3_TC1(TOut);	//0°C =< T < 5°C ZE25 sensor Temperature Compensation
	} else
	if ((TOut >= 5.0) && (TOut < 20.0))
	{
		ZE25_TC = ZE25_O3_TC2(TOut);	//5°C =< T < 20°C ZE25 sensor Temperature Compensation
	}
#endif

#if (CH2O_FROM_EC)
	float32_t V0 = mux1_inputs[0] + ((float32_t)CH2O_Corr/1000.0);
#endif
	float32_t V1 = mux1_inputs[1] + ((float32_t)O3_Corr/1000.0);

#if (ZE_SENSOR_TC)
	V0 = V0/ZE08_TC;
	V1 = V1/ZE25_TC;
#endif

#if (CH2O_FROM_EC)
	//Calculate CH2O ppm
//	ppm_CH2O = (float32_t)fabs((double)ZE08_CH2O(V0));
	ppm_CH2O = (float32_t)ZE08_CH2O(V0);
	if (ppm_CH2O < 0)
		ppm_CH2O = 0;
#endif
	//Calculate O3 ppm
//	ppm_O3 = (float32_t)fabs((double)ZE08_CH2O(V1));
	ppm_O3 = (float32_t)ZE25_O3(V1);
	if (ppm_O3 < 0)
		ppm_O3 = 0;
}

/*
 * @fn      read_SMO_sensors() - (Semiconductor Metal Oxide Sensor)
 * @brief   Converts the voltage value read by the AD converter into the SMO sensor
 * 			resistive value (Rs). Then converts them to the corresponding value in ppm.
 * 			For Gas_Sensor_Board V1.0 it is calculated in this manner:
 * 			1) Connect the Non Inverting input to a well-know voltage source Vref
 * 			2) The output of the non-inverting amplifier is: Vad=Vref*(1+(Rf/Rs))
 * 			3) Calculate Rs=Rf/a, where a=(Vad/Vref)-1
 * 			For Gas_Sensor_Board V2.0 and following it is calculated in this manner:
 * 			1) The output of the unit-gain non-inverting amplifier is: Vad=Vref*(Rs/(Rf+Rs))
 * 			2) Calculate Rs=(Vad*Rf)/a, where a=(Vref-Vad)
 * @return	null
**/
void read_SMO_sensors(void)
{
#if (GSB_HW_VER == 10)
//	const float32_t VRef = 0.318;
	const float32_t VRef = 0.293;
	const float32_t AD_Sensitivity = 0.003;
#elif ((GSB_HW_VER == 20) || (GSB_HW_VER == 21))
	const float32_t VRef = 3.3;
#endif
#if !(CH2O_FROM_EC)
	extern int8_t CH2O_Corr;	//In mVolts: 1 = 1mV Correction
//	extern uint32_t SMD1001_CH2O_Rf;	//The load resistance of the IDM SMD1001 formaldehyde sensor is set at 10Kohm on the board
	extern uint32_t SMD1001_CH2O_Vo;	//In mVolts
	extern float32_t SMD1001_CH2O_Vs;
#endif
#if !(NO2_FROM_EC)
	extern uint32_t MiCS_6814_NO2_Rf;
	float32_t a_NO2;
	extern uint32_t MiCS_6814_NO2_Ro;	//In ohm
	extern float32_t MiCS_6814_NO2_Rs;	//In ohm
	extern int8_t NO2_Corr;		//In mVolts/10: 1 = 10mV Correction
#endif
	extern uint32_t MiCS_6814_NH3_Rf;
	extern uint32_t MiCS_6814_CO_Rf;
	float32_t a_NH3, a_CO;
	extern int8_t NH3_Corr;		//In mVolts/10: 1 = 10mV Correction
	extern int8_t CO_Corr;		//In mVolts/10: 1 = 10mV Correction
	extern uint32_t MiCS_6814_CO_Ro;	//In ohm
	extern uint32_t MiCS_6814_NH3_Ro;	//In ohm
	extern float32_t MiCS_6814_CO_Rs;	//In ohm
	extern float32_t MiCS_6814_NH3_Rs;	//In ohm
#if (SMO_SENSOR_TC)
	extern double_t Temperature;
	extern uint8_t Humidity;
	float32_t TOut, HOut;
#if !(CH2O_FROM_EC)
	float32_t SMD1001_CH2O_TC, SMD1001_CH2O_RHC;
#endif
#endif	//SMO_SENSOR_TC

#if (SMO_SENSOR_TC)
	TOut = (float32_t)Temperature;
	HOut = (float32_t)Humidity;
#endif

#if !(CH2O_FROM_EC)
	//Read and correct the CH2O sensor output voltage (SMD1001)
	float32_t V0 = mux1_inputs[0] + ((float32_t)CH2O_Corr/1000.0);
#endif
#if !(NO2_FROM_EC)
	//Read and correct the voltage across NO2 Rs
	float32_t V2 = mux1_inputs[2] + ((float32_t)NO2_Corr/100.0);
#endif
	//Read and correct the voltage across NH3 Rs
	float32_t V3 = mux1_inputs[3] + ((float32_t)NH3_Corr/100.0);
	//Read and correct the voltage across CO Rs
	float32_t V4 = mux1_inputs[4] + ((float32_t)CO_Corr/100.0);

#if (GSB_HW_VER == 10)
	//Check the voltage across NO2 Rs
	if (V2 < VRef)		//The minimum value of V2 so that MiCS_6814_NO2_Rs is a real value
		V2 = VRef + AD_Sensitivity;
	//Calculate NO2 Rs
#if !(NO2_FROM_EC)
	a_NO2 = (float32_t)fabs((double)((V2/VRef) - 1.0));
	MiCS_6814_NO2_Rs = (float32_t)(MiCS_6814_NO2_Rf)/a_NO2;
#endif
	//Check the voltage across NH3 Rs
	if (V3 < VRef)		//The minimum value of V3 so that MiCS_6814_NH3_Rs is a real value
		V3 = VRef + AD_Sensitivity;
	//Calculate NH3 Rs
	a_NH3 = (float32_t)fabs((double)((V3/VRef) - 1.0));
	MiCS_6814_NH3_Rs = (float32_t)(MiCS_6814_NH3_Rf)/a_NH3;
	//Check the voltage across CO Rs
	if (V4 < VRef)		//The minimum value of V4 so that MiCS_6814_CO_Rs is a real value
		V4 = VRef + AD_Sensitivity;
	//Calculate CO Rs
	a_CO  = (float32_t)fabs((double)((V4/VRef) - 1.0));
	MiCS_6814_CO_Rs = (float32_t)(MiCS_6814_CO_Rf)/a_CO;
#elif ((GSB_HW_VER == 20) || (GSB_HW_VER == 21))
#if !(CH2O_FROM_EC)
	//Calculate CH2O Vs
	SMD1001_CH2O_Vs = V0;
	if (SMD1001_CH2O_Vs < (SMD1001_CH2O_Vo/1000.0))	//The voltage value detected (Vs) cannot be less
		SMD1001_CH2O_Vs = (SMD1001_CH2O_Vo/1000.0);	//than that in pure air (Vo) (See SMD1001 data sheet)
#if (SMO_SENSOR_TC)
	//Calculate the SMD1001 temperature correction
	if (TOut < 10.0)
		SMD1001_CH2O_TC = SMD1001_CH2O_TC1(TOut);
	else
		SMD1001_CH2O_TC = SMD1001_CH2O_TC2(TOut);
	//Calculate the SMD1001 humidity correction
	SMD1001_CH2O_RHC = SMD1001_CH2O_RHC(HOut);
	//Apply the SMD1001 temperature correction
	SMD1001_CH2O_Vs = SMD1001_CH2O_Vs/SMD1001_CH2O_TC;
	//Apply the SMD1001 humidity correction
	SMD1001_CH2O_Vs = SMD1001_CH2O_Vs/SMD1001_CH2O_RHC;
#endif	//SMO_SENSOR_TC
#endif	//!(CH2O_FROM_EC)
#if !(NO2_FROM_EC)
	//Calculate NO2 Rs
	a_NO2 = VRef - V2;
	MiCS_6814_NO2_Rs = (V2 * (float32_t)MiCS_6814_NO2_Rf)/a_NO2;
#endif
	//Calculate NH3 Rs
	a_NH3 = VRef - V3;
	MiCS_6814_NH3_Rs =  (V3 * (float32_t)MiCS_6814_NH3_Rf)/a_NH3;
	//Calculate CO Rs
	a_CO  = VRef - V4;
	MiCS_6814_CO_Rs = (V4 * (float32_t)MiCS_6814_CO_Rf)/a_CO;
#endif	//((GSB_HW_VER == 20) || (GSB_HW_VER == 21))

#if (SMO_SENSOR_TC)
	//Apply the MiCS_6814 temperature/humidity correction
	MiCS_6814_NO2_Rs = MiCS_6814_TC(MiCS_6814_NO2_Rs,TOut,HOut);
	MiCS_6814_NH3_Rs = MiCS_6814_TC(MiCS_6814_NH3_Rs,TOut,HOut);
	MiCS_6814_CO_Rs = MiCS_6814_TC(MiCS_6814_CO_Rs,TOut,HOut);
#endif

#if !(CH2O_FROM_EC)
	//Calculate CH2O ppm
	float32_t Arg_CH2O = SMD1001_CH2O_Vs/((float32_t)(SMD1001_CH2O_Vo/1000));	//SMD1001_CH2O_Vo is stored in mVolts!
	if (Arg_CH2O <= 1.83)
		ppm_CH2O = SMD1001_CH2O_1(Arg_CH2O);
	else
		ppm_CH2O = SMD1001_CH2O_2(Arg_CH2O);
	if (ppm_CH2O < 0)
		ppm_CH2O = 0;
#endif
#if !(NO2_FROM_EC)
	//Calculate NO2 ppm
	float32_t Arg_NO2 = MiCS_6814_NO2_Rs/((float32_t)MiCS_6814_NO2_Ro);
	ppm_NO2 = MiCS_6814_NO2((double)(Arg_NO2));
#endif
	//Calculate NH3 ppm
	float32_t Arg_NH3 = MiCS_6814_NH3_Rs/((float32_t)MiCS_6814_NH3_Ro);
	ppm_NH3 = MiCS_6814_NH3((double)(Arg_NH3));
	//Calculate CO ppm
	float32_t Arg_CO = MiCS_6814_CO_Rs/((float32_t)MiCS_6814_CO_Ro);
	ppm_CO = MiCS_6814_CO((double)(Arg_CO));
}

/*
 * @fn      read_EC_sensors() - (ElectroChemical Sensors)
 * @brief   Read the voltage values from the Electrochemical Sensors
 * 			and converts them to the corresponding value in ppm
 * @return	null
**/
void read_EC_sensors(void)
{
	extern int8_t SO2_Corr;		//In mVolts: 1 = 1mV Correction
#if (GSB_HW_VER == 10)
	extern int8_t C6H6_Corr;	//In mVolts: 1 = 1mV Correction
#elif ((GSB_HW_VER == 20) || (GSB_HW_VER == 21))
	extern int8_t NO2_Corr;		//In mVolts: 1 = 1mV Correction
#endif
#if (EC_SENSOR_TC)
	extern double_t Temperature;
	float32_t SO2_TC, TOut;
#if (GSB_HW_VER == 10)
	float32_t C6H6_TC;
#elif ((GSB_HW_VER == 20) || (GSB_HW_VER == 21))
	float32_t NO2_TC;
#endif

	TOut = (float32_t)Temperature;
	SO2_TC = ME4_SO2_TC(TOut);		//ME4_SO2 sensor Temperature Compensation
#if (GSB_HW_VER == 10)
	C6H6_TC = ME4_C6H6_TC(TOut);	//ME4_C6H6 sensor Temperature Compensation
#elif ((GSB_HW_VER == 20) || (GSB_HW_VER == 21))
	NO2_TC = ME4_NO2_TC(TOut);		//ME4_NO2 sensor Temperature Compensation
#endif
#endif	//EC_SENSOR_TC

	float32_t V5 = mux1_inputs[5] + ((float32_t)SO2_Corr/1000.0);
#if (GSB_HW_VER == 10)
	float32_t V6 = mux1_inputs[6] + ((float32_t)C6H6_Corr/1000.0);
#elif (GSB_HW_VER == 20)
	float32_t V6 = mux1_inputs[10] + ((float32_t)NO2_Corr/1000.0);
#elif (GSB_HW_VER == 21)
	float32_t V6 = mux1_inputs[6] + ((float32_t)NO2_Corr/1000.0);
#endif

#if (EC_SENSOR_TC)
	V5 = V5/SO2_TC;
#if (GSB_HW_VER == 10)
	V6 = V6/C6H6_TC;
#elif ((GSB_HW_VER == 20) || (GSB_HW_VER == 21))
	V6 = V6/NO2_TC;
#endif
#endif

	//Calculate SO2 ppm
//	ppm_SO2 = (float32_t)fabs((double)ME4_SO2(V5));
	ppm_SO2 = (float32_t)ME4_SO2(V5);
	if (ppm_SO2 < 0)
		ppm_SO2 = 0;
#if (GSB_HW_VER == 10)
	//Calculate C6H6 ppm
//	ppm_C6H6 = (float32_t)fabs((double)ME4_C6H6(V6));
	ppm_C6H6 = (float32_t)ME4_C6H6(V6);
	if (ppm_C6H6 < 0)
		ppm_C6H6 = 0;
#elif ((GSB_HW_VER == 20) || (GSB_HW_VER == 21))
	//Calculate NO2 ppm
#if (NO2_FROM_EC)
//	ppm_NO2 = (float32_t)fabs((double)ME4_NO2(V6));
	ppm_NO2 = (float32_t)ME4_NO2(V6);
	if (ppm_NO2 < 0)
		ppm_NO2 = 0;
#endif
#endif
}

/*
  * @brief  Updates the total analog values read from gas sensors.
  * @param  Pointer to the buffer that stores the data read.
  * @retval Interface status (MANDATORY: return 0 -> no Error).
 */
ANLG_Error_et ANLG_Get_Measurement(ANLG_MeasureTypeDef_st *Measurement_Value)
{
	/* The integration window of the analog sensors is set at 5 minutes (300/5),
	 * to prevent intense but short-term polluting events from distorting the
	 * calculation of the air quality in the long term.
	 * (Eg: the housewife who throws the white wine into the roast or use
	 * a product containing alcohol for cleaning...!! :))
	 *
	 * The moving average filter has two time constants (the integration windows):
	 * the first (used when the current value is greater than the average value
	 * calculated up to then) is given by the value of the "AverageWindow_5m" constant.
	 * The second (used when the current value is less than or equal to the average
	 * value calculated up to then) is given by the value of the "AverageWindow_1m" constant.
	 */
	static const uint32_t AverageWindow_5m = 60;	//Analog sensors integration window of the is fixed at 5 minutes
	static const uint32_t AverageWindow_1m = 12;	//Analog sensors integration window1 of the is fixed at 1 minutes
	static float32_t ch2o_avg = 0; static float32_t o3_avg = 0;
	static float32_t no2_avg = 0; static float32_t nh3_avg = 0;
	static float32_t co_avg = 0; static float32_t so2_avg = 0;
	static float32_t ch2o_avg1 = 0; static float32_t o3_avg1 = 0;
	static float32_t no2_avg1 = 0; static float32_t nh3_avg1 = 0;
	static float32_t co_avg1 = 0; static float32_t so2_avg1 = 0;
	static float32_t ch2o_new_sample, o3_new_sample, no2_new_sample, nh3_new_sample;
	static float32_t co_new_sample, so2_new_sample;
	ANLG_Error_et ret = 0;

	if(!Test_Mode)
	{
		read_analogs();
		read_ZE_sensors();
		read_SMO_sensors();
		read_EC_sensors();
	}

	Measurement_Value->CH2O = CH2O_ppm2ugm3(ppm_CH2O);	//Calculate the CH2O concentration in ug/m3
	ch2o_new_sample = Measurement_Value->CH2O;
	ch2o_avg = approxMovingAverage(ch2o_avg, ch2o_new_sample, AverageWindow_5m);
	ch2o_avg1 = approxMovingAverage(ch2o_avg1, ch2o_new_sample, AverageWindow_1m);
	if (ch2o_avg1 > ch2o_avg)
		Measurement_Value->CH2O = ch2o_avg;
	else
		Measurement_Value->CH2O = ch2o_avg1;

	Measurement_Value->O3 = O3_ppm2ugm3(ppm_O3);		//Calculate the O3 concentration in ug/m3
	o3_new_sample = Measurement_Value->O3;
	o3_avg = approxMovingAverage(o3_avg, o3_new_sample, AverageWindow_5m);
	o3_avg1 = approxMovingAverage(o3_avg1, o3_new_sample, AverageWindow_1m);
	if (o3_avg1 > o3_avg)
		Measurement_Value->O3 = o3_avg;
	else
		Measurement_Value->O3 = o3_avg1;

	Measurement_Value->NO2 = NO2_ppm2ugm3(ppm_NO2);		//Calculate the NO2 concentration in ug/m3
	no2_new_sample = Measurement_Value->NO2;
	no2_avg = approxMovingAverage(no2_avg, no2_new_sample, AverageWindow_5m);
	no2_avg1 = approxMovingAverage(no2_avg1, no2_new_sample, AverageWindow_1m);
	if (no2_avg1 > no2_avg)
		Measurement_Value->NO2 = no2_avg;
	else
		Measurement_Value->NO2 = no2_avg1;

	Measurement_Value->NH3 = NH3_ppm2ugm3(ppm_NH3);		//Calculate the NH3 concentration in ug/m3
	nh3_new_sample = Measurement_Value->NH3;
	nh3_avg = approxMovingAverage(nh3_avg, nh3_new_sample, AverageWindow_5m);
	nh3_avg1 = approxMovingAverage(nh3_avg1, nh3_new_sample, AverageWindow_1m);
	if (nh3_avg1 > nh3_avg)
		Measurement_Value->NH3 = nh3_avg;
	else
		Measurement_Value->NH3 = nh3_avg1;

	Measurement_Value->CO = CO_ppm2ugm3(ppm_CO);		//Calculate the CO concentration in mg/m3
	co_new_sample = Measurement_Value->CO;
	co_avg = approxMovingAverage(co_avg, co_new_sample, AverageWindow_5m);
	co_avg1 = approxMovingAverage(co_avg1, co_new_sample, AverageWindow_1m);
	if (co_avg1 > co_avg)
		Measurement_Value->CO = co_avg;
	else
		Measurement_Value->CO = co_avg1;

	Measurement_Value->SO2 = SO2_ppm2ugm3(ppm_SO2);		//Calculate the SO2 concentration in ug/m3
	so2_new_sample = Measurement_Value->SO2;
	so2_avg = approxMovingAverage(so2_avg, so2_new_sample, AverageWindow_5m);
	so2_avg1 = approxMovingAverage(so2_avg1, so2_new_sample, AverageWindow_1m);
	if (so2_avg1 > so2_avg)
		Measurement_Value->SO2 = so2_avg;
	else
		Measurement_Value->SO2 = so2_avg1;

	Measurement_Value->C6H6  = 0;						//Calculate the C6H6 concentration in ug/m3
//	Measurement_Value->C6H6  = C6H6_ppm2ugm3(ppm_C6H6);	//Calculate the C6H6 concentration in ug/m3
	Measurement_Value->AIN8  = ain1_buf[7];
	Measurement_Value->AIN9  = ain1_buf[8];
	Measurement_Value->AIN10 = ain1_buf[9];
	Measurement_Value->AIN11 = ain1_buf[10];
	Measurement_Value->AIN12 = ain1_buf[11];
	Measurement_Value->AIN13 = ain1_buf[12];
	Measurement_Value->AIN14 = ain1_buf[13];
	Measurement_Value->AIN15 = ain1_buf[14];
	Measurement_Value->AIN16 = ain1_buf[15];

	return ret;
}
