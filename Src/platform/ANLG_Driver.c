/*
 * ANLG_Driver.c
 *
 *  Created on: 10 apr 2021
 *      Author: Tommaso Sabatini
 */

#include "platform/ANLG_Driver.h"
#include "compiler/compiler.h"

float R1 = 1500.0; float R2 = 56000.0; float Vref = 3.3;
uint16_t ADC_RESOLUTION = 4096;	//ADC_RESOLUTION is 4096 for 12 bit, 1024 for 10 bit and 256 for 8 bit
uint8_t ain1_buf[16], ain2_buf[16];
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
			b[15] = mask;
		}
		break;
		default:
			break;
	}
}

/* @fn 		Ain1_Check()
 * @brief 	Check if there was an event from an analog input and manage it
 * @return	null
 * */
void Ain2_Check(uint8_t* b, uint8_t* a, AIN2_FUNCTION An_Func)
{
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
		case AIN_14_2_:
		{
			mask = *a;
			b[14] = mask;
		}
		break;
		case AIN_15_2_:
		{
			mask = *a;
			b[15] = mask;
		}
		break;
		default:
			break;
	}
}

void read_analogs(void)
{
	uint8_t i, j, k, n;
	uint16_t mask1 = 0xFF8;	//4088: Filter for 7mV sensitivity
	const uint8_t mask1_shift = 4;
//	uint16_t mask1 = 0xFF0;	//4080: Filter for 13mV sensitivity
//	uint16_t mask1 = 0xF80;	//3968: Filter for 100mV sensitivity
	uint16_t mask2 = 0xFF0;	//4080: Filter for 13mV sensitivity
	const uint8_t mask2_shift = 4;
//	uint16_t mask2 = 0xF80;	//3968: Filter for 100mV sensitivity
//	uint16_t mask2 = 0xFC0;	//4032: Filter for 52mV sensitivity
	uint16_t mask3 = 0xFE0;	//4064: Filter for 26mV sensitivity
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
//			memset(&ovfl_1[i][0], 0x00, 8);
			Ovfl_Time0 = HAL_GetTick();
			BIT_CLEAR(AnlgOvflStatusReg, i);
			mux1_inputs[i] = (float)(((((adc_values[0] & mask1)) * ((R1 + R2) / R2) * Vref)) / ADC_RESOLUTION);
			ain1_values[i] = ((adc_values[0] & mask1) >> mask1_shift);	//Only the ADC msb 11..5 are used
			//Check for ain1 variations
//			if ((ain1_values[i] != prev_ain1_values[i]) || (service_timer0_expired))	//Uncomment to check only if analog input changes
//			{
				Ain1_Check(&ain1_buf[0], &ain1_values[i], i);
//				service_timer0_expired = true;					//Uncomment to send message on analog input change
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
//			memset(&ovfl_2[i][0], 0x00, 8);
			Ovfl_Time1 = HAL_GetTick();
			BIT_CLEAR(AnlgOvflStatusReg, i<<16);
			mux2_inputs[i] = (float)(((((adc_values[1] & mask2)) * ((R1 + R2) / R2) * Vref)) / ADC_RESOLUTION);
			ain2_values[i] = ((adc_values[1] & mask2) >>  mask2_shift);	//Only the ADC msb 11..5 are used
			//Check for ain2 variations
//			if (ain2_values[i] != prev_ain2_values[i])	//Uncomment to check only if current value changes
//			{
				Ain2_Check(&ain2_buf[0], &ain2_values[i], i);
//				service_timer0_expired = true;					//Uncomment to send message on current change
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
//	Vsense = (float)((((adc_values[2] & mask3) * Vref) / ADC_RESOLUTION));	//Read Internal Temperature (Volt)
	Vrsense = (float)(adc_values[2] & mask3);								//Read Internal Temperature (Row Data)
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
			CDC_Transmit_FS(L50_menu_items_row7a,strlen((const char*)L50_menu_items_row7a));
			HAL_Delay(CDC_delay);
			CDC_Transmit_FS(L50_menu_items_row7b,strlen((const char*)L50_menu_items_row7b));
			HAL_Delay(CDC_delay);
			CDC_Transmit_FS(L50_menu_items_row4a,strlen((const char*)L50_menu_items_row4a));
			HAL_Delay(CDC_delay);
			CDC_Transmit_FS(L50_menu_items_row4b,strlen((const char*)L50_menu_items_row4b));
			HAL_Delay(CDC_delay);
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
	extern int8_t CH2O_Corr;	//In mVolts/10In mVolts/10: 1 = 10mV Correction
	extern int8_t O3_Corr;		//In mVolts/10In mVolts/10: 1 = 10mV Correction

	float32_t V0 = mux1_inputs[0] + ((float32_t)CH2O_Corr/100.0);
	float32_t V1 = mux1_inputs[1] + ((float32_t)O3_Corr/100.0);

	//Calculate CH2O ppm
//	ppm_CH2O = (float32_t)fabs((double)ZE08_CH2O(V0));
	ppm_CH2O = (float32_t)ZE08_CH2O(V0);
	if (ppm_CH2O < 0)
		ppm_CH2O = 0;
	//Calculate O3 ppm
//	ppm_O3 = (float32_t)fabs((double)ZE08_CH2O(V1));
	ppm_O3 = (float32_t)ZE08_CH2O(V1);
	if (ppm_O3 < 0)
		ppm_O3 = 0;
}

/*
 * @fn      read_SMO_sensors() - (Semiconductor Metal Oxide Sensor)
 * @brief   Converts the voltage value read by the AD converter into the SMO sensor
 * 			resistive value (Rs). Then converts them to the corresponding value in ppm.
 * 			It is calculated in this manner:
 * 			1) Connect the Non Inverting input to a well-know voltage source Vref
 * 			2) The output of the non-inverting amplifier is: Vad=Vref*(1+(Rf/Rs))
 * 			3) Calculate Rs=Rf/a, where a=(Vad/Vref)-1
 * @return	null
**/
void read_SMO_sensors(void)
{
//	const float32_t VRef = 0.318;
	const float32_t VRef = 0.293;
	const float32_t Rf_NO2 = 6340.0;
	const float32_t Rf_NH3 = 80600.0;
	const float32_t Rf_CO = 806000.0;
	float32_t a_NO2, a_NH3, a_CO;
	extern int8_t NO2_Corr;		//In mVolts/10In mVolts/10: 1 = 10mV Correction
	extern int8_t NH3_Corr;		//In mVolts/10In mVolts/10: 1 = 10mV Correction
	extern int8_t CO_Corr;		//In mVolts/10In mVolts/10: 1 = 10mV Correction
	extern uint32_t MiCS_6814_CO_Ro;	//In ohm
	extern uint32_t MiCS_6814_NH3_Ro;	//In ohm
	extern uint32_t MiCS_6814_NO2_Ro;	//In ohm
	extern float32_t Rs_CO;				//In ohm
	extern float32_t Rs_NO2;			//In ohm
	extern float32_t Rs_NH3;			//In ohm

	float32_t V2 = mux1_inputs[2] + ((float32_t)NO2_Corr/100.0);
	float32_t V3 = mux1_inputs[3] + ((float32_t)NH3_Corr/100.0);
	float32_t V4 = mux1_inputs[4] + ((float32_t)CO_Corr/100.0);

	a_NO2 = (float32_t)fabs((double)((V2/VRef) - 1.0));
	a_NH3 = (float32_t)fabs((double)((V3/VRef) - 1.0));
	a_CO  = (float32_t)fabs((double)((V4/VRef) - 1.0));

	//Calculate NO2 ppm
	Rs_NO2 = Rf_NO2/a_NO2;
	ppm_NO2 = MiCS_6814_NO2((double)(Rs_NO2/MiCS_6814_NO2_Ro));

	//Calculate NH3 ppm
	Rs_NH3 = Rf_NH3/a_NH3;
	ppm_NH3 = MiCS_6814_NH3((double)(Rs_NH3/MiCS_6814_NH3_Ro));

	//Calculate CO ppm
	Rs_CO = Rf_CO/a_CO;
	ppm_CO = MiCS_6814_CO((double)(Rs_CO/MiCS_6814_CO_Ro));
}

/*
 * @fn      read_ME_sensors()
 * @brief   Read the voltage values from the Winsen ME series Electrochemical Detection Modules
 * 			and converts them to the corresponding value in ppm
 * @return	null
**/
void read_ME_sensors(void)
{
	extern int8_t SO2_Corr;		//In mVolts/10In mVolts/10: 1 = 10mV Correction
	extern int8_t C6H6_Corr;	//In mVolts/10In mVolts/10: 1 = 10mV Correction

	float32_t V5 = mux1_inputs[5] + ((float32_t)SO2_Corr/100.0);
	float32_t V6 = mux1_inputs[6] + ((float32_t)C6H6_Corr/100.0);

	//Calculate SO2 ppm
//	ppm_SO2 = (float32_t)fabs((double)ME4_SO2(V5));
	ppm_SO2 = (float32_t)ME4_SO2(V5);
	if (ppm_SO2 < 0)
		ppm_SO2 = 0;
	//Calculate C6H6 ppm
//	ppm_C6H6 = (float32_t)fabs((double)ME4_C6H6(V6));
	ppm_C6H6 = (float32_t)ME4_C6H6(V6);
	if (ppm_C6H6 < 0)
		ppm_C6H6 = 0;
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
	 * calculated up to then) is given by the value of the "AverageWindow" constant.
	 * The second (used when the current value is less than or equal to the average
	 * value calculated up to then) is given by the value of the "AverageWindow1" constant.
	 */
	static const uint32_t AverageWindow = 60;	//Analog sensors integration window of the is fixed at 5 minutes
	static const uint32_t AverageWindow1 = 12;	//Analog sensors integration window1 of the is fixed at 1 minutes
	const float32_t CorrectionFactor = (1+(1/AverageWindow));
	const float32_t CorrectionFactor1 = (1+(1/AverageWindow1));
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
		read_ME_sensors();
	}

	Measurement_Value->CH2O = CH2O_ppm2ugm3(ppm_CH2O);	//Calculate the CH2O concentration in ug/m3
	ch2o_new_sample = Measurement_Value->CH2O;
	ch2o_avg = approxMovingAverage(ch2o_avg, ch2o_new_sample, AverageWindow, CorrectionFactor);
	ch2o_avg1 = approxMovingAverage(ch2o_avg1, ch2o_new_sample, AverageWindow1, CorrectionFactor1);
	if (ch2o_avg1 > ch2o_avg)
		Measurement_Value->CH2O = ch2o_avg;
	else
		Measurement_Value->CH2O = ch2o_avg1;

	Measurement_Value->O3 = O3_ppm2ugm3(ppm_O3);		//Calculate the O3 concentration in ug/m3
	o3_new_sample = Measurement_Value->O3;
	o3_avg = approxMovingAverage(o3_avg, o3_new_sample, AverageWindow, CorrectionFactor);
	o3_avg1 = approxMovingAverage(o3_avg1, o3_new_sample, AverageWindow1, CorrectionFactor1);
	if (o3_avg1 > o3_avg)
		Measurement_Value->O3 = o3_avg;
	else
		Measurement_Value->O3 = o3_avg1;

	Measurement_Value->NO2 = NO2_ppm2ugm3(ppm_NO2);		//Calculate the NO2 concentration in ug/m3
	no2_new_sample = Measurement_Value->NO2;
	no2_avg = approxMovingAverage(no2_avg, no2_new_sample, AverageWindow, CorrectionFactor);
	no2_avg1 = approxMovingAverage(no2_avg1, no2_new_sample, AverageWindow1, CorrectionFactor1);
	if (no2_avg1 > no2_avg)
		Measurement_Value->NO2 = no2_avg;
	else
		Measurement_Value->NO2 = no2_avg1;

	Measurement_Value->NH3 = NH3_ppm2ugm3(ppm_NH3);		//Calculate the NH3 concentration in ug/m3
	nh3_new_sample = Measurement_Value->NH3;
	nh3_avg = approxMovingAverage(nh3_avg, nh3_new_sample, AverageWindow, CorrectionFactor);
	nh3_avg1 = approxMovingAverage(nh3_avg1, nh3_new_sample, AverageWindow1, CorrectionFactor1);
	if (nh3_avg1 > nh3_avg)
		Measurement_Value->NH3 = nh3_avg;
	else
		Measurement_Value->NH3 = nh3_avg1;

	Measurement_Value->CO = CO_ppm2ugm3(ppm_CO);		//Calculate the CO concentration in mg/m3
	co_new_sample = Measurement_Value->CO;
	co_avg = approxMovingAverage(co_avg, co_new_sample, AverageWindow, CorrectionFactor);
	co_avg1 = approxMovingAverage(co_avg1, co_new_sample, AverageWindow1, CorrectionFactor1);
	if (co_avg1 > co_avg)
		Measurement_Value->CO = co_avg;
	else
		Measurement_Value->CO = co_avg1;

	Measurement_Value->SO2 = SO2_ppm2ugm3(ppm_SO2);		//Calculate the SO2 concentration in ug/m3
	so2_new_sample = Measurement_Value->SO2;
	so2_avg = approxMovingAverage(so2_avg, so2_new_sample, AverageWindow, CorrectionFactor);
	so2_avg1 = approxMovingAverage(so2_avg1, so2_new_sample, AverageWindow1, CorrectionFactor1);
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

