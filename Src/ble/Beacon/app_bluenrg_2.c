/**
  ******************************************************************************
  * @file    app_bluenrg_2.c
  * @author  SRA Application Team
  * @brief   BlueNRG-2 initialization and applicative code
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include "OpModes.h"		//Added By Me!!!
#if (BEACON_APP==1)			//Added By Me!!!

/* Includes ------------------------------------------------------------------*/
#include "Beacon/app_bluenrg_2.h"
#include "bluenrg_conf.h"
#include "hci.h"
#include "bluenrg1_types.h"
#include "bluenrg1_gap.h"
#include "bluenrg1_aci.h"
#include "bluenrg1_hci_le.h"

/* USER CODE BEGIN Includes */
#include "Beacon/gatt_db.h"
#include "platform/port.h"
/* USER CODE END Includes */

/* Private macros ------------------------------------------------------------*/
/** @brief Macro that stores Value into a buffer in Little Endian Format (2 bytes)*/
#define HOST_TO_LE_16(buf, val)    ( ((buf)[0] =  (uint8_t) (val)    ) , \
                                   ((buf)[1] =  (uint8_t) (val>>8) ) )

/** @brief Macro that stores Value into a buffer in Little Endian Format (4 bytes) */
#define HOST_TO_LE_32(buf, val)    ( ((buf)[0] =  (uint8_t) (val)     ) , \
                                   ((buf)[1] =  (uint8_t) (val>>8)  ) , \
                                   ((buf)[2] =  (uint8_t) (val>>16) ) , \
                                   ((buf)[3] =  (uint8_t) (val>>24) ) )

/** @brief Macro that stores Value into a buffer in Big Endian Format (2 bytes) */
#define HOST_TO_BE_16(buf, val)    ( ((buf)[1] =  (uint8_t) (val)    ) , \
                                   ((buf)[0] =  (uint8_t) (val>>8) ) )

/** @brief Macro that stores Value into a buffer in Big Endian Format (4 bytes) */
#define HOST_TO_BE_32(buf, val)    ( ((buf)[3] =  (uint8_t) (val)     ) , \
                                   ((buf)[2] =  (uint8_t) (val>>8)  ) , \
                                   ((buf)[1] =  (uint8_t) (val>>16) ) , \
                                   ((buf)[0] =  (uint8_t) (val>>24) ) )

/* Private define ------------------------------------------------------------*/
/* Set to 1 for enabling Flags AD Type position at the beginning
   of the advertising packet */
#define ENABLE_FLAGS_AD_TYPE_AT_BEGINNING 0

#if (ADV_DATA_TYPE != ADV_NONCONN_IND)
   #warning "The advertising data type should be of 'non-connectable and no scan response' type."
#endif
#if (ADV_INTERV_MIN > ADV_INTERV_MAX)
   #error "The min advertising interval cannot be greater than the max advertising interval."
#endif
#if ((ADV_INTERV_MIN <= 0) || (ADV_INTERV_MIN > 40959))
  #error "Invalid min advertising interval! Please select a value between 0 and 40959 ms."
#endif
#if ((ADV_INTERV_MAX <= 0) || (ADV_INTERV_MAX > 40959))
  #error "Invalid max advertising interval! Please select a value between 0 and 40959 ms."
#endif

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
volatile uint32_t HCI_ProcessEvent;
//uint8_t  local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'B','l','u','e','N','R','G','_','B','e','a','c','o','n'};
uint8_t  local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME, DEVICE_NAME};

uint8_t scan_resp_data[] =
{
	// Set advertising device name as Local Node Name
	11, AD_TYPE_COMPLETE_LOCAL_NAME,
	DEVICE_NAME			//Local Name
};

uint8_t manuf_data[] =
{
	// Advertising data: Flags AD Type
	0x02, AD_TYPE_FLAGS,
	0x06,		//AD_TYPE_FLAGS Payload
	// Advertising data: manufacturer specific data
	27, AD_TYPE_MANUFACTURER_SPECIFIC_DATA,
	0xFF, 0xFF,										//CID: Test Company identifier code (My CID to be assigned)
	0x55,											//Device ID (manuf_data[7])
	0x00,											//Custom Data (manuf_data[8])
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	//Custom Data (manuf_data[9..16])
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	//Custom Data (manuf_data[17..24])
	0x00, 0x00,										//Custom Data (manuf_data[25..26])
	0x00, 0x00,										//Custom Data (manuf_data[27..28])
	0x00,        									//Custom Data (manuf_data[29])
	0x80        									//Packet Identifier (AdvCodes - manuf_data[30])
};

/*uint8_t manuf_data[] =
{
	// Advertising data: Flags AD Type
	0x02, AD_TYPE_FLAGS,
	0x06,		//AD_TYPE_FLAGS Payload
	// Advertising data: manufacturer specific data
	3, AD_TYPE_16_BIT_SERV_UUID,
	0x30, 0x00,		//CID: Company identifier code (Default is 0x0030 - STMicroelectronics: To be customized for specific identifier)
	23, AD_TYPE_MANUFACTURER_SPECIFIC_DATA,
	0x55,											//Custom Data (manuf_data[9])
	0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,	//Custom Data (manuf_data[10..17])
	0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10,	//Custom Data (manuf_data[18..25])
	0x11, 0x12,										//Custom Data (manuf_data[26..27])
	0x13, 0x14,										//Custom Data (manuf_data[28..29])
	0x15        									//Packet Identifier (AdvCodes - manuf_data[30])
//	0xC8        									//2's complement of the Tx power (-56dBm at 1meter);
};*/

const uint8_t AdvCodes[] =	//manuf_data[30]
{
	0x01,		//Environmental code: Adv ID 0x01
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
//
	0x02,		//Air quality absolute values: Adv ID = 0x02
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
//
	0x03,		//Air quality averaged values: Adv ID = 0x03
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
//
	0x04,		//Air pollution absolute and averaged: Adv ID = 0x04
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
//
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80,		//Status Report: Adv ID = 0x80
	0x80		//Status Report: Adv ID = 0x80
};

/*
 * N_ADV_CODES specifies how often (in seconds) a Beacon with a given Adv_Code is sent.
 * For example, when N_ADV_CODES is equal to 60, Beacons with Adv_Code 0xXX created by
 * the "MX_BlueNRG_2_Process" function, will be sent every minute.
 * Since each Adv_Code specified in the Adv_Codes[] array is sent every second,
 * then the Status Report Beacon, with Adv_Code = 0x80, will be sent every second.
 */
#define N_ADV_CODES 60
//#define N_ADV_CODES 5
/*
 * SecondaryBeaconSwitchTime specifies how often a set of Secondary Beacons,
 * created by the "MX_BlueNRG_2_Process" function, will be sent.
 * In this case a set of Secondary Beacons will be sent every hour.
 */
uint16_t SecondaryBeaconSwitchTime = (uint16_t)((3600 / N_ADV_CODES) / (ADV_INTERV_MAX / 1600));
//uint16_t SecondaryBeaconSwitchTime = 10;
const uint8_t man_data_offset = 2;	//Advertising data: manufacturer specific data offset
uint8_t Adv_Code = 0;
uint8_t beacon_number = 0;
bool adv_data_sent = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
#if (USE_STM32F4XX_NUCLEO==1)
	static void User_Init(void);
#endif
static void User_Process(void);
static void Device_Init(void);
static void Start_Beaconing(void);

void EnterStopMode(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

#if PRINT_CSV_FORMAT
extern volatile uint32_t ms_counter;
/**
 * @brief  This function is a utility to print the log time
 *         in the format HH:MM:SS:MSS (ST BlueNRG GUI time format)
 * @param  None
 * @retval None
 */
void print_csv_time(void){
  uint32_t ms = HAL_GetTick();
  PRINT_CSV("%02ld:%02ld:%02ld.%03ld", (long)(ms/(60*60*1000)%24), (long)(ms/(60*1000)%60), (long)((ms/1000)%60), (long)(ms%1000));
}
#endif

void MX_BlueNRG_2_Init(void)
{
  /* USER CODE BEGIN SV */

  /* USER CODE END SV */

  /* USER CODE BEGIN BlueNRG_2_Init_PreTreatment */

  /* USER CODE END BlueNRG_2_Init_PreTreatment */

  /* Initialize the peripherals and the BLE Stack */
#if (USE_STM32F4XX_NUCLEO==1)
  User_Init();
#endif
//hci_init(NULL, NULL);
  hci_init(APP_UserEvtRx, NULL);	//APP_UserEvtRx: ACI events callback function pointer This callback is
  	  	  	  	  	  	  	  	  	//triggered when an user event is received from the BLE core device.
  /* Init the Device */
  Device_Init();

  /* Start Beacon Non Connectable Mode*/
  Start_Beaconing();

  PRINT_DBG("BlueNRG-2 BLE Beacon Application\r\n");

  /* USER CODE BEGIN BlueNRG_2_Init_PostTreatment */

  /* USER CODE END BlueNRG_2_Init_PostTreatment */
}

#if (MY_BEACON)
/*
 *  @brief	BlueNRG-2 background task; fills the beacon payload as follows:
 *
 * Environmental Data: manuf_data[30] = 0x01;
 * manuf_data[8..9]: T_Out; manuf_data[10..13]: P0_Out; manuf_data[14..15]: Hum_Out
 * manuf_data[16..17]: T2_Out; manuf_data[18..19]: T3_Out; manuf_data[20..23]: P_Out;
 * manuf_data[24]: forecast;
 *
 * Air Quality Data: manuf_data[30] = 0x02;
 * manuf_data[8..9]: eq_TVOC; manuf_data[10..11]: eq_CO2; manuf_data[12..13]: CO
 * manuf_data[14..15]: NO2; manuf_data[16..17]: NH3; manuf_data[18..19]: CH2O;
 * manuf_data[20..21]: O3; manuf_data[22..23]: SO2; manuf_data[24..25]: C6H6;
 * manuf_data[29]: Gas_AQI; bit 7 = RiskReport
 *
 * Air Quality Data - Average values: manuf_data[30] = 0x03;
 * manuf_data[8..9]: eq_TVOC_1h_Mean; manuf_data[10..11]: eq_CO2_1h_Mean; manuf_data[12..13]: CO_8h_Mean
 * manuf_data[14..15]: NO2_1h_Mean; manuf_data[16..17]: NH3_8h_Mean; manuf_data[18..19]: CH2O_8h_Mean;
 * manuf_data[20..21]: O3_1h_Mean; manuf_data[22..23]: SO2_1h_Mean; manuf_data[24..25]: C6H6_24h_Mean;
 * manuf_data[29]: AVG_Gas_AQI; bit 7 = RiskReport
 *
 * Air Pollution Data: manuf_data[30] = 0x04
 * manuf_data[8..9]: MC_1p0; manuf_data[10..11]: MC_2p5; manuf_data[12..13]: MC_4p0;
 * manuf_data[14..15]: MC_10p0; manuf_data[16..17]: NC_0p5;
 * manuf_data[18..19]: MC_1p0_24h_Mean; manuf_data[20..21]: MC_2p5_24h_Mean; manuf_data[22..23]: MC_4p0_24h_Mean;
 * manuf_data[24..25]: MC_10p0_24h_Mean;
 * manuf_data[28], manuf_data[29]: AVG_PMx_AQI;
 *
 * Status Report: manuf_data[30] = 0x05
 * manuf_data[8..11]: SensorStatusReg; manuf_data[12..15]: AnlgOvflStatusReg;
 *
 * @param  None
 * @retval None
 */
void MX_BlueNRG_2_Process(void)
{
	/* USER CODE BEGIN BlueNRG_2_Process_PreTreatment */
#if (USE_SENSUS191==1)
//	uint16_t Lux_Out = 0x55AA;
	static uint8_t ret = 0;
	uint8_t mycmd = 0;
	bool ConfCmd = false;

//	beacon_number = 0;
/*
 * The data types that the beacon packet can contain (Environmental, Air Quality, Air Pollution)
 * are distinguished by the value of Adv_Code, entered in the Minor_Number field
 */
	do
	{
		mycmd = app.usbbuf[0];
//		mycmd = usart3app.usartbuf[0];
		ConfCmd = (bool)(((mycmd == 0x10) || (mycmd == 0x7F) || (mycmd == 0x12)));

		memset(&manuf_data[8], 0xFF, 22);

		manuf_data[man_data_offset+28] = Adv_Code;
		if(Adv_Code == 0x01)	//Main Beacon Adv_Code 0x01
		{
			//Send Environmental Data
			HOST_TO_LE_16(manuf_data+man_data_offset+6, T_Out);
			HOST_TO_LE_32(manuf_data+man_data_offset+8, P0_Out);	//Sea-level pressure
	#if (HUMIDITY_SENSOR_PRESENT)
			HOST_TO_LE_16(manuf_data+man_data_offset+12, Hum_Out);
			HOST_TO_LE_16(manuf_data+man_data_offset+14, T2_Out);	//Dew Point
			HOST_TO_LE_16(manuf_data+man_data_offset+16, T3_Out);	//Heat Index
	#endif
			HOST_TO_LE_32(manuf_data+man_data_offset+18, P_Out);	//Absolute Pressure
			manuf_data[man_data_offset+22] = Z_forecast;
		} else
		if(Adv_Code == 0x02)	//Main Beacon Adv_Code 0x02
		{
			//Send Air Quality Data. Absolute values
	#if (VOC_SENSOR_PRESENT)
			HOST_TO_LE_16(manuf_data+man_data_offset+6, eq_TVOC);
			HOST_TO_LE_16(manuf_data+man_data_offset+8, eq_CO2);
	#endif
	#if (GAS_SENSOR_MODULE_PRESENT)
			HOST_TO_LE_16(manuf_data+man_data_offset+10, CO);
			HOST_TO_LE_16(manuf_data+man_data_offset+16, CH2O);
			HOST_TO_LE_16(manuf_data+man_data_offset+12, NO2);
			HOST_TO_LE_16(manuf_data+man_data_offset+14, NH3);
		#if (OUTDOOR_MODE)
			HOST_TO_LE_16(manuf_data+man_data_offset+18, O3);
			HOST_TO_LE_16(manuf_data+man_data_offset+20, SO2);
			HOST_TO_LE_16(manuf_data+man_data_offset+22, C6H6);
		#endif
	#endif	//GAS_SENSOR_MODULE_PRESENT
			manuf_data[man_data_offset+27] = Gas_AQI;
		} else
		if(Adv_Code == 0x03)	//Main Beacon Adv_Code 0x03
		{
			//Send Air Quality Data. Averaged values
	#if (VOC_SENSOR_PRESENT)
			HOST_TO_LE_16(manuf_data+man_data_offset+6, eq_TVOC_1h_Mean);
			HOST_TO_LE_16(manuf_data+man_data_offset+8, eq_CO2_1h_Mean);
	#endif
	#if (GAS_SENSOR_MODULE_PRESENT)
			HOST_TO_LE_16(manuf_data+man_data_offset+10, CO_8h_Mean);
			HOST_TO_LE_16(manuf_data+man_data_offset+16, CH2O_8h_Mean);
			HOST_TO_LE_16(manuf_data+man_data_offset+12, NO2_1h_Mean);
			HOST_TO_LE_16(manuf_data+man_data_offset+14, NH3_8h_Mean);
		#if (OUTDOOR_MODE)
			HOST_TO_LE_16(manuf_data+man_data_offset+18, O3_1h_Mean);
			HOST_TO_LE_16(manuf_data+man_data_offset+20, SO2_1h_Mean);
			HOST_TO_LE_16(manuf_data+man_data_offset+22, C6H6_24h_Mean);
		#endif
	#endif	//GAS_SENSOR_MODULE_PRESENT
			manuf_data[man_data_offset+26] = T_AVG_Gas_AQI;
			manuf_data[man_data_offset+27] = AVG_Gas_AQI;
		} else
		if(Adv_Code == 0x04)	//Main Beacon Adv_Code 0x04
		{
			//Send Air pollution Data
	#if (PARTICULATE_SENSOR_PRESENT)
			HOST_TO_LE_16(manuf_data+man_data_offset+6, MC_1p0);
			HOST_TO_LE_16(manuf_data+man_data_offset+8, MC_2p5);
			HOST_TO_LE_16(manuf_data+man_data_offset+10, MC_4p0);
			HOST_TO_LE_16(manuf_data+man_data_offset+12, MC_10p0);
			HOST_TO_LE_16(manuf_data+man_data_offset+14, NC_0p5);
			HOST_TO_LE_16(manuf_data+man_data_offset+16, MC_1p0_24h_Mean);
			HOST_TO_LE_16(manuf_data+man_data_offset+18, MC_2p5_24h_Mean);
			HOST_TO_LE_16(manuf_data+man_data_offset+20, MC_4p0_24h_Mean);
			HOST_TO_LE_16(manuf_data+man_data_offset+22, MC_10p0_24h_Mean);
			manuf_data[man_data_offset+25] = T_AVG_PMx_AQI;
			manuf_data[man_data_offset+26] = PMx_AQI;
			manuf_data[man_data_offset+27] = AVG_PMx_AQI;
	#endif	//PARTICULATE_SENSOR_PRESENT
		} else
		if(Adv_Code == 0x80)	//Main Status Report Beacon; sent every seconds
		{
			//Send Device Status
			HOST_TO_BE_32(manuf_data+man_data_offset+6, SensorStatusReg);
			HOST_TO_LE_32(manuf_data+man_data_offset+10, BLE_TimeStamp);
			HOST_TO_BE_32(manuf_data+man_data_offset+14, StatusReg);
		} else
		if(Adv_Code == 0x81)	//Secondary Beacon Adv_Code 0x81
		{
			//Send Environmental Daily Min-Max Data
			HOST_TO_LE_16(manuf_data+man_data_offset+6, T_Min);		//Daily minimum temperature
			HOST_TO_LE_32(manuf_data+man_data_offset+8, P_Min);		//Daily minimum Sea-level pressure
	#if (HUMIDITY_SENSOR_PRESENT)
			HOST_TO_LE_16(manuf_data+man_data_offset+12, H_Min);	//Daily minimum relative humidity
			HOST_TO_LE_16(manuf_data+man_data_offset+14, H_Max);	//Daily maximum relative humidity
	#endif
			HOST_TO_LE_16(manuf_data+man_data_offset+16, T_Max);	//Daily maximum temperature
			HOST_TO_LE_32(manuf_data+man_data_offset+18, P_Max);	//Daily maximum Sea-level pressure
		} else
		if(Adv_Code == 0x83)	//Secondary Beacon Adv_Code 0x83
		{
			//Send Air Quality maximum values of the averages in the day, starting from midnight
	#if (VOC_SENSOR_PRESENT)
			HOST_TO_LE_16(manuf_data+man_data_offset+6, eq_TVOC_1h_Mean_Max);	//1h average maximum value
			HOST_TO_LE_16(manuf_data+man_data_offset+8, eq_CO2_1h_Mean_Max);	//1h average maximum value
	#endif
	#if (GAS_SENSOR_MODULE_PRESENT)
			HOST_TO_LE_16(manuf_data+man_data_offset+10, CO_8h_Mean_Max);		//8h average maximum value
			HOST_TO_LE_16(manuf_data+man_data_offset+16, CH2O_8h_Mean_Max);		//8h average maximum value
			HOST_TO_LE_16(manuf_data+man_data_offset+12, NO2_1h_Mean_Max);		//1h average maximum value
			HOST_TO_LE_16(manuf_data+man_data_offset+14, NH3_8h_Mean_Max);		//8h average maximum value
		#if (OUTDOOR_MODE)
			HOST_TO_LE_16(manuf_data+man_data_offset+18, O3_1h_Mean_Max);		//1h average maximum value
			HOST_TO_LE_16(manuf_data+man_data_offset+20, SO2_1h_Mean_Max);		//1h average maximum value
			HOST_TO_LE_16(manuf_data+man_data_offset+22, C6H6_24h_Mean_Max);	//Daily average maximum value
		#endif
	#endif	//GAS_SENSOR_MODULE_PRESENT
		} else
		if(Adv_Code == 0x84)	//Secondary Beacon Adv_Code 0x84
		{
			//Send Air pollution Data maximum values of the averages in the day, starting from midnight
	#if (PARTICULATE_SENSOR_PRESENT)
			HOST_TO_LE_16(manuf_data+man_data_offset+18, MC_2p5_24h_Mean_Max);
			HOST_TO_LE_16(manuf_data+man_data_offset+22, MC_10p0_24h_Mean_Max);
	#endif	//PARTICULATE_SENSOR_PRESENT
		}

		if (adv_data_sent)
		{
			adv_data_sent = false;
			ret = aci_gap_update_adv_data(sizeof(manuf_data), (uint8_t *)manuf_data);

			if(ret != 0) {
				PRINT_DBG ("Error in aci_gap_update_adv_data() 0x%04x\r\n", ret);
				while(1);
			}
		}
	/* USER CODE END BlueNRG_2_Process_PreTreatment */

	/* USER CODE BEGIN BlueNRG_2_Process_PostTreatment */
		hci_user_evt_proc();
		User_Process();
	}
	while ((Adv_Code != 0x80) && (!ConfCmd));
#endif	//USE_SENSUS191==0
	/* USER CODE END BlueNRG_2_Process_PostTreatment */
}
#else	//MY_BEACON==0
/*
 * BlueNRG-2 background task
 */
void MX_BlueNRG_2_Process(void)
{
  /* USER CODE BEGIN BlueNRG_2_Process_PreTreatment */
#if (USE_SENSUS191==1)
  static uint8_t ret = 0;
  static uint8_t n = 0; //This counter has only the function of demonstrating the possibility
  // of transmitting a custom payload in the last 21 bytes of the beacon message. Here only byte 25 is updated.
  if(service_timer0_expired)
  {
	  manuf_data[man_data_offset+25] = ++n;
		ret = aci_gap_update_adv_data(sizeof(manuf_data), (uint8_t *)manuf_data);
	  if(ret != 0) {
	    PRINT_DBG ("Error in aci_gap_update_adv_data() 0x%04x\r\n", ret);
	    while(1);
	  }
	  service_timer0_expired = false;
  }
#endif
  /* USER CODE END BlueNRG_2_Process_PreTreatment */

  hci_user_evt_proc();
  User_Process();

  /* USER CODE BEGIN BlueNRG_2_Process_PostTreatment */

  /* USER CODE END BlueNRG_2_Process_PostTreatment */
}
#endif

#if (USE_STM32F4XX_NUCLEO==1)
/**
 * @brief  Initialize User process.
 *
 * @param  None
 * @retval None
 */
static void User_Init(void)
{
  BSP_LED_Init(LED2);

  BSP_COM_Init(COM1);
}
#endif

/**
 * @brief  User application process
 * @param  None
 * @retval None
 */
static void User_Process(void)
{
#if (USE_SENSUS191==0)
  EnterStopMode();		//Comment if you want SysTick equal to 1 mS
#endif
}

static void Device_Init(void)
{
  static uint8_t ret = 0;
  uint8_t device_name[] = {'S','e','n','s','u','s',' ','1','9','1'};
  uint16_t service_handle;
  uint16_t dev_name_char_handle;
  uint16_t appearance_char_handle;
  uint8_t bdaddr[] = {0xf5, 0x00, 0x00, 0xE1, 0x80, 0x02};	//Big Endian!!
//uint8_t bdaddr[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};	//Big Endian!!
  extern FLASH_DATA_ORG FlashDataOrg;
/*	 5    4    3    2    1   0  Byte order
  +----+----+----+----+----+----+
  | F5 | 00 | 00 | E1 | 80 | 02 |	MAC displayed: 02:80:E1:00:00:F5
  +----+----+----+----+----+----+	OUI: 02:80:E1 - NIC Specific: 00:00:F5
  (from https://www.shiksha.com/online-courses/articles/mac-address-format-and-types) */

  /* Set the custom MAC Address */
  if (((FlashDataOrg.b_status.s12 == 0xFFFFFFFF) || (FlashDataOrg.b_status.s12 == 0x00)) ||
	  ((FlashDataOrg.b_status.s13 == 0xFFFFFFFF) || (FlashDataOrg.b_status.s13 == 0x00)))
  {
	FlashDataOrg.b_status.s12 =  ((uint32_t)(bdaddr[0] & 0xFF) << 16) |
								   ((uint32_t)(bdaddr[1] & 0xFF) << 8) | (uint32_t)(bdaddr[2] & 0xFF);
	FlashDataOrg.b_status.s13 =  ((uint32_t)(bdaddr[3] & 0xFF) << 16) |
								   ((uint32_t)(bdaddr[4] & 0xFF) << 8) | (uint32_t)(bdaddr[5] & 0xFF);
  } else
  {
	(((bdaddr)[2] =  (uint8_t) (FlashDataOrg.b_status.s12)    ) , \
	 ((bdaddr)[1] =  (uint8_t) (FlashDataOrg.b_status.s12>>8) ) , \
	 ((bdaddr)[0] =  (uint8_t) (FlashDataOrg.b_status.s12>>16)));
	(((bdaddr)[5] =  (uint8_t) (FlashDataOrg.b_status.s13)    ) , \
	 ((bdaddr)[4] =  (uint8_t) (FlashDataOrg.b_status.s13>>8) ) , \
	 ((bdaddr)[3] =  (uint8_t) (FlashDataOrg.b_status.s13>>16)));
  }
  /* Sw reset of the device */
  hci_reset();
  /**
   *  To support both the BlueNRG-2 and the BlueNRG-2N a minimum delay of 2000ms is required at device boot
   */
  HAL_Delay(2000);

#if (USE_SENSUS191==0)
  /* Set the TX power -2 dBm */
  aci_hal_set_tx_power_level(1, 4);
#else
  /* Set the TX power 8 dBm */
  aci_hal_set_tx_power_level(1, 7);
#endif
  if(ret != 0) {
    PRINT_DBG ("Error in aci_hal_set_tx_power_level() 0x%04x\r\n", ret);
    while(1);
  }
  else
    PRINT_DBG ("aci_hal_set_tx_power_level() --> SUCCESS\r\n");

  /* Set the public address */
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, bdaddr);
  if(ret != 0) {
    PRINT_DBG ("Error in aci_hal_write_config_data() 0x%04x\r\n", ret);
    while(1);
  }
  else
    PRINT_DBG ("aci_hal_write_config_data() --> SUCCESS\r\n");

  /* Init the GATT */
  ret = aci_gatt_init();
  if(ret != 0) {
    PRINT_DBG ("Error in aci_gatt_init() 0x%04x\r\n", ret);
    while(1);
  }
  else
    PRINT_DBG ("aci_gatt_init() --> SUCCESS\r\n");

  /* Init the GAP */
  ret = aci_gap_init(GAP_PERIPHERAL_ROLE, 0, sizeof(device_name), &service_handle, &dev_name_char_handle, &appearance_char_handle);
  if(ret != 0) {
    PRINT_DBG ("Error in aci_gap_init() 0x%04x\r\n", ret);
    while(1);
  }
  else
    PRINT_DBG ("aci_gap_init() --> SUCCESS\r\n");

  ret = aci_hal_set_radio_activity_mask(0x0002);	//Enable the aci_hal_end_of_radio_activity_event(uint16_t Radio_Activity_Mask)
  if (ret != BLE_STATUS_SUCCESS) {					//call back function. Radio_Activity_Mask = 0x0002: Advertising
    PRINT_DBG ("Error in aci_hal_set_radio_activity_mask() 0x%04x\r\n", ret);
    while(1);
  }
  else
    PRINT_DBG ("aci_hal_set_radio_activity_mask() --> SUCCESS\r\n");
}

/**
* @brief  Start beaconing
* @param  None
* @retval None
*/
static void Start_Beaconing(void)
{
  uint8_t ret = BLE_STATUS_SUCCESS;

  /* disable scan response */
  ret = hci_le_set_scan_response_data(0,NULL);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINT_DBG ("Error in hci_le_set_scan_resp_data() 0x%04x\r\n", ret);
    while(1);
  }
  else
    PRINT_DBG ("hci_le_set_scan_resp_data() --> SUCCESS\r\n");

  /* Set scan response data */
  hci_le_set_scan_response_data(sizeof(scan_resp_data), scan_resp_data);

  /* put device in non connectable mode */
  ret = aci_gap_set_discoverable(ADV_IND,//ADV_NONCONN_IND,  /*< Advertise as non-connectable, undirected. */
                                 ADV_INTERV_MIN,             /*< Set the min advertising interval (0.625 us increment). */
                                 ADV_INTERV_MAX,             /*< Set the max advertising interval (0.625 us increment). */
                                 PUBLIC_ADDR, NO_WHITE_LIST_USE, /*< Use the public address, with no white list. */
                                 0, NULL,                    /*< Do not use a local name. */	//Modified By Me!!!
//								 sizeof(local_name), (uint8_t*)&local_name, /*< Use a local name. */	//Modified By Me!!!
                                 0, NULL,                    /*< Do not include the service UUID list. */
                                 0, 0);                      /*< Do not set a slave connection interval. */

  if (ret != BLE_STATUS_SUCCESS) {
    PRINT_DBG ("Error in aci_gap_set_discoverable() 0x%04x\r\n", ret);
    while(1);
  }
  else
    PRINT_DBG ("aci_gap_set_discoverable() --> SUCCESS\r\n");

ret = aci_gap_delete_ad_type(AD_TYPE_TX_POWER_LEVEL);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINT_DBG ("Error in aci_gap_delete_ad_type() 0x%04x\r\n", ret);
    while(1);
  }
  else
    PRINT_DBG ("aci_gap_delete_ad_type() --> SUCCESS\r\n");

  ret = aci_gap_update_adv_data(sizeof(manuf_data), (uint8_t *)manuf_data);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINT_DBG ("Error in aci_gap_update_adv_data() 0x%04x\r\n", ret);
    while(1);
  }
  else
    PRINT_DBG ("aci_gap_update_adv_data() --> SUCCESS\r\n");
}

/**
 * @brief  Enter the STOP mode
 * @param  None
 * @retval None
 */
void EnterStopMode(void)
{
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
}

/**
 * @brief  Restart the BLE module
 * @param  None
 * @retval None
 */
void BLE_Restart(void)
{
	HAL_NVIC_DisableIRQ(HCI_TL_SPI_EXTI_IRQn);
	HCI_TL_SPI_Reset();
	HCI_TL_SPI_DeInit();

	MX_BlueNRG_2_Init();
}

/**
 * @brief	This event is generated when the device completes a radio activity and provide
 * 			information when a new radio activity will be performed.
 * 			The provided information includes type of radio activity and absolute time in system
 *			ticks when a new radio activity is schedule, if any. Application can use this
 *			information to schedule user activities synchronous to selected radio activities
 * @param	Last_State: Completed radio events
 * @param	Next_State Incoming radio events
 * @param	Next_State_SysTime: 32bit absolute current time expressed in internal time units.
 * @retval	None
 */
void aci_hal_end_of_radio_activity_event(uint8_t Last_State, uint8_t Next_State, uint32_t Next_State_SysTime)
{
	static uint16_t SecondaryBeaconCicleNumber = 0;
	static uint8_t Adv_Code_Modifier = 0x00;	//When = 0x80 secondary beacons will be transmitted
#if (USE_IWDGT)
	extern IWDG_HandleTypeDef hiwdg;
#endif
#if (USE_BKUP_SRAM)
	uint32_t counter_time;
#endif

	if (Next_State == 0x01)	/* 0x01: Advertising */
	{
		/* Update the code to be transmitted in the next Beacon Advertising packet */
		Adv_Code = AdvCodes[beacon_number];
		Adv_Code |= Adv_Code_Modifier;
		/*
		 * CycleNumber counts after how many transmissions of the primary "beacon set" (Adv_Code = 0b0xxx xxxx)
		 * a secondary "beacon set" (here identified with Adc_Cobe = 0b1xxx xxxx) is transmitted
		 */
		if (++beacon_number == (uint8_t)N_ADV_CODES)
		{
			beacon_number = 0;
			if ((++SecondaryBeaconCicleNumber > SecondaryBeaconSwitchTime) || (MinMaxStored))
			{	//The secondary "beacon set" 0b1xxx xxxx contains the daily min-max values. It is still transmitted at midnight.
				Adv_Code_Modifier = 0x80;
				SecondaryBeaconCicleNumber = 0;
			} else
			{
				Adv_Code_Modifier = 0x00;
			}
		}
		adv_data_sent = true;

		/**
		 * If for some reason the transmission of the Beacon is interrupted,
		 * this function is no longer called, in this case activating the WatchDog Timer
		 */
#if (USE_IWDGT)
		HAL_IWDG_Refresh(&hiwdg);
#endif
//It signals to the main () process that the callback has been executed at least once
#if (USE_BKUP_SRAM)
		counter_time = RTC_GetCounter();
		BakUpRTC_Data[0] = BLE_MAGIC_NUMBER;
		/**
		 * The current timestamp will be used in the main() function to recognize
		 * if the reset was generated by the WatchDog Timer or by a power-cycle
		 * It will be stored by the "Store_MeanValues_BackupRTC" function,
		 * called every second in the "HAL_RTCEx_RTCEventCallback" function, in port.c
		 */
		BakUpRTC_Data[1] = (uint8_t)(counter_time & 0xFF);
		BakUpRTC_Data[2] = (uint8_t)((counter_time >> 8) & 0xFF);
		BakUpRTC_Data[3] = (uint8_t)((counter_time >> 16) & 0xFF);
		BakUpRTC_Data[4] = (uint8_t)((counter_time >> 24) & 0xFF);
#endif
	}
}

#endif	//BEACON_APP - Added By Me!!!
