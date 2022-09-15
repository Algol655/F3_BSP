/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "application/MEMS_app.h"
#if (BLE_SUPPORT==1)
	#include "Sensor/app_bluenrg_2.h"
	#include "OpModes.h"
#endif
#if (IO_EXP_PRESENT==1)
	#include "platform/mcp23017.h"
#endif
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
__attribute__((__section__(".user_data"))) const char userConfig[64];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#if (IMU_PRESENT==1)						//Defined in main.h
//	const double_t g = 9.80665;				//g is the acceleration of gravity
	const double_t g_to_ms2 = 9.80665;		//g to m/s^2 conversion factor
	const double_t mg_to_g = 0.001;			//mg to g conversion factor
	const double_t mg_to_ms2 = 0.00980665;	//mg to m/s^2 conversion factor
	const double_t mdps_to_dps = 0.001;		//mdps to dps conversion factor
	const double_t mGauss_to_uT = 0.1;		//mGauss to uTesla conversion factor
	const double_t mGauss_to_uT50 = 0.002;	//mGauss to uTesla/50 conversion factor
	const double_t uT50_to_mGauss = 500.0;	//uTesla/50 to mGauss conversion factor
#endif
#if ((PRESSURE_SENSOR_PRESENT==0) && (IMU_PRESENT==1))
	double_t	Temperature = 19.3;
	double_t	Pressure = 1013.25;
	double_t	Altitude = 10.0;
#endif
#if (GUI_SUPPORT==1)
	const double_t AccelConvFactor = 0.001;	//mg to g conversion factor
	uint8_t DataLoggerActive = 0;
	uint8_t Sensors_Enabled = 0;
#else
	const double_t AccelConvFactor = 0.00980665;	//mg to m/s^2 conversion factor
	uint8_t DataLoggerActive = 1;
	uint8_t Sensors_Enabled = 1;
#endif
bool button_pressed = false;
int64_t MEMS_LclData;

FLASH_DATA_ORG FlashDataOrg = {.b_date_offset = 0x00, .b_time_offset = 0x04,
.b_mdata.DeviceName_offset = 0x08, .b_mdata.HW_Version_offset = 0x0C, .b_mdata.SW_Version_offset = 0x10,
.b_mdata.Vendor_ID_offset = 0x14, .b_mdata.Prdct_Code_offset = 0x18, .b_mdata.Rev_Number_offset = 0x1C, .b_mdata.Ser_Number_offset = 0x20,
.b_status.s0_offset = 0x24, .b_status.s1_offset = 0x28, .b_status.s2_offset = 0x2C, .b_status.s3_offset = 0x30,
.b_status.s4_offset = 0x34, .b_status.s5_offset = 0x38, .b_status.s6_offset = 0x3C, .b_status.s7_offset = 0x40,
.b_status.s8_offset = 0x44, .b_status.s9_offset = 0x48, .b_status.sa_offset = 0x4C, .b_status.sb_offset = 0x50,
.b_status.sc_offset = 0x54, .b_status.sd_offset = 0x58, .b_status.se_offset = 0x5C, .b_status.sf_offset = 0x60};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#if	(USB_SUPPORT==1) 					//this is set in the port.h file
	extern void usb_run(void);
	extern int usb_init(void);
	#if ((DATA_MODE==0) && (GUI_SUPPORT==1))
		extern void send_usbmessage(uint8_t*, int);
	#endif
#endif
#if ((TLCD_SUPPORT==1) || (GLCD_SUPPORT==1))
	extern void FormatDisplayString(int *len, uint8_t* Buff, SENSOR_TYPE stype);
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_RTC_Init();
  MX_USB_DEVICE_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_CRC_Init();
  MX_TIM6_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  MX_TIM7_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
//HAL_UART_MspDeInit(&huart2);			//To avoid the error that would occur if a serial communication
  HAL_UART_MspDeInit(&huart3);			//is already active when the program starts
  HAL_NVIC_ClearPendingIRQ(EXTI0_IRQn);
  HAL_NVIC_ClearPendingIRQ(EXTI2_IRQn);
  HAL_NVIC_ClearPendingIRQ(EXTI4_IRQn);
  HAL_NVIC_ClearPendingIRQ(I2C1_EV_IRQn);
  HAL_NVIC_ClearPendingIRQ(TIM1_UP_TIM10_IRQn);
  HAL_NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
  HAL_NVIC_ClearPendingIRQ(TIM3_IRQn);
  HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
  __HAL_RTC_SECOND_ENABLE_IT(&hrtc,RTC_FLAG_SEC);	//Enable RTC per second interrupt
  HAL_RTCEx_SetSmoothCalib(&hrtc, 0, 0, *((uint32_t*)(DATA_EEPROM_BASE+FlashDataOrg.b_status.s7_offset)));	//Adjust RTC precision
  HAL_NVIC_DisableIRQ(EXTI2_IRQn);
  HAL_NVIC_DisableIRQ(EXTI4_IRQn);
  HAL_NVIC_DisableIRQ(RTC_Alarm_IRQn);
  HAL_GPIO_DeInit(D3_LCD_BLE_INT_GPIO_Port, D3_LCD_BLE_INT_Pin);	//Pin Init will be made by MX_BlueNRG_2_Init() if BLE is enabled

  HAL_TIM_Base_Stop_IT(&htim3);						//Stop Timer 3
  __HAL_TIM_SET_COUNTER(&htim3, 0);					//Clear Timer 3
  __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);	//Clear Timer 3 Update Interrupt Flag
  HAL_TIM_Base_Stop_IT(&htim6);						//Stop Timer 6
  __HAL_TIM_SET_COUNTER(&htim6, 0);					//Clear Timer 6
  __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);	//Clear Timer 6 Update Interrupt Flag
  HAL_TIM_Base_Start_IT(&htim7);					//Start Timer 7
  ServiceTimersInit();

//HAL_GPIO_WritePin(PC_11_DISC_GPIO_Port, PC_11_DISC_Pin, GPIO_PIN_RESET);	//Pull-up Enable on USBDP (Only for Olimex STM32-H103 Board)
  leds_test = false; Link_Ok = true;
  lcl_imu_data_rdy = false; send_lcl_imu_data = false; send_lcl_imu_data_to_ble = false; display_imu_data = false;
  lcl_prs_data_rdy = false; send_lcl_prs_data = false; display_prs_data = false;
  lcl_hum_data_rdy = false; send_lcl_hum_data = false; display_hum_data = false;
  lcl_uvx_data_rdy = false; send_lcl_uvx_data = false; display_uvx_data = false;
  lcl_voc_data_rdy = false; send_lcl_voc_data = false; display_voc_data = false;
  lcl_pms_data_rdy = false; send_lcl_pms_data = false; display_pms_data = false;
  stby_timer = 0; service_timer3 = 0;
  service_timer0_expired = false; WarmUpPeriod_expired = false;
  timer5s_expired = false; update_1s = false; update_5s = false;
  update_16Hz = 0; update_25Hz = 0; update_50Hz = 0; update_100Hz = 0;
  t_flip = 0; NumberOfDevices = 0; SensorStatusReg = 0, PreviousPage = 1;
  internal_notifies_0 = 0; internal_notifies_1 = 0;

#if (USE_BKUP_SRAM)
  extern bool StartDataStrmng;

  enable_backup_rtc();
  readBkpRTC((uint8_t *)BakUpRTC_Data, sizeof(BakUpRTC_Data), 0);
  disable_backup_rtc();
  if (BakUpRTC_Data[0] == BLE_MAGIC_NUMBER)
  {
	  BakUpRTC_Data[0] = 0;
	  enable_backup_rtc();
	  writeBkpRTC((uint8_t *)BakUpRTC_Data, sizeof(BakUpRTC_Data), 0);
	  disable_backup_rtc();
	  StartDataStrmng = (bool)FlipFlop(false, 0xFF, 0x01);
  }
#endif
#if (BLE_SUPPORT==1)
//  HAL_NVIC_DisableIRQ(EXTI0_IRQn);			//Made by BSP
  	MX_BlueNRG_2_Init();
#endif
#if	(USB_SUPPORT==1) 							//this is set in the port.h file
    // enable the USB functionality
    usb_init();
#endif
#if (TLCD_SUPPORT==1)
    TLCD_status = MX_TLCD_Init();
    send_tlcdmessage(WELCOME_STRING1, 26);
    send_tlcdmessage(MY_FW_VERSION, 26);
    Sleep(3000);
    LCD_Clear();
#elif (GLCD_SUPPORT==1)
    GLCD_status = MX_GLCD_Init();
    SendWelcomeMessage();
    Sleep(3000);
#endif
#if (CAN_SUPPORT==1)
    CAN_Config(&hcan1);
//  CAN_Config(&hcan2);
#endif
#if (USART_SUPPORT==1)
//  USART_Config(&huart2);
	USART_Config(&huart3);
#endif
#if (IO_EXP_PRESENT==1)
    MCP23017_status = MX_MCP23017_Init();
	#if (TLCD_SUPPORT==1)
	if (MCP23017_status == (uint8_t)MCP23017_OK)
	{
		send_tlcdmessage("IOEXP ",6);
	}
	#endif
#endif
#if (GAS_SENSOR_MODULE_PRESENT==1)
	  ADC_Config(&hadc1);						//Configure the ADC peripheral
#endif
	AB_Init();									//Initialize Sensor System
#if (TLCD_SUPPORT==1)
	send_tlcdmessage(ReadyDevices, strlen(ReadyDevices));
#elif (GLCD_SUPPORT==1)
	SendReadyDevicesMessage();
#endif
#if ((BLE_SUPPORT) && (BEACON_APP))
  	button_manage();
#endif
	leds_test = true;							//Blink LEDs if init Ok
//  LPS25HB_status = LPS25HB_Get_Measurement(LPS25HB_BADDR, &PRS_Values);		//Sensor Test...
//  LSM9DS1_status = LSM9DS1_Temperature_Get(&Temperatura_Raw, &Temperature);	//Sensor Test...
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
#if ((GUI_SUPPORT==0) && (BLE_SUPPORT==0))
		int n = 0;								//n = 0 enable Remote Control Mode
#else
		int n = 1;
#endif
#if (PRESSURE_SENSOR_PRESENT==1)
	//Acquire data from pressure sensor and fill Msg stream
		if (lcl_prs_data_rdy && Sensors_Enabled)	//Set in process_timer3_irq. Pressure/Temp and Time-Stamp data are acquired in timer3 irq
		{
			lcl_prs_data_rdy = false;
			Pressure_Sensor_Handler(&PRS_Values, &dataseq1[0]);
		}
#endif
#if (HUMIDITY_SENSOR_PRESENT==1)
	//Acquire data from humidity sensor and fill Msg stream
		if (lcl_hum_data_rdy && Sensors_Enabled)	//Set in process_timer3_irq. Pressure/Temp and Time-Stamp data are acquired in timer3 irq
		{
			lcl_hum_data_rdy = false;
			Humidity_Sensor_Handler(&HUM_Values, &dataseq1[0]);
		}
#endif
#if (UVx_SENSOR_PRESENT==1)
	//Acquire data from UVx sensor and fill Msg stream
		if (lcl_uvx_data_rdy && Sensors_Enabled)	//Set in process_timer3_irq. Pressure/Temp and Time-Stamp data are acquired in timer3 irq
		{
			lcl_uvx_data_rdy = false;
			UVx_Sensor_Handler(&UVx_Values, &dataseq1[0]);
		}
#endif
#if (VOC_SENSOR_PRESENT==1)
	//Acquire data from VOC sensor and fill Msg stream
		if (lcl_voc_data_rdy && Sensors_Enabled)	//Set in process_timer3_irq. Pressure/Temp and Time-Stamp data are acquired in timer3 irq
		{
			lcl_voc_data_rdy = false;
			VOC_Sensor_Handler(&VOC_Values, &dataseq1[0]);
		}
#endif
#if (PARTICULATE_SENSOR_PRESENT==1)
	//Acquire data from Particulate Matter sensor and fill Msg stream
		if (lcl_pms_data_rdy && Sensors_Enabled)	//Set in process_timer3_irq. Pressure/Temp and Time-Stamp data are acquired in timer3 irq
		{
			lcl_pms_data_rdy = false;
			Particulate_Sensor_Handler(&PMS_Values, &dataseq1[0]);
		}
#endif
#if (GAS_SENSOR_MODULE_PRESENT==1)
	//Acquire data from analog Gas sensor board and fill Msg stream
		if (lcl_gas_data_rdy && Sensors_Enabled)	//Set in process_timer3_irq. Pressure/Temp and Time-Stamp data are acquired in timer3 irq
		{
			lcl_gas_data_rdy = false;
			Gas_Sensor_Handler(&GAS_Values, &dataseq1[0]);
		}
#endif
#if (IMU_PRESENT==1)
		if (lcl_imu_data_rdy && Sensors_Enabled)	//Set in process_timer1_irq. IMU and Time Stamp data are acquired in timer1-OC1 irq
		{
			lcl_imu_data_rdy = false;
			//Acquire data from enabled IMU sensors, call periodically calibration functions and fill Msg stream
			IMU_Handler(&IMU_Values, &dataseq1[0]);
			//Gyroscope Calibration
			GC_Data_Handler(&IMU_Values, &dataseq1[0]);
			//Accelerometer Calibration
			AC_Data_Handler(&IMU_Values, &dataseq1[0]);
			//Activity Recognition
			AR_Data_Handler(&IMU_Values, &dataseq1[0]);
			//Magnetometer Calibration
			MC_Data_Handler(&IMU_Values, &dataseq1[0]);
			//Activity Recognition
			AR_Data_Handler(&IMU_Values, &dataseq1[0]);
			//Sensor Fusion specific part
			FX_Data_Handler(&IMU_Values, &dataseq1[0]);
			//Velocity Estimate from Linear Acceleration
//			VE_Data_Handler(&Linear_Acceleration_9X_0_data[0], &Linear_Acceleration_9X_1_data[0],
// 							&Linear_Speed_9X_0_data[0], &Linear_Speed_9X_1_data[0], &Calibrated_Gyroscope_g_1_status[0]);
			send_lcl_imu_data_to_ble = true;
	#if (GUI_SUPPORT==1)
			AB_Handler();
	#elif (SEND_IMU_DATA_TO_TERMINAL)
			AC_Handler(&n, &IMU_Values, &usbVCOMout[0]);
	#endif
		}
#else	//IMU_PRESENT==0
		if (update_5s && Sensors_Enabled)	//Set in process_timer3_irq. IMU and Time Stamp data are acquired in timer3 irq
		{
			update_5s = false;
	#if (GUI_SUPPORT==1)
			AB_Handler();
	#endif
		}
#endif	//IMU_PRESENT==0
#if (IO_EXP_PRESENT==1)
		read_ports();
#endif
#if ((PRESSURE_SENSOR_PRESENT==1) && !(GUI_SUPPORT))	//Don't send data to VCP if GUI enabled
		if (send_lcl_prs_data)
		{
			send_lcl_prs_data = false;
	#if ((TLCD_SUPPORT==1) || (GLCD_SUPPORT==1))
			FormatDisplayString(&n, &usbVCOMout[0], PRESSURE_SENSOR);
	#endif
		}
#endif
#if ((HUMIDITY_SENSOR_PRESENT==1) && !(GUI_SUPPORT))	//Don't send data to VCP if GUI enabled
		if ((send_lcl_hum_data) && !(send_lcl_prs_data))
		{
			send_lcl_hum_data = false;
	#if ((TLCD_SUPPORT==1) || (GLCD_SUPPORT==1))
			FormatDisplayString(&n, &usbVCOMout[0], HUMIDITY_SENSOR);
	#endif
		}
#endif
#if ((UVx_SENSOR_PRESENT==1) && !(GUI_SUPPORT))			//Don't send data to VCP if GUI enabled
		if ((send_lcl_uvx_data) && !(send_lcl_prs_data) && !(send_lcl_hum_data))
		{
			send_lcl_uvx_data = false;
	#if ((TLCD_SUPPORT==1) || (GLCD_SUPPORT==1))
			FormatDisplayString(&n, &usbVCOMout[0], UVx_SENSOR);
	#endif
		}
#endif
#if ((VOC_SENSOR_PRESENT==1) && !(GUI_SUPPORT))			//Don't send data to VCP if GUI enabled
		if ((send_lcl_voc_data) && !(send_lcl_prs_data) && !(send_lcl_hum_data) && \
		   !(send_lcl_uvx_data))
		{
			send_lcl_voc_data = false;
	#if ((TLCD_SUPPORT==1) || (GLCD_SUPPORT==1))
			FormatDisplayString(&n, &usbVCOMout[0], VOC_SENSOR);
	#endif
		}
#endif
#if ((PARTICULATE_SENSOR_PRESENT==1) && !(GUI_SUPPORT))	//Don't send data to VCP if GUI enabled
		if ((send_lcl_pms_data) && !(send_lcl_prs_data) && !(send_lcl_hum_data) && \
		   !(send_lcl_uvx_data) && !(send_lcl_voc_data))
		{
			send_lcl_pms_data = false;
	#if ((TLCD_SUPPORT==1) || (GLCD_SUPPORT==1))
			FormatDisplayString(&n, &usbVCOMout[0], PARTICULATE_SENSOR);
	#endif
		}
#endif
//  MX_X_CUBE_MEMS1_Process();
#if (DATA_MODE==0)
	#if ((BLE_SUPPORT) && (BEACON_APP))
		MX_BlueNRG_2_Process();
	#endif
		if(n > 0)
		{
	#if (GUI_SUPPORT==0)
			if ((display_imu_data) || (display_prs_data) || (display_hum_data) || (display_uvx_data) || \
				(display_voc_data) || (display_pms_data))
			{
				display_imu_data = false;
				display_prs_data = false;
				display_hum_data = false;
				display_uvx_data = false;
				display_voc_data = false;
				display_pms_data = false;
		#if (TLCD_SUPPORT==1)
				send_tlcdmessage((char*)&usbVCOMout[0], n);
		#elif (GLCD_SUPPORT==1)
				ReDrawPage_S0(t_flip);
				if (t_flip != 1)
					ReDrawPage_S1(t_flip);
		#endif
		#if ((BLE_SUPPORT) && (SENSOR_APP))
				MX_BlueNRG_2_Process();
		#endif
			}
	#else	//GUI_SUPPORT==1
		#if	(USB_SUPPORT==1)
			send_usbmessage(&dataseq[0], Message_Length);	//Data to be displayed by STM UnicleoGUI
		#elif (USART_SUPPORT==1)
			send_usart3message(&dataseq[0], Message_Length);//Data to be displayed by STM UnicleoGUI
		#endif
	#endif	//GUI_SUPPORT==0
		}
	#if ((GUI_SUPPORT==0) && (BLE_SUPPORT==0))	//Defined in main.h
		else			//When n = 0 Remote Control Mode can send/receive message from SENSUS191
		{				//n = 0 when SENSUS191 displays the Welcome or the Devices Present screens
		#if (GLCD_SUPPORT==1)
			if ((update_1s) && (t_flip == 1))	//Clock is updated every seconds in page 1
			{
				update_1s = false;
				ReDrawPage_S1(1);
			}
		#endif
		}
	#endif	//(GUI_SUPPORT==0) && (BLE_SUPPORT==0)
	#if	(USB_SUPPORT==1)
		usb_run();
	#elif (USART_SUPPORT==1)
		usart3_run();
	#endif
#endif	//DATA_MODE==0

#if (CAN_SUPPORT==1)
    can1_run();
#endif

	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC
                              |RCC_PERIPHCLK_USB;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the Systick interrupt time
  */
  __HAL_RCC_PLLI2S_ENABLE();
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
