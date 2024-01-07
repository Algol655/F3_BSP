/*! ----------------------------------------------------------------------------
 * @file	SerialProtocol.c
 * @brief	Serial Application Programs Collection.
 *          T.B.D.
 *
 * @author  Tommaso Sabatini, 2019
 */

#include "application/SerialProtocol.h"
const uint8_t Message[] = "\r\n\n  Warning! A system restart has been booked.\r\n  The system will reboot at midnight..";
const uint8_t Message1[] = "\r\n\n  The system restart reservation has been revoked";

/**
 * @brief  Byte stuffing process for one byte
 * @param  Dest destination
 * @param  Source source
 * @retval Total number of bytes processed
*/
int ByteStuffCopyByte(uint8_t *Dest, uint8_t Source)
{
  switch(Source)
  {
    case TMsg_EOF:
      Dest[0] = TMsg_BS;
      Dest[1] = TMsg_BS_EOF;
      return 2;
    case TMsg_BS:
      Dest[0] = TMsg_BS;
      Dest[1] = TMsg_BS;
      return 2;
    default:
      Dest[0] = Source;
      return 1;
  }
}

/**
 * @brief  Byte stuffing process for a Msg
 * @param  Dest destination
 * @param  Source source
 * @retval Total number of bytes processed
 */
int ByteStuffCopy(uint8_t *Dest, uint8_t *Source, uint16_t *Len)
{
  int i, Count;
  static uint8_t Buff[512];

  memcpy(&Buff[0], &Source[0], *Len);
  Count = 0;
  for (i = 0; i < *Len; i++)
  {
    Count += ByteStuffCopyByte(&Dest[Count], Buff[i]);
  }
  Dest[Count] = TMsg_EOF;
  Count++;
  return Count;
}

/**
 * @brief  Compute and add checksum and TMsg_EOF (oxF0)
 * @param  Msg pointer to the message
 * @retval None
 */
void CHK_ComputeAndAdd(uint8_t* Buff, uint16_t* len, int8_t Add_TOF)
{
	uint8_t chk = 0;
	uint8_t i;

	for (i = 0; i < *len; i++)
	{
		chk -= Buff[i];
	}
	Buff[i] = chk;
	if (Add_TOF)
	{
		Buff[i+1] = TMsg_EOF;
		*len = i+2;
	}
	else
	{
		*len = i+1;
	}
}

/**
 * @brief  Unbuild a Number from an array (LSB first)
 * @param  Source source
 * @param  Len number of bytes
 * @retval Rebuild unsigned int variable
 */
uint32_t Deserialize(uint8_t *Source, uint32_t Len)
{
	uint32_t app;

	app = Source[--Len];
	while(Len > 0)
	{
		app <<= 8;
		app += Source[--Len];
	}
	return app;
}

/**
 * @brief  Build the reply header
 * @param  Msg the pointer to the message to be built
 * @retval None
 */
void Build_Reply_Header(uint8_t* Buff)
{
	Buff[0] = Buff[1];
	Buff[1] = DEV_ADDR;
	Buff[2] += CMD_Reply_Add;
}

/**
 * @brief  Initialize the streaming header
 * @param  Msg the pointer to the header to be initialized
 * @retval None
 */
void Init_Streaming_Header(uint8_t* Buff)
{
	Buff[0] = DataStreamingDest;
	Buff[1] = DEV_ADDR;
	Buff[2] = CMD_Start_Data_Strmng;
}

/**
 * @brief  Handle a message from VCP
 * @param  Msg the pointer to the message to be handled
 * @retval 1 if the message is correctly handled, 0 otherwise
 */
int HandleUSB_MSG(uint8_t* Buff)
/*  DestAddr | SouceAddr | CMD | PAYLOAD
 *      1          1        1       N
 *  The STCmdP serial protocol is used here.
 *  (Described in UM1986 STM Document)
 */
{
	int ret = 0;
#if (GUI_SUPPORT==1)
	uint16_t Len;
	extern uint8_t Sensors_Enabled;
	extern uint8_t DataLoggerActive;
//	uint8_t PresentationString[] = {"MEMS shield demo,201,8.0.0,8.2.0,IKS01A2"};
	uint8_t PresentationString[] = {"MEMS shield demo,201,"FW_VERSION","LIB_VERSION",IKS01A2"};
#endif
#if (PARTICULATE_SENSOR_PRESENT==1)
	extern SPS30_Error_et sps30_start_measurement();
	extern SPS30_Error_et sps30_stop_measurement();
#endif
	static char dd[3] = {'\0'}; static char MM[3] = {'\0'};	static char yy[3] = {'\0'};
	static char HH[3] = {'\0'}; static char MN[3] = {'\0'}; static char SS[3] = {'\0'};
	static uint8_t y = 0; static uint8_t m = 0; static uint8_t d = 0; static uint8_t dw = 0;
	static uint8_t hh = 0; static uint8_t mm = 0; static uint8_t ss = 0;
	static DateTime_t Stamp; static uint8_t len = 0;

//	if (Msg->Len != 3U)
//	{
//		return 0;
//	}
	if (Buff[0] != DEV_ADDR)
	{
		return 0;
	}
	memset(&dataseq[0], 0x00, sizeof(dataseq));
	switch (Buff[2])   				//CMD
	{
#if (GUI_SUPPORT==1)
		case CMD_Ping:
		{
			Len = 3;
			memcpy(&dataseq[0], Buff, Len);
			Build_Reply_Header(&dataseq[0]);
			CHK_ComputeAndAdd(&dataseq[0], &Len, 1);
			Message_Length = Len;
		}
		break;

		case CMD_Enter_DFU_Mode:
		{
			Len = 3;
			memcpy(&dataseq[0], Buff, Len);
			Build_Reply_Header(&dataseq[0]);
			Message_Length = Len;
		}
		break;

		case CMD_Read_PresString:
		{
			Len = 3;
			memcpy(&dataseq[0], Buff, Len);
			Build_Reply_Header(&dataseq[0]);
			memcpy(&dataseq[Len], &PresentationString[0], sizeof(PresentationString));
			Len += sizeof(PresentationString)-1;
			CHK_ComputeAndAdd(&dataseq[0], &Len, 1);
			Message_Length = Len;
		}
		break;

/*		case CMD_PRESSURE_Init:
		{
			Build_Reply_Header(Msg);
			Serialize_s32(&Msg->Data[3], LPS22HB_UNICLEO_ID, 4);
			Msg->Len = 3 + 4;
			UART_SendMsg(Msg);
		}
		break;

		case CMD_HUMIDITY_TEMPERATURE_Init:
		{
			Build_Reply_Header(Msg);
			Serialize_s32(&Msg->Data[3], HTS221_UNICLEO_ID, 4);
			Msg->Len = 3 + 4;
			UART_SendMsg(Msg);
		}
		break;

		case CMD_ACCELERO_GYRO_Init:
		{
			Build_Reply_Header(Msg);
			Serialize_s32(&Msg->Data[3], LSM6DSL_UNICLEO_ID, 4);
			Msg->Len = 3 + 4;
			UART_SendMsg(Msg);
			}
		break;

		case CMD_MAGNETO_Init:
		{
			Build_Reply_Header(Msg);
			Serialize_s32(&Msg->Data[3], LSM303AGR_UNICLEO_ID_MAG, 4);
			Msg->Len = 3 + 4;
			UART_SendMsg(Msg);
		}
		break; */

		case CMD_Start_Data_Strmng:
		{
			memcpy(&dataseq[0], Buff, 5);
			Sensors_Enabled = Deserialize(&dataseq[0], 4);

			// Start enabled sensors
/*			if ((SensorsEnabled & PRESSURE_SENSOR) == PRESSURE_SENSOR)
			{
				(void)IKS01A2_ENV_SENSOR_Enable(IKS01A2_LPS22HB_0, ENV_PRESSURE);
			}
			if ((SensorsEnabled & TEMPERATURE_SENSOR) == TEMPERATURE_SENSOR)
			{
				(void)IKS01A2_ENV_SENSOR_Enable(IKS01A2_HTS221_0, ENV_TEMPERATURE);
			}
			if ((SensorsEnabled & HUMIDITY_SENSOR) == HUMIDITY_SENSOR)
			{
				(void)IKS01A2_ENV_SENSOR_Enable(IKS01A2_HTS221_0, ENV_HUMIDITY);
			}
			if ((SensorsEnabled & ACCELEROMETER_SENSOR) == ACCELEROMETER_SENSOR)
			{
				(void)IKS01A2_MOTION_SENSOR_Enable(IKS01A2_LSM6DSL_0, MOTION_ACCELERO);
			}
			if ((SensorsEnabled & GYROSCOPE_SENSOR) == GYROSCOPE_SENSOR)
			{
				(void)IKS01A2_MOTION_SENSOR_Enable(IKS01A2_LSM6DSL_0, MOTION_GYRO);
			}
			if ((SensorsEnabled & MAGNETIC_SENSOR) == MAGNETIC_SENSOR)
			{
				(void)IKS01A2_MOTION_SENSOR_Enable(IKS01A2_LSM303AGR_MAG_0, MOTION_MAGNETO);
			} */

	#if (IMU_PRESENT==1)
			TIM_OC_Timers_Start(&htim1);		//ReStart Output Compare Timers
			MotionFX_manager_start_9X();
	#endif
			RTC_Handler(&hrtc, &dataseq[0]);
			DataLoggerActive = 1;
	#if (PARTICULATE_SENSOR_PRESENT==1)
			sps30_start_measurement();			//PowerOn SPS30 PMx sensor
	#endif
			Build_Reply_Header(&dataseq[0]);
			Len = 7;
			CHK_ComputeAndAdd(&dataseq[0], &Len, 1);
			Message_Length = Len;
		}
		break;

		case CMD_Stop_Data_Strmng:
		{
			memcpy(&dataseq[0], Buff, 5);
			DataLoggerActive = 0;
			TIM_OC_Timers_Stop(&htim1);			//Stop Output Compare Timers
	#if (IMU_PRESENT==1)
			MotionFX_manager_stop_9X();
	#endif
			//Disable all sensors
/*			(void)IKS01A2_ENV_SENSOR_Disable(IKS01A2_LPS22HB_0, ENV_PRESSURE);
			(void)IKS01A2_ENV_SENSOR_Disable(IKS01A2_HTS221_0, ENV_TEMPERATURE);
			(void)IKS01A2_ENV_SENSOR_Disable(IKS01A2_HTS221_0, ENV_HUMIDITY);
			(void)IKS01A2_MOTION_SENSOR_Disable(IKS01A2_LSM6DSL_0, MOTION_ACCELERO);
			(void)IKS01A2_MOTION_SENSOR_Disable(IKS01A2_LSM6DSL_0, MOTION_GYRO);
			(void)IKS01A2_MOTION_SENSOR_Disable(IKS01A2_LSM303AGR_MAG_0, MOTION_MAGNETO); */

			Sensors_Enabled = 0;
	#if (PARTICULATE_SENSOR_PRESENT==1)
			sps30_stop_measurement();			//PowerOff SPS30 PMx sensor
	#endif
			Len = 3;
			Build_Reply_Header(&dataseq[0]);
			CHK_ComputeAndAdd(&dataseq[0], &Len, 1);
			Message_Length = Len;
		}
		break;

		case CMD_Set_DateTime:
		{
			Len = 3;
			memcpy(&dataseq[0], Buff, Len);
			Build_Reply_Header(&dataseq[0]);
//			RTC_TimeRegulate(&hrtc, Buff[3], Buff[4], Buff[5], FORMAT_BIN);
//			RTC_DateRegulate(&hrtc, Buff[6], Buff[7], Buff[8], Buff[9]);
			RTC_DateTimeRegulate(&hrtc, Buff[6], Buff[7], Buff[8], Buff[9], Buff[3], Buff[4], Buff[5], FORMAT_BIN);
			CHK_ComputeAndAdd(&dataseq[0], &Len, 1);
			Message_Length = Len;
		}
		break;

		case CMD_Sensor:
		{
			/* Check if the command length is at least 5 bytes */
			if (size(Buff) < 3)
			{
				return 0;
			}
			Handle_Sensor_command(Buff);
			ret = 0;
		}
		break;

/*		case CMD_ChangeSF:
		{
			Enabled6X = Msg->Data[3];
			if (Enabled6X == 1U)
			{
				MotionFX_manager_stop_9X();
				MotionFX_manager_start_6X();
			}
			else
			{
				MotionFX_manager_stop_6X();
				MotionFX_manager_start_9X();
			}
				Build_Reply_Header(Msg);
				UART_SendMsg(Msg);
			}
			break; */
#endif	//GUI_SUPPORT==1
		case CMD_Set_Date_Time:			//From Remote Controller
		{
			memcpy(&HH[0], &Buff[3], 2);
			hh = (uint8_t)xtoi(HH);
			memcpy(&MN[0], &Buff[5], 2);
			mm = (uint8_t)xtoi(MN);
			memcpy(&SS[0], &Buff[7], 2);
			ss = (uint8_t)xtoi(SS);
			memcpy(&yy[0], &Buff[11], 2);
			y = (uint8_t)xtoi(yy);
			memcpy(&MM[0], &Buff[13], 2);
			m = (uint8_t)xtoi(MM);
			memcpy(&dd[0], &Buff[15], 2);
			d = (uint8_t)xtoi(dd);
			dw = (uint8_t)xtoi((char*)&Buff[17]);

//			RTC_TimeRegulate(&hrtc, hh, mm, ss, FORMAT_BCD);
//			RTC_DateRegulate(&hrtc, y, m, d, dw);
			RTC_DateTimeRegulate(&hrtc, y, m, d, dw, hh, mm, ss, FORMAT_BCD);
			//STM32F1xx loses the date after a reset or a power cycle;
			//therefore we have to memorize the set date in the Flash
//			Write_Flash(0, 0);	//Valid only for STM32F1xx Series

			SendCntrlMsg = true;
			dataseq[0] = 'O';
			dataseq[1] = 'K';
			Message_Length = 2;
		}
		break;

		case CMD_Get_Date_Time:				//From Remote Controller
		{
			RTC_DateTimeStamp(&hrtc, &Stamp);

			len = sprintf((char*)&dataseq[0], "%02u/", Stamp.date[1]);
			len += sprintf((char*)&dataseq[len], "%02u/", Stamp.date[0]);
			len += sprintf((char*)&dataseq[len], "%04u ", Stamp.date[2]);
			dataseq[6] = '2'; dataseq[7] = '0';
			len += sprintf((char*)&dataseq[len], "%02u:", Stamp.time[0]);
			len += sprintf((char*)&dataseq[len], "%02u:", Stamp.time[1]);
			len += sprintf((char*)&dataseq[len], "%02u", Stamp.time[2]);

			SendCntrlMsg = true;
			Message_Length = 19;
		}
		break;

		case CMD_OpenTestEnv_Mode:			//From Remote Controller
		{
			top_menu();
			memset(&Buff[0], 0x00, BUFFLEN);			//Clear VCP Rx Buffer
			memset(&dataseq[0], 0x00, sizeof(dataseq));	//Clear VCP Tx Buffer
			local_buff_offset = 0;
			local_buff_length = 0;
			app.usblen = 0;
		}
		break;

		case CMD_Reset:
		case CMD_Rst:						//From Remote Controller
		{
			NVIC_SystemReset();
		}
		break;
		case CMD_Restart_Reservation:		//From Remote Controller
		{
			if (!Restart_Reverved)
			{
				Restart_Reverved = true;
				CDC_Transmit_FS((uint8_t*)Message, strlen((const char*)Message));
			} else
			{
				Restart_Reverved = false;
				CDC_Transmit_FS((uint8_t*)Message1, strlen((const char*)Message1));
			}
		}
		break;
		default:
			ret = 0;
			break;
	}

  return ret;
}

/**
 * @brief  Handle a message from USART2
 * @param  Msg the pointer to the message to be handled
 * @retval 1 if the message is correctly handled, 0 otherwise
 */
int HandleUSART2_MSG(uint8_t* Buff)
/*  DestAddr | SouceAddr | CMD | PAYLOAD
 *      1          1        1       N
 */
{
	int ret = 1;

	/*	switch(application_mode)
		{
			case STAND_ALONE:
			{

			}
			break;

			case USB_TO_SPI:
			{

			}
			break;

			case USB_PRINT_ONLY:
			{

			}
			break;

			default:
				break;
		} */
	return ret;
}

/**
 * @brief  Handle a message from USART3
 * @param  Msg the pointer to the message to be handled
 * @retval 1 if the message is correctly handled, 0 otherwise
 */
int HandleUSART3_MSG(uint8_t* Buff)
/*  DestAddr | SouceAddr | CMD | PAYLOAD
 *      1          1        1       N
 */
{
	int ret = 0;
	static char dd[3] = {'\0'}; static char MM[3] = {'\0'};	static char yy[3] = {'\0'};
	static char HH[3] = {'\0'}; static char MN[3] = {'\0'}; static char SS[3] = {'\0'};
	static uint8_t y = 0; static uint8_t m = 0; static uint8_t d = 0; static uint8_t dw = 0;
	static uint8_t hh = 0; static uint8_t mm = 0; static uint8_t ss = 0;
	static DateTime_t Stamp; static uint8_t len = 0;

//	if (Msg->Len != 3U)
//	{
//		return 0;
//	}
	if (Buff[0] != DEV_ADDR)
	{
		return 0;
	}
	memset(&dataseq[0], 0x00, sizeof(dataseq));
	switch (Buff[2])   				//CMD
	{
		case CMD_Set_Date_Time:			//From Remote Controller
		{
			memcpy(&HH[0], &Buff[3], 2);
			hh = (uint8_t)xtoi(HH);
			memcpy(&MN[0], &Buff[5], 2);
			mm = (uint8_t)xtoi(MN);
			memcpy(&SS[0], &Buff[7], 2);
			ss = (uint8_t)xtoi(SS);
			memcpy(&yy[0], &Buff[11], 2);
			y = (uint8_t)xtoi(yy);
			memcpy(&MM[0], &Buff[13], 2);
			m = (uint8_t)xtoi(MM);
			memcpy(&dd[0], &Buff[15], 2);
			d = (uint8_t)xtoi(dd);
			dw = (uint8_t)xtoi((char*)&Buff[17]);

//			RTC_TimeRegulate(&hrtc, hh, mm, ss, FORMAT_BCD);
//			RTC_DateRegulate(&hrtc, y, m, d, dw);
			RTC_DateTimeRegulate(&hrtc, y, m, d, dw, hh, mm, ss, FORMAT_BCD);
			//STM32F1xx loses the date after a reset or a power cycle;
			//therefore we have to memorize the set date in the Flash
//			Write_Flash(0, 0);	//Valid only for STM32F1xx Series

			SendCntrlMsg = true;
			dataseq[0] = 'O';
			dataseq[1] = 'K';
			Message_Length = 2;
		}
		break;

		case CMD_Get_Date_Time:				//From Remote Controller
		{
			RTC_DateTimeStamp(&hrtc, &Stamp);

			len = sprintf((char*)&dataseq[0], "%02u/", Stamp.date[1]);
			len += sprintf((char*)&dataseq[len], "%02u/", Stamp.date[0]);
			len += sprintf((char*)&dataseq[len], "%04u ", Stamp.date[2]);
			dataseq[6] = '2'; dataseq[7] = '0';
			len += sprintf((char*)&dataseq[len], "%02u:", Stamp.time[0]);
			len += sprintf((char*)&dataseq[len], "%02u:", Stamp.time[1]);
			len += sprintf((char*)&dataseq[len], "%02u", Stamp.time[2]);

			SendCntrlMsg = true;
			Message_Length = 19;
		}
		break;

		case CMD_OpenTestEnv_Mode:			//From Remote Controller
		{
			top_menu();
			memset(&Buff[0], 0x00, BUFFLEN);			//Clear VCP Rx Buffer
			memset(&dataseq[0], 0x00, sizeof(dataseq));	//Clear VCP Tx Buffer
			usart3_local_buff_offset = 0;
			usart3_local_buff_length = 0;
			usart3app.usartlen = 0;
		}
		break;

		case CMD_Reset:
		case CMD_Rst:						//From Remote Controller
		{
			NVIC_SystemReset();
		}
		break;
		case CMD_Restart_Reservation:		//From Remote Controller
		{
			if (!Restart_Reverved)
			{
				Restart_Reverved = true;
				HAL_UART_Transmit_DMA(&huart3, (uint8_t*)Message, strlen((const char*)Message));
			} else
			{
				Restart_Reverved = false;
				HAL_UART_Transmit_DMA(&huart3, (uint8_t*)Message1, strlen((const char*)Message1));
			}
		}
		break;
		default:
			ret = 0;
			break;
	}

	return ret;
}
