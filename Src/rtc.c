/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    rtc.c
  * @brief   This file provides code for the configuration
  *          of the RTC instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "rtc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

RTC_HandleTypeDef hrtc;

/* RTC init function */
void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */
#if (RTC_SET_VALUES==1)
  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */
#endif
  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
#if (RTC_SET_VALUES==1)
  RTC_DateTypeDef DateToUpdate = {0};
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0x1;
  DateToUpdate.Year = 0x21;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
#else
  //Following a reset or a power cycle we must restore the date stored in the RTC registers.
  //Due to the problems encountered, the rtc_read_backup_reg and rtc_write_backup_reg functions
  //are not used for date backup and restore. The date is stored in flash using the WriteFlash
  //function and restored in the RTC registers by means of the ReadFlash function,
  //subsequently called by the AB_Init function.
/*  DateToUpdate.WeekDay = rtc_read_backup_reg(RTC_BKP_DR8);
  DateToUpdate.Month = rtc_read_backup_reg(RTC_BKP_DR5);
  DateToUpdate.Date = rtc_read_backup_reg(RTC_BKP_DR6);
  DateToUpdate.Year = rtc_read_backup_reg(RTC_BKP_DR7);
  if (HAL_RTC_SetDateTime(rtcHandle, &DateToUpdate, &&sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  } */

  /* When AsynchPrediv is 0xFFFFFFFF the RTC prescaler is automatically set to obtain 1 second,
   * otherwise the prescaler value is set to the value read by the flash.
   * When the crystal ocillation frequency is 32768, the prescaler value must be 32767
   * to obtain 1 second
   */
  extern FLASH_DATA_ORG FlashDataOrg;

  hrtc.Init.AsynchPrediv = *((uint32_t*)(DATA_EEPROM_BASE+FlashDataOrg.b_status.s8_offset));
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
#endif
  /* USER CODE END RTC_Init 2 */

}

void HAL_RTC_MspInit(RTC_HandleTypeDef* rtcHandle)
{

  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspInit 0 */

  /* USER CODE END RTC_MspInit 0 */
    HAL_PWR_EnableBkUpAccess();
    /* Enable BKP CLK enable for backup registers */
    __HAL_RCC_BKP_CLK_ENABLE();
    /* RTC clock enable */
    __HAL_RCC_RTC_ENABLE();

    /* RTC interrupt Init */
    HAL_NVIC_SetPriority(RTC_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(RTC_IRQn);
  /* USER CODE BEGIN RTC_MspInit 1 */

  /* USER CODE END RTC_MspInit 1 */
  }
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef* rtcHandle)
{

  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspDeInit 0 */

  /* USER CODE END RTC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_RTC_DISABLE();

    /* RTC interrupt Deinit */
    HAL_NVIC_DisableIRQ(RTC_IRQn);
  /* USER CODE BEGIN RTC_MspDeInit 1 */

  /* USER CODE END RTC_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/*************************************************************************
 *                          MY RTC CODE
 ************************************************************************/
/**
  * @brief  Gets the RTC counter value.
  * @param  None
  * @retval RTC counter value.
  */
uint32_t RTC_GetCounter(void)
{
  uint16_t high1 = 0, high2 = 0, low = 0;

  high1 = RTC->CNTH;
  low   = RTC->CNTL;
  high2 = RTC->CNTH;

  if (high1 != high2)
  { /* In this case the counter roll over during reading of CNTL and CNTH registers,
       read again CNTL register then return the counter value */
    return (((uint32_t) high2 << 16 ) | RTC->CNTL);
  }
  else
  { /* No counter roll over during reading of CNTL and CNTH registers, counter
       value is equal to first value of CNTL and CNTH */
    return (((uint32_t) high1 << 16 ) | low);
  }
}

/**
 * @brief  Handles the time+date getting
 * @param  Stamp time + date
 * @retval None
 */
void RTC_DateTimeStamp(RTC_HandleTypeDef* rtcHandle, DateTime_t *Stamp)
{
	RTC_DateTypeDef date;
	RTC_TimeTypeDef time;

	if (HAL_RTC_GetDateTime(rtcHandle, &date, &time, FORMAT_BIN) != HAL_OK)
	{
		HAL_RTC_MspDeInit(rtcHandle);
		HAL_RTC_MspInit(rtcHandle);
	}
/*	if (HAL_RTC_GetTime(rtcHandle, &time, FORMAT_BIN) != HAL_OK)
	{
		HAL_RTC_MspDeInit(rtcHandle);
		HAL_RTC_MspInit(rtcHandle);
	}
	if (HAL_RTC_GetDate(rtcHandle, &date, FORMAT_BIN) != HAL_OK)
	{
		HAL_RTC_MspDeInit(rtcHandle);
		HAL_RTC_MspInit(rtcHandle);
	} */

	Stamp->date[0] = (uint8_t)date.Month;
	Stamp->date[1] = (uint8_t)date.Date;
	Stamp->date[2] = (uint8_t)date.Year;
	Stamp->time[0] = (uint8_t)time.Hours;
	Stamp->time[1] = (uint8_t)time.Minutes;
	Stamp->time[2] = (uint8_t)time.Seconds;
}

/**
 * @brief  Handles the time+date getting
 * @param  Msg the time+date part of the stream
 * @retval None
 */
void RTC_Handler(RTC_HandleTypeDef* rtcHandle, uint8_t* Buff)
{
	uint8_t sub_sec = 0;
//	uint32_t ans_uint32;
//	int32_t ans_int32;
	RTC_DateTypeDef sdatestructureget;
	RTC_TimeTypeDef stimestructure;

	if(rtcHandle->Instance==RTC)
	{
		if (HAL_RTC_GetDateTime(rtcHandle, &sdatestructureget, &stimestructure, FORMAT_BIN) != HAL_OK)
		{
			HAL_RTC_MspDeInit(rtcHandle);
			HAL_RTC_MspInit(rtcHandle);
		}
/*		if (HAL_RTC_GetTime(rtcHandle, &stimestructure, FORMAT_BIN) != HAL_OK)
		{
			HAL_RTC_MspDeInit(rtcHandle);
			HAL_RTC_MspInit(rtcHandle);
		}
		if (HAL_RTC_GetDate(rtcHandle, &sdatestructureget, FORMAT_BCD) != HAL_OK)
		{
			HAL_RTC_MspDeInit(rtcHandle);
			HAL_RTC_MspInit(rtcHandle);
		} */

		/* To be MISRA C-2012 compliant the original calculation:
		sub_sec = ((((((int)RtcSynchPrediv) - ((int)stimestructure.SubSeconds)) * 100) / (RtcSynchPrediv + 1)) & 0xFF);
		has been split to separate expressions */
//		ans_int32 = (RtcSynchPrediv - (int32_t)stimestructure.SubSeconds) * 100;
//		ans_int32 /= RtcSynchPrediv + 1;
//		ans_uint32 = (uint32_t)ans_int32 & 0xFFU;
//		sub_sec = (uint8_t)ans_uint32;

		Buff[3] = (uint8_t)stimestructure.Hours;
		Buff[4] = (uint8_t)stimestructure.Minutes;
		Buff[5] = (uint8_t)stimestructure.Seconds;
		Buff[6] = sub_sec;

		if (!(Buff[3] | Buff[4]))
		{
			MidNight = true;
		} else
		{
			MidNight = false;
		}
	}
}

/**
 * @brief  Configures the current date
 * @param  y the year value to be set
 * @param  m the month value to be set
 * @param  d the day value to be set
 * @param  dw the day-week value to be set
 * @retval None
 */
void RTC_DateRegulate(RTC_HandleTypeDef* rtcHandle, uint8_t y, uint8_t m, uint8_t d, uint8_t dw)
{
	RTC_DateTypeDef sdatestructure;

	sdatestructure.Year = y;
	sdatestructure.Month = m;
	sdatestructure.Date = d;
	sdatestructure.WeekDay = dw;

	if (HAL_RTC_SetDate(rtcHandle, &sdatestructure, FORMAT_BCD) != HAL_OK)
	{
		/* Initialization Error */
		HAL_RTC_MspDeInit(rtcHandle);
		HAL_RTC_MspInit(rtcHandle);
	}
}

/**
 * @brief  Configures the current time
 * @param  hh the hour value to be set
 * @param  mm the minute value to be set
 * @param  ss the second value to be set
 * @retval None
 */
void RTC_TimeRegulate(RTC_HandleTypeDef* rtcHandle, uint8_t hh, uint8_t mm, uint8_t ss, uint32_t Format)
{
	RTC_TimeTypeDef stimestructure;

//	stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
	stimestructure.Hours = hh;
	stimestructure.Minutes = mm;
	stimestructure.Seconds = ss;
//	stimestructure.SubSeconds = 0;
//	stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
//	stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

	if (HAL_RTC_SetTime(rtcHandle, &stimestructure, Format) != HAL_OK)
	{
		/* Initialization Error */
		HAL_RTC_MspDeInit(rtcHandle);
		HAL_RTC_MspInit(rtcHandle);
	}
}

/**
 * @brief  Configures the current date & time
 * @param  y the year value to be set
 * @param  m the month value to be set
 * @param  d the day value to be set
 * @param  dw the day-week value to be set
 * @param  hh the hour value to be set
 * @param  mm the minute value to be set
 * @param  ss the second value to be set
 * @retval None
 */
void RTC_DateTimeRegulate(RTC_HandleTypeDef* rtcHandle, uint8_t y, uint8_t m, uint8_t d, uint8_t dw,
														uint8_t hh, uint8_t mm, uint8_t ss, uint32_t Format)
{
	RTC_DateTypeDef sdatestructure;
	RTC_TimeTypeDef stimestructure;

	sdatestructure.Year = y;
	sdatestructure.Month = m;
	sdatestructure.Date = d;
	sdatestructure.WeekDay = dw;
	stimestructure.Hours = hh;
	stimestructure.Minutes = mm;
	stimestructure.Seconds = ss;

	if (HAL_RTC_SetDateTime(rtcHandle, &sdatestructure, &stimestructure, Format) != HAL_OK)
	{
		/* Initialization Error */
		HAL_RTC_MspDeInit(rtcHandle);
		HAL_RTC_MspInit(rtcHandle);
	}
}

void enable_backup_rtc(void)
{
	/*PWREN : Enable backup domain access; Enable the PWR clock */
	__HAL_RCC_PWR_CLK_ENABLE();
	/*DBP : Enable access to Backup domain */
	HAL_PWR_EnableBkUpAccess();
}

void disable_backup_rtc(void)
{
	/*PWREN : Disable backup domain access  */
	__HAL_RCC_PWR_CLK_DISABLE();
	/*DBP : Disable access to Backup domain */
	HAL_PWR_DisableBkUpAccess();
}

/**
  * @brief  Writes a buffer in the RTC Backup data register.
  * @param  data: pointer to the data structure to be written in the RTC Backup data register.
  * @param  bytes: number of bytes to write in the RTC Backup data register.
  * 		Since the RTC Backup data register is made up of 16bit registers, it must be a multiple of 2
  * @param  offset: is the displacement of the RTC Backup data register from where "data" will be written
  *			It can take values from 0 .. ((RTC_BKUP_SIZE / 2) -1)
  * @retval 0 = Success
  */
int8_t writeBkpRTC(uint8_t *data, uint16_t bytes, uint16_t offset)
{
	uint32_t base_addr = (uint32_t)BKP_BASE;
	uint16_t i;
	uint16_t val = 0;
	uint16_t ofst = 0;
	uint8_t bank = 0;

	ofst = offset;
	if( bytes + offset > RTC_BKUP_SIZE+2)
	{
	/* ERROR : the last byte is outside the backup SRAM region */
		return -1;
	} else if(bytes % 2 )
	{
	/* ERROR: data start or num bytes are not word aligned */
	return -2;
	} else
	{
		bytes >>= 1;	/* divide by 2 because writing half-words */
		offset <<= 2;	/* multiply by 4 because addressing words */
	}

	/* Enable clock to BKRTC */
	__HAL_RCC_BKP_CLK_ENABLE();
	for( i = 0; i < bytes; i++ )
	{
		if ((uint8_t)(i+ofst) < 10)
			bank = 0;
		else
			bank = 20;
		val =  ((uint16_t)(*(data + i*2 + ofst + 1) & 0xFF) << 8) | (uint16_t)(*(data + i*2 + ofst) & 0xFF);
		*(__IO uint32_t *)(base_addr + bank + offset + i*4 + 4) = val;
//		HAL_RTCEx_BKUPWrite(&hrtc, (offset << 2), val);
	}
	/* Disable clock to BKPRTC */
	__HAL_RCC_BKP_CLK_DISABLE();

	return 0;
}

/**
  * @brief  Reads a buffer from the RTC Backup data register.
  * @param  data: pointer to the data structure to be read from the RTC Backup data register.
  * @param  bytes: number of bytes to read from the RTC Backup data register.
  * 		Since the RTC Backup data register is made up of 16bit registers, it must be a multiple of 2
  * @param  offset: is the displacement of the RTC Backup data register from where values will be read
  *			It can take values from 0 .. ((RTC_BKUP_SIZE / 2) -1)
  * @retval 0 = Success
  */
uint8_t readBkpRTC(uint8_t *data, uint16_t bytes, uint16_t offset)
{
	uint32_t base_addr = (uint32_t)BKP_BASE;
	uint16_t i;
	uint16_t val = 0;
	uint16_t ofst = 0;
	uint8_t bank = 0;

	ofst = offset;
	if( bytes + offset > RTC_BKUP_SIZE+2)
	{
	/* ERROR : the last byte is outside the backup SRAM region */
		return -1;
	} else if(bytes % 2)
	{
		/* ERROR: data start or num bytes are not word aligned */
		return -2;
	} else
	{
		bytes >>= 1;      /* divide by 2 because writing half-words */
		offset <<= 2;	/* multiply by 4 because addressing words */
	}

	/* read should be 16 bit aligned */
	for( i = 0; i < bytes; i++ )
	{
		if ((uint8_t)(i+ofst) < 10)
			bank = 0;
		else
			bank = 20;
		val = *(__IO uint32_t *)(base_addr + bank + offset + i*4 + 4);
		*(data + i*2 + ofst) = (uint8_t)(val & 0xFF);
		*(data + i*2 + +ofst + 1) = (uint8_t)((val >> 8) & 0xFF);
	}

	return 0;
}
/* USER CODE END 1 */
