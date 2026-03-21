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
  * @brief  Enters the RTC configuration mode.
  * @param  None
  * @retval None
  */
void RTC_EnterConfigMode(void)
{
	/* Set the CNF flag to enter in the Configuration Mode */
	RTC->CRL |= RTC_CRL_CNF;
}

/**
  * @brief  Exits from the RTC configuration mode.
  * @param  None
  * @retval None
  */
void RTC_ExitConfigMode(void)
{
	/* Reset the CNF flag to exit from the Configuration Mode */
	RTC->CRL &= (uint16_t)~((uint16_t)RTC_CRL_CNF);
}

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
	{ 	/* In this case the counter roll over during reading of CNTL and CNTH registers,
		read again CNTL register then return the counter value */
		return (((uint32_t) high2 << 16 ) | RTC->CNTL);
	}
	else
	{	/* No counter roll over during reading of CNTL and CNTH registers, counter
		value is equal to first value of CNTL and CNTH */
		return (((uint32_t) high1 << 16 ) | low);
	}
}

/**
  * @brief  Sets the RTC counter value.
  * @param  CounterValue: RTC counter new value.
  * @retval None
  */
void RTC_SetCounter(uint32_t CounterValue)
{
	RTC_EnterConfigMode();
	/* Set RTC COUNTER MSB word */
	RTC->CNTH = CounterValue >> 16;
	/* Set RTC COUNTER LSB word */
	RTC->CNTL = (CounterValue & RTC_LSB_MASK);
	RTC_ExitConfigMode();
}

/**
  * @brief  Daylight Saving Time, reads the store operation bit.
  * @param  hrtc RTC handle
  * @retval store operation bit status (1 or 0)
  */
uint32_t RTC_DST_ReadStoreOperation(void)
{
	uint32_t val;

	val = (StatusReg & 0x10000000) >> 28;

	return val;
}

void RTC_DST_SetStoreOperation(void)
{
	StatusReg |= 0x10000000;
	HOST_TO_BKPR_LE_32(BakUpRTC_Data+70, StatusReg);

	enable_backup_rtc();
	writeBkpRTC((uint8_t *)BakUpRTC_Data, sizeof(BakUpRTC_Data), 0);
	disable_backup_rtc();
}

void RTC_DST_ClearStoreOperation(void)
{
	StatusReg &= 0x01111111;
	HOST_TO_BKPR_LE_32(BakUpRTC_Data+70, StatusReg);

	enable_backup_rtc();
	writeBkpRTC((uint8_t *)BakUpRTC_Data, sizeof(BakUpRTC_Data), 0);
	disable_backup_rtc();
}

void RTC_DST_Add1Hour(void)
{
	uint32_t counter;

	counter = RTC_GetCounter();
	counter += 3600;
	RTC_SetCounter(counter);
}

void RTC_DST_Sub1Hour(void)
{
	uint32_t counter;

	counter = RTC_GetCounter();
	counter -= 3600;
	RTC_SetCounter(counter);
}

//#pragma GCC optimize ("O0")
uint32_t CheckDayLigth(RTC_HandleTypeDef* rtcHandle, uint8_t sec, uint8_t min, uint8_t hour,
												     uint8_t day, uint8_t month, uint8_t year)
{
	uint8_t Index;
	uint32_t Counter, DayLigthPeriod;
	int32_t OffSet;
	const uint8_t StartYear = 23;
	/*
	 * Epoch of the entry into force of daylight saving time and the return to solar time in the years
	 * 2023..2035, keeping in mind that the system start date is 01/01/2000 (Unix epoch time: 946684800)
	 */
	const uint32_t EpochDayLightValues[13][2] =
	{
		{733111200,   751863600},	/* 26/03/2023 02:00:00, 29/10/2023 03:00:00 - initializers for row indexed by  0 */
		{765165600,   783313200},	/* 31/03/2024 02:00:00, 27/10/2024 03:00:00 - initializers for row indexed by  1 */
		{796615200,   814762800},	/* 30/03/2025 02:00:00, 26/10/2025 03:00:00 - initializers for row indexed by  2 */
		{828064800,   846212400},	/* 29/03/2026 02:00:00, 25/10/2026 03:00:00 - initializers for row indexed by  3 */
		{859514400,   878266800},	/* 28/03/2027 02:00:00, 31/10/2027 03:00:00 - initializers for row indexed by  4 */
		{890964000,   909716400},	/* 26/03/2028 02:00:00, 29/10/2028 03:00:00 - initializers for row indexed by  5 */
		{922413600,   941166000},	/* 25/03/2029 02:00:00, 28/10/2029 03:00:00 - initializers for row indexed by  6 */
		{954468000,   972615600},	/* 31/03/2030 02:00:00, 27/10/2030 03:00:00 - initializers for row indexed by  7 */
		{985917600,  1004065200},	/* 30/03/2031 02:00:00, 26/10/2031 03:00:00 - initializers for row indexed by  8 */
		{1017367200, 1036119600},	/* 28/03/2032 02:00:00, 31/10/2032 03:00:00 - initializers for row indexed by  9 */
		{1048816800, 1067569200},	/* 27/03/2033 02:00:00, 30/10/2033 03:00:00 - initializers for row indexed by 10 */
		{1080266400, 1099018800},	/* 26/03/2034 02:00:00, 29/10/2034 03:00:00 - initializers for row indexed by 11 */
		{1111716000, 1130468400} 	/* 25/03/2035 02:00:00, 28/10/2035 03:00:00 - initializers for row indexed by 12 */
	};

//	Counter = RTC_ToEpoch(sec, min, hour, day, month, year);
	Counter = RTC_GetCounter();
	DayLigthPeriod = RTC_DST_ReadStoreOperation();
	Index = year - StartYear;
	if ((Index < 0) || (Index > 12))
		return Counter;
	/*
	 * When switching back to standard time, one hour is subtracted from the RTC Counter value.
	 * This causes a bounce between the daylight saving time setting and the standard time setting in the comparison on line 329.
     * To avoid this bounce, when switching back to standard time, an offset of one hour is added to the second member of the
     * comparison on line 329.
     * The offset value returns to zero when more than one hour has passed since switching back to standard time
     * and when daylight saving time is in effect.
	 */
	if (!DayLigthPeriod)
	{
		OffSet = Counter - (EpochDayLightValues[Index][1] - 3600);
		if ((OffSet < 0) || (OffSet > 3600))
		{
			OffSet = 0;
		} else
		if ((OffSet >= 0) && (OffSet <= 3600))
		{
			OffSet = 3600;
		}
	} else
	{
		OffSet = 0;
	}

	if ((Counter > EpochDayLightValues[Index][0]) && ((Counter + (uint32_t)OffSet) < EpochDayLightValues[Index][1]))
	{
		if (!DayLigthPeriod)
		{
			RTC_DST_Add1Hour();
			RTC_DST_SetStoreOperation();
		}
	} else
	{
		if (DayLigthPeriod)
		{
			RTC_DST_Sub1Hour();
			RTC_DST_ClearStoreOperation();
		}
	}

	return Counter;
}

/**
 * @brief  Handles the data timestamp
 * @param  Stamp time + date
 * @retval None
 */
void RTC_DateTimeStamp(RTC_HandleTypeDef* rtcHandle, DateTime_t *Stamp)
{
	RTC_DateTypeDef date;
	RTC_TimeTypeDef time;
	extern FLASH_DATA_ORG FlashDataOrg;

	if(rtcHandle->Instance==RTC)
	{
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

#if (RTC_SET_VALUES==0)
		Stamp->epoch_timestamp = CheckDayLigth(rtcHandle, time.Seconds, time.Minutes, time.Hours, date.Date, date.Month, date.Year);
#endif

		Stamp->date[0] = (uint8_t)date.Month;
		Stamp->date[1] = (uint8_t)date.Date;
		Stamp->date[2] = (uint8_t)date.Year;
		Stamp->time[0] = (uint8_t)time.Hours;
		Stamp->time[1] = (uint8_t)time.Minutes;
		Stamp->time[2] = (uint8_t)time.Seconds;
		Stamp->time[3] = 0;

		memcpy(&FlashDataOrg.b_date, &Stamp->date[0], 3);
		memcpy(&FlashDataOrg.b_time, &Stamp->time[0], 4);

		if (!(Stamp->time[0] | Stamp->time[1]))
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
