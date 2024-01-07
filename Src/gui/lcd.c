/*
 * lcd.c
 *
 *  Created on: 10/06/2018
 *      Author: Olivier Van den Eede (20x2 text LCD display driver only)
 *      Modified by: Tommaso Sabatini 18/03/2020
 *      Added Graphic LCD functions by: Tommaso Sabatini 03/08/2020
 */

#include "platform/port.h"
#include "gui/lcd.h"
#include "main.h"
#define SHOW_PM_NUMBER (0)

uint8_t SLen;
const uint8_t NumENV_Pages = 2;
#if (FULL_MODE==1)
	const uint8_t NumGAS_Pages = 2;
#else
	const uint8_t NumGAS_Pages = 1;
#endif
#if (SHOW_PM_NUMBER==1)
	const uint8_t NumPM_Pages = 3;
#else
	const uint8_t NumPM_Pages = 1;
#endif
const char sensor_fail[] = "FAIL";
const char sensor_fail1a[] = " FAIL";
const char sensor_fail1b[] = " FAIL ";
const char sensor_fail1c[] = " FAIL  ";
const char sensor_fail2p[] = "FAIL  ";
#if ((UVx_SENSOR_PRESENT==1) && ((GLCD_SUPPORT==1) || (TLCD_SUPPORT==1)))
	static bool UVx_Display = false;
	static bool ClearDisplay = false;
#endif

#if (TLCD_SUPPORT==1)

const uint8_t ROW_16[] = {0x00, 0x40, 0x10, 0x50};
const uint8_t ROW_20[] = {0x00, 0x40, 0x14, 0x54};
const uint8_t NRows = 2;
/************************************** Static declarations **************************************/
/*
static void lcd_write_data(Lcd_HandleTypeDef * lcd, uint8_t data);
static void lcd_write_command(Lcd_HandleTypeDef * lcd, uint8_t command);
static void lcd_write(Lcd_HandleTypeDef * lcd, uint8_t data, uint8_t len); */

/************************************** Function definitions **************************************/

/**
 * Create new Lcd_HandleTypeDef and initialize the Lcd
 */
Lcd_HandleTypeDef Lcd_create(
		Lcd_PortType port[], Lcd_PinType pin[],
		Lcd_PortType rs_port, Lcd_PinType rs_pin,
		Lcd_PortType en_port, Lcd_PinType en_pin, Lcd_ModeTypeDef mode)
{
	Lcd_HandleTypeDef LCD;

	LCD.mode = mode;

	LCD.en_pin = en_pin;
	LCD.en_port = en_port;

	LCD.rs_pin = rs_pin;
	LCD.rs_port = rs_port;

	LCD.data_pin = pin;
	LCD.data_port = port;

	Lcd_init(&LCD);

	return LCD;
}

/**
 * Initialize 16x2-lcd without cursor
 */
void Lcd_init(Lcd_HandleTypeDef * lcd)
{
	if(lcd->mode == LCD_4_BIT_MODE)
	{
		lcd_write_command(lcd, 0x33);
		lcd_write_command(lcd, 0x32);
		lcd_write_command(lcd, FUNCTION_SET | OPT_N);				// 4-bit mode
	}
	else
		lcd_write_command(lcd, FUNCTION_SET | OPT_DL | OPT_N);

	lcd_write_command(lcd, CLEAR_DISPLAY);						// Clear screen
	lcd_write_command(lcd, DISPLAY_ON_OFF_CONTROL | OPT_D);		// Lcd-on, cursor-off, no-blink
	lcd_write_command(lcd, ENTRY_MODE_SET | OPT_INC);			// Increment cursor
}

/**
 * Write a number on the current position
 */
void Lcd_int(Lcd_HandleTypeDef * lcd, int number)
{
	uint8_t buffer[11];
	sprintf((char*)&buffer[0], "%d", number);

	Lcd_string(lcd, (char*)buffer, 11);
}

/**
 * Write a string on the current position
 */
void Lcd_string(Lcd_HandleTypeDef *lcd, char *string, int len)
{
	for(uint8_t i = 0; i < len; i++)
	{
		if (string[i] == 0x0D)
		{
			Column = 0;
			Lcd_cursor(lcd, Row, Column);
		} else
		if (string[i] == 0x0A)
		{
			if (++Row > NRows-1)
			{
				Row = 0;
			}
			Lcd_cursor(lcd, Row, Column);
		} else
		lcd_write_data(lcd, string[i]);
	}
}

/**
 * Set the cursor position
 */
void Lcd_cursor(Lcd_HandleTypeDef *lcd, uint8_t row, uint8_t col)
{
	#ifdef LCD20xN
	lcd_write_command(lcd, SET_DDRAM_ADDR + ROW_20[row] + col);
	#endif

	#ifdef LCD16xN
	lcd_write_command(lcd, SET_DDRAM_ADDR + ROW_16[row] + col);
	#endif
}

/**
 * Clear the screen
 */
void Lcd_clear(Lcd_HandleTypeDef *lcd)
{
	lcd_write_command(lcd, CLEAR_DISPLAY);
	Row = 0; Column = 0;
}

/************************************** Static function definition **************************************/

/**
 * Write a byte to the command register
 */
void lcd_write_command(Lcd_HandleTypeDef *lcd, uint8_t command)
{
	HAL_GPIO_WritePin(lcd->rs_port, lcd->rs_pin, LCD_COMMAND_REG);		// Write to command register

	if(lcd->mode == LCD_4_BIT_MODE)
	{
		lcd_write(lcd, (command >> 4), LCD_NIB);
		lcd_write(lcd, command & 0x0F, LCD_NIB);
	}
	else
	{
		lcd_write(lcd, command, LCD_BYTE);
	}
}

/**
 * Write a byte to the data register
 */
void lcd_write_data(Lcd_HandleTypeDef *lcd, uint8_t data)
{
	HAL_GPIO_WritePin(lcd->rs_port, lcd->rs_pin, LCD_DATA_REG);			// Write to data register

	if(lcd->mode == LCD_4_BIT_MODE)
	{
		lcd_write(lcd, data >> 4, LCD_NIB);
		lcd_write(lcd, data & 0x0F, LCD_NIB);
	}
	else
	{
		lcd_write(lcd, data, LCD_BYTE);
	}
}

/**
 * Set len bits on the bus and toggle the enable line
 */
void lcd_write(Lcd_HandleTypeDef *lcd, uint8_t data, uint8_t Len)
{
/*	for(uint8_t i = 0; i < Len; i++)
	{
		HAL_GPIO_WritePin(lcd->data_port[i], lcd->data_pin[i], (data >> i) & 0x01);
	}
	HAL_GPIO_WritePin(lcd->data_port[0], lcd->data_pin[0], (data >> 0) & 0x01);
	HAL_GPIO_WritePin(lcd->data_port[1], lcd->data_pin[1], (data >> 1) & 0x01);
	HAL_GPIO_WritePin(lcd->data_port[2], lcd->data_pin[2], (data >> 2) & 0x01);
	HAL_GPIO_WritePin(lcd->data_port[3], lcd->data_pin[3], (data >> 3) & 0x01);
	HAL_GPIO_WritePin(lcd->data_port[4], lcd->data_pin[4], (data >> 4) & 0x01);
	HAL_GPIO_WritePin(lcd->data_port[5], lcd->data_pin[5], (data >> 5) & 0x01);
	HAL_GPIO_WritePin(lcd->data_port[6], lcd->data_pin[6], (data >> 6) & 0x01);
	HAL_GPIO_WritePin(lcd->data_port[7], lcd->data_pin[7], (data >> 7) & 0x01); */

// 	HAL_GPIO_WritePin(D0_LCD_GPIO_Port, D0_LCD_Pin, (data >> 0) & 0x01);
//	HAL_GPIO_WritePin(D1_LCD_GPIO_Port, D1_LCD_Pin, (data >> 1) & 0x01);
//	HAL_GPIO_WritePin(D2_LCD_GPIO_Port, D2_LCD_Pin, (data >> 2) & 0x01);
//	HAL_GPIO_WritePin(D3_LCD_GPIO_Port, D3_LCD_Pin, (data >> 3) & 0x01);
	HAL_GPIO_WritePin(D4_LCD_SPI2_NSS_GPIO_Port, D4_LCD_SPI2_NSS_Pin, (data >> 0) & 0x01);
	HAL_GPIO_WritePin(D5_LCD_SPI2_SCK_GPIO_Port, D5_LCD_SPI2_SCK_Pin, (data >> 1) & 0x01);
	HAL_GPIO_WritePin(D6_LCD_SPI2_MISO_GPIO_Port, D6_LCD_SPI2_MISO_Pin, (data >> 2) & 0x01);
	HAL_GPIO_WritePin(D7_LCD_SPI2_MOSI_GPIO_Port, D7_LCD_SPI2_MOSI_Pin, (data >> 3) & 0x01);

	HAL_GPIO_WritePin(lcd->en_port, lcd->en_pin, 1);
	DELAY(1);
	HAL_GPIO_WritePin(lcd->en_port, lcd->en_pin, 0); 		// Data receive on falling edge
}

void send_tlcdmessage(char *string, int len)
{
	Lcd_string(&my_lcd, (char*)&string[0], len);
}

void LCD_Clear()
{
	Lcd_clear(&my_lcd);
	Lcd_cursor(&my_lcd, 0, 0);
}

/**
 * Initalize Text LCD system
 */
LCD_Error_et MX_TLCD_Init()
{
	PM_toggle = 1; NumberOfPages = 1;

	Config_SPI2_Port_AsGPIO(GPIO_PIN_RESET);

//	Lcd_PortType ports[] = {D0_LCD_GPIO_Port, D1_LCD_GPIO_Port, D2_LCD_GPIO_Port, D3_LCD_GPIO_Port,
//							D4_LCD_SPI2_NSS_GPIO_Port, D5_LCD_SPI2_SCK_GPIO_Port, D6_LCD_SPI2_MISO_GPIO_Port, D7_LCD_SPI2_MOSI_GPIO_Port};
	Lcd_PortType ports[] = {D4_LCD_SPI2_NSS_GPIO_Port, D5_LCD_SPI2_SCK_GPIO_Port, D6_LCD_SPI2_MISO_GPIO_Port, D7_LCD_SPI2_MOSI_GPIO_Port};
//	Lcd_PinType pins[] = {D0_LCD_Pin, D1_LCD_Pin, D2_LCD_Pin, D3_LCD_Pin,
//						  D4_LCD_SPI2_NSS_Pin, D5_LCD_SPI2_SCK_Pin, D6_LCD_SPI2_MISO_Pin, D7_LCD_SPI2_MOSI_Pin};
	Lcd_PinType pins[] = {D4_LCD_SPI2_NSS_Pin, D5_LCD_SPI2_SCK_Pin, D6_LCD_SPI2_MISO_Pin, D7_LCD_SPI2_MOSI_Pin};
	my_lcd = Lcd_create(ports, pins, RS_LCD_GPIO_Port, RS_LCD_Pin, EN_LCD_GPIO_Port, EN_LCD_Pin, LCD_4_BIT_MODE);

	return LCD_OK;
}

#elif (GLCD_SUPPORT==1)

static u8g2_t u8g2;
/**
 * GLCD Functions
 */
uint8_t u8x8_stm32_gpio_and_delay(U8X8_UNUSED u8x8_t *u8x8,
		U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int,
		U8X8_UNUSED void *arg_ptr)
{
	switch (msg)
	{
		case U8X8_MSG_GPIO_AND_DELAY_INIT:
			HAL_Delay(1);
		break;
		case U8X8_MSG_DELAY_MILLI:
			HAL_Delay(arg_int);
		break;
		case U8X8_MSG_GPIO_DC:
			HAL_GPIO_WritePin(RS_LCD_GPIO_Port, RS_LCD_Pin, arg_int);
		break;
		case U8X8_MSG_GPIO_RESET:
			HAL_GPIO_WritePin(EN_LCD_GPIO_Port, EN_LCD_Pin, arg_int);
		break;
	}

	return 1;
}

uint8_t u8x8_byte_4wire_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
	switch (msg)
	{
		case U8X8_MSG_BYTE_SEND:
			HAL_SPI_Transmit(&hspi2, (uint8_t *) arg_ptr, arg_int, 10000);
		break;
		case U8X8_MSG_BYTE_INIT:
		break;
		case U8X8_MSG_BYTE_SET_DC:
			HAL_GPIO_WritePin(RS_LCD_GPIO_Port, RS_LCD_Pin, arg_int);
		break;
		case U8X8_MSG_BYTE_START_TRANSFER:
			HAL_GPIO_WritePin(D4_LCD_SPI2_NSS_GPIO_Port,
					          D4_LCD_SPI2_NSS_Pin, Disp_Area);
			HAL_GPIO_WritePin(RS_LCD_GPIO_Port, RS_LCD_Pin, GPIO_PIN_SET);
		break;
		case U8X8_MSG_BYTE_END_TRANSFER:
			HAL_GPIO_WritePin(RS_LCD_GPIO_Port, RS_LCD_Pin, GPIO_PIN_RESET);
		break;
		default:
		return 0;
	}

	return 1;
}

void GLCD_Clear(bool next_page, bool clear)
{
	if (next_page)
	{
		u8g2_NextPage(&u8g2);
	} else
	{
		u8g2_SendBuffer(&u8g2);
	}
	if (clear)
	{
		u8g2_ClearBuffer(&u8g2);
	}
}

void send_glcdmessage(char *str, uint8_t xpos, uint8_t ypos, uint8_t yspace, uint8_t len, bool clearbuff, bool next_page)
{
	char * pch;
	uint8_t y_pos = ypos;
	uint8_t i = 1;

	u8g2_DrawStr(&u8g2, xpos, y_pos, &str[0]);
	pch=strchr(str,'\n');
	while (pch!=NULL)
	{
//		printf ("found at %d\n",pch-str+1);
		i++;
		u8g2_DrawStr(&u8g2, xpos, (uint8_t)(i*y_pos+yspace*(i-1)), &str[pch-str+1]);
		pch=strchr(pch+1,'\n');
	}

	GLCD_Clear(next_page, clearbuff);
}

LCD_Error_et ReDrawPage_S0(uint8_t PageNumb)
{
//	const uint8_t mystring[] = "P    Td    Tp\r\n";
	const uint8_t mystring[] = {0xB0,0x43,0x0D,0x0A,0x00};	//°C\r\n";

	Disp_Area = 0;
	if (!(PageNumb == PreviousPage))
	{
//		u8g2_ClearDisplay(&u8g2);
		u8g2_ClearBuffer(&u8g2);
	}
	switch (PageNumb)
	{
		case 0x01:	//Environmental page
		{
			if (PM_toggle == 2)		//+1 because PM_toggle is incremented at the end of the
			{						//PRESSURE_SENSOR section of the FormatDisplayString function
		#if (UVx_SENSOR_PRESENT==1)
				if ((UVx_Display) || (ClearDisplay))
				{
					u8g2_ClearDisplay(&u8g2);
					ClearDisplay = false;
				}
		#endif
				//Resends string, so it will be displayed immediately
				u8g2_DrawStr(&u8g2, TextXPos, TextYPos, (char*)&usbVCOMout[0]);

				u8g2_DrawRFrame(&u8g2, 0, 0, 46, 31, 3);
	//			u8g2_DrawLine(&u8g2, 0, 15, 45, 15);
				u8g2_DrawRFrame(&u8g2, 47, 0, 40, 31, 3);
				u8g2_DrawRFrame(&u8g2, 88, 0, 51, 31, 3);
				u8g2_DrawRFrame(&u8g2, 140, 0, 52, 31, 3);
				u8g2_SetFont(&u8g2, u8g2_font_t0_12b_tf);
		#if (CALC_ALTITUDE==1)
				u8g2_DrawStr(&u8g2, 20, 12, "T      RH      Pr      H\r\n");
		#elif (CALC_DEWPOINT==1)
				u8g2_DrawStr(&u8g2, 20, 12, "T      RH\r\n");
				u8g2_DrawStr(&u8g2, 112, 12, "Pr   Td    Tp\r\n");
				u8g2_SetFont(&u8g2, u8g2_font_t0_11_tf);
				u8g2_DrawStr(&u8g2, 159, 13, (char*)&mystring[0]);
		#endif
				u8g2_SetFont(&u8g2, u8g2_font_unifont_t_weather);
				switch (forecast & 0x3F)	//Mask Steady and pressure increase/decrease bits
				{
					case 0x00:			//Sunny
					{
						u8g2_DrawGlyph(&u8g2, 92, 16, 0x0033);		//Sunny
					}
					break;

					case 0x01:			//Cloudy
					{
					//	u8g2_DrawGlyph(&u8g2, 92, 16, 0x0034);		//Partially Cloudy
						u8g2_DrawGlyph(&u8g2, 92, 16, 0x0035);		//Cloudy
					}
					break;

					case 0x02:			//Rainy
					{
					//	u8g2_DrawGlyph(&u8g2, 92, 16, 0x0036);		//Light Rain
						u8g2_DrawGlyph(&u8g2, 92, 16, 0x0037);		//Rain
					}
					break;

					case 0x03:			//Stormy
					{
					//	u8g2_DrawGlyph(&u8g2, 92, 16, 0x0038);		//Heavy Rain
						u8g2_DrawGlyph(&u8g2, 92, 16, 0x0039);		//Thunderstorm
					//	u8g2_DrawGlyph(&u8g2, 92, 16, 0x003A);		//Storm
					}
					break;
				}
				u8g2_SetFont(&u8g2, u8g2_font_unifont_t_0_86);
				if (BIT_CHECK(forecast,6))
				{
					u8g2_DrawGlyph(&u8g2, 125, 14, 0x003D);			//Pressure Steady
				} else
				if (BIT_CHECK(forecast,7))
				{
					u8g2_DrawGlyph(&u8g2, 125, 16, 0x2B07);			//Pressure Decreasing
				} else
					u8g2_DrawGlyph(&u8g2, 125, 16, 0x2B06);			//Pressure Increasing
			} else
			if (PM_toggle == 1)		//+1 because PM_toggle is incremented at the end of the
			{						//PRESSURE_SENSOR section of the FormatDisplayString function
				u8g2_ClearDisplay(&u8g2);
				//Resends string, so it will be displayed immediately
				u8g2_DrawStr(&u8g2, TextXPos, TextYPos, (char*)&usbVCOMout[0]);

				u8g2_SetFont(&u8g2, u8g2_font_t0_12b_tf);
			#if (VEML6075)
				u8g2_DrawRFrame(&u8g2, 0, 0, 63, 31, 3);
				u8g2_DrawRFrame(&u8g2, 64, 0, 63, 31, 3);
				u8g2_DrawRFrame(&u8g2, 128, 0, 63, 31, 3);
				u8g2_DrawStr(&u8g2, 25, 12, "UVA       UVB     UV Index\r\n");
			#elif (LTR390UV)
				u8g2_DrawRFrame(&u8g2, 0, 0, 65, 31, 3);
				u8g2_DrawRFrame(&u8g2, 66, 0, 63, 31, 3);
				u8g2_DrawRFrame(&u8g2, 130, 0, 62, 31, 3);
				u8g2_DrawStr(&u8g2, 10, 12, "ILLUMIN.   UV Index     SPL\r\n");
			#endif
			}
		}
		break;
		case 0x02:	//Air Quality page
		{
	#if (GAS_SENSOR_MODULE_PRESENT==1)
		#if (FULL_MODE==1)
			if (PM_toggle == 2)		//+1 because PM_toggle is incremented at the end of the
			{						//VOC_SENSOR section of the FormatDisplayString function
				u8g2_ClearBuffer(&u8g2);
				//Resends string, so it will be displayed immediately
				u8g2_DrawStr(&u8g2, TextXPos, TextYPos, (char*)&usbVCOMout[0]);

				u8g2_SetFont(&u8g2, u8g2_font_5x8_mf);
				u8g2_DrawStr(&u8g2, 6, 18, " (ppb)    (mg/mc)  (ug/mc)   (ug/mc)\r\n");
				u8g2_SetFont(&u8g2, u8g2_font_t0_12b_tf);
				u8g2_DrawStr(&u8g2, 9, 10, "eTVOC     CO     NO2     NH3\r\n");

				u8g2_DrawRFrame(&u8g2, 0, 0, 47, 31, 3);
				u8g2_DrawRFrame(&u8g2, 48, 0, 47, 31, 3);
				u8g2_DrawRFrame(&u8g2, 96, 0, 47, 31, 3);
				u8g2_DrawRFrame(&u8g2, 144, 0, 48, 31, 3);
			} else
			if (PM_toggle == 1)		//+1 because PM_toggle is incremented at the end of the
			{						//VOC_SENSOR section of the FormatDisplayString function
				u8g2_ClearBuffer(&u8g2);
				//Resends string, so it will be displayed immediately
				u8g2_DrawStr(&u8g2, TextXPos, TextYPos, (char*)&usbVCOMout[0]);

				u8g2_SetFont(&u8g2, u8g2_font_5x8_mf);
			#if (DISPLAY_C6H6)
				u8g2_DrawStr(&u8g2, 6, 18, "(ug/mc)   (ug/mc)  (ug/mc)   (ug/mc)\r\n");
			#else
				u8g2_DrawStr(&u8g2, 6, 18, "(ug/mc)   (ug/mc)  (ug/mc)    (ppm)\r\n");
			#endif
				u8g2_SetFont(&u8g2, u8g2_font_t0_12b_tf);
			#if (DISPLAY_C6H6)
				u8g2_DrawStr(&u8g2, 9, 10, " CH20     O3     SO2     C6H6\r\n");
			#else
				u8g2_DrawStr(&u8g2, 9, 10, " CH20     O3     SO2     eCO2\r\n");
			#endif
				u8g2_DrawRFrame(&u8g2, 0, 0, 47, 31, 3);
				u8g2_DrawRFrame(&u8g2, 48, 0, 47, 31, 3);
				u8g2_DrawRFrame(&u8g2, 96, 0, 47, 31, 3);
				u8g2_DrawRFrame(&u8g2, 144, 0, 48, 31, 3);
			}
		#else		//FULL_MODE==0
			u8g2_ClearBuffer(&u8g2);
			//Resends string, so it will be displayed immediately
			u8g2_DrawStr(&u8g2, TextXPos, TextYPos, (char*)&usbVCOMout[0]);

			u8g2_SetFont(&u8g2, u8g2_font_5x8_mf);
			u8g2_DrawStr(&u8g2, 6, 18, " (ppb)     (ppm)   (ug/mc)   (mg/mc)\r\n");
			u8g2_SetFont(&u8g2, u8g2_font_t0_12b_tf);
			u8g2_DrawStr(&u8g2, 9, 10, "eTVOC    eCO2    CH20     CO\r\n");

			u8g2_DrawRFrame(&u8g2, 0, 0, 47, 31, 3);
			u8g2_DrawRFrame(&u8g2, 48, 0, 47, 31, 3);
			u8g2_DrawRFrame(&u8g2, 96, 0, 47, 31, 3);
			u8g2_DrawRFrame(&u8g2, 144, 0, 48, 31, 3);
		#endif		//FULL_MODE
	#else			//GAS_SENSOR_MODULE_PRESENT==0
			//Resends string, so it will be displayed immediately
			u8g2_DrawStr(&u8g2, TextXPos, TextYPos, (char*)&usbVCOMout[0]);

			u8g2_SetFont(&u8g2, u8g2_font_5x8_mf);
			u8g2_DrawStr(&u8g2, 59, 18, "(ppb)     (ppm)\r\n");

			u8g2_DrawRFrame(&u8g2, 44, 0, 51, 31, 3);
			u8g2_DrawRFrame(&u8g2, 96, 0, 51, 31, 3);
			u8g2_SetFont(&u8g2, u8g2_font_t0_12b_tf);
			u8g2_DrawStr(&u8g2, 56, 10, "eTVOC    eCO2\r\n");
	#endif			//GAS_SENSOR_MODULE_PRESENT
		}
		break;
		case 0x03:	//Air Pollution page
		{
	#if (SHOW_PM_NUMBER==1)
			if (PM_toggle == 2)		//+1 because PM_toggle is incremented at the end of the
			{						//PARTICULATE_SENSOR section of the FormatDisplayString function
				u8g2_ClearBuffer(&u8g2);
				//Resends string, so it will be displayed immediately
				u8g2_DrawStr(&u8g2, TextXPos, TextYPos, (char*)&usbVCOMout[0]);

				u8g2_SetFont(&u8g2, u8g2_font_5x8_mf);
				u8g2_DrawStr(&u8g2, 6, 18, "(ug/mc)   (ug/mc)  (ug/mc)   (ug/mc)\r\n");
				u8g2_SetFont(&u8g2, u8g2_font_t0_12b_tf);
				u8g2_DrawStr(&u8g2, 9, 10, "PM1.0   PM2.5   PM4.0    PM10\r\n");

				u8g2_DrawRFrame(&u8g2, 0, 0, 47, 31, 3);
				u8g2_DrawRFrame(&u8g2, 48, 0, 47, 31, 3);
				u8g2_DrawRFrame(&u8g2, 96, 0, 47, 31, 3);
				u8g2_DrawRFrame(&u8g2, 144, 0, 48, 31, 3);
			} else
			if (PM_toggle == 3)		//+1 because PM_toggle is incremented at the end of the
			{						//PARTICULATE_SENSOR section of the FormatDisplayString function
				u8g2_ClearBuffer(&u8g2);
				//Resends string, so it will be displayed immediately
				u8g2_DrawStr(&u8g2, TextXPos, TextYPos, (char*)&usbVCOMout[0]);

				u8g2_SetFont(&u8g2, u8g2_font_5x8_mf);
				u8g2_DrawStr(&u8g2, 6, 18, "(n/cmc)   (n/cmc)  (n/cmc)   (n/cmc)\r\n");
				u8g2_SetFont(&u8g2, u8g2_font_t0_12b_tf);
				u8g2_DrawStr(&u8g2, 9, 10, "PM1.0   PM2.5   PM4.0    PM10\r\n");

				u8g2_DrawRFrame(&u8g2, 0, 0, 47, 31, 3);
				u8g2_DrawRFrame(&u8g2, 48, 0, 47, 31, 3);
				u8g2_DrawRFrame(&u8g2, 96, 0, 47, 31, 3);
				u8g2_DrawRFrame(&u8g2, 144, 0, 48, 31, 3);
			} else
			if (PM_toggle == 1)		//+1 because PM_toggle is incremented at the end of the
			{						//PARTICULATE_SENSOR section of the FormatDisplayString function
				u8g2_ClearDisplay(&u8g2);
				//Resends string, so it will be displayed immediately
				u8g2_DrawStr(&u8g2, TextXPos, TextYPos, (char*)&usbVCOMout[0]);

				u8g2_SetFont(&u8g2, u8g2_font_5x8_mf);
				u8g2_DrawStr(&u8g2, 33, 18, "(n/cmc)        (um)\r\n");

				u8g2_DrawRFrame(&u8g2, 25, 0, 47, 31, 3);
				u8g2_DrawRFrame(&u8g2, 77, 0, 90, 31, 3);
				u8g2_SetFont(&u8g2, u8g2_font_t0_12b_tf);
				u8g2_DrawStr(&u8g2, 35, 10, "PM0.5   Tip.Part.Size\r\n");
			}
	#else	//SHOW_PM_NUMBER = 1
			u8g2_ClearBuffer(&u8g2);
			//Resends string, so it will be displayed immediately
			u8g2_DrawStr(&u8g2, TextXPos, TextYPos, (char*)&usbVCOMout[0]);

			u8g2_SetFont(&u8g2, u8g2_font_5x8_mf);
			u8g2_DrawStr(&u8g2, 6, 18, "(ug/mc)   (ug/mc)  (ug/mc)   (ug/mc)\r\n");
			u8g2_SetFont(&u8g2, u8g2_font_t0_12b_tf);
			u8g2_DrawStr(&u8g2, 9, 10, "PM1.0   PM2.5   PM4.0    PM10\r\n");

			u8g2_DrawRFrame(&u8g2, 0, 0, 47, 31, 3);
			u8g2_DrawRFrame(&u8g2, 48, 0, 47, 31, 3);
			u8g2_DrawRFrame(&u8g2, 96, 0, 47, 31, 3);
			u8g2_DrawRFrame(&u8g2, 144, 0, 48, 31, 3);
	#endif	//SHOW_PM_NUMBER = 0
		}
		break;
	}

	GLCD_Clear(false, true);
	PreviousPage = PageNumb;

	return LCD_OK;
}

LCD_Error_et ReDrawPage_S1(uint8_t PageNumb)
{
	Disp_Area = 1;
	static DateTime_t Stamp;
	static uint8_t len = 0;
	static AirQualityParameters_st AQ_Level;
#if (VOC_SENSOR_PRESENT==1)
	extern uint16_t eq_TVOC, eq_CO2, eq_TVOC_1h_Mean, eq_CO2_1h_Mean;
#endif
#if (PARTICULATE_SENSOR_PRESENT==1)
	extern uint16_t MC_10p0, MC_2p5;
	extern uint16_t MC_1p0_24h_Mean, MC_2p5_24h_Mean, MC_4p0_24h_Mean, MC_10p0_24h_Mean;
#endif
#if (GAS_SENSOR_MODULE_PRESENT==1)
	extern uint16_t CH2O, CO, CH2O_8h_Mean, CO_8h_Mean;
	#if (FULL_MODE==1)
		extern uint16_t NO2, NH3, O3, SO2, C6H6;
		extern uint16_t NO2_1h_Mean, NH3_8h_Mean, O3_1h_Mean, SO2_1h_Mean, C6H6_24h_Mean;
	#endif
#endif
	//Calculate the air quality index
#if (GAS_SENSOR_MODULE_PRESENT==1)
	#if (FULL_MODE==1)
	AQ_Level = AirQuality(eq_TVOC, eq_CO2, eq_TVOC_1h_Mean, eq_CO2_1h_Mean,
										   CH2O, CO, NO2, NH3, O3, SO2, C6H6, MC_10p0, MC_2p5,
										   CH2O_8h_Mean, CO_8h_Mean, NO2_1h_Mean, NH3_8h_Mean,
										   O3_1h_Mean, SO2_1h_Mean, C6H6_24h_Mean,
										   MC_10p0_24h_Mean, MC_2p5_24h_Mean);
	#else	//FULL_MODE==0
	AQ_Level = AirQuality(eq_TVOC, eq_CO2, eq_TVOC_1h_Mean, eq_CO2_1h_Mean,
										   CH2O, CO, 0, 0, 0, 0, 0, MC_10p0, MC_2p5,
										   CH2O_8h_Mean, CO_8h_Mean, 0, 0, 0, 0, 0,
										   MC_10p0_24h_Mean, MC_2p5_24h_Mean);
	#endif	//FULL_MODE
#else	//GAS_SENSOR_MODULE_PRESENT==0
	#if (PARTICULATE_SENSOR_PRESENT)
	AQ_Level = AirQuality(eq_TVOC, eq_CO2, eq_TVOC_1h_Mean, eq_CO2_1h_Mean,
										   0, 0, 0, 0, 0, 0, 0, MC_10p0, MC_2p5,
										   0, 0, 0, 0, 0, 0, 0,
										   MC_10p0_24h_Mean, MC_2p5_24h_Mean);
	#else
	AQ_Level = AirQuality(eq_TVOC, eq_CO2, eq_TVOC_1h_Mean, eq_CO2_1h_Mean,
										   0, 0, 0, 0, 0, 0, 0, 0, 0,
										   0, 0, 0, 0, 0, 0, 0,
										   0, 0);
	#endif
#endif //GAS_SENSOR_MODULE_PRESENT

	if (!(PageNumb == PreviousPage))
	{
		u8g2_ClearDisplay(&u8g2);
	}
	switch (PageNumb)
	{
		case 0x01:	//Environmental page
		{
			if ((PM_toggle == 2) || (PM_toggle == 1))	//+1 because PM_toggle is incremented at the end of the
			{											//PRESSURE_SENSOR section of the FormatDisplayString function
				RTC_DateTimeStamp(&hrtc, &Stamp);

				u8g2_DrawRFrame(&u8g2, 0, 0, 137, 11, 3);
				u8g2_SetFont(&u8g2, u8g2_font_6x10_mf);
//				u8g2_DrawStr(&u8g2, 10, 9, "AQI (1 hour average)");
				u8g2_DrawStr(&u8g2, 10, 9, " Air Quality Index ");

				u8g2_DrawRFrame(&u8g2, 0, 10, 68, 22, 3);
				u8g2_DrawRFrame(&u8g2, 69, 10, 68, 22, 3);

				u8g2_SetFont(&u8g2, u8g2_font_5x7_tf);
				u8g2_DrawStr(&u8g2, 10, 19, "GAS.POULL.");
				u8g2_SetFont(&u8g2, u8g2_font_5x8_tf);	//The AQI shown is always the lower between..
				if (AQ_Level.AvgGasAirQualityIndex <= AQ_Level.GasAirQualityIndex)
					u8g2_DrawStr(&u8g2, 2, 30, AQ_Level.AvgGasAirQualityClass);	//..the average...
				else
					u8g2_DrawStr(&u8g2, 2, 30, AQ_Level.GasAirQualityClass);	//...and the instantaneous one

				u8g2_SetFont(&u8g2, u8g2_font_5x7_tf);
				u8g2_DrawStr(&u8g2, 80, 19, "PMx POULL.");
				u8g2_SetFont(&u8g2, u8g2_font_5x8_tf);	//The AQI shown is always the lower between..
				if (AQ_Level.AvgPMxAirQualityIndex <= AQ_Level.PMxAirQualityIndex)
					u8g2_DrawStr(&u8g2, 71, 30, AQ_Level.AvgPMxAirQualityClass);	//..the average...
				else
					u8g2_DrawStr(&u8g2, 71, 30, AQ_Level.PMxAirQualityClass);	//...and the instantaneous one

				len = sprintf((char*)&usbVCOMout[0], "%02u.", Stamp.date[1]);
				len += sprintf((char*)&usbVCOMout[len], "%02u.", Stamp.date[0]);
				len += sprintf((char*)&usbVCOMout[len], "%04u ", Stamp.date[2]);
				usbVCOMout[6] = '2'; usbVCOMout[7] = '0';
				u8g2_SetFont(&u8g2, u8g2_font_helvR08_tf);
				u8g2_DrawStr(&u8g2, 139, 12, (char*)&usbVCOMout[0]);

				len = sprintf((char*)&usbVCOMout[0], "%02u:", Stamp.time[0]);
				len += sprintf((char*)&usbVCOMout[len], "%02u:", Stamp.time[1]);
				len += sprintf((char*)&usbVCOMout[len], "%02u", Stamp.time[2]);
				usbVCOMout[8] = 0x00; usbVCOMout[9] = 0x00;

				u8g2_SetFont(&u8g2, u8g2_font_helvR10_tf);
				u8g2_DrawStr(&u8g2, 138, 27, (char*)&usbVCOMout[0]);
			} else
//			if (PM_toggle == 1)		//+1 because PM_toggle is incremented at the end of the
			{						//PRESSURE_SENSOR section of the FormatDisplayString function
			}
		}
		break;
		case 0x02:	//Air Quality page
		{
	#if (GAS_SENSOR_MODULE_PRESENT==1)
		#if (FULL_MODE==1)
			if (PM_toggle == 2)		//+1 because PM_toggle is incremented at the end of the
			{						//VOC_SENSOR section of the FormatDisplayString function
				u8g2_SetFont(&u8g2, u8g2_font_5x8_mf);
				u8g2_DrawStr(&u8g2, 6, 18, " (ppb)    (mg/mc)  (ug/mc)   (mg/mc)\r\n");
				u8g2_SetFont(&u8g2, u8g2_font_t0_11_mf);
				sprintf((char*)&etvoc_v_1h_Mean[0],  "%u", eq_TVOC_1h_Mean);
				u8g2_DrawStr(&u8g2, 2, 30, (char*)&etvoc_v_1h_Mean[0]);
				sprintf((char*)&co_v_8h_Mean[0],  "%u", CO_8h_Mean);
				u8g2_DrawStr(&u8g2, 50, 30, (char*)&co_v_8h_Mean[0]);
				sprintf((char*)&no2_v_1h_Mean[0],  "%u", NO2_1h_Mean);
				u8g2_DrawStr(&u8g2, 98, 30, (char*)&no2_v_1h_Mean[0]);
				sprintf((char*)&nh3_v_8h_Mean[0],  "%u", NH3_8h_Mean);
				u8g2_DrawStr(&u8g2, 146, 30, (char*)&nh3_v_8h_Mean[0]);

				u8g2_SetFont(&u8g2, u8g2_font_5x8_mf);
				u8g2_DrawStr(&u8g2, 36, 30, "2k");
				u8g2_DrawStr(&u8g2, 84, 30, "10");
				u8g2_DrawStr(&u8g2, 127, 30, "200");
				u8g2_DrawStr(&u8g2, 176, 30, "1.8");

				u8g2_DrawRFrame(&u8g2, 0, 0, 47, 12, 3);
				u8g2_DrawRFrame(&u8g2, 48, 0, 47, 12, 3);
				u8g2_DrawRFrame(&u8g2, 96, 0, 47, 12, 3);
				u8g2_DrawRFrame(&u8g2, 144, 0, 48, 12, 3);

				u8g2_SetFont(&u8g2, u8g2_font_6x10_mf);
				u8g2_DrawStr(&u8g2, 6, 10, "1h AVG\r\n");
				u8g2_DrawStr(&u8g2, 54, 10, "8h AVG\r\n");
				u8g2_DrawStr(&u8g2, 102, 10, "1h AVG\r\n");
				u8g2_DrawStr(&u8g2, 150, 10, "8h AVG\r\n");

				u8g2_DrawRFrame(&u8g2, 0, 11, 47, 21, 3);
				u8g2_DrawRFrame(&u8g2, 48, 11, 47, 21, 3);
				u8g2_DrawRFrame(&u8g2, 96, 11, 47, 21, 3);
				u8g2_DrawRFrame(&u8g2, 144, 11, 48, 21, 3);
			}
			if (PM_toggle == 1)		//+1 because PM_toggle is incremented at the end of the
			{						//VOC_SENSOR section of the FormatDisplayString function
				u8g2_SetFont(&u8g2, u8g2_font_5x8_mf);
			#if (DISPLAY_C6H6)
				u8g2_DrawStr(&u8g2, 6, 18, "(ug/mc)   (ug/mc)  (ug/mc)   (ug/mc)\r\n");
			#else
				u8g2_DrawStr(&u8g2, 6, 18, "(ug/mc)   (ug/mc)  (ug/mc)    (ppm)\r\n");
			#endif
				u8g2_SetFont(&u8g2, u8g2_font_t0_11_mf);
				sprintf((char*)&ch2o_v_8h_Mean[0],  "%u", CH2O_8h_Mean);
				u8g2_DrawStr(&u8g2, 2, 30, (char*)&ch2o_v_8h_Mean[0]);
				sprintf((char*)&o3v_8h_Mean[0],  "%u", O3_1h_Mean);
				u8g2_DrawStr(&u8g2, 50, 30, (char*)&o3v_8h_Mean[0]);
				sprintf((char*)&so2_v_24h_Mean[0],  "%u", SO2_1h_Mean);
				u8g2_DrawStr(&u8g2, 98, 30, (char*)&so2_v_24h_Mean[0]);
			#if (DISPLAY_C6H6)
				sprintf((char*)&c6h6_v_24h_Mean[0],  "%u", C6H6_24h_Mean);
				u8g2_DrawStr(&u8g2, 146, 30, (char*)&c6h6_v_24h_Mean[0]);

				u8g2_SetFont(&u8g2, u8g2_font_5x8_mf);
				u8g2_DrawStr(&u8g2, 31, 30, "120");
				u8g2_DrawStr(&u8g2, 79, 30, "180");
				u8g2_DrawStr(&u8g2, 127, 30, "350");
				u8g2_DrawStr(&u8g2, 185, 30, "2");
			#else
				sprintf((char*)&eco2_v_1h_Mean[0],  "%u", eq_CO2_1h_Mean);
				u8g2_DrawStr(&u8g2, 146, 30, (char*)&eco2_v_1h_Mean[0]);

				u8g2_SetFont(&u8g2, u8g2_font_5x8_mf);
				u8g2_DrawStr(&u8g2, 31, 30, "120");
				u8g2_DrawStr(&u8g2, 79, 30, "180");
				u8g2_DrawStr(&u8g2, 127, 30, "350");
				u8g2_DrawStr(&u8g2, 180, 30, "5k");
			#endif

				u8g2_DrawRFrame(&u8g2, 0, 0, 47, 12, 3);
				u8g2_DrawRFrame(&u8g2, 48, 0, 47, 12, 3);
				u8g2_DrawRFrame(&u8g2, 96, 0, 47, 12, 3);
				u8g2_DrawRFrame(&u8g2, 144, 0, 48, 12, 3);

				u8g2_SetFont(&u8g2, u8g2_font_6x10_mf);
				u8g2_DrawStr(&u8g2, 6, 10, "8h AVG\r\n");
				u8g2_DrawStr(&u8g2, 54, 10, "1h AVG\r\n");
				u8g2_DrawStr(&u8g2, 102, 10, "1h AVG\r\n");
			#if (DISPLAY_C6H6)
				u8g2_DrawStr(&u8g2, 148, 10, "24h AVG\r\n");
			#else
				u8g2_DrawStr(&u8g2, 150, 10, "1h AVG\r\n");
			#endif
				u8g2_DrawRFrame(&u8g2, 0, 11, 47, 21, 3);
				u8g2_DrawRFrame(&u8g2, 48, 11, 47, 21, 3);
				u8g2_DrawRFrame(&u8g2, 96, 11, 47, 21, 3);
				u8g2_DrawRFrame(&u8g2, 144, 11, 48, 21, 3);
			}
		#else		//FULL_MODE==0
//			if (PM_toggle == 2)		//+1 because PM_toggle is incremented at the end of the
//			{						//VOC_SENSOR section of the FormatDisplayString function
				u8g2_SetFont(&u8g2, u8g2_font_5x8_mf);
				u8g2_DrawStr(&u8g2, 6, 18, " (ppb)     (ppm)   (ug/mc)   (mg/mc)\r\n");
				u8g2_SetFont(&u8g2, u8g2_font_t0_11_mf);
				sprintf((char*)&etvoc_v_1h_Mean[0],  "%u", eq_TVOC_1h_Mean);
				u8g2_DrawStr(&u8g2, 2, 30, (char*)&etvoc_v_1h_Mean[0]);
				sprintf((char*)&eco2_v_1h_Mean[0],  "%u", eq_CO2_1h_Mean);
				u8g2_DrawStr(&u8g2, 50, 30, (char*)&eco2_v_1h_Mean[0]);
				sprintf((char*)&ch2o_v_8h_Mean[0],  "%u", CH2O_8h_Mean);
				u8g2_DrawStr(&u8g2, 98, 30, (char*)&ch2o_v_8h_Mean[0]);
				sprintf((char*)&co_v_8h_Mean[0],  "%u", CO_8h_Mean);
				u8g2_DrawStr(&u8g2, 146, 30, (char*)&co_v_8h_Mean[0]);

				u8g2_SetFont(&u8g2, u8g2_font_5x8_mf);
				u8g2_DrawStr(&u8g2, 36, 30, "2k");
				u8g2_DrawStr(&u8g2, 84, 30, "5k");
				u8g2_DrawStr(&u8g2, 127, 30, "120");
				u8g2_DrawStr(&u8g2, 180, 30, "10");

				u8g2_DrawRFrame(&u8g2, 0, 0, 192, 12, 3);
/*				u8g2_DrawRFrame(&u8g2, 0, 0, 47, 12, 3);
				u8g2_DrawRFrame(&u8g2, 48, 0, 47, 12, 3);
				u8g2_DrawRFrame(&u8g2, 96, 0, 47, 12, 3);
				u8g2_DrawRFrame(&u8g2, 144, 0, 48, 12, 3); */

				u8g2_SetFont(&u8g2, u8g2_font_6x10_mf);
				u8g2_DrawStr(&u8g2, 31, 10, "Mean values in 8 hours\r\n");
/*				u8g2_DrawStr(&u8g2, 6, 10, "8h AVG\r\n");
				u8g2_DrawStr(&u8g2, 54, 10, "8h AVG\r\n");
				u8g2_DrawStr(&u8g2, 102, 10, "8h AVG\r\n");
				u8g2_DrawStr(&u8g2, 148, 10, "8h AVG\r\n"); */

				u8g2_DrawRFrame(&u8g2, 0, 11, 47, 21, 3);
				u8g2_DrawRFrame(&u8g2, 48, 11, 47, 21, 3);
				u8g2_DrawRFrame(&u8g2, 96, 11, 47, 21, 3);
				u8g2_DrawRFrame(&u8g2, 144, 11, 48, 21, 3);
//			}
		#endif		//FULL_MODE
	#else			//GAS_SENSOR_MODULE_PRESENT==0
			u8g2_SetFont(&u8g2, u8g2_font_5x8_tf);
			if (AQ_Level.AvgGasAirQualityIndex <= AQ_Level.GasAirQualityIndex)	//The AQI shown is always the lower between..
				u8g2_DrawStr(&u8g2, 65, 12, AQ_Level.AvgGasAirQualityClass);		//..the average...
			else
				u8g2_DrawStr(&u8g2, 65, 12, AQ_Level.GasAirQualityClass);		//...and the instantaneous one
			u8g2_SetFont(&u8g2, u8g2_font_unifont_t_emoticons);
			switch (AVG_Gas_AQI)
			{
				case 0x00:			//Exellent
				{
//					u8g2_SetFont(&u8g2, u8g2_font_t0_12_mf);
//					u8g2_DrawStr(&u8g2, 2, 10, "Air Quality Index 1: Excellent!\r\n");
					u8g2_SetFont(&u8g2, u8g2_font_unifont_t_emoticons);
					u8g2_DrawGlyph(&u8g2, 89, 28, 0x0023);		//Smiley 1
				}
				break;
				case 0x01:			//Good
				{
//					u8g2_SetFont(&u8g2, u8g2_font_t0_12_mf);
//					u8g2_DrawStr(&u8g2, 2, 10, "   Air Quality Index 2: Good\r\n");
					u8g2_SetFont(&u8g2, u8g2_font_unifont_t_emoticons);
					u8g2_DrawGlyph(&u8g2, 89, 28, 0x0062);		//Smiley 2
				}
				break;
				case 0x02:			//Moderate
				{
//					u8g2_SetFont(&u8g2, u8g2_font_t0_12_mf);
//					u8g2_DrawStr(&u8g2, 2, 10, "  Air Quality Index 3: Medium\r\n");
					u8g2_SetFont(&u8g2, u8g2_font_unifont_t_emoticons);
					u8g2_DrawGlyph(&u8g2, 89, 28, 0x0030);		//Straight face
				}
				break;
				case 0x03:			//Poor
				{
//					u8g2_SetFont(&u8g2, u8g2_font_t0_12_mf);
//					u8g2_DrawStr(&u8g2, 2, 10, "   Air Quality Index 4: Poor\r\n");
					u8g2_SetFont(&u8g2, u8g2_font_unifont_t_emoticons);
					u8g2_DrawGlyph(&u8g2, 89, 28, 0x0040);		//Angry face
				}
				break;
				case 0x04:			//Unhealthy
				{
//					u8g2_SetFont(&u8g2, u8g2_font_t0_12_mf);
//					u8g2_DrawStr(&u8g2, 2, 10, "Air Quality Index 5: Unhealthy!\r\n");
					u8g2_SetFont(&u8g2, u8g2_font_unifont_t_emoticons);
					u8g2_DrawGlyph(&u8g2, 89, 28, 0x0043);		//Frown face
				}
				break;
				case 0x05:			//Dangerous
				{
//					u8g2_SetFont(&u8g2, u8g2_font_t0_12_mf);
//					u8g2_DrawStr(&u8g2, 2, 10, "Air Quality Index 6: Dangerous!\r\n");
					u8g2_SetFont(&u8g2, u8g2_font_unifont_t_emoticons);
					u8g2_DrawGlyph(&u8g2, 89, 28, 0x0044);		//Horror face
				}
				break;
			}
	#endif		//GAS_SENSOR_MODULE_PRESENT
		}
		break;
		case 0x03:	//Air Pollution page
		{
	#if(PARTICULATE_SENSOR_PRESENT)
			if (PM_toggle == 2)		//+1 because PM_toggle is incremented at the end of the
			{						//PARTICULATE_SENSOR section of the FormatDisplayString function
				u8g2_SetFont(&u8g2, u8g2_font_5x8_mf);
				u8g2_DrawStr(&u8g2, 6, 19, "(ug/mc)   (ug/mc)  (ug/mc)   (ug/mc)\r\n");
				u8g2_SetFont(&u8g2, u8g2_font_t0_11_mf);
				sprintf((char*)&mc_1p0_v_24h_Mean[0],  "%u", MC_1p0_24h_Mean);
				u8g2_DrawStr(&u8g2, 7, 30, (char*)&mc_1p0_v_24h_Mean[0]);
				sprintf((char*)&mc_2p5_v_24h_Mean[0],  "%u", MC_2p5_24h_Mean);
				u8g2_DrawStr(&u8g2, 55, 30, (char*)&mc_2p5_v_24h_Mean[0]);
				sprintf((char*)&mc_4p0_v_24h_Mean[0],  "%u", MC_4p0_24h_Mean);
				u8g2_DrawStr(&u8g2, 103, 30, (char*)&mc_4p0_v_24h_Mean[0]);
				sprintf((char*)&mc_10p0_v_24h_Mean[0],  "%u", MC_10p0_24h_Mean);
				u8g2_DrawStr(&u8g2, 151, 30, (char*)&mc_10p0_v_24h_Mean[0]);
			} else
			if (PM_toggle == 3)		//+1 because PM_toggle is incremented at the end of the
			{						//PARTICULATE_SENSOR section of the FormatDisplayString function
				u8g2_SetFont(&u8g2, u8g2_font_5x8_mf);
				u8g2_DrawStr(&u8g2, 6, 19, "(ug/mc)   (ug/mc)  (ug/mc)   (ug/mc)\r\n");
				u8g2_SetFont(&u8g2, u8g2_font_t0_11_mf);
				sprintf((char*)&mc_1p0_v_24h_Mean[0],  "%u", MC_1p0_24h_Mean);
				u8g2_DrawStr(&u8g2, 7, 30, (char*)&mc_1p0_v_24h_Mean[0]);
				sprintf((char*)&mc_2p5_v_24h_Mean[0],  "%u", MC_2p5_24h_Mean);
				u8g2_DrawStr(&u8g2, 55, 30, (char*)&mc_2p5_v_24h_Mean[0]);
				sprintf((char*)&mc_4p0_v_24h_Mean[0],  "%u", MC_4p0_24h_Mean);
				u8g2_DrawStr(&u8g2, 103, 30, (char*)&mc_4p0_v_24h_Mean[0]);
				sprintf((char*)&mc_10p0_v_24h_Mean[0],  "%u", MC_10p0_24h_Mean);
				u8g2_DrawStr(&u8g2, 151, 30, (char*)&mc_10p0_v_24h_Mean[0]);
			} else
			if (PM_toggle == 1)		//+1 because PM_toggle is incremented at the end of the
			{						//PARTICULATE_SENSOR section of the FormatDisplayString function
				u8g2_SetFont(&u8g2, u8g2_font_5x8_mf);
				u8g2_DrawStr(&u8g2, 6, 19, "(ug/mc)   (ug/mc)  (ug/mc)   (ug/mc)\r\n");
				u8g2_SetFont(&u8g2, u8g2_font_t0_11_mf);
				sprintf((char*)&mc_1p0_v_24h_Mean[0],  "%u", MC_1p0_24h_Mean);
				u8g2_DrawStr(&u8g2, 7, 30, (char*)&mc_1p0_v_24h_Mean[0]);
				sprintf((char*)&mc_2p5_v_24h_Mean[0],  "%u", MC_2p5_24h_Mean);
				u8g2_DrawStr(&u8g2, 55, 30, (char*)&mc_2p5_v_24h_Mean[0]);
				sprintf((char*)&mc_4p0_v_24h_Mean[0],  "%u", MC_4p0_24h_Mean);
				u8g2_DrawStr(&u8g2, 103, 30, (char*)&mc_4p0_v_24h_Mean[0]);
				sprintf((char*)&mc_10p0_v_24h_Mean[0],  "%u", MC_10p0_24h_Mean);
				u8g2_DrawStr(&u8g2, 151, 30, (char*)&mc_10p0_v_24h_Mean[0]);
			}
			u8g2_SetFont(&u8g2, u8g2_font_5x8_mf);
			u8g2_DrawStr(&u8g2, 36, 30, "--");
			u8g2_DrawStr(&u8g2, 84, 30, "25");
			u8g2_DrawStr(&u8g2, 132, 30, "--");
			u8g2_DrawStr(&u8g2, 180, 30, "50");

			u8g2_DrawRFrame(&u8g2, 0, 0, 192, 12, 3);
//			u8g2_SetFont(&u8g2, u8g2_font_t0_11_mf);
			u8g2_SetFont(&u8g2, u8g2_font_6x10_mf);
			u8g2_DrawStr(&u8g2, 25, 10, "Mean values in 24 hours\r\n");

			u8g2_DrawRFrame(&u8g2, 0, 11, 47, 21, 3);
			u8g2_DrawRFrame(&u8g2, 48, 11, 47, 21, 3);
			u8g2_DrawRFrame(&u8g2, 96, 11, 47, 21, 3);
			u8g2_DrawRFrame(&u8g2, 144, 11, 48, 21, 3);
	#endif	//PARTICULATE_SENSOR_PRESENT
		}
		break;
	}

	GLCD_Clear(false, true);
	PreviousPage = PageNumb;
	Disp_Area = 0;

	return LCD_OK;
}

LCD_Error_et SendWelcomeMessage()
{
	extern uint8_t Version[];
	char MyVersion[40];

	memcpy(&MyVersion[0], &Version[2], 36);
	MyVersion[36] = 0x0D; MyVersion[37] = 0x0A; MyVersion[38] = 0x00;

	u8g2_SetFont(&u8g2, u8g2_font_t0_12_mf);
	u8g2_SetFontMode(&u8g2, 0);

	Disp_Area = 1;
	u8g2_ClearDisplay(&u8g2);
    send_glcdmessage(WELCOME_STRING2, 1, 12, 1, strlen(WELCOME_STRING2), false, true);

	Disp_Area = 0;
	u8g2_SetFont(&u8g2, u8g2_font_t0_14_mf);
	u8g2_ClearDisplay(&u8g2);
    send_glcdmessage(WELCOME_STRING1, 10, 15, 2, 26, false, true);
    u8g2_SetFont(&u8g2, u8g2_font_5x7_tf);
    send_glcdmessage(MyVersion, 6, 25, 2, 26, true, true);

    return LCD_OK;
}

LCD_Error_et SendReadyDevicesMessage()
{
	u8g2_SetFont(&u8g2, u8g2_font_5x7_tf);
	send_glcdmessage(ReadyDevices, 0, 6, 2, strlen(ReadyDevices), true, true);

	return LCD_OK;
}

LCD_Error_et MX_GLCD_Init()
{
	ClearBuff = false; NextPage = true; PM_toggle = 1; /*NumberOfPages = 1;*/ Disp_Area = 0;
//N.B.!! Comment "NumberOfPages = 1" if it has already been set previously
	u8g2_Setup_st7920_s_192x32_f(&u8g2, U8G2_R0, u8x8_byte_4wire_hw_spi, u8x8_stm32_gpio_and_delay);
//	u8g2_Setup_ssd1306_128x64_noname_1(&u8g2, U8G2_R0, u8x8_byte_4wire_hw_spi, u8x8_stm32_gpio_and_delay);
	u8g2_InitDisplay(&u8g2);
	u8g2_SetPowerSave(&u8g2, 0);

	u8g2_FirstPage(&u8g2);
	u8g2_ClearBuffer(&u8g2);
	u8g2_SetFont(&u8g2, u8g2_font_t0_14_mf);
	u8g2_SetFontMode(&u8g2, 0);

	return LCD_OK;
}

#endif

/**
 * Returns a string "str" centered in string of a length width "new_length".
 * Padding is done using the specified fill character "placeholder".
 */
char *str_center(char str[], unsigned int new_length, char placeholder)
{
    size_t str_length = strlen(str);

    // if a new length is less or equal length of the original string, returns the original string
    if (new_length <= str_length)
        return str;

    char *buffer;
    unsigned int i, total_rest_length;

    buffer = malloc(sizeof(char) * new_length + 1);

    // length of a wrapper of the original string
    total_rest_length = new_length - str_length;

    // write a prefix to buffer
    i = 0;
    while (i < (total_rest_length / 2))
    {
        buffer[i] = placeholder;
        ++i;
    }
//	buffer[i+1] = '\0';		//Original
    buffer[i] = '\0';

    // write the original string
    strcat(buffer, str);

    // write a postfix to the buffer
    i += str_length;
    while (i < new_length)
    {
        buffer[i] = placeholder;
        ++i;
    }
//	buffer[i+1] = '\0';		//Original
    buffer[i] = '\0';

    free(buffer);

    return buffer;
}

void FillAirQualityField(char* field, uint16_t value, uint8_t ovfl_check)
{
	if (ovfl_check)
	{
		strcpy(field, "OVFL");
	} else
	if(value)
		sprintf((char*)&field[0], "%u", value);
	else
		strcpy(field, "< 1");
}
void FormatDisplayString(int *len, uint8_t* Buff, SENSOR_TYPE stype)
{
#if ((TLCD_SUPPORT==1) || (GLCD_SUPPORT==1))
	uint8_t Sh = 0;
	#if (PRESSURE_SENSOR_PRESENT==1)
		extern double_t Temperature;
		extern float32_t press_value;
		#if(CALC_ALTITUDE==1)
			extern double_t Altitude;
		#endif
	#endif
	#if (HUMIDITY_SENSOR_PRESENT==1)
		extern uint8_t	Humidity;
		#if(CALC_DEWPOINT==1)
			extern float TemperatureD;
		#endif
		#if (DISPLAY_SIMMER_INDEX==1)
			extern float SI;
		#elif (DISPLAY_HEAT_INDEX==1)
			extern float HI;
		#endif
	#endif
	#if (UVx_SENSOR_PRESENT==1)
		#if (VEML6075)
			extern float32_t UVa, UVb, UV_Index;
			if (UV_Index < 0.5)
			{
				UVx_Display = false;			//Not display UVx page if UV-Index < 0.5
			} else
			{
				UVx_Display = true;
				ClearDisplay = true;
			}
		#elif (LTR390UV)
			extern float32_t Lux, UV_Index;
			UVx_Display = true;
			ClearDisplay = true;
		#endif	//VEML6075
	#endif	// UVx_SENSOR_PRESENT
	#if (VOC_SENSOR_PRESENT==1)
		extern uint16_t eq_TVOC, eq_CO2;
	#endif
	#if (PARTICULATE_SENSOR_PRESENT==1)
		extern uint16_t MC_1p0, MC_2p5, MC_4p0, MC_10p0, NC_0p5, NC_1p0, NC_2p5, NC_4p0, NC_10p0;
		extern float TypicalParticleSize;
	#endif
	#if (GAS_SENSOR_MODULE_PRESENT==1)
		extern uint16_t CH2O, CO;
		#if (FULL_MODE==1)
			extern uint16_t NO2, NH3, C6H6, O3, SO2;
		#endif
	#endif

	switch (stype)
	{
		case IMU:
		{
		}
		break;

		case PRESSURE_SENSOR:
		{
	#if (PRESSURE_SENSOR_PRESENT==1)
		#if (TLCD_SUPPORT==1)
			if (t_flip == 1)	//t_flip identifies the display page number: Page 1 -> Environmental Page
			{
				if ((SensorStatusReg) & (PRESSURE_SENSOR_OK))
				{
			#if (UVx_SENSOR_PRESENT)
					if (!UVx_Display)
			#endif
						PM_toggle = 1;			//Not display UVx page when UV-Index < 0.5
					if (PM_toggle == 1)
					{
						//Format first TLCD row
						*len = sprintf((char*)&Buff[0], "T.: %4.1f %cC  ", Temperature, 0xDF);
						*len += sprintf((char*)&Buff[*len], "RH: %2u%%    \r\n", Humidity);
						//Format second TLCD row
						*len += sprintf((char*)&Buff[*len], "Pr.:%4.1fmb ", Pressure);
					#if (CALC_ALTITUDE==1)
						*len += sprintf((char*)&Buff[*len], "Alt: %4.1fm\r\n", Altitude);
					#elif (CALC_DEWPOINT==1)
						*len += sprintf((char*)&Buff[*len], "Td: %4.1f %cC\r\n", TemperatureD, 0xDF);
					#endif
					}
				#if (UVx_SENSOR_PRESENT==1)
					else
					if (PM_toggle == 2)
					{
						//Format first TLCD row
						*len = sprintf((char*)&Buff[0], "UVA: %.1f  ", UVa);
						*len += sprintf((char*)&Buff[*len], "UVB: %.1f    \r\n", UVb);
						//Format second TLCD row
						*len += sprintf((char*)&Buff[*len], "UV Index: %.3f          \r\n", UV_Index);
					}
				#endif
				} else
				{
					if (PM_toggle == 1)
					{
						//Format first TLCD row
						*len = sprintf((char*)&Buff[0], "T.: FAIL    ");
						*len += sprintf((char*)&Buff[*len], "RH: %2u%%    \r\n", Humidity);
						//Format second TLCD row
						*len += sprintf((char*)&Buff[*len], "Pr.: FAIL   ");
					#if (CALC_ALTITUDE==1)
						*len += sprintf((char*)&Buff[*len], "Alt: FAIL\r\n");
					#elif (CALC_DEWPOINT==1)
						*len += sprintf((char*)&Buff[*len], "Td: FAIL\r\n");
					#endif
					}
				#if (UVx_SENSOR_PRESENT==1)
					else
					if (PM_toggle == 2)
					{
						//Format first TLCD row
						*len = sprintf((char*)&Buff[0], "UVA: %.1f ", UVa);
						*len += sprintf((char*)&Buff[*len], "UVB: %.1f    \r\n", UVb);
						//Format second TLCD row
						*len += sprintf((char*)&Buff[*len], "UV Index: %.3f          \r\n", UV_Index);
					}
				#endif
				}
				if (++PM_toggle > NumENV_Pages)
				{
					PM_toggle = 1;
				}
			}
		#elif (GLCD_SUPPORT==1)
			if (t_flip == 1)	//t_flip identifies the display page number: Page 1 -> Environmental Page
			{
				memset((void *)&Buff[0], 0, 32);

				u8g2_SetFont(&u8g2, u8g2_font_t0_12_mf);
				u8g2_SetFontMode(&u8g2, 0);
			#if (UVx_SENSOR_PRESENT)
				if (!UVx_Display)
			#endif
					PM_toggle = 1;			//Not display UVx page when UV-Index < 0.5
				if (((SensorStatusReg) & (PRESSURE_SENSOR_OK)) || ((SensorStatusReg) & (HUMIDITY_SENSOR_OK)))
				{
					if (PM_toggle == 1)
					{
						TextXPos = 5; TextYPos = 27; Sh = 0;

						sprintf((char*)&temp_v[0],  "%.1f%cC", Temperature, 0xB0);
						ptr = str_center(temp_v, TEMP_FRAME_W, ENV_FRAME_FILL_CHAR);
						memcpy((void *)&Buff[0], ptr, TEMP_FRAME_W);
				#if (HUMIDITY_SENSOR_PRESENT==1)
						sprintf((char*)&hum_v[0],  "%2u%%", Humidity);
						ptr = str_center(hum_v, HUM_FRAME_W, ENV_FRAME_FILL_CHAR);
						memcpy((void *)&Buff[TEMP_FRAME_W], ptr, HUM_FRAME_W);
				#else
						memcpy((void *)&Buff[TEMP_FRAME_W], &sensor_fail1c[0], 7);
						memcpy((void *)&Buff[TEMP_FRAME_W + HUM_FRAME_W + PRESS_FRAME_W], &sensor_fail[0], 4);
						memcpy((void *)&Buff[TEMP_FRAME_W + HUM_FRAME_W + PRESS_FRAME_W + (TEMPd_FRAME_W+Sh)], &sensor_fail[0], 4);
				#endif
						sprintf((char*)&press_v[0],  "%.1fmb", press_value);
						ptr = str_center(press_v, PRESS_FRAME_W, ENV_FRAME_FILL_CHAR);
						memcpy((void *)&Buff[TEMP_FRAME_W + HUM_FRAME_W], ptr, PRESS_FRAME_W);
				#if (CALC_ALTITUDE==1)
						sprintf((char*)&alt_v[0],  "%.1fm", Altitude);
						ptr = str_center(alt_v, ALT_FRAME_W, ENV_FRAME_FILL_CHAR);
						memcpy((void *)&Buff[TEMP_FRAME_W + HUM_FRAME_W + PRESS_FRAME_W], ptr, ALT_FRAME_W);

						sprintf((char*)&tempp_v[0],  "%2u", (uint8_t)lrint(Temperature));
						ptr = str_center(tempp_v, TEMPp_FRAME_W, ENV_FRAME_FILL_CHAR);
						memcpy((void *)&Buff[TEMP_FRAME_W + HUM_FRAME_W + PRESS_FRAME_W + TEMPd_FRAME_W], ptr, TEMPp_FRAME_W);
				#elif ((CALC_DEWPOINT==1) && (HUMIDITY_SENSOR_PRESENT==1))
						sprintf((char*)&tempd_v[0],  "%.1f", TemperatureD);
						ptr = str_center(tempd_v, TEMPd_FRAME_W, ENV_FRAME_FILL_CHAR);
						memcpy((void *)&Buff[TEMP_FRAME_W + HUM_FRAME_W + PRESS_FRAME_W], ptr, TEMPd_FRAME_W);
					#if (DISPLAY_SIMMER_INDEX==1)
						sprintf((char*)&tempp_v[0],  "%2u", (uint8_t)lrintf(SI));
						if ((uint8_t)lrintf(SI) > 9)
						{	//This crap is just to improve the Apparent Temperature display (SI or HI) (!!!)
							Buff[TEMP_FRAME_W + HUM_FRAME_W + PRESS_FRAME_W + TEMPd_FRAME_W] = 0x20;
							Sh = 1;
						}
					#elif (DISPLAY_HEAT_INDEX==1)
						sprintf((char*)&tempp_v[0],  "%2u", (uint8_t)lrintf(HI));
						if ((uint8_t)lrintf(HI) > 9)
						{	//This crap is just to improve the Apparent Temperature display (SI or HI) (!!!)
							Buff[TEMP_FRAME_W + HUM_FRAME_W + PRESS_FRAME_W + TEMPd_FRAME_W] = 0x20;
							Sh = 1;
						}
					#endif
						ptr = str_center(tempp_v, TEMPp_FRAME_W, ENV_FRAME_FILL_CHAR);
						memcpy((void *)&Buff[TEMP_FRAME_W + HUM_FRAME_W + PRESS_FRAME_W + (TEMPd_FRAME_W+Sh)], ptr, TEMPp_FRAME_W);

				#endif
				#if (CALC_ALTITUDE==1)
						*len = TEMP_FRAME_W + HUM_FRAME_W + PRESS_FRAME_W + ALT_FRAME_W + TEMPp_FRAME_W;
				#else
						*len = TEMP_FRAME_W + HUM_FRAME_W + PRESS_FRAME_W + TEMPd_FRAME_W + TEMPp_FRAME_W + Sh;
				#endif
//						*len = sprintf((char*)&Buff[0], "ABCDEFGHILMNOPQRSTUVZ0123456789");
					}
				#if ((UVx_SENSOR_PRESENT) || (ALS_SENSOR_PRESENT))
					else
					if (PM_toggle == 2)
					{
						TextXPos = 3; TextYPos = 27;
					#if (VEML6075)
						sprintf((char*)&uva_v[0], "%.1f", UVa);
						ptr = str_center(uva_v, UVA_FRAME_W, UVx_FRAME_FILL_CHAR);
						memcpy((void *)&Buff[0], ptr, UVA_FRAME_W);

						sprintf((char*)&uvb_v[0], "%.1f", UVb);
						ptr = str_center(uvb_v, UVB_FRAME_W, UVx_FRAME_FILL_CHAR);
						memcpy((void *)&Buff[UVA_FRAME_W], ptr, UVB_FRAME_W);

						sprintf((char*)&uvi_v[0], "%.3f", UV_Index);
						ptr = str_center(uvi_v, UVIDX_FRAME_W, UVx_FRAME_FILL_CHAR);
						memcpy((void *)&Buff[UVA_FRAME_W + UVB_FRAME_W], ptr, UVIDX_FRAME_W);

						*len = UVA_FRAME_W + UVB_FRAME_W + UVIDX_FRAME_W;
					#elif (LTR390UV)
						sprintf((char*)&uva_v[0], "%.1f lx", Lux);
						ptr = str_center(uva_v, UVA_FRAME_W, UVx_FRAME_FILL_CHAR);
						memcpy((void *)&Buff[0], ptr, UVA_FRAME_W);

						sprintf((char*)&uvi_v[0], "%.2f", UV_Index);
						ptr = str_center(uvi_v, UVIDX_FRAME_W+1, UVx_FRAME_FILL_CHAR);
						memcpy((void *)&Buff[UVA_FRAME_W], ptr, UVIDX_FRAME_W);

						*len = UVA_FRAME_W + UVIDX_FRAME_W;
					#endif // VEML6075
					}
				#endif //(UVx_SENSOR_PRESENT) || (ALS_SENSOR_PRESENT)
				} else
				{
					TextXPos = 11; TextYPos = 27;
					*len += sprintf((char*)&Buff[0], " FAIL           FAIL        \r\n");
				}
				if (++PM_toggle > NumENV_Pages)
				{
					PM_toggle = 1;
				}
				NextPage = true;
				ClearBuff = false;
				SLen = *len;
			}
		#endif
	#endif
		}
		break;

		case HUMIDITY_SENSOR:
		{
	#if (HUMIDITY_SENSOR_PRESENT==1)
		#if (TLCD_SUPPORT==1)
			if ((t_flip == 1) & (PM_toggle==2))	//t_flip identifies the display page number: Page 1 -> Environmental Page
			{												//PM_Toggle+1 because PM_toggle is incremented at the end of the
				if (!((SensorStatusReg) & (HUMIDITY_SENSOR_OK)))	//TVOC section of the FormatDisplayString function
				{
					memcpy((void *)&Buff[17], &sensor_fail[0], 4);
				}
			}
		#elif (GLCD_SUPPORT==1)
			if ((t_flip == 1) & (PM_toggle==2))				//t_flip identifies the display page number: Page 1 -> Environmental Page
			{												//PM_Toggle+1 because PM_toggle is incremented at the end of the
				u8g2_SetFont(&u8g2, u8g2_font_t0_12_mf);	//TVOC section of the FormatDisplayString function
				u8g2_SetFontMode(&u8g2, 0);
				TextXPos = 5; TextYPos = 27;

				if (((SensorStatusReg) & (HUMIDITY_SENSOR_OK)) && !((SensorStatusReg) & (PRESSURE_SENSOR_OK)))
				{
					sprintf((char*)&temp_v[0],  "%.1f%cC", Temperature, 0xB0);
					ptr = str_center(temp_v, TEMP_FRAME_W, ENV_FRAME_FILL_CHAR);
					memcpy((void *)&Buff[0], ptr, TEMP_FRAME_W);

					sprintf((char*)&hum_v[0],  "%2u%%", Humidity);
					ptr = str_center(hum_v, HUM_FRAME_W, ENV_FRAME_FILL_CHAR);
					memcpy((void *)&Buff[TEMP_FRAME_W], ptr, HUM_FRAME_W);

					Sh = 1;
					memcpy((void *)&Buff[TEMP_FRAME_W + (HUM_FRAME_W+Sh)], &sensor_fail1c[0], 7);

				} else
				if (!((SensorStatusReg) & (HUMIDITY_SENSOR_OK)))
				{
					memcpy((void *)&Buff[TEMP_FRAME_W+1], &sensor_fail[0], 4);
					memcpy((void *)&Buff[TEMP_FRAME_W + HUM_FRAME_W + PRESS_FRAME_W], &sensor_fail[0], 4);
					memcpy((void *)&Buff[TEMP_FRAME_W + HUM_FRAME_W + PRESS_FRAME_W + (TEMPd_FRAME_W+Sh)], &sensor_fail[0], 4);
				}

				*len = TEMP_FRAME_W + HUM_FRAME_W + PRESS_FRAME_W + TEMPd_FRAME_W + TEMPp_FRAME_W + Sh;

				NextPage = true;
				ClearBuff = false;
				SLen = *len;
			}
		#endif
	#endif
		}
		break;

		case UVx_SENSOR:
		{
	#if (UVx_SENSOR_PRESENT==1)
		#if (TLCD_SUPPORT==1)
			if ((t_flip == 1) & (PM_toggle==1))	//t_flip identifies the display page number: Page 1 -> Environmental Page
			{												//PM_Toggle+1 because PM_toggle is incremented at the end of the
				if (!((SensorStatusReg) & (UVx_SENSOR_OK)))	//TVOC section of the FormatDisplayString function
				{
					memcpy((void *)&Buff[5], &sensor_fail[0], 4);
					memcpy((void *)&Buff[15], &sensor_fail[0], 4);
					memcpy((void *)&Buff[34], &sensor_fail2p[0], 6);
				}
			}
		#elif (GLCD_SUPPORT==1)
		if ((t_flip == 1) & (PM_toggle==1))	//t_flip identifies the display page number: Page 1 -> Environmental Page
		{										//PM_Toggle+1 because PM_toggle is incremented at the end of the
//			memset((void *)&Buff[0], 0, 32);			//TVOC section of the FormatDisplayString function

			u8g2_SetFont(&u8g2, u8g2_font_t0_12_mf);
			u8g2_SetFontMode(&u8g2, 0);
			if (((SensorStatusReg) & (UVx_SENSOR_OK)) && !((SensorStatusReg) & (PRESSURE_SENSOR_OK)))
			{
				TextXPos = 3; TextYPos = 27;
			#if (VEML6075)
				sprintf((char*)&uva_v[0], "%.1f", UVa);
				ptr = str_center(uva_v, UVA_FRAME_W, UVx_FRAME_FILL_CHAR);
				memcpy((void *)&Buff[0], ptr, UVA_FRAME_W);

				sprintf((char*)&uvb_v[0], "%.1f", UVb);
				ptr = str_center(uvb_v, UVB_FRAME_W, UVx_FRAME_FILL_CHAR);
				memcpy((void *)&Buff[UVA_FRAME_W], ptr, UVB_FRAME_W);
			#elif (LTR390UV)
				sprintf((char*)&uva_v[0], "%.1f lx", Lux);
				ptr = str_center(uva_v, UVA_FRAME_W, UVx_FRAME_FILL_CHAR);
				memcpy((void *)&Buff[0], ptr, UVA_FRAME_W);
			#endif // VEML6075
				sprintf((char*)&uvi_v[0], "%.2f", UV_Index);
				ptr = str_center(uvi_v, UVIDX_FRAME_W, UVx_FRAME_FILL_CHAR);
				memcpy((void *)&Buff[UVA_FRAME_W], ptr, UVIDX_FRAME_W);

				*len = UVA_FRAME_W + UVB_FRAME_W + UVIDX_FRAME_W;
			} else
			if (!((SensorStatusReg) & (UVx_SENSOR_OK)))
			{
				memcpy((void *)&Buff[2], &sensor_fail1b[0], 6);
				memcpy((void *)&Buff[13], &sensor_fail1b[0], 6);
				memcpy((void *)&Buff[23], &sensor_fail1a[0], 5);
			}
			NextPage = true;
			ClearBuff = false;
			SLen = *len;
		}
		#endif
	#endif
		}
		break;

		case VOC_SENSOR:
		{
		#if (DISPLAY_C6H6==0)
			bool PM_Toggle_1 = false;
		#endif
			bool PM_Toggle_2 = false;
	#if (VOC_SENSOR_PRESENT==1)
		#if (TLCD_SUPPORT==1)
			if (t_flip == 2)	//t_flip identifies the display page number: Page 2 -> Air Quality Page
			{
				if ((SensorStatusReg) & (VOC_SENSOR_OK))
				{
					//Format first TLCD row
					*len = sprintf((char*)&Buff[*len], "eTVOC: %3uppb\r\n", eq_TVOC);
					//Format second TLCD row
					*len += sprintf((char*)&Buff[*len], "eCO2:%5uppm\r\n", eq_CO2);
				} else
				{
					*len = sprintf((char*)&Buff[*len], "TVOC-CO2 sensors failure\r\n");
				}
			}
		#elif (GLCD_SUPPORT==1)
			if (t_flip == 2)	//t_flip identifies the display page number: Page 2 -> Air Quality Page
			{
				memset((void *)&Buff[0], 0, 32);

				u8g2_SetFont(&u8g2, u8g2_font_t0_12_mf);
				u8g2_SetFontMode(&u8g2, 0);
				if (((SensorStatusReg) & (VOC_SENSOR_OK)) || ((SensorStatusReg) & (GAS_SENSORS_OK)))
				{
			#if (GAS_SENSOR_MODULE_PRESENT==1)
				#if (FULL_MODE==1)
					if (PM_toggle == 1)
					{
						TextXPos = 4; TextYPos = 29;
					#if (DISPLAY_C6H6==0)
						PM_Toggle_1 = true;
					#endif
						PM_Toggle_2 = false;

						if ((SensorStatusReg) & (VOC_SENSOR_OK))
						{
							FillAirQualityField(etvoc_v, eq_TVOC, 0);
							ptr = str_center(etvoc_v, eTVOC_FRAME_W, AIRQ_FRAME_FILL_CHAR);
							memcpy((void *)&Buff[0], ptr, eTVOC_FRAME_W);
						} else
						{
							ptr = str_center((char *)sensor_fail, FAIL_MSG_W, AIRQ_FRAME_FILL_CHAR);
							memcpy((void *)&Buff[0], ptr, FAIL_MSG_W);
						}

						FillAirQualityField(co_v, CO, (uint8_t)BIT_CHECK(AnlgOvflStatusReg,4));
						ptr = str_center(co_v, CO_FRAME_W, AIRQ_FRAME_FILL_CHAR);
						memcpy((void *)&Buff[eTVOC_FRAME_W], ptr, CO_FRAME_W);

						FillAirQualityField(no2_v, NO2, (uint8_t)BIT_CHECK(AnlgOvflStatusReg,2));
						ptr = str_center(no2_v, NO2_FRAME_W, AIRQ_FRAME_FILL_CHAR);
						memcpy((void *)&Buff[eTVOC_FRAME_W+CO_FRAME_W], ptr, NO2_FRAME_W);

						FillAirQualityField(nh3_v, NH3, (uint8_t)BIT_CHECK(AnlgOvflStatusReg,3));
						ptr = str_center(nh3_v, NH3_FRAME_W, AIRQ_FRAME_FILL_CHAR);
						memcpy((void *)&Buff[eTVOC_FRAME_W+CO_FRAME_W+NO2_FRAME_W], ptr, NH3_FRAME_W);

						*len = eTVOC_FRAME_W + CO_FRAME_W + NO2_FRAME_W + NH3_FRAME_W;
					} else
					if (PM_toggle == 2)
					{
						TextXPos = 4; TextYPos = 29;
					#if (DISPLAY_C6H6==0)
						PM_Toggle_1 = false;
					#endif
						PM_Toggle_2 = true;

						FillAirQualityField(ch2o_v, CH2O, (uint8_t)BIT_CHECK(AnlgOvflStatusReg,0));
						ptr = str_center(ch2o_v, CH2O_FRAME_W, AIRQ_FRAME_FILL_CHAR);
						memcpy((void *)&Buff[0], ptr, CH2O_FRAME_W);

						FillAirQualityField(o3_v, O3, (uint8_t)BIT_CHECK(AnlgOvflStatusReg,1));
						ptr = str_center(o3_v, O3_FRAME_W, AIRQ_FRAME_FILL_CHAR);
						memcpy((void *)&Buff[CH2O_FRAME_W], ptr, O3_FRAME_W);

						FillAirQualityField(so2_v, SO2, (uint8_t)BIT_CHECK(AnlgOvflStatusReg,5));
						ptr = str_center(so2_v, SO2_FRAME_W, AIRQ_FRAME_FILL_CHAR);
						memcpy((void *)&Buff[CH2O_FRAME_W+O3_FRAME_W], ptr, SO2_FRAME_W);
					#if (DISPLAY_C6H6)
						FillAirQualityField(c6h6_v, C6H6, (uint8_t)BIT_CHECK(AnlgOvflStatusReg,6));
						ptr = str_center(c6h6_v, C6H6_FRAME_W, AIRQ_FRAME_FILL_CHAR);
						memcpy((void *)&Buff[CH2O_FRAME_W+O3_FRAME_W+SO2_FRAME_W], ptr, C6H6_FRAME_W);

						*len = CH2O_FRAME_W + O3_FRAME_W + SO2_FRAME_W + C6H6_FRAME_W;
					#else
						FillAirQualityField(eco2_v, eq_CO2, 0);
						ptr = str_center(eco2_v, eCO2_FRAME_W, AIRQ_FRAME_FILL_CHAR);
						memcpy((void *)&Buff[CH2O_FRAME_W+O3_FRAME_W+SO2_FRAME_W], ptr, eCO2_FRAME_W);

						*len = CH2O_FRAME_W + O3_FRAME_W + SO2_FRAME_W + eCO2_FRAME_W;
					#endif
						Buff[*len+1] = '\r';
						Buff[*len+2] = '\n';
					}
//					*len = sprintf((char*)&Buff[0], "ABCDEFGHILMNOPQRSTUVZ0123456789");
				#else	//FULL_MODE==0
					if (PM_toggle == 1)
					{
						TextXPos = 4; TextYPos = 29;
					#if (DISPLAY_C6H6==0)
						PM_Toggle_1 = true;
					#endif
						PM_Toggle_2 = false;

						if ((SensorStatusReg) & (VOC_SENSOR_OK))
						{
							FillAirQualityField(etvoc_v, eq_TVOC, 0);
							ptr = str_center(etvoc_v, eTVOC_FRAME_W, AIRQ_FRAME_FILL_CHAR);
							memcpy((void *)&Buff[0], ptr, eTVOC_FRAME_W);
						} else
						{
							ptr = str_center((char *)sensor_fail, FAIL_MSG_W, AIRQ_FRAME_FILL_CHAR);
							memcpy((void *)&Buff[0], ptr, FAIL_MSG_W);
						}

						FillAirQualityField(eco2_v, eq_CO2, 0);
						ptr = str_center(eco2_v, eCO2_FRAME_W, AIRQ_FRAME_FILL_CHAR);
						memcpy((void *)&Buff[eTVOC_FRAME_W], ptr, eCO2_FRAME_W);

						FillAirQualityField(ch2o_v, CH2O, (uint8_t)BIT_CHECK(AnlgOvflStatusReg,0));
						ptr = str_center(ch2o_v, CH2O_FRAME_W, AIRQ_FRAME_FILL_CHAR);
						memcpy((void *)&Buff[eTVOC_FRAME_W+eCO2_FRAME_W], ptr, CH2O_FRAME_W);

						FillAirQualityField(co_v, CO, (uint8_t)BIT_CHECK(AnlgOvflStatusReg,4));
						ptr = str_center(co_v, CO_FRAME_W, AIRQ_FRAME_FILL_CHAR);
						memcpy((void *)&Buff[eTVOC_FRAME_W+eCO2_FRAME_W+CH2O_FRAME_W], ptr, CO_FRAME_W);

						*len = eTVOC_FRAME_W + eCO2_FRAME_W + CH2O_FRAME_W + CO_FRAME_W;
						Buff[*len+1] = '\r';
						Buff[*len+2] = '\n';
					}
//					*len = sprintf((char*)&Buff[0], "ABCDEFGHILMNOPQRSTUVZ0123456789");
				#endif		//FULL_MODE
			#else			//GAS_SENSOR_MODULE_PRESENT==0
					TextXPos = 46; TextYPos = 29;

					FillAirQualityField(etvoc_v, eq_TVOC, 0);
					ptr = str_center(etvoc_v, eTVOC_FRAME_W, AIRQ_FRAME_FILL_CHAR);
					memcpy((void *)&Buff[0], ptr, eTVOC_FRAME_W);

					FillAirQualityField(eco2_v, eq_CO2, 0);
					ptr = str_center(eco2_v, eCO2_FRAME_W, AIRQ_FRAME_FILL_CHAR);
					memcpy((void *)&Buff[eTVOC_FRAME_W], ptr, eCO2_FRAME_W);

					*len = eTVOC_FRAME_W + eCO2_FRAME_W;
					Buff[*len+1] = '\r';
					Buff[*len+2] = '\n';

//					*len = sprintf((char*)&Buff[0], "ABCDEFGHILMNOPQRST");
			#endif	//GAS_SENSOR_MODULE_PRESENT
				}
				if (++PM_toggle > NumGAS_Pages)
				{
					PM_toggle = 1;
				}
				if (!((SensorStatusReg) & (VOC_SENSOR_OK)))
				{
					ptr = str_center((char *)sensor_fail, FAIL_MSG_W, AIRQ_FRAME_FILL_CHAR);
					memcpy((void *)&Buff[0], ptr, FAIL_MSG_W);

					*len = eTVOC_FRAME_W;
				}
				if (!((SensorStatusReg) & (GAS_SENSORS_OK)))
				{
					ptr = str_center((char *)sensor_fail1a, FAIL_MSG_W+1, AIRQ_FRAME_FILL_CHAR);
					memcpy((void *)&Buff[eTVOC_FRAME_W], ptr, FAIL_MSG_W+1);
					memcpy((void *)&Buff[eTVOC_FRAME_W+CO_FRAME_W], ptr, FAIL_MSG_W+1);
				#if (DISPLAY_C6H6)
					memcpy((void *)&Buff[eTVOC_FRAME_W+CO_FRAME_W+NO2_FRAME_W], ptr, FAIL_MSG_W+1);
				#else
					if (PM_Toggle_1)
					{
						memcpy((void *)&Buff[eTVOC_FRAME_W+CO_FRAME_W+NO2_FRAME_W], ptr, FAIL_MSG_W+1);
					}
				#endif
					if (PM_Toggle_2)
					{
						ptr = str_center((char *)sensor_fail, FAIL_MSG_W, AIRQ_FRAME_FILL_CHAR);
						memcpy((void *)&Buff[0], ptr, FAIL_MSG_W);
					}

					*len = eTVOC_FRAME_W + CO_FRAME_W + NO2_FRAME_W + NH3_FRAME_W;
				}

				NextPage = true;
				ClearBuff = false;
				SLen = *len;
			}
		#endif	//GLCD_SUPPORT==1
	#endif	//VOC_SENSOR_PRESENT==1
		}
		break;

		case PARTICULATE_SENSOR:
		{
	#if (PARTICULATE_SENSOR_PRESENT==1)
		#if (TLCD_SUPPORT==1)
		if (t_flip == 3)	//t_flip identifies the display page number: Page 3 -> Air Pollution Page
		{
			if ((SensorStatusReg) & (PM_SENSOR_OK))
			{
				if (PM_toggle == 1)
				{
					//Format first TLCD row
					*len = sprintf((char*)&Buff[*len], "PM1.0:%3uug/mc", MC_1p0);
					*len += sprintf((char*)&Buff[*len], " %3u/cmc\r\n", NC_1p0);
					//Format second TLCD row
					*len += sprintf((char*)&Buff[*len], "PM2.5:%3uug/mc", MC_2p5);
					*len += sprintf((char*)&Buff[*len], " %3u/cmc\r\n", NC_2p5);
				} else
				if (PM_toggle == 2)
				{
					//Format first TLCD row
					*len = sprintf((char*)&Buff[*len], "PM4.0:%3uug/mc", MC_4p0);
					*len += sprintf((char*)&Buff[*len], " %3u/cmc\r\n", NC_4p0);
					//Format second TLCD row
					*len += sprintf((char*)&Buff[*len], "PM10 :%3uug/mc", MC_10p0);
					*len += sprintf((char*)&Buff[*len], " %3u/cmc\r\n", NC_10p0);
				} else
				if (PM_toggle == 3)
				{
					//Format first TLCD row
					*len = sprintf((char*)&Buff[*len], "PM0.5:%3u/cmc           \r\n", NC_0p5);
					//Format second TLCD row
					*len += sprintf((char*)&Buff[*len], "Tip.Part.Size: %.3fum\r\n", TypicalParticleSize);
				}
				if (++PM_toggle > NumPM_Pages)
				{
					PM_toggle = 1;
				}
			} else
			{
				*len = sprintf((char*)&Buff[*len], "PMx sensor failure      \r\n");
			}
		}
		#elif (GLCD_SUPPORT==1)
		if (t_flip == 3)	//t_flip identifies the display page number: Page 3 -> Air Pollution Page
		{
			memset((void *)&Buff[0], 0, 32);

			u8g2_SetFont(&u8g2, u8g2_font_t0_12_mf);
			u8g2_SetFontMode(&u8g2, 0);
			if ((SensorStatusReg) & (PM_SENSOR_OK))
			{
				if (PM_toggle == 1)
				{
					TextXPos = 3; TextYPos = 29;

					sprintf((char*)&mc_1p0_v[0],  "%u", MC_1p0);
					ptr = str_center(mc_1p0_v, MC_1p0_FRAME_W, AIRP_FRAME_FILL_CHAR);
					memcpy((void *)&Buff[0], ptr, MC_1p0_FRAME_W);

					sprintf((char*)&mc_2p5_v[0],  "%u", MC_2p5);
					ptr = str_center(mc_2p5_v, MC_2p5_FRAME_W, AIRP_FRAME_FILL_CHAR);
					memcpy((void *)&Buff[MC_1p0_FRAME_W], ptr, MC_2p5_FRAME_W);

					sprintf((char*)&mc_4p0_v[0],  "%u", MC_4p0);
					ptr = str_center(mc_4p0_v, MC_4p0_FRAME_W, AIRP_FRAME_FILL_CHAR);
					memcpy((void *)&Buff[MC_1p0_FRAME_W+MC_2p5_FRAME_W], ptr, MC_4p0_FRAME_W);

					sprintf((char*)&mc_10p0_v[0],  "%u", MC_10p0);
					ptr = str_center(mc_10p0_v, MC_10p0_FRAME_W, AIRP_FRAME_FILL_CHAR);
					memcpy((void *)&Buff[MC_1p0_FRAME_W+MC_2p5_FRAME_W+MC_4p0_FRAME_W], ptr, MC_10p0_FRAME_W);

					*len = MC_1p0_FRAME_W + MC_2p5_FRAME_W + MC_4p0_FRAME_W + MC_10p0_FRAME_W;
				} else
				if (PM_toggle == 2)
				{
					TextXPos = 3; TextYPos = 29;

					sprintf((char*)&nc_1p0_v[0],  "%u", NC_1p0);
					ptr = str_center(nc_1p0_v, NC_1p0_FRAME_W, AIRP_FRAME_FILL_CHAR);
					memcpy((void *)&Buff[0], ptr, NC_1p0_FRAME_W);

					sprintf((char*)&nc_2p5_v[0],  "%u", NC_2p5);
					ptr = str_center(nc_2p5_v, NC_2p5_FRAME_W, AIRP_FRAME_FILL_CHAR);
					memcpy((void *)&Buff[NC_1p0_FRAME_W], ptr, NC_2p5_FRAME_W);

					sprintf((char*)&nc_4p0_v[0],  "%u", NC_4p0);
					ptr = str_center(nc_4p0_v, NC_4p0_FRAME_W, AIRP_FRAME_FILL_CHAR);
					memcpy((void *)&Buff[NC_1p0_FRAME_W+NC_2p5_FRAME_W], ptr, NC_4p0_FRAME_W);

					sprintf((char*)&nc_10p0_v[0],  "%u", NC_10p0);
					ptr = str_center(nc_10p0_v, NC_10p0_FRAME_W, AIRP_FRAME_FILL_CHAR);
					memcpy((void *)&Buff[NC_1p0_FRAME_W+NC_2p5_FRAME_W+NC_4p0_FRAME_W], ptr, NC_10p0_FRAME_W);

					*len = NC_1p0_FRAME_W + NC_2p5_FRAME_W + NC_4p0_FRAME_W + NC_10p0_FRAME_W;
				} else
				if (PM_toggle == 3)
				{
					TextXPos = 27; TextYPos = 29;

					sprintf((char*)&nc_0p5_v[0],  "%u", NC_0p5);
					ptr = str_center(nc_0p5_v, NC_0p5_FRAME_W, AIRP_FRAME_FILL_CHAR);
					memcpy((void *)&Buff[0], ptr, NC_0p5_FRAME_W);

					sprintf((char*)&tps_v[0],  " %.3f", TypicalParticleSize);
					ptr = str_center(tps_v, TPS_FRAME_W, AIRP_FRAME_FILL_CHAR);
					memcpy((void *)&Buff[NC_0p5_FRAME_W], ptr, TPS_FRAME_W);

					*len = NC_0p5_FRAME_W + TPS_FRAME_W;
					Buff[*len+1] = '\r';
					Buff[*len+2] = '\n';
				}
//				*len = sprintf((char*)&Buff[0], "ABCDEFGHILMNOPQRSTUVZ0123456789");
				if (++PM_toggle > NumPM_Pages)
				{
					PM_toggle = 1;
				}
			} else
			{
				TextXPos = 11; TextYPos = 29;
				*len += sprintf((char*)&Buff[0], "FAIL    FAIL    FAIL    FAIL\r\n");
			}
			NextPage = true;
			ClearBuff = false;
			SLen = *len;
		}
		#endif
	#endif
		}
		break;
		default:
			break;
	}
#endif
}
