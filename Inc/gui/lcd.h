/*
 * lcd.c
 *
 *  Created on: 10/06/2018
 *      Author: Olivier Van den Eede (20x2 text LCD display driver only)
 *      Modified by: Tommaso Sabatini 18/03/2020
 *      Added Graphic LCD functions by: Tommaso Sabatini 03/08/2020
 */

#ifndef LCD_H_
#define LCD_H_

#include "application/Z_WeatherForecast.h"
#include "application/AirQuality.h"
#include "gui/u8g2.h"

#define LCD20xN 			// For 20xN LCDs
//#define LCD16xN			// For 16xN LCDs
// For row start addresses
extern const uint8_t ROW_16[];
extern const uint8_t ROW_20[];

#define FAIL_MSG_W		7U
#define TEMP_FRAME_W	7U
#define HUM_FRAME_W		7U
#define PRESS_FRAME_W	9U
#define ALT_FRAME_W		9U
#define TEMPd_FRAME_W	4U
#define TEMPp_FRAME_W	4U
#define ENV_FRAME_FILL_CHAR	0x20

#define UVA_FRAME_W		10U
#define UVB_FRAME_W		10U
#define UVIDX_FRAME_W	11U
#define UVx_FRAME_FILL_CHAR	0x20

#if (GAS_SENSOR_MODULE_PRESENT==1)
	#if (OUTDOOR_MODE)
		#define eTVOC_FRAME_W	7U
		#define eCO2_FRAME_W	9U
		#define CO_FRAME_W		8U
		#define NO2_FRAME_W		8U
		#define NH3_FRAME_W		8U
		#define CH2O_FRAME_W	7U
		#define O3_FRAME_W		8U
		#define SO2_FRAME_W		8U
		#define C6H6_FRAME_W	8U
	#else
		#define eTVOC_FRAME_W	7U
		#define eCO2_FRAME_W	9U
		#define CH2O_FRAME_W	8U
		#define CO_FRAME_W		8U
		#define NO2_FRAME_W		8U
		#define NH3_FRAME_W		8U
	#endif
#else
	#define eTVOC_FRAME_W	8U
	#define eCO2_FRAME_W	9U
#endif
#define AIRQ_FRAME_FILL_CHAR 0x20

#define MC_1p0_FRAME_W	7U
#define MC_2p5_FRAME_W	8U
#define MC_4p0_FRAME_W	8U
#define MC_10p0_FRAME_W	8U
#define NC_0p5_FRAME_W	8U
#define NC_1p0_FRAME_W	7U
#define NC_2p5_FRAME_W	8U
#define NC_4p0_FRAME_W	8U
#define NC_10p0_FRAME_W	8U
#define TPS_FRAME_W		15U
#define AIRP_FRAME_FILL_CHAR 0x20

#define DISPLAY_SIMMER_INDEX	(0)
#define DISPLAY_HEAT_INDEX		(1)
#define DISPLAY_C6H6			(0)

#define GLCD_ROW_MAX_LENGTH		(uint8_t)0x1EU	//Maximum number of characters (30) in one graphic display line
#define TLCD_ROW_MAX_LENGTH		(uint8_t)0x14U	//Maximum number of characters (20) in one text display line
/************************************** Command register **************************************/
#define CLEAR_DISPLAY 0x01

#define RETURN_HOME 0x02

#define ENTRY_MODE_SET 0x04
#define OPT_S	0x01					// Shift entire display to right
#define OPT_INC 0x02					// Cursor increment

#define DISPLAY_ON_OFF_CONTROL 0x08
#define OPT_D	0x04					// Turn on display
#define OPT_C	0x02					// Turn on cursor
#define OPT_B 	0x01					// Turn on cursor blink

#define CURSOR_DISPLAY_SHIFT 0x10		// Move and shift cursor
#define OPT_SC 0x08
#define OPT_RL 0x04

#define FUNCTION_SET 0x20
#define OPT_DL 0x10						// Set interface data length
#define OPT_N 0x08						// Set number of display lines
#define OPT_F 0x04						// Set alternate font

#define SET_DDRAM_ADDR 0x80				// Set DDRAM address

/************************************** Helper macros **************************************/
#define DELAY(X) HAL_Delay(X)

/************************************** LCD defines **************************************/
#define LCD_NIB 4
#define LCD_BYTE 8
#define LCD_DATA_REG 1
#define LCD_COMMAND_REG 0

/************************************** LCD typedefs **************************************/
#define Lcd_PortType GPIO_TypeDef*
#define Lcd_PinType uint16_t

typedef enum 
{
	LCD_4_BIT_MODE,
	LCD_8_BIT_MODE
} Lcd_ModeTypeDef;

typedef enum
{
	LCD_OK	=	 0x00,
	LCD_ERROR =	 0x01
} LCD_Error_et;

typedef struct 
{
	Lcd_PortType *data_port;
	Lcd_PinType *data_pin;

	Lcd_PortType rs_port;
	Lcd_PinType rs_pin;

	Lcd_PortType en_port;
	Lcd_PinType en_pin;

	Lcd_ModeTypeDef mode;

} Lcd_HandleTypeDef;

Lcd_HandleTypeDef my_lcd;

uint8_t Row, Column;

/************************************** TEXT LCD Public functions **************************************/
void Lcd_init(Lcd_HandleTypeDef * lcd);
void Lcd_int(Lcd_HandleTypeDef * lcd, int number);
void Lcd_string(Lcd_HandleTypeDef *lcd, char *string, int len);
void Lcd_cursor(Lcd_HandleTypeDef * lcd, uint8_t row, uint8_t col);
void Lcd_clear(Lcd_HandleTypeDef *lcd);
void lcd_write_command(Lcd_HandleTypeDef *lcd, uint8_t command);
void lcd_write_data(Lcd_HandleTypeDef *lcd, uint8_t data);
void lcd_write(Lcd_HandleTypeDef *lcd, uint8_t data, uint8_t Len);
Lcd_HandleTypeDef Lcd_create(
		Lcd_PortType port[], Lcd_PinType pin[],
		Lcd_PortType rs_port, Lcd_PinType rs_pin,
		Lcd_PortType en_port, Lcd_PinType en_pin, Lcd_ModeTypeDef mode);
void send_tlcdmessage(char *string, int len);
void LCD_Clear(void);
LCD_Error_et MX_TLCD_Init();

/************************************** GRAPHIC LCD Public variables **************************************/
uint8_t TextXPos, TextYPos, PM_toggle, NumberOfPages, Disp_Area;
bool ClearBuff, NextPage;
char *ptr;
uint8_t HumVal[10], Len;
char temp_v[10]; char tempd_v[8]; char tempp_v[8]; char hum_v[8]; char press_v[10]; char alt_v[10];
char uva_v[10]; char uvb_v[10]; char uvi_v[10];
char etvoc_v[10]; char eco2_v[10];
char etvoc_v_1h_Mean[10]; char eco2_v_1h_Mean[10];
char mc_1p0_v[10]; char mc_2p5_v[10]; char mc_4p0_v[10]; char mc_10p0_v[10];
char nc_0p5_v[10]; char nc_1p0_v[10]; char nc_2p5_v[10]; char nc_4p0_v[10]; char nc_10p0_v[10]; char tps_v[10];
char mc_1p0_v_24h_Mean[10]; char mc_2p5_v_24h_Mean[10]; char mc_4p0_v_24h_Mean[10]; char mc_10p0_v_24h_Mean[10];
char co_v[10]; char no2_v[10]; char nh3_v[10]; char ch2o_v[10]; char o3_v[10]; char so2_v[10]; char c6h6_v[10];
char co_v_8h_Mean[10]; char no2_v_1h_Mean[10]; char nh3_v_8h_Mean[10]; char ch2o_v_8h_Mean[10];
char o3v_8h_Mean[10]; char so2_v_24h_Mean[10]; char c6h6_v_24h_Mean[10];

/************************************** GRAPHIC LCD Public functions **************************************/
uint8_t u8x8_stm32_gpio_and_delay(U8X8_UNUSED u8x8_t *u8x8,
		U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int,
		U8X8_UNUSED void *arg_ptr);
uint8_t u8x8_byte_4wire_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
void send_glcdmessage(char *str, uint8_t xpos, uint8_t ypos, uint8_t yspace, uint8_t len, bool clearbuff, bool next_page);
void GLCD_Clear(bool next_page, bool clear);
LCD_Error_et ReDrawPage_S0(uint8_t PageNumb);
LCD_Error_et ReDrawPage_S1(uint8_t PageNumb);
LCD_Error_et SendWelcomeMessage();
LCD_Error_et SendReadyDevicesMessage();
char *str_center(char str[], unsigned int new_length, char placeholder);
LCD_Error_et MX_GLCD_Init();
//void FormatDisplayString(int *len, uint8_t* Buff, SENSOR_TYPE stype);

#endif /* LCD_H_ */
