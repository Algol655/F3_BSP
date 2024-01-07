/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

#include "hci_tl_interface.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "compiler/compiler.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CCS811_nWAKE_Pin GPIO_PIN_13
#define CCS811_nWAKE_GPIO_Port GPIOC
#define RS_LCD_Pin GPIO_PIN_0
#define RS_LCD_GPIO_Port GPIOC
#define SEL_1_Pin GPIO_PIN_1
#define SEL_1_GPIO_Port GPIOC
#define IO_EXP_INT_Pin GPIO_PIN_2
#define IO_EXP_INT_GPIO_Port GPIOC
#define IO_EXP_INT_EXTI_IRQn EXTI2_IRQn
#define BUTTON_Pin GPIO_PIN_0
#define BUTTON_GPIO_Port GPIOA
#define BUTTON_EXTI_IRQn EXTI0_IRQn
#define ADC1_IN1_Pin GPIO_PIN_1
#define ADC1_IN1_GPIO_Port GPIOA
#define ADC1_IN2_Pin GPIO_PIN_2
#define ADC1_IN2_GPIO_Port GPIOA
#define ADC1_IN3_Pin GPIO_PIN_3
#define ADC1_IN3_GPIO_Port GPIOA
#define SPI1_NSS_Pin GPIO_PIN_4
#define SPI1_NSS_GPIO_Port GPIOA
#define SPI1_SCK_Pin GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
#define SPI1_MISO_Pin GPIO_PIN_6
#define SPI1_MISO_GPIO_Port GPIOA
#define SPI1_MOSI_Pin GPIO_PIN_7
#define SPI1_MOSI_GPIO_Port GPIOA
#define D3_LCD_BLE_INT_Pin GPIO_PIN_4
#define D3_LCD_BLE_INT_GPIO_Port GPIOC
#define D3_LCD_BLE_INT_EXTI_IRQn EXTI4_IRQn
#define D2_LCD_BLE_CS_Pin GPIO_PIN_5
#define D2_LCD_BLE_CS_GPIO_Port GPIOC
#define RESTART_Pin GPIO_PIN_0
#define RESTART_GPIO_Port GPIOB
#define SEL_3_Pin GPIO_PIN_1
#define SEL_3_GPIO_Port GPIOB
#define I2C2_SCL_Pin GPIO_PIN_10
#define I2C2_SCL_GPIO_Port GPIOB
#define I2C2_SDA_Pin GPIO_PIN_11
#define I2C2_SDA_GPIO_Port GPIOB
#define D4_LCD_SPI2_NSS_Pin GPIO_PIN_12
#define D4_LCD_SPI2_NSS_GPIO_Port GPIOB
#define D5_LCD_SPI2_SCK_Pin GPIO_PIN_13
#define D5_LCD_SPI2_SCK_GPIO_Port GPIOB
#define D6_LCD_SPI2_MISO_Pin GPIO_PIN_14
#define D6_LCD_SPI2_MISO_GPIO_Port GPIOB
#define D7_LCD_SPI2_MOSI_Pin GPIO_PIN_15
#define D7_LCD_SPI2_MOSI_GPIO_Port GPIOB
#define SEL_0_Pin GPIO_PIN_8
#define SEL_0_GPIO_Port GPIOC
#define D1_LCD_BLE_RST_Pin GPIO_PIN_9
#define D1_LCD_BLE_RST_GPIO_Port GPIOC
#define SEL_2_Pin GPIO_PIN_10
#define SEL_2_GPIO_Port GPIOA
#define USART3_TX_Pin GPIO_PIN_10
#define USART3_TX_GPIO_Port GPIOC
#define USART3_RX_Pin GPIO_PIN_11
#define USART3_RX_GPIO_Port GPIOC
#define PC_12_LED_Pin GPIO_PIN_12
#define PC_12_LED_GPIO_Port GPIOC
#define EN_LCD_Pin GPIO_PIN_2
#define EN_LCD_GPIO_Port GPIOD
#define I2C1_SCL_Pin GPIO_PIN_6
#define I2C1_SCL_GPIO_Port GPIOB
#define I2C1_SDA_Pin GPIO_PIN_7
#define I2C1_SDA_GPIO_Port GPIOB
#define CAN1_RX_Pin GPIO_PIN_8
#define CAN1_RX_GPIO_Port GPIOB
#define CAN1_TX_Pin GPIO_PIN_9
#define CAN1_TX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/**************************/
/*   Board FW Settings    */
/**************************/
#define NORMAL_MODE (1)					//As an alternative to DATA_MODE, GUI_SUPPORT
#define DATA_MODE	(0)					//1 when the node is used as an end-point of a wireless data link (e.g. BLE, LORA, UWB ...)
#define IO_EXP_PRESENT (1)
//Sensors defines section				//When IMU_PRESENT = 1 UnicleoGUI graphics only the accelerometer, Gyroscope and Magnet. data.
#define IMU_PRESENT (0)					//When IMU_PRESENT = 0 UnicleoGUI graphics only the environmental sensor data.
#define PRESSURE_SENSOR_PRESENT (1)
#define HUMIDITY_SENSOR_PRESENT (1)
#define UVx_SENSOR_PRESENT		(0)
#define ALS_SENSOR_PRESENT		(0)
#define VOC_SENSOR_PRESENT		(1)
#define PARTICULATE_SENSOR_PRESENT (1)
#define GAS_SENSOR_MODULE_PRESENT  (1)	//Must be understood as "USE_ADC"
#define FULL_MODE		(1)
//Board defines section
#define USE_STM32F4XX_NUCLEO	(0)
#define USE_SENSUS191	(1)
#define BOOTLOADER_MODE (1)				//0 = UART Mode; 1 = DFU Mode (USB)
#define STBY_MODE		(0)				//0 = Disable entering Stand By mode
#define DATA_EEPROM_BASE (0x0803F800)	//Last flash sector (127) is used as EEPROM to store/load data. See processor reference manual
#define USE_BKUP_SRAM	(1)				//1 = Internal Back-Up SRAM is used
#define RTC_BKUP_SIZE	(84)			//Internal Back-Up SRAM size: 42 x 16bit (84 bytes). See processor reference manual
#define USE_IWDGT 		(1)				//1 = IWDGT activated (=1 only in BLE BEACON MODE!!!)
/**************************/
/*    Project Settings    */
/**************************/
#define CALC_DEWPOINT	(1)				//When at 1 the DewPoint is calculated and displayed. CALC_ALTITUDE must be 0
#define CALC_ALTITUDE	(0)				//When at 1 the altitude calculated according to barometric formula
										//to the atmospheric pressure will be displayed. CALC_DEWPOINT must be 0
//GUI defines section
#define TLCD_SUPPORT	(0)				//1 to support a Text LCD 24xN. GLCD_SUPPORT and TFT_SUPPORT must both be zero!
#define GLCD_SUPPORT	(0)				//1 to support a Graphic LCD 192x64. TLCD_SUPPORT and TFT_SUPPORT must both be zero!
#define TFT_SUPPORT		(0)				//1 to support a TouchScreen TFT Graphic LCD. TLCD_SUPPORT and GLCD_SUPPORT must both be zero!
#define BLE_SUPPORT		(1)				//1: Sensor data are sent to the BLE for the "ST BLE Sensor" app. GUI_SUPPORT must be zero
#define GUI_SUPPORT		(0)				//Set to 1 when STM UNICLEO Graphical User Interface is used to Display/Control sensors data
										//Warning!! You have to enable all sensors when Unicleo mode is enabled and BLE_SUPPORT must be zero
										//Warning!! When UnicleoGUI == 1 the possibility of displaying the data on a local display is excluded
#define WELCOME_STRING1 "****** SENSUS-191 ******\r\n"
#define WELCOME_STRING2 "     Personal Environmental\n\r       Monitoring Station"

/* a=target variable, b=bit number to act upon 0-n */
#define BIT_MASK(b)				(1ULL << (b))
#define BIT_SET(a,b) 			((a) |= (1ULL<<(b)))
#define BIT_CLEAR(a,b) 			((a) &= ~(1ULL<<(b)))
#define BIT_FLIP(a,b) 			((a) ^= (1ULL<<(b)))
#define BIT_CHECK(a,b)			(((a) & (1ULL << (b))) ? 1 : 0)
//#define BIT_CHECK(a,b) 		(!!((a) & (1ULL<<(b))))        // '!!' to make sure this returns 0 or 1
//#define BIT_CHECK(a,b)		(((a)>>(b)) & 1)

/* x=target variable, y=mask */
#define BITMASK_SET(x,y)		((x) |= (y))
#define BITMASK_CLEAR(x,y)		((x) &= (~(y)))
#define BITMASK_FLIP(x,y)		((x) ^= (y))
#define BITMASK_CHECK_ALL(x,y)	(((x) & (y)) == (y))   // warning: evaluates y twice
#define BITMASK_CHECK_ANY(x,y)	((x) & (y))

/* center text macros */
#define CALC_CENTER_POSITION_PREV(WIDTH, STR) (((WIDTH + ((int)strlen(STR))) % 2) \
       ? ((WIDTH + ((int)strlen(STR)) + 1)/2) : ((WIDTH + ((int)strlen(STR)))/2))
#define CALC_CENTER_POSITION_POST(WIDTH, STR) (((WIDTH - ((int)strlen(STR))) % 2) \
       ? ((WIDTH - ((int)strlen(STR)) - 1)/2) : ((WIDTH - ((int)strlen(STR)))/2))
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
