/*! ----------------------------------------------------------------------------
 * @file	port.c
 * @brief	HW specific definitions and functions for portability
 *          All the Devices Callbacks & ISR are here!
 *
 * @author  Tommaso Sabatini, 2018
 */

#ifndef PORT_H_
#define PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#if defined(STM32F405xx)
	#include "stm32f4xx_hal.h"
#elif defined(STM32F105xC)
	#include "stm32f1xx_hal.h"
#endif
#include "compiler/compiler.h"
#include "usbd_cdc_if.h"
#include "usart.h"
#include "tim.h"
#include "rtc.h"				//Here to avoid circular redefinition of "usart_t" typedef
#include "main.h"
#include "gpio.h"
#include "can.h"
#include "spi.h"
#include "can/deca_can1.h"
#include "usart/deca_usart3.h"
#include "adc.h"
#include "gui/lcd.h"

#define BUFFLEN 			(2048)	//It must be >= the maximum size of the message to be transmitted
#define DATA_FRAME_LEN_MAX 	(1006)	//32*31+9+2+1+2
#define MSG_PAYLOAD_LEN_MAX DATA_FRAME_LEN_MAX-9-2
#if ((DATA_MODE==0) && (NORMAL_MODE==1))
	#define CANBUFFLEN 		(32)	//CAN Rx/Tx circular buffers size. (deca_canx.c)
	#define USARTBUFFLEN	(256)	//USART Rx/Tx circular buffers size. (deca_usartx.c)
#elif ((DATA_MODE==1) && (NORMAL_MODE==0))
	#define CANBUFFLEN 		(1024)
	#define USARTBUFFLEN	(2048)
#endif
#define USB_SUPPORT			(1) 	//In Data mode set VCP as EndPoint_1
#define CAN_SUPPORT			(0)		//In Data mode set CAN1 as EndPoint_1
#define CAN_REPEATER_MODE	(1)		//When set to "1" (in CAN Data mode) the device works as a wireless transparent CAN-BUS EndPoint
#define USART_SUPPORT		(0)		//In Data mode set USART2 as EndPoint_1
#define APPLICATION_RUN_CYCLE	(5)	//Moved from CCS811_Driver.h
#define BLE_MAGIC_NUMBER	(0x55)	//This is the value written in location 0 of the Back-Up RTC used to notify main()
									//that an BLE event must be managed at the next reset or power-on
#define TX_BUFF_LEN 		(512)
#define RX_TIMEOUT	(5000)			//Time (ms) in which the peripheral remains in Rx before signaling "no data"
uint8_t usbVCOMout[TX_BUFF_LEN];	//Display "Frame Buffer".
uint8_t dataseq[TX_BUFF_LEN];		//STCmdP serial protocol command protocol Tx Buffer - Used by UnicleoGUI message stream.
uint8_t dataseq1[TX_BUFF_LEN];		//STCmdP serial protocol command protocol Tx Buffer - Used by Report message stream.

uint8_t msg_payload[MSG_PAYLOAD_LEN_MAX];	//Max payload = FRAME_LEN_MAX - msg_hdr (9 bytes)  - CRC (2 bytes) = 500 bytes
bool Link_Ok;
uint8_t d_zero;

typedef struct
{
    uint16_t		usblen;							// < Rx message length from USB
    uint8_t			usbbuf[MSG_PAYLOAD_LEN_MAX-2];	// < for RX from USB. Last word is reserved for msg_length
    bool 			usbcts;							// < USB CTS; Used to stop host TX
    bool			disabled;						// < Used to enable CDC reception regardless of usbcts status
	uint32_t		usbrx_time;						// < Time in which the VCP remains in Rx before "RX_TIMEOUT" expires
}__packed app_t;

typedef struct
{
    uint16_t		canlen;							// < Rx message length from CAN
	uint8_t			canbuf[CANBUFFLEN];				// < for RX from CAN
}__packed can_t;

typedef struct
{
    uint16_t		usartlen;						// < Rx message length from Usart
	uint8_t			usartbuf[USARTBUFFLEN];			// < for RX from Usart
	uint32_t		usartrx_time;					// < Time in which the usart remains in Rx before "RX_TIMEOUT" expires
}__packed usart_t;

app_t 	app;
can_t 	can1app, can2app;
usart_t	usart2app, usart3app;

#define num_digital_in				(16U-1U)			//Number of digital inputs
#if (IMU_PRESENT==1)
	extern uint8_t MagCalRequest, AccCalRequest, ActRecRequest, GyroCalRequest;
#endif
#if (VOC_SENSOR_PRESENT==1)
	extern bool load_baseline, baseline_loaded, CCS811_Save_Baseline_Reserved;
#endif
uint8_t Message_Length;

#define DEVICE_ID   (0xDECA0130)        			//!< DW1000 MP device ID

/****************************************************************************//**
 * 								Types definitions
 *******************************************************************************/
//typedef unsigned char	uint8;
//typedef unsigned long uint32;
typedef uint64_t	uint64 ;
typedef int64_t		int64 ;

#ifndef FALSE
#define FALSE			0
#endif

#ifndef TRUE
#define TRUE			1
#endif

typedef enum
{
    LED_PC6, //Status Green LED
    LED_PC7, //Button Blue LED
    LED_PC8, //LED7
    LED_PC9, //LED8
    LED_ALL,
    LEDn
} led_t;

typedef enum sensor_type
{
	IMU = 0x01,					//IMU device number
	PRESSURE_SENSOR = 0x02,		//PRESSURE_SENSOR device number
	HUMIDITY_SENSOR = 0x03,		//HUMIDITY_SENSOR device number
	UVx_SENSOR = 0x04,			//UVx_SENSOR device number
	VOC_SENSOR = 0x05,			//VOC_SENSOR device number
	PARTICULATE_SENSOR = 0x06	//PARTICULATE_SENSOR device number
} SENSOR_TYPE;

typedef enum sensor_status
{
	PRESSURE_SENSOR_OK = 0x00000001,	//PRESSURE_SENSOR check passed
	HUMIDITY_SENSOR_OK = 0x00000002,	//HUMIDITY_SENSOR check passed
	UVx_SENSOR_OK = 0x00000004,			//UVx_SENSOR check passed
	ENV_SENSOR_OK = 0x00000007,			//One or more Environmental sensor Ok
	VOC_SENSOR_OK = 0x00000008,			//VOC_SENSOR check passed
	PM_SENSOR_OK =  0x00000010,			//PARTICULATE_SENSOR check passed
	GAS_SENSORS_OK = 0x00000020,		//Gas Sensors board check passed
	GAS_SENSORS_FULL_OK = 0x00000040,	//Full Equipped Gas Sensors board check passed
	IMU_OK = 0x00000080,				//IMU check passed
	PRESSURE_SENSOR_INST = 0x00010000,	//PRESSURE_SENSOR Installed
	HUMIDITY_SENSOR_INST = 0x00020000,	//HUMIDITY_SENSOR Installed
	UVx_SENSOR_INST = 0x00040000,		//UVx_SENSOR Installed
	VOC_SENSOR_INST = 0x00080000,		//VOC_SENSOR Installed
	PM_SENSOR_INST =  0x00100000,		//PARTICULATE_SENSOR Installed
	GAS_SENSORS_INST =0x00200000,		//Gas Sensors Board Installed
	GAS_SENSORS_FULL_INST =0x00400000,	//Full Equipped Gas Sensors Board Installed
	IMU_INST = 0x00800000				//IMU Installed
} SENSOR_STATUS;

typedef struct board_master_data
{
	uint32_t DeviceName;
	uint8_t DeviceName_offset;	//offset from 0x0803F800: 0x08
	uint32_t HW_Version;
	uint8_t HW_Version_offset;	//offset from 0x0803F800: 0x0C
	uint32_t SW_Version;
	uint8_t SW_Version_offset;	//offset from 0x0803F800: 0x10
	uint32_t Vendor_ID;
	uint8_t Vendor_ID_offset;	//offset from 0x0803F800: 0x14
	uint32_t Prdct_Code;
	uint8_t Prdct_Code_offset;	//offset from 0x0803F800: 0x18
	uint32_t Rev_Number;
	uint8_t Rev_Number_offset;	//offset from 0x0803F800: 0x1C
	uint32_t Ser_Number;
	uint8_t Ser_Number_offset;	//offset from 0x0803F800: 0x20
} BOARD_MASTER_DATA;

typedef struct board_status
{
	uint32_t s0;		//In this application used to store the VOC Sensor BaseLine values
	uint8_t s0_offset;	//offset from 0x0803F800: 0x24
	uint32_t s1;		//In this application used to store the total uptime timer
	uint8_t s1_offset;	//offset from 0x0803F800: 0x28
	uint32_t s2;		//In this application used to store the SPS30 fan cleaning time interval
	uint8_t s2_offset;	//offset from 0x0803F800: 0x2C
	uint32_t s3;		//In this application used to store the LPS25HB Temperature calibration value
	uint8_t s3_offset;	//offset from 0x0803F800: 0x30
	uint32_t s4;		//In this application used to store the LPS25HB Pressure calibration value
	uint8_t s4_offset;	//offset from 0x0803F800: 0x34
	uint32_t s5;		//In this application used to store the HTS221 Humidity calibration value
	uint8_t s5_offset;	//offset from 0x0803F800: 0x38
	uint32_t s6;		//In this application used to store the WeatherF_DeltaP value in WeatherForecast function
	uint8_t s6_offset;	//offset from 0x0803F800: 0x3C
	uint32_t s7;		//In this application used to store the smooth RTC digital calibration value
	uint8_t s7_offset;	//offset from 0x0803F800: 0x40
	uint32_t s8;		//In this application used to store the RTC Asinch Prediv value
	uint8_t s8_offset;	//offset from 0x0803F800: 0x44
	uint32_t s9;		//Bit 0..7: CH2O_Corr; Bit 7..15: O3_Corr; Bit 16..23: NO2_Corr; Bit 24..32: NH3_Corr
	uint8_t s9_offset;	//offset from 0x0803F800: 0x48
	uint32_t sa;		//Bit 0..7: CO_Corr; Bit 7..15: SO2_Corr; Bit 16..23: C6H6_Corr; Bit 24..32: Spare_Corr
	uint8_t sa_offset;	//offset from 0x0803F800: 0x4C
	uint32_t sb;		//Bit 0..32: MiCS_6814_CO_Ro
	uint8_t sb_offset;	//offset from 0x0803F800: 0x50
	uint32_t sc;		//Bit 0..32: MiCS_6814_NH3_Ro
	uint8_t sc_offset;	//offset from 0x0803F800: 0x54
	uint32_t sd;		//Bit 0..32: MiCS_6814_NO2_Ro
	uint8_t sd_offset;	//offset from 0x0803F800: 0x58
	uint32_t se;		//Bit 0..16: MSL: Altitude in meters of the city where the device is located
	uint8_t se_offset;	//offset from 0x0803F800: 0x5C
	uint32_t sf;
	uint8_t sf_offset;	//offset from 0x0803F800: 0x60
} BOARD_STATUS;

typedef struct flash_data_org
{
	uint32_t b_date;
	uint8_t b_date_offset;		//offset from 0x0803F800: 0x00
	uint32_t b_time;
	uint8_t b_time_offset;		//offset from 0x0803F800: 0x04
	BOARD_MASTER_DATA b_mdata;
	uint8_t b_mdata_offset;		//offset from 0x0803F800: 0x08
	BOARD_STATUS b_status;
	uint8_t b_status_offset;	//offset from 0x0803F800: 0x24
} FLASH_DATA_ORG;

SENSOR_TYPE Sensor_Type;
SENSOR_STATUS Sensor_Status;

HAL_StatusTypeDef MCP23017_status, LSM9DS1_status, LPS25HB_status, HTS221_status, VEML6075_status;
HAL_StatusTypeDef CCS811_status, SPS30_status, ANLG_status, TLCD_status, GLCD_status;
bool Test_Mode, leds_test, timer5s_expired, update_5s, conversion_ended, RiskReport;
bool input_changed, I2C_done, can_tx_done, refresh, WarmUpPeriod_expired;
bool display_imu_data, send_lcl_imu_data, send_lcl_imu_data_to_ble, lcl_imu_data_rdy;
bool display_prs_data, send_lcl_prs_data, lcl_prs_data_rdy;
bool display_uvx_data, send_lcl_uvx_data, lcl_uvx_data_rdy;
bool display_hum_data, send_lcl_hum_data, lcl_hum_data_rdy;
bool display_voc_data, send_lcl_voc_data, lcl_voc_data_rdy;
bool display_pms_data, send_lcl_pms_data, lcl_pms_data_rdy;
bool display_gas_data, send_lcl_gas_data, lcl_gas_data_rdy;
bool service_timer0_expired;
bool update_1s, update1h, update1d;
uint8_t t_flip, NumberOfDevices, PreviousPage;
uint8_t Z_forecast, forecast, Gas_AQI, AVG_Gas_AQI, AVG_PMx_AQI;
uint16_t stby_timer, stby_timer_timeout;
uint16_t led_pc6_timer, led_pc7_timer, led_pc8_timer, led_pc9_timer;
uint16_t service_timer0, service_timer3;	//Service_timer3 uses 5s timer3 tick
uint32_t SensorStatusReg, AnlgOvflStatusReg;
uint8_t internal_notifies_0, internal_notifies_1;
volatile uint32_t update_16Hz;
volatile uint32_t update_25Hz;
volatile uint32_t update_50Hz;
volatile uint32_t update_100Hz;
volatile uint32_t AC_TimeStamp, AR_TimeStamp;
char ReadyDevices[52];			//Will be filled in AB_Init() function whit the initialized device list
uint8_t BakUpRTC_Data[RTC_BKUP_SIZE];

/****************************************************************************//**
 * 								port function prototypes
 *******************************************************************************/
void Write_Flash(uint32_t data, uint8_t f_offset);
void Read_Flash(uint32_t *data, uint8_t f_offset);
void enter_stby_mode(void);
void NVIC_SetVectorTable(uint32_t NVIC_VectTab, uint32_t Offset);
void jump_to_bootloader(void);
void RunActivePartition(uint32_t AppAddress);
int size(uint8_t *ptr);
uint8_t InRange(uint16_t min, uint16_t max, uint16_t value);
int xtoi(char *hexstring);
uint8_t FlipFlop(bool ResetFF, uint8_t mask, uint8_t period);
float32_t approxMovingAverage(float32_t avg, float32_t new_sample, uint32_t N, float32_t CorrectionFactor);
uint8_t ByteToBcd(uint8_t Value);
int32_t max(int32_t args, ...);
int32_t min(int32_t args, ...);
int usleep(useconds_t usec);
void Sleep(uint32_t Delay);
unsigned long portGetTickCnt(void);
void led_on (led_t led);
void led_off (led_t led);
void led_toggle (led_t led);
void write_port(uint16_t port_number, uint8_t value);

/****************************************************************************//**
 * 								CallBacks section
 *******************************************************************************/
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_SYSTICK_Callback();
void HAL_RTCEx_RTCEventCallback(RTC_HandleTypeDef *hrtc);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle);

/****************************************************************************//**
 * 								IRQ section
 *******************************************************************************/
void process_timer1_irq(void);
void button_manage(void);
void process_pn_menu_irq(void);
void process_timer3_irq(void);
void process_timer7_irq(void);
void process_IO_Expander_irq(void);
uint8_t process_CAN_RX_irq(CAN_HandleTypeDef* hcan, CAN_RxHeaderTypeDef Buf_Head, uint8_t* Buf_Data, uint32_t Len_Head, uint32_t Len_Data);
uint8_t process_CAN_TX_irq(CAN_HandleTypeDef* hcan);
uint8_t process_ADC_irq(ADC_HandleTypeDef* hadc);
uint8_t process_USART_RX_irq(UART_HandleTypeDef* huart, uint8_t* Buf, uint32_t Len);
uint8_t process_USART_TX_irq(UART_HandleTypeDef *UartHandle);

/*! ------------------------------------------------------------------------------------------------------------------
 * USB report section
 */
int usb_ready(void);
int usart_ready(UART_HandleTypeDef *UartHandle);
HAL_StatusTypeDef 	flush_report_buff(void);

typedef struct
{
	uint8_t *buf;
	int head;
	int tail;
	int maxLen;
} circBuf_t;

extern uint16_t local_buff_length;
uint16_t local_buff_offset;
uint8_t circBufPush(circBuf_t* c, uint8_t* data, uint16_t Len);
uint8_t circBufPop(circBuf_t* c, uint8_t* data, uint16_t Len);
HAL_StatusTypeDef flush_local_buff(void);

#endif /* PORT_H_ */

/*
 * Taken from the Linux Kernel
 *
 */
#ifndef _LINUX_CIRC_BUF_H
#define _LINUX_CIRC_BUF_H 1

/*struct circ_buf
{
	char *buf;
	int head;
	int tail;
};*/

/* Return count in buffer.  */
#define CIRC_CNT(head,tail,size) (((head) - (tail)) & ((size)-1))

/* Return space available, 0..size-1.  We always leave one free char
   as a completely full buffer has head == tail, which is the same as
   empty.  */
#define CIRC_SPACE(head,tail,size) CIRC_CNT((tail),((head)+1),(size))

/* Return count up to the end of the buffer.  Carefully avoid
   accessing head and tail more than once, so they can change
   underneath us without returning inconsistent results.  */
#define CIRC_CNT_TO_END(head,tail,size) \
	({int end = (size) - (tail); \
	  int n = ((head) + end) & ((size)-1); \
	  n < end ? n : end;})

/* Return space available up to the end of the buffer.  */
#define CIRC_SPACE_TO_END(head,tail,size) \
	({int end = (size) - 1 - (head); \
	  int n = (end + (tail)) & ((size)-1); \
	  n <= end ? n : end+1;})

#endif /* _LINUX_CIRC_BUF_H  */
