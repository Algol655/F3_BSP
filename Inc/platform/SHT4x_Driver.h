/*
 * SHT4x_Driver.h
 *
 *  Created on: Sep 10, 2023
 *      Author: Tommaso Sabatini
 */

#ifndef PLATFORM_SHT4X_DRIVER_H_
#define PLATFORM_SHT4X_DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
// User include starts here
#include "port.h"
#include "i2c.h"
// User include ends here

/**
* brief  I2C address.
*/
#define SHT4x_BADDR	(uint8_t)0x44		//7-bit unshifted I2C Address
//#define SHT4x_BADDR	(uint8_t)0x88	//7-bit shifted I2C Address

/**
* brief Device Register Commands.
  all measurement commands return T (CRC) RH (CRC)
*/
#define SHT4X_CMD_MEASURE_HPM_20_100	0x15	//activate heater with 20mW for 0.1s including a high precision measurement just before deactivation
#define SHT4X_CMD_MEASURE_HPM_20_1000	0x1E	//activate heater with 20mW for 1s including a high precision measurement just before deactivation
#define SHT4X_CMD_MEASURE_HPM_110_100	0x24	//activate heater with 110mW for 0.1s including a high precision measurement just before deactivation
#define SHT4X_CMD_MEASURE_HPM_110_1000	0x2F	//activate heater with 110mW for 1s including a high precision measurement just before deactivation
#define SHT4X_CMD_MEASURE_HPM_200_100	0x32	//activate heater with 200mW for 0.1s including a high precision measurement just before deactivation
#define SHT4X_CMD_MEASURE_HPM_200_1000	0x39	//activate heater with 200mW for 1s including a high precision measurement just before deactivation
#define SHT4X_CMD_SW_RESET				0x94	//Soft Reset
#define SHT4X_CMD_READ_SERIAL 			0x89	//Read Serial Number
#define SHT4X_CMD_MEASURE_LPM 			0xE0	//Measure T & RH with lowest precision (low repeatability)
#define SHT4X_CMD_MEASURE_MPM 			0xF6	//Measure T & RH with medium precision (medium repeatability)
#define SHT4X_CMD_MEASURE_HPM 			0xFD	//Measure T & RH with high precision (high repeatability)

#define SHT4X_CMD_DURATION_USEC				1000
#define SHT4X_MEASUREMENT_DURATION_USEC		10000	//10ms "high repeatability"
#define SHT4X_MEASUREMENT_DURATION_LPM_USEC	2500	//2.5ms "low repeatability"

#define SHT4X_CRC8_POLYNOMIAL			0x31
#define SHT4X_CRC8_INIT					0xFF
#define SHT4X_CRC8_LEN					1

#define SHT4X_COMMAND_SIZE				2
#define SHT4X_WORD_SIZE					2
#define SHT4X_NUM_WORDS(x) 				(sizeof(x) / SHT4X_WORD_SIZE)
#define SHT4X_MAX_BUFFER_WORDS			32

/**
* brief SHT4x Humidity & Temperature Min Max Init Values.	//Added By Me!!!
*/
#define SHT4x_UPPER_H_LIMIT	(100*10)
#define SHT4x_LOWER_H_LIMIT	(10*10)
#define SHT4x_UPPER_T_LIMIT	(80*10)
#define SHT4x_LOWER_T_LIMIT	(-40*10)

/**
* brief  SHT4x Measure Type definition.
*/
typedef struct
{
  int16_t Tout;
  int16_t Tout_DailyMin;
  int16_t Tout_DailyMax;
  uint16_t Hout;
  uint16_t Hout_DailyMin;
  uint16_t Hout_DailyMax;
} SHT4x_MeasureTypeDef_st;

/**
* brief  Error code type.
*/
typedef enum
{
  SHT4x_OK = (uint8_t)0,
  SHT4x_ERROR = (uint8_t)1,
  SHT4x_ERR_BAD_DATA = (uint8_t)2,
  SHT4x_CRC_FAIL = (uint8_t)3,
  SHT4x_UNKNOWN_DEVICE = (uint8_t)4
} SHT4x_Error_et;

/**
* brief  State type.
*/
typedef enum
{
  SHT4x_DISABLE = (uint8_t)0,
  SHT4x_ENABLE = !SHT4x_DISABLE
} SHT4x_State_et;

#define IS_SHT4x_State(MODE) ((MODE == SHT4x_ENABLE) || (MODE == SHT4x_DISABLE))

/**
* brief  Bit status type.
*/
typedef enum
{
  SHT4x_RESET = (uint8_t)0,
  SHT4x_SET = !SHT4x_RESET
} SHT4x_BitStatus_et;

#define IS_SHT4x_BitStatus(MODE) ((MODE == SHT4x_RESET) || (MODE == SHT4x_SET))

/** SHT4x_I2C_BUS_Functions_Prototypes
*/
//void SHT4x_i2c_init(void);
//void SHT4x_i2c_release(void);
//SHT4x_Error_et SHT4x_i2c_select_bus(uint8_t bus_idx);
SHT4x_Error_et SHT4x_i2c_read(uint8_t address, uint8_t* data, uint16_t count);
SHT4x_Error_et SHT4x_i2c_write(uint8_t address, const uint8_t* data, uint16_t count);
void SHT4x_sleep_usec(uint32_t useconds);

/** SHT4x_Common_Functions_Prototypes
*/
uint16_t SHT4x_bytes_to_uint16_t(const uint8_t* bytes);
uint32_t SHT4x_bytes_to_uint32_t(const uint8_t* bytes);
float SHT4x_bytes_to_float(const uint8_t* bytes);
uint8_t SHT4x_common_generate_crc(const uint8_t* data, uint16_t count);
int8_t SHT4x_common_check_crc(const uint8_t* data, uint16_t count, uint8_t checksum);
SHT4x_Error_et SHT4x_i2c_general_call_reset(void);
uint16_t SHT4x_fill_cmd_send_buf(uint8_t* buf, uint16_t cmd, const uint16_t* args, uint8_t num_args);
SHT4x_Error_et SHT4x_i2c_read_words(uint8_t address, uint16_t* data_words, uint16_t num_words);
SHT4x_Error_et SHT4x_i2c_read_words_as_bytes(uint8_t address, uint8_t* data, uint16_t num_words);
SHT4x_Error_et SHT4x_i2c_write_cmd(uint8_t address, uint16_t command);
SHT4x_Error_et SHT4x_i2c_write_cmd_with_args(uint8_t address, uint16_t command, const uint16_t* data_words, uint16_t num_words);
SHT4x_Error_et SHT4x_i2c_delayed_read_cmd(uint8_t address, uint16_t cmd, uint32_t delay_us, uint16_t* data_words, uint16_t num_words);
SHT4x_Error_et SHT4x_i2c_read_cmd(uint8_t address, uint16_t cmd, uint16_t* data_words, uint16_t num_words);

/** SHT4x_Exported_Functions
*/
SHT4x_Error_et SHT4x_probe(void);
SHT4x_Error_et SHT4x_measure_blocking_read(int32_t* temperature, int32_t* humidity);
SHT4x_Error_et SHT4x_measure(void);
SHT4x_Error_et SHT4x_read(int32_t* temperature, int32_t* humidity);
void SHT4x_enable_low_power_mode(uint8_t enable_low_power_mode);
SHT4x_Error_et SHT4x_read_serial(uint32_t* serial);
const char* SHT4x_get_driver_version(void);
uint8_t SHT4x_get_configured_address(void);
SHT4x_Error_et SHT4x_Get_Measurement(uint8_t B_Addr, SHT4x_MeasureTypeDef_st *Measurement_Value);
SHT4x_Error_et MX_SHT4x_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* PLATFORM_SHT4X_DRIVER_H_ */
