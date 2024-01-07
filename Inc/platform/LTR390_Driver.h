/*
 * LTR390_Driver.h
 *
 *  Created on: Oct 9, 2023
 *      Author: Tommaso Sabatini
 */

#ifndef PLATFORM_LTR390_DRIVER_H_
#define PLATFORM_LTR390_DRIVER_H_

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
//#define LTR390_BADDR	(uint8_t)0x53	//7-bit unshifted I2C Address
#define LTR390_BADDR	(uint8_t)0xA6	//7-bit shifted I2C Address

/************************************************************************/
/* LTR390 REGISTER DEFINITIONS                                          */
/************************************************************************/
/**
* @brief I2C Register Address
* REGISTER NAME
*/
#define LTR390_MAIN_CTRL	(uint8_t)0x00U	// Main control register
#define LTR390_MEAS_RATE	(uint8_t)0x04U	// Resolution and data rate
#define LTR390_GAIN			(uint8_t)0x05U	// ALS and UVS gain range
#define LTR390_PART_ID		(uint8_t)0x06U	// Part id/revision register
#define LTR390_MAIN_STATUS	(uint8_t)0x07U	// Main status register
#define LTR390_ALSDATA_0	(uint8_t)0x0DU	// ALS ADC measurement data, LSB
#define LTR390_ALSDATA_1	(uint8_t)0x0EU	// ALS ADC measurement data
#define LTR390_ALSDATA_2	(uint8_t)0x0FU	// ALS ADC measurement data, MSB
#define LTR390_UVSDATA_0	(uint8_t)0x10U	// UVS ADC measurement data, LSB
#define LTR390_UVSDATA_1	(uint8_t)0x11U	// UVS ADC measurement data
#define LTR390_UVSDATA_2	(uint8_t)0x12U	// UVS ADC measurement data, MSB
#define LTR390_INT_CFG		(uint8_t)0x19U	// Interrupt configuration
#define LTR390_INT_PST		(uint8_t)0x1AU	// Interrupt persistence setting
#define LTR390_THRESH_UP_0	(uint8_t)0x21U	// ALS/UVS interrupt upper threshold, LSB
#define LTR390_THRESH_UP_1	(uint8_t)0x22U	// ALS/UVS interrupt upper threshold, intervening bits
#define LTR390_THRESH_UP_2	(uint8_t)0x23U	// ALS/UVS interrupt upper threshold, MSB
#define LTR390_THRESH_LOW_0	(uint8_t)0x24U	// ALS/UVS interrupt lower threshold, LSB
#define LTR390_THRESH_LOW_1	(uint8_t)0x25U	// ALS/UVS interrupt lower threshold, intervening bits
#define LTR390_THRESH_LOW_2	(uint8_t)0x26U	// ALS/UVS interrupt lower threshold, MSB

/**
* @brief Registers Init Values.
* Values write in the registers in the MX_LPS22HB_Init() function
*/
//LTR390 Register Addresses Values
#define LTR390_MAIN_CTRL_VAL	(uint8_t)0x0AU	// 0x00 - Main control register (r/w)
#define LTR390_MEAS_RATE_VAL	(uint8_t)0x02U	// 0x04 - Resolution and data rate (r/w)
#define LTR390_GAIN_VAL			(uint8_t)0x00U	// 0x05 - ALS and UVS gain range (r/w)
#define LTR390_PART_ID_VAL		(uint8_t)0x00U	// 0x06 - Part id/revision register (r)
#define LTR390_MAIN_STATUS_VAL	(uint8_t)0x00U	// 0x07 - Main status register (r)
#define LTR390_ALSDATA_0_VAL	(uint8_t)0x00U	// 0x0D - ALS ADC measurement data, LSB (r)
#define LTR390_ALSDATA_1_VAL	(uint8_t)0x00U	// 0x0E - ALS ADC measurement data (r)
#define LTR390_ALSDATA_2_VAL	(uint8_t)0x00U	// 0x0F - ALS ADC measurement data, MSB (r)
#define LTR390_UVSDATA_0_VAL	(uint8_t)0x00U	// 0x10 - UVS ADC measurement data, LSB (r)
#define LTR390_UVSDATA_1_VAL	(uint8_t)0x00U	// 0x11 - UVS ADC measurement data (r)
#define LTR390_UVSDATA_2_VAL	(uint8_t)0x00U	// 0x12 - UVS ADC measurement data, MSB (r)
#define LTR390_INT_CFG_VAL		(uint8_t)0x00U	// 0x19 - Interrupt configuration (r/w)
#define LTR390_INT_PST_VAL		(uint8_t)0x00U	// 0x1A - Interrupt persistence setting (r/w)
#define LTR390_THRESH_UP_0_VAL	(uint8_t)0x00U	// 0x21 - ALS/UVS interrupt upper threshold, LSB (r/w)
#define LTR390_THRESH_UP_1_VAL	(uint8_t)0x00U	// 0x22 - ALS/UVS interrupt upper threshold, intervening bits (r/w)
#define LTR390_THRESH_UP_2_VAL	(uint8_t)0x00U	// 0x23 - ALS/UVS interrupt upper threshold, MSB (r/w)
#define LTR390_THRESH_LOW_0_VAL	(uint8_t)0x00U	// 0x24 - ALS/UVS interrupt lower threshold, LSB (r/w)
#define LTR390_THRESH_LOW_1_VAL	(uint8_t)0x00U	// 0x25 - ALS/UVS interrupt lower threshold, intervening bits (r/w)
#define LTR390_THRESH_LOW_2_VAL	(uint8_t)0x00U	// 0x26 - ALS/UVS interrupt lower threshold, MSB (r/w)

/**
 * Set resolution and sampling time of module, the sampling time must be greater than the time for collecting resolution
 * +------------+------------+------------+------------+------------+------------+------------+------------+
 * |    bit7    |    bit6    |    bit5    |    bit4    |    bit3    |    bit2    |    bit1    |    bit0    |
 * +------------+------------+------------+------------+------------+------------+------------+------------+
 * |  Reserved  |        ALS/UVS Resolution            |  Reserved  |   ALS/UVS Measurement Rate           |
 * +------------+--------------------------------------+------------+--------------------------------------+
 * | ALS/UVS Resolution       |000|20 Bit, Conversion time = 400ms                                         |
 * |                          |001|19 Bit, Conversion time = 200ms                                         |
 * |                          |010|18 Bit, Conversion time = 100ms(default)                                |
 * |                          |011|17 Bit, Conversion time = 50ms                                          |
 * |                          |100|16 Bit, Conversion time = 25ms                                          |
 * |                          |101|13 Bit, Conversion time = 12.5ms                                        |
 * |                          |110/111|Reserved                                                            |
 * +-------------------------------------------------------------------------------------------------------+
 * | ALS/UVS Measurement Rate |000|25ms                                                                    |
 * |                          |001|50ms                                                                    |
 * |                          |010|100ms (default)                                                         |
 * |                          |011|200ms                                                                   |
 * |                          |100|500ms                                                                   |
 * |                          |101|1000ms                                                                  |
 * |                          |110/111|2000ms                                                              |
 * +-------------------------------------------------------------------------------------------------------+
 */

/**
 * Set sensor gain
 * +------------+------------+------------+------------+------------+------------+------------+------------+
 * |    bit7    |    bit6    |    bit5    |    bit4    |    bit3    |    bit2    |    bit1    |    bit0    |
 * +------------+------------+------------+------------+------------+------------+------------+------------+
 * |                                    Reserved                    |          ALS/UVS Gain Range          |
 * +----------------------------------------------------------------+--------------------------------------+
 * | ALS/UVS Gain Range       |000|Gain Range: 1                                                           |
 * |                          |001|Gain Range: 3 (default)                                                 |
 * |                          |010|Gain Range: 6                                                           |
 * |                          |011|Gain Range: 9                                                           |
 * |                          |100|Gain Range: 18                                                          |
 * |                          |110/111|Reserved                                                            |
 * +-------------------------------------------------------------------------------------------------------+
 */

#ifdef __cplusplus
}
#endif

#endif /* PLATFORM_LTR390_DRIVER_H_ */
