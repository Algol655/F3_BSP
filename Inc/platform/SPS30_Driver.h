/*
 * Copyright (c) 2018, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SPS30_DRIVER_H
#define SPS30_DRIVER_H

// User include starts here
#include "port.h"
#include "i2c.h"
// User include ends here

#ifdef __cplusplus
extern "C" {
#endif

/*************************************************************************/
/*                          sps_git_version.h							 */
/*************************************************************************/
const char *SPS_DRV_VERSION_STR;

/*************************************************************************/
/*                          sensirion_arch_config.h						 */
/*************************************************************************/
/* Define the endianness of your architecture:
 * 0: little endian, 1: big endian
 * Use the following code to determine if unsure:
 * ```c
 * #include <stdio.h>
 *
 * int is_big_endian(void) {
 *     union {
 *         unsigned int u;
 *         char c[sizeof(unsigned int)];
 *     } e = { 0 };
 *     e.c[0] = 1;
 *
 *     return (e.i != 1);
 * }
 *
 * int main(void) {
 *     printf("Use #define SENSIRION_BIG_ENDIAN %d\n", is_big_endian());
 *
 *     return 0;
 * }
 * ```
 */
#define SENSIRION_BIG_ENDIAN 0

/**
 * The clock period of the i2c bus in microseconds. Increase this, if your GPIO
 * ports cannot support a 200 kHz output rate. (2 * 1 / 10usec == 200Khz)
 *
 * This is only relevant for the sw-i2c HAL (bit-banging on GPIO pins). The
 * pulse length is half the clock period, the number should thus be even.
 */
#define SENSIRION_I2C_CLOCK_PERIOD_USEC 10

/*************************************************************************/
/*                          sensirion_common.h							 */
/*************************************************************************/
#if SENSIRION_BIG_ENDIAN
#define be16_to_cpu(s) (s)
#define be32_to_cpu(s) (s)
#define be64_to_cpu(s) (s)
#define SENSIRION_WORDS_TO_BYTES(a, w) ()

#else /* SENSIRION_BIG_ENDIAN */

#define be16_to_cpu(s) (((uint16_t)(s) << 8) | (0xff & ((uint16_t)(s)) >> 8))
#define be32_to_cpu(s)                                                         \
    (((uint32_t)be16_to_cpu(s) << 16) | (0xffff & (be16_to_cpu((s) >> 16))))
#define be64_to_cpu(s)                                                         \
    (((uint64_t)be32_to_cpu(s) << 32) |                                        \
     (0xffffffff & ((uint64_t)be32_to_cpu((s) >> 32))))
/**
 * Convert a word-array to a bytes-array, effectively reverting the
 * host-endianness to big-endian
 * @a:  word array to change (must be (uint16_t *) castable)
 * @w:  number of word-sized elements in the array (SENSIRION_NUM_WORDS(a)).
 */
#define SENSIRION_WORDS_TO_BYTES(a, w)                                         \
    for (uint16_t *__a = (uint16_t *)(a), __e = (w), __w = 0; __w < __e;       \
         ++__w) {                                                              \
        __a[__w] = be16_to_cpu(__a[__w]);                                      \
    }
#endif /* SENSIRION_BIG_ENDIAN */

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof(*(x)))
#endif

#define CRC8_POLYNOMIAL 0x31
#define CRC8_INIT 0xFF
#define CRC8_LEN 1

#define SENSIRION_COMMAND_SIZE 2
#define SENSIRION_WORD_SIZE 2
#define SENSIRION_NUM_WORDS(x) (sizeof(x) / SENSIRION_WORD_SIZE)
#define SENSIRION_MAX_BUFFER_WORDS 32

#define SENSIRION_AUTOCLEAN_INTERVAL ((168*60*60)/APPLICATION_RUN_CYCLE)	//168 Hours

typedef enum
{
	SPS30_OK	= 0x00,
	SPS30_ERROR	= 0x01,
	SPS30_BUSY	= 0x02U,
	SPS30_TIMEOUT = 0x03U
} SPS30_Error_et;

uint8_t sensirion_common_generate_crc(uint8_t *data, uint16_t count);

SPS30_Error_et sensirion_common_check_crc(uint8_t *data, uint16_t count, uint8_t checksum);

/**
 * sensirion_fill_cmd_send_buf() - create the i2c send buffer for a command and
 *                                 a set of argument words. The output buffer
 *                                 interleaves argument words with their
 *                                 checksums.
 * @buf:        The generated buffer to send over i2c. Then buffer length must
 *              be at least SENSIRION_COMMAND_LEN + num_args *
 *              (SENSIRION_WORD_SIZE + CRC8_LEN).
 * @cmd:        The i2c command to send. It already includes a checksum.
 * @args:       The arguments to the command. Can be NULL if none.
 * @num_args:   The number of word arguments in args.
 *
 * @return      The number of bytes written to buf
 */
uint16_t sensirion_fill_cmd_send_buf(uint8_t *buf, uint16_t cmd, const uint16_t *args, uint8_t num_args);

/**
 * sensirion_i2c_read_words() - read data words from sensor
 *
 * @address:    Sensor i2c address
 * @data_words: Allocated buffer to store the read words.
 *              The buffer may also have been modified on SPS30_ERROR return.
 * @num_words:  Number of data words to read (without CRC bytes)
 *
 * @return      SPS30_OK on success, an error code otherwise
 */
SPS30_Error_et sensirion_i2c_read_words(uint8_t address, uint16_t *data_words, uint16_t num_words);

/**
 * sensirion_i2c_read_words_as_bytes() - read data words as byte-stream from
 *                                       sensor
 *
 * Read bytes without adjusting values to the uP's word-order.
 *
 * @address:    Sensor i2c address
 * @data:       Allocated buffer to store the read bytes.
 *              The buffer may also have been modified on SPS30_ERROR return.
 * @num_words:  Number of data words(!) to read (without CRC bytes)
 *              Since only word-chunks can be read from the sensor the size
 *              is still specified in sensor-words (num_words = num_bytes *
 *              SENSIRION_WORD_SIZE)
 *
 * @return      SPS30_OK on success, an error code otherwise
 */
SPS30_Error_et sensirion_i2c_read_words_as_bytes(uint8_t address, uint8_t *data, uint16_t num_words);

/**
 * sensirion_i2c_write_cmd() - writes a command to the sensor
 * @address:    Sensor i2c address
 * @command:    Sensor command
 *
 * @return      SPS30_OK on success, an error code otherwise
 */
SPS30_Error_et sensirion_i2c_write_cmd(uint8_t address, uint16_t command);

/**
 * sensirion_i2c_write_cmd_with_args() - writes a command with arguments to the
 *                                       sensor
 * @address:    Sensor i2c address
 * @command:    Sensor command
 * @data:       Argument buffer with words to send
 * @num_words:  Number of data words to send (without CRC bytes)
 *
 * @return      SPS30_OK on success, an error code otherwise
 */
SPS30_Error_et sensirion_i2c_write_cmd_with_args(uint8_t address, uint16_t command,
										  const uint16_t *data_words,
										  uint16_t num_words);

/**
 * sensirion_i2c_delayed_read_cmd() - send a command, wait for the sensor to
 *                                    process and read data back
 * @address:    Sensor i2c address
 * @cmd:        Command
 * @delay:      Time in microseconds to delay sending the read request
 * @data_words: Allocated buffer to store the read data
 * @num_words:  Data words to read (without CRC bytes)
 *
 * @return      SPS30_OK on success, an error code otherwise
 */
SPS30_Error_et sensirion_i2c_delayed_read_cmd(uint8_t address, uint16_t cmd,
									   uint32_t delay_us, uint16_t *data_words,
									   uint16_t num_words);
/**
 * sensirion_i2c_read_cmd() - reads data words from the sensor after a command
 *                            is issued
 * @address:    Sensor i2c address
 * @cmd:        Command
 * @data_words: Allocated buffer to store the read data
 * @num_words:  Data words to read (without CRC bytes)
 *
 * @return      SPS30_OK on success, an error code otherwise
 */
SPS30_Error_et sensirion_i2c_read_cmd(uint8_t address, uint16_t cmd,
							   uint16_t *data_words, uint16_t num_words);

/*************************************************************************/
/*                          sensirion_i2c.h								 */
/*************************************************************************/
/**
 * Release all resources initialized by sensirion_i2c_init().
 */
void sensirion_i2c_release(void);

/**
 * Execute one read transaction on the I2C bus, reading a given number of bytes.
 * If the device does not acknowledge the read command, an error shall be
 * returned.
 *
 * @param address 7-bit I2C address to read from
 * @param data    pointer to the buffer where the data is to be stored
 * @param count   number of bytes to read from I2C and store in the buffer
 * @returns 0 on success, error code otherwise
 */
SPS30_Error_et sensirion_i2c_read(uint8_t address, uint8_t *data, uint16_t count);

/**
 * Execute one write transaction on the I2C bus, sending a given number of
 * bytes. The bytes in the supplied buffer must be sent to the given address. If
 * the slave device does not acknowledge any of the bytes, an error shall be
 * returned.
 *
 * @param address 7-bit I2C address to write to
 * @param data    pointer to the buffer containing the data to write
 * @param count   number of bytes to read from the buffer and send over I2C
 * @returns 0 on success, error code otherwise
 */
SPS30_Error_et sensirion_i2c_write(uint8_t address, const uint8_t *data, uint16_t count);

/*************************************************************************/
/*                          	sps30.h									 */
/*************************************************************************/
#define SPS30_I2C_ADDRESS 0x69
#define SPS30_MAX_SERIAL_LEN 32
#define SPS30_MAX_PRODUCT_TYPE_LEN 8
#define SPS30_MEASUREMENT_DURATION_mSEC 1000	// 1s measurement intervals */
#define SPS30_RESET_DELAY_mSEC 50				// 50ms delay after resetting the sensor

#define SPS_CMD_START_MEASUREMENT 0x0010
#define SPS_CMD_START_MEASUREMENT_ARG 0x0300
#define SPS_CMD_STOP_MEASUREMENT 0x0104
#define SPS_CMD_READ_MEASUREMENT 0x0300
#define SPS_CMD_GET_DATA_READY 0x0202
#define SPS_CMD_AUTOCLEAN_INTERVAL 0x8004
#define SPS_CMD_GET_PRODUCT_TYPE 0xd002
#define SPS_CMD_GET_FIRMWARE_VERSION 0xd100
#define SPS_CMD_GET_SERIAL 0xd033
#define SPS_CMD_RESET 0xd304
#define SPS_CMD_START_MANUAL_FAN_CLEANING 0x5607
#define SPS_CMD_DELAY_mSEC 10
#define SPS_WRITE_DELAY_mSEC 15

typedef struct
{
	float32_t mc_1p0;
	float32_t mc_2p5;
	float32_t mc_4p0;
	float32_t mc_10p0;
	float32_t nc_0p5;
	float32_t nc_1p0;
	float32_t nc_2p5;
	float32_t nc_4p0;
	float32_t nc_10p0;
	float32_t typical_particle_size;
	float32_t mc_1p0_mean;	//24h average
	float32_t mc_2p5_mean;	//24h average
	float32_t mc_4p0_mean;	//24h average
	float32_t mc_10p0_mean;	//24h average
} SPS30_MeasureTypeDef_st;

/**
 * sps_get_driver_version() - Return the driver version
 * Return:  Driver version string
 */
const char *sps_get_driver_version(void);

/**
 * sps30_probe() - check if SPS sensor is available and initialize it
 *
 * Note that Pin 4 must be pulled to ground for the sensor to operate in i2c
 * mode (this driver). When left floating, the sensor operates in UART mode
 * which is not compatible with this i2c driver.
 *
 * Return:  0 on success, an error code otherwise
 */
SPS30_Error_et sps30_probe();

/**
 * sps30_read_product_type - read the product type
 * @product_type:  Memory where the product_type value is written into
 *
 * Return:  0 on success, an error code otherwise
 */
SPS30_Error_et sps30_read_product_type(char *product_type);

/**
 * sps30_read_firmware_version - read the firmware version
 * @major:  Memory where the firmware major version is written into
 * @minor:  Memory where the firmware minor version is written into
 *
 * Return:  0 on success, an error code otherwise
 */
SPS30_Error_et sps30_read_firmware_version(uint8_t *major, uint8_t *minor);

/**
 * sps30_get_serial() - retrieve the serial number
 *
 * Note that serial must be discarded when the return code is non-zero.
 *
 * @serial: Memory where the serial number is written into as hex string (zero
 *          terminated). Must be at least SPS30_MAX_SERIAL_LEN long.
 * Return:  0 on success, an error code otherwise
 */
SPS30_Error_et sps30_get_serial(char *serial);

/**
 * sps30_start_measurement() - start measuring
 *
 * Once the measurement is started, measurements are retrievable once per second
 * with sps30_read_measurement.
 *
 * Return:  0 on success, an error code otherwise
 */
SPS30_Error_et sps30_start_measurement();

/**
 * sps30_stop_measurement() - stop measuring
 *
 * Return:  0 on success, an error code otherwise
 */
SPS30_Error_et sps30_stop_measurement();

/**
 * sps30_read_datda_ready() - reads the current data-ready flag
 *
 * The data-ready flag indicates that new (not yet retrieved) measurements are
 * available
 *
 * @data_ready: Memory where the data-ready flag (0|1) is stored.
 * Return:      0 on success, an error code otherwise
 */
SPS30_Error_et sps30_read_data_ready(uint16_t *data_ready);

/**
 * sps30_read_measurement() - read a measurement
 *
 * Read the last measurement.
 *
 * Return:  0 on success, an error code otherwise
 */
SPS30_Error_et sps30_read_measurement(SPS30_MeasureTypeDef_st *measurement);

/**
 * sps30_get_fan_auto_cleaning_interval() - read the current(*) auto-cleaning
 * interval
 *
 * Note that interval_seconds must be discarded when the return code is
 * non-zero.
 *
 * (*) Note that due to a firmware bug, the reported interval is only updated on
 * sensor restart/reset. If the interval was thus updated after the last reset,
 * the old value is still reported. Power-cycle the sensor or call sps30_reset()
 * first if you need the latest value.
 *
 * @interval_seconds:   Memory where the interval in seconds is stored
 * Return:              0 on success, an error code otherwise
 */
SPS30_Error_et sps30_get_fan_auto_cleaning_interval(uint32_t *interval_seconds);

/**
 * sps30_set_fan_auto_cleaning_interval() - set the current auto-cleaning
 * interval
 *
 * @interval_seconds:   Value in seconds used to sets the auto-cleaning
 *                      interval, 0 to disable auto cleaning
 * Return:              0 on success, an error code otherwise
 */
SPS30_Error_et sps30_set_fan_auto_cleaning_interval(uint32_t interval_seconds);

/**
 * sps30_get_fan_auto_cleaning_interval_days() - convenience function to read
 * the current(*) auto-cleaning interval in days
 *
 * note that the value is simply cut, not rounded or calculated nicely, thus
 * using this method is not precise when used in conjunction with
 * sps30_set_fan_auto_cleaning_interval instead of
 * sps30_set_fan_auto_cleaning_interval_days
 *
 * Note that interval_days must be discarded when the return code is non-zero.
 *
 * (*) Note that due to a firmware bug, the reported interval is only updated on
 * sensor restart/reset. If the interval was thus updated after the last reset,
 * the old value is still reported. Power-cycle the sensor or call sps30_reset()
 * first if you need the latest value.
 *
 * @interval_days:  Memory where the interval in days is stored
 * Return:          0 on success, an error code otherwise
 */
SPS30_Error_et sps30_get_fan_auto_cleaning_interval_days(uint8_t *interval_days);

/**
 * sps30_set_fan_auto_cleaning_interval_days() - convenience function to set the
 * current auto-cleaning interval in days
 *
 * @interval_days:  Value in days used to sets the auto-cleaning interval, 0 to
 *                  disable auto cleaning
 * Return:          0 on success, an error code otherwise
 */
SPS30_Error_et sps30_set_fan_auto_cleaning_interval_days(uint8_t interval_days);

/**
 * sps30_start_manual_fan_cleaning() - Immediately trigger the fan cleaning
 *
 * Note that this command can only be run when the sensor is in measurement
 * mode, i.e. after sps30_start_measurement() without subsequent
 * sps30_stop_measurement().
 *
 * Return:          0 on success, an error code otherwise
 */
SPS30_Error_et sps30_start_manual_fan_cleaning();

/**
 * sps30_reset() - reset the SGP30
 *
 * The sensor reboots to the same state as before the reset but takes a few
 * seconds to resume measurements.
 *
 * The caller should wait at least SPS30_RESET_DELAY_USEC microseconds before
 * interacting with the sensor again in order for the sensor to restart.
 * Interactions with the sensor without this delay might fail.
 *
 * Note that the interface-select configuration is reinterpreted, thus Pin 4
 * must be pulled to ground during the reset period for the sensor to remain in
 * i2c mode.
 *
 * Return:          0 on success, an error code otherwise
 */
SPS30_Error_et sps30_reset();

SPS30_Error_et SPS30_Get_Measurement(SPS30_MeasureTypeDef_st *Measurement_Values);

/*******************************************************************************
* Function Name	: MX_SPS30_Init
* Description   : SPS30 Global init
*         		: I2C writing function
* Input       	: None
* Output      	: None
* Return      	: Status [SPS30_ERROR, SPS30_OK]
*******************************************************************************/
SPS30_Error_et MX_SPS30_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* SPS30_DRIVER_H */
