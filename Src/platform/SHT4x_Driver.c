/*
 * SHT4x_Driver.c
 *
 *  Created on: Sep 11, 2023
 *      Author: Tommaso Sabatini
 */

/* Includes ------------------------------------------------------------------*/
#include "platform/SHT4x_Driver.h"

static uint8_t sht4x_cmd_measure = SHT4X_CMD_MEASURE_HPM;
//static uint16_t sht4x_cmd_measure_delay_us = SHT4X_MEASUREMENT_DURATION_USEC;
const char * SHT4x_DRIVER_VERSION_STR = "5.3.0";

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize all hard- and software components that are needed for the I2C
 * communication.
 */
/*void SHT4x_i2c_init(void)
{
//	Done by CubeMX in MX_I2C1_Init(void) function
} */

/**
 * Release all resources initialized by SHT4x_i2c_init().
 */
/*void SHT4x_i2c_release(void)
{
} */

/**
 * Select the current i2c bus by index.
 * All following i2c operations will be directed at that bus.
 *
 * THE IMPLEMENTATION IS OPTIONAL ON SINGLE-BUS SETUPS (all sensors on the same
 * bus)
 *
 * @param bus_idx   Bus index to select
 * @returns         0 on success, an error code otherwise
 */
/*SHT4x_Error_et SHT4x_i2c_select_bus(uint8_t bus_idx)
{
} */

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
SHT4x_Error_et SHT4x_i2c_read(uint8_t address, uint8_t* data, uint16_t count)
{
	return (int8_t)HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(address << 1), data, count, 100);
}

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
SHT4x_Error_et SHT4x_i2c_write(uint8_t address, const uint8_t* data, uint16_t count)
{
	return (int8_t)HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(address << 1), (uint8_t*)data, count, 100);
}

/**
 * Sleep for a given number of microseconds. The function should delay the
 * execution for at least the given time, but may also sleep longer.
 *
 * @param useconds the sleep time in microseconds
 */
void SHT4x_sleep_usec(uint32_t useconds)
{
	usleep(useconds);
}

/**
 * SHT4x_bytes_to_uint16_t() - Convert an array of bytes to an uint16_t
 *
 * Convert an array of bytes received from the sensor in big-endian/MSB-first
 * format to an uint16_t value in the correct system-endianness.
 *
 * @param bytes An array of at least two bytes (MSB first)
 * @return      The byte array represented as uint16_t
 */
uint16_t SHT4x_bytes_to_uint16_t(const uint8_t* bytes)
{
	return (uint16_t)bytes[0] << 8 | (uint16_t)bytes[1];
}

/**
 * SHT4x_bytes_to_uint32_t() - Convert an array of bytes to an uint32_t
 *
 * Convert an array of bytes received from the sensor in big-endian/MSB-first
 * format to an uint32_t value in the correct system-endianness.
 *
 * @param bytes An array of at least four bytes (MSB first)
 * @return      The byte array represented as uint32_t
 */
uint32_t SHT4x_bytes_to_uint32_t(const uint8_t* bytes)
{
	return (uint32_t)bytes[0] << 24 | (uint32_t)bytes[1] << 16 |
		   (uint32_t)bytes[2] << 8 | (uint32_t)bytes[3];
}

/**
 * SHT4x_bytes_to_float() - Convert an array of bytes to a float
 *
 * Convert an array of bytes received from the sensor in big-endian/MSB-first
 * format to an float value in the correct system-endianness.
 *
 * @param bytes An array of at least four bytes (MSB first)
 * @return      The byte array represented as float
 */
float SHT4x_bytes_to_float(const uint8_t* bytes)
{
    union
	{
    	uint32_t u32_value;
    	float float32;
    } tmp;

    tmp.u32_value = SHT4x_bytes_to_uint32_t(bytes);
    return tmp.float32;
}

uint8_t SHT4x_common_generate_crc(const uint8_t* data, uint16_t count)
{
	uint16_t current_byte;
	uint8_t crc = SHT4X_CRC8_INIT;
	uint8_t crc_bit;

	/* calculates 8-Bit checksum with given polynomial */
	for (current_byte = 0; current_byte < count; ++current_byte)
	{
		crc ^= (data[current_byte]);
		for (crc_bit = 8; crc_bit > 0; --crc_bit)
		{
			if (crc & 0x80)
				crc = (crc << 1) ^ SHT4X_CRC8_POLYNOMIAL;
			else
				crc = (crc << 1);
		}
	}
   	return crc;
}

int8_t SHT4x_common_check_crc(const uint8_t* data, uint16_t count, uint8_t checksum)
{
	if (SHT4x_common_generate_crc(data, count) != checksum)
		return SHT4x_ERROR;
	return SHT4x_OK;
}

/**
 * SHT4x_i2c_general_call_reset() - Send a general call reset.
 *
 * @warning This will reset all attached I2C devices on the bus which support
 *          general call reset.
 *
 * @return  NO_ERROR on success, an error code otherwise
 */
SHT4x_Error_et SHT4x_i2c_general_call_reset(void)
{
	const uint8_t data = 0x06;
	return SHT4x_i2c_write(0, &data, (uint16_t)sizeof(data));
}

/**
 * SHT4x_fill_cmd_send_buf() - create the i2c send buffer for a command and
 *                                 a set of argument words. The output buffer
 *                                 interleaves argument words with their
 *                                 checksums.
 * @buf:        The generated buffer to send over i2c. Then buffer length must
 *              be at least SHT4x_COMMAND_LEN + num_args *
 *              (SHT4x_WORD_SIZE + CRC8_LEN).
 * @cmd:        The i2c command to send. It already includes a checksum.
 * @args:       The arguments to the command. Can be NULL if none.
 * @num_args:   The number of word arguments in args.
 *
 * @return      The number of bytes written to buf
 */
uint16_t SHT4x_fill_cmd_send_buf(uint8_t* buf, uint16_t cmd, const uint16_t* args, uint8_t num_args)
{
	uint8_t crc;
	uint8_t i;
	uint16_t idx = 0;

	buf[idx++] = (uint8_t)((cmd & 0xFF00) >> 8);
	buf[idx++] = (uint8_t)((cmd & 0x00FF) >> 0);

	for (i = 0; i < num_args; ++i)
	{
		buf[idx++] = (uint8_t)((args[i] & 0xFF00) >> 8);
		buf[idx++] = (uint8_t)((args[i] & 0x00FF) >> 0);

		crc = SHT4x_common_generate_crc((uint8_t*)&buf[idx - 2], SHT4X_WORD_SIZE);
		buf[idx++] = crc;
	}
	return idx;
}

/**
 * SHT4x_i2c_read_words() - read data words from sensor
 *
 * @address:    Sensor i2c address
 * @data_words: Allocated buffer to store the read words.
 *              The buffer may also have been modified on STATUS_FAIL return.
 * @num_words:  Number of data words to read (without CRC bytes)
 *
 * @return      NO_ERROR on success, an error code otherwise
 */
SHT4x_Error_et SHT4x_i2c_read_words(uint8_t address, uint16_t* data_words, uint16_t num_words)
{
	SHT4x_Error_et ret;
	uint8_t i;
	const uint8_t* word_bytes;

	ret = SHT4x_i2c_read_words_as_bytes(address, (uint8_t*)data_words, num_words);
	if (ret != SHT4x_OK)
		return ret;

	for (i = 0; i < num_words; ++i)
	{
		word_bytes = (uint8_t*)&data_words[i];
		data_words[i] = ((uint16_t)word_bytes[0] << 8) | word_bytes[1];
    }

	return SHT4x_OK;
}

/**
 * SHT4x_i2c_read_words_as_bytes() - read data words as byte-stream from
 *                                       sensor
 *
 * Read bytes without adjusting values to the uP's word-order.
 *
 * @address:    Sensor i2c address
 * @data:       Allocated buffer to store the read bytes.
 *              The buffer may also have been modified on STATUS_FAIL return.
 * @num_words:  Number of data words(!) to read (without CRC bytes)
 *              Since only word-chunks can be read from the sensor the size
 *              is still specified in sensor-words (num_words = num_bytes *
 *              SHT4x_WORD_SIZE)
 *
 * @return      NO_ERROR on success, an error code otherwise
 */
SHT4x_Error_et SHT4x_i2c_read_words_as_bytes(uint8_t address, uint8_t* data, uint16_t num_words)
{
	SHT4x_Error_et ret;
	uint16_t i, j;
	uint16_t size = num_words * (SHT4X_WORD_SIZE + SHT4X_CRC8_LEN);
	uint16_t word_buf[SHT4X_MAX_BUFFER_WORDS];
	uint8_t* const buf8 = (uint8_t*)word_buf;

	ret = SHT4x_i2c_read(address, buf8, size);
	if (ret != SHT4x_OK)
		return ret;

	/* check the CRC for each word */
	for (i = 0, j = 0; i < size; i += SHT4X_WORD_SIZE + SHT4X_CRC8_LEN)
	{
		ret = SHT4x_common_check_crc(&buf8[i], SHT4X_WORD_SIZE, buf8[i + SHT4X_WORD_SIZE]);
		if (ret != SHT4x_OK)
			return ret;

		data[j++] = buf8[i];
		data[j++] = buf8[i + 1];
	}

	return SHT4x_OK;
}

/**
 * SHT4x_i2c_write_cmd() - writes a command to the sensor
 * @address:    Sensor i2c address
 * @command:    Sensor command
 *
 * @return      NO_ERROR on success, an error code otherwise
 */
SHT4x_Error_et SHT4x_i2c_write_cmd(uint8_t address, uint16_t command)
{
	uint8_t buf[SHT4X_COMMAND_SIZE];

	SHT4x_fill_cmd_send_buf(buf, command, NULL, 0);
	return SHT4x_i2c_write(address, buf, SHT4X_COMMAND_SIZE);
}

/**
 * SHT4x_i2c_write_cmd_with_args() - writes a command with arguments to the
 *                                       sensor
 * @address:    Sensor i2c address
 * @command:    Sensor command
 * @data:       Argument buffer with words to send
 * @num_words:  Number of data words to send (without CRC bytes)
 *
 * @return      NO_ERROR on success, an error code otherwise
 */
SHT4x_Error_et SHT4x_i2c_write_cmd_with_args(uint8_t address, uint16_t command, const uint16_t* data_words, uint16_t num_words)
{
	uint8_t buf[SHT4X_MAX_BUFFER_WORDS];
	uint16_t buf_size;

	buf_size = SHT4x_fill_cmd_send_buf(buf, command, data_words, num_words);
	return SHT4x_i2c_write(address, buf, buf_size);
}

/**
 * SHT4x_i2c_delayed_read_cmd() - send a command, wait for the sensor to
 *                                    process and read data back
 * @address:    Sensor i2c address
 * @cmd:        Command
 * @delay:      Time in microseconds to delay sending the read request
 * @data_words: Allocated buffer to store the read data
 * @num_words:  Data words to read (without CRC bytes)
 *
 * @return      NO_ERROR on success, an error code otherwise
 */
SHT4x_Error_et SHT4x_i2c_delayed_read_cmd(uint8_t address, uint16_t cmd, uint32_t delay_us, uint16_t* data_words, uint16_t num_words)
{
	SHT4x_Error_et ret;
	uint8_t buf[SHT4X_COMMAND_SIZE];

	SHT4x_fill_cmd_send_buf(buf, cmd, NULL, 0);
	ret = SHT4x_i2c_write(address, buf, SHT4X_COMMAND_SIZE);
	if (ret != SHT4x_OK)
		return ret;

	if (delay_us)
		SHT4x_sleep_usec(delay_us);

	return SHT4x_i2c_read_words(address, data_words, num_words);
}

/**
 * SHT4x_i2c_read_cmd() - reads data words from the sensor after a command
 *                            is issued
 * @address:    Sensor i2c address
 * @cmd:        Command
 * @data_words: Allocated buffer to store the read data
 * @num_words:  Data words to read (without CRC bytes)
 *
 * @return      NO_ERROR on success, an error code otherwise
 */
SHT4x_Error_et SHT4x_i2c_read_cmd(uint8_t address, uint16_t cmd, uint16_t* data_words, uint16_t num_words)
{
    return SHT4x_i2c_delayed_read_cmd(address, cmd, 0, data_words, num_words);
}

/**
 * Detects if a sensor is connected by reading out the ID register.
 * If the sensor does not answer or if the answer is not the expected value,
 * the test fails.
 *
 * @return 0 if a sensor was detected
 */
SHT4x_Error_et SHT4x_probe(void)
{
	uint32_t serial;

	return SHT4x_read_serial(&serial);
}

/**
 * Starts a measurement and then reads out the results. This function blocks
 * while the measurement is in progress. The duration of the measurement depends
 * on the sensor in use, please consult the datasheet.
 * Temperature is returned in [degree Celsius], multiplied by 1000,
 * and relative humidity in [percent relative humidity], multiplied by 1000.
 *
 * @param temperature   the address for the result of the temperature
 * measurement
 * @param humidity      the address for the result of the relative humidity
 * measurement
 * @return              0 if the command was successful, else an error code.
 */
SHT4x_Error_et SHT4x_measure_blocking_read(int32_t* temperature, int32_t* humidity)
{
	SHT4x_Error_et ret;

	ret = SHT4x_measure();
	if (ret)
		return ret;
//	SHT4x_sleep_usec(sht4x_cmd_measure_delay_us);
	Sleep(10);

	return SHT4x_read(temperature, humidity);
}

/**
 * Starts a measurement in high precision mode. Use SHT4x_read() to read out the
 * values, once the measurement is done. The duration of the measurement depends
 * on the sensor in use, please consult the datasheet.
 *
 * @return     0 if the command was successful, else an error code.
 */
SHT4x_Error_et SHT4x_measure(void)
{
	return SHT4x_i2c_write(SHT4x_BADDR, &sht4x_cmd_measure, 1);
}

/**
 * Reads out the results of a measurement that was previously started by
 * SHT4x_measure(). If the measurement is still in progress, this function
 * returns an error.
 * Temperature is returned in [degree Celsius], multiplied by 1000,
 * and relative humidity in [percent relative humidity], multiplied by 1000.
 *
 * @param temperature   the address for the result of the temperature
 * measurement
 * @param humidity      the address for the result of the relative humidity
 * measurement
 * @return              0 if the command was successful, else an error code.
 */
SHT4x_Error_et SHT4x_read(int32_t* temperature, int32_t* humidity)
{
	uint16_t words[2];
	SHT4x_Error_et ret = SHT4x_i2c_read_words(SHT4x_BADDR, words, SHT4X_NUM_WORDS(words));
	/**
	 * formulas for conversion of the sensor signals, optimized for fixed point
	 * algebra:
	 * Temperature = 175 * S_T / 65535 - 45
	 * Relative Humidity = 125 * (S_RH / 65535) - 6
	 */
	*temperature = ((21875 * (int32_t)words[0]) >> 13) - 45000;
	*humidity = ((15625 * (int32_t)words[1]) >> 13) - 6000;

	return ret;
}

/**
 * Enable or disable the SHT's low power mode
 *
 * @param enable_low_power_mode 1 to enable low power mode, 0 to disable
 */
void SHT4x_enable_low_power_mode(uint8_t enable_low_power_mode)
{
	if (enable_low_power_mode)
	{
		sht4x_cmd_measure = SHT4X_CMD_MEASURE_LPM;
//		sht4x_cmd_measure_delay_us = SHT4X_MEASUREMENT_DURATION_LPM_USEC;
		Sleep(3);
	} else
	{
		sht4x_cmd_measure = SHT4X_CMD_MEASURE_HPM;
//		sht4x_cmd_measure_delay_us = SHT4X_MEASUREMENT_DURATION_USEC;
		Sleep(10);
	}
}

/**
 * Read out the serial number
 *
 * @param serial    the address for the result of the serial number
 * @return          0 if the command was successful, else an error code.
 */
SHT4x_Error_et SHT4x_read_serial(uint32_t* serial)
{
	const uint8_t cmd = SHT4X_CMD_READ_SERIAL;
	SHT4x_Error_et ret;
	uint16_t serial_words[SHT4X_NUM_WORDS(*serial)];

	ret = SHT4x_i2c_write(SHT4x_BADDR, &cmd, 1);
	if (ret)
		return ret;

	SHT4x_sleep_usec(SHT4X_CMD_DURATION_USEC);
	ret = SHT4x_i2c_read_words(SHT4x_BADDR, serial_words, SHT4X_NUM_WORDS(serial_words));
	*serial = ((uint32_t)serial_words[0] << 16) | serial_words[1];

	return ret;
}

/**
 * Return the driver version
 *
 * @return Driver version string
 */
const char* SHT4x_get_driver_version(void)
{
	return SHT4x_DRIVER_VERSION_STR;
}

/**
 * Returns the configured SHT4x address.
 *
 * @return SHT4x_ADDRESS
 */
uint8_t SHT4x_get_configured_address(void)
{
	return SHT4x_BADDR;
}

SHT4x_Error_et SHT4x_Get_Measurement(uint8_t B_Addr, SHT4x_MeasureTypeDef_st *Measurement_Value)
{
	int32_t Tout, Hout;

	if (SHT4x_measure_blocking_read(&Tout, &Hout))
		return SHT4x_ERROR;
    Measurement_Value->Tout = (int16_t)(Tout/100);
    Measurement_Value->Hout = (uint16_t)(Hout/100);

	return SHT4x_OK;
}

SHT4x_Error_et MX_SHT4x_Init(void)
{
	uint32_t Serial;
	int32_t temperature, humidity;

	Sleep(1);	//1ms maximal power-up time
	if (SHT4x_probe())
		return SHT4x_ERROR;

	if (SHT4x_read_serial(&Serial))
		return SHT4x_ERROR;

	if (SHT4x_measure_blocking_read(&temperature, &humidity))
		return SHT4x_ERROR;

	return SHT4x_OK;
}

#ifdef __cplusplus
}
#endif

