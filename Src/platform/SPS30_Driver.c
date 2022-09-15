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

#include "platform/SPS30_Driver.h"

/*************************************************************************/
/*                          sensirion_common.c							 */
/*************************************************************************/
uint8_t sensirion_common_generate_crc(uint8_t *data, uint16_t count)
{
	uint16_t current_byte;
	uint8_t crc = CRC8_INIT;
	uint8_t crc_bit;
	
	/* calculates 8-Bit checksum with given polynomial */
	for (current_byte = 0; current_byte < count; ++current_byte) {
		crc ^= (data[current_byte]);
		for (crc_bit = 8; crc_bit > 0; --crc_bit) {
			if (crc & 0x80)
				crc = (crc << 1) ^ CRC8_POLYNOMIAL;
			else
				crc = (crc << 1);
		}
	}
	
	return crc;
}

SPS30_Error_et sensirion_common_check_crc(uint8_t *data, uint16_t count,
                                  	  	  uint8_t checksum)
{
	if (sensirion_common_generate_crc(data, count) != checksum)
		return SPS30_ERROR;
	
	return SPS30_OK;
}

uint16_t sensirion_fill_cmd_send_buf(uint8_t *buf, uint16_t cmd,
                                     const uint16_t *args, uint8_t num_args)
{
	uint8_t crc;
	uint8_t i;
	uint16_t idx = 0;
	
	buf[idx++] = (uint8_t)((cmd & 0xFF00) >> 8);
	buf[idx++] = (uint8_t)((cmd & 0x00FF) >> 0);
	
	for (i = 0; i < num_args; ++i) {
		buf[idx++] = (uint8_t)((args[i] & 0xFF00) >> 8);
		buf[idx++] = (uint8_t)((args[i] & 0x00FF) >> 0);
	
		crc = sensirion_common_generate_crc((uint8_t *)&buf[idx - 2], SENSIRION_WORD_SIZE);
		buf[idx++] = crc;
	}
	
	return idx;
}

SPS30_Error_et sensirion_i2c_read_words_as_bytes(uint8_t address, uint8_t *data, uint16_t num_words)
{
	int16_t ret;
	uint16_t i, j;
	uint16_t size = num_words * (SENSIRION_WORD_SIZE + CRC8_LEN);
	uint16_t word_buf[SENSIRION_MAX_BUFFER_WORDS];
	uint8_t *const buf8 = (uint8_t *)word_buf;
	
	ret = sensirion_i2c_read(address, buf8, size);
	if (ret != SPS30_OK)
		return ret;
	
	/* check the CRC for each word */
	for (i = 0, j = 0; i < size; i += SENSIRION_WORD_SIZE + CRC8_LEN)
	{
	
		ret = sensirion_common_check_crc(&buf8[i], SENSIRION_WORD_SIZE,
										buf8[i + SENSIRION_WORD_SIZE]);
		if (ret != SPS30_OK)
			return ret;
	
		data[j++] = buf8[i];
		data[j++] = buf8[i + 1];
	}
	
	return SPS30_OK;
}

SPS30_Error_et sensirion_i2c_read_words(uint8_t address, uint16_t *data_words, uint16_t num_words)
{
	int16_t ret;
	uint8_t i;
	
	ret = sensirion_i2c_read_words_as_bytes(address, (uint8_t *)data_words,	num_words);
	if (ret != SPS30_OK)
		return ret;
	
	for (i = 0; i < num_words; ++i)
		data_words[i] = be16_to_cpu(data_words[i]);
	
	return SPS30_OK;
}

SPS30_Error_et sensirion_i2c_write_cmd(uint8_t address, uint16_t command)
{
	uint8_t buf[SENSIRION_COMMAND_SIZE];
	
	sensirion_fill_cmd_send_buf(buf, command, NULL, 0);
	
	return sensirion_i2c_write(address, buf, SENSIRION_COMMAND_SIZE);
}

SPS30_Error_et sensirion_i2c_write_cmd_with_args(uint8_t address, uint16_t command,
                                          const uint16_t *data_words,
                                          uint16_t num_words)
{
	uint8_t buf[SENSIRION_MAX_BUFFER_WORDS];
	uint16_t buf_size;
	
	buf_size = sensirion_fill_cmd_send_buf(buf, command, data_words, num_words);
	
	return sensirion_i2c_write(address, buf, buf_size);
}

SPS30_Error_et sensirion_i2c_delayed_read_cmd(uint8_t address, uint16_t cmd,
                                       	   	  uint32_t delay_us, uint16_t *data_words,
											  uint16_t num_words)
{
	int16_t ret;
	uint8_t buf[SENSIRION_COMMAND_SIZE];
	
	sensirion_fill_cmd_send_buf(buf, cmd, NULL, 0);
	ret = sensirion_i2c_write(address, buf, SENSIRION_COMMAND_SIZE);
	if (ret != SPS30_OK)
		return ret;
	
	if (delay_us)
		usleep(delay_us);
	
	return sensirion_i2c_read_words(address, data_words, num_words);
}

SPS30_Error_et sensirion_i2c_read_cmd(uint8_t address, uint16_t cmd, uint16_t *data_words, uint16_t num_words)
{
	return sensirion_i2c_delayed_read_cmd(address, cmd, 0, data_words, num_words);
}

/*************************************************************************/
/*                          sensirion_i2c.c								 */
/*************************************************************************/
/**
 * Release all resources initialized by sensirion_i2c_init().
 */
void sensirion_i2c_release(void) 
{

}

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
SPS30_Error_et sensirion_i2c_read(uint8_t address, uint8_t *data, uint16_t count)
{
	return (int8_t)HAL_I2C_Master_Receive(&hi2c2, (uint16_t)(address << 1), data, count, 100);
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
SPS30_Error_et sensirion_i2c_write(uint8_t address, const uint8_t *data, uint16_t count)
{
	return (int8_t)HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)(address << 1), (uint8_t *)data, count, 100);
}

/*************************************************************************/
/*                          sps30.c										 */
/*************************************************************************/
const char *sps_get_driver_version()
{
	return SPS_DRV_VERSION_STR;
}

SPS30_Error_et sps30_probe()
{
	char serial[SPS30_MAX_SERIAL_LEN];

	return sps30_get_serial(serial);
}

SPS30_Error_et sps30_read_product_type(char *product_type)
{
	uint16_t i;
	int16_t ret;
	union
	{
		char product_type[SPS30_MAX_PRODUCT_TYPE_LEN];
		uint16_t __enforce_alignment;
	} buffer;

	ret = sensirion_i2c_read_cmd(SPS30_I2C_ADDRESS, SPS_CMD_GET_PRODUCT_TYPE,
								(uint16_t *)buffer.product_type,
								SENSIRION_NUM_WORDS(buffer.product_type));
	if (ret != SPS30_OK)
		return ret;

	SENSIRION_WORDS_TO_BYTES(buffer.product_type, SENSIRION_NUM_WORDS(buffer.product_type));
	for (i = 0; i < SPS30_MAX_PRODUCT_TYPE_LEN; ++i)
	{
		product_type[i] = buffer.product_type[i];
		if (product_type[i] == '\0')
			return 0;
	}

	return 0;
}

SPS30_Error_et sps30_read_firmware_version(uint8_t *major, uint8_t *minor)
{
	uint16_t version;
	int16_t ret;
	
	ret = sensirion_i2c_read_cmd(SPS30_I2C_ADDRESS,	SPS_CMD_GET_FIRMWARE_VERSION, &version, 1);
	*major = (version & 0xff00) >> 8;
	*minor = (version & 0x00ff);
	
	return ret;
}

SPS30_Error_et sps30_get_serial(char *serial)
{
	uint16_t i;
	int16_t ret;
	union
	{
		char serial[SPS30_MAX_SERIAL_LEN];
		uint16_t __enforce_alignment;
	} buffer;
	
	ret = sensirion_i2c_read_cmd(SPS30_I2C_ADDRESS, SPS_CMD_GET_SERIAL,
								(uint16_t *)buffer.serial,
								SENSIRION_NUM_WORDS(buffer.serial));
	if (ret != SPS30_OK)
		return ret;
	
	SENSIRION_WORDS_TO_BYTES(buffer.serial, SENSIRION_NUM_WORDS(buffer.serial));
	for (i = 0; i < SPS30_MAX_SERIAL_LEN; ++i)
	{
		serial[i] = buffer.serial[i];
		if (serial[i] == '\0')
			return 0;
	}
	
	return 0;
}

SPS30_Error_et sps30_start_measurement()
{
	const uint16_t arg = SPS_CMD_START_MEASUREMENT_ARG;
	
	int16_t ret = sensirion_i2c_write_cmd_with_args(
		SPS30_I2C_ADDRESS, SPS_CMD_START_MEASUREMENT, &arg,
		SENSIRION_NUM_WORDS(arg));
	
	Sleep(SPS_CMD_DELAY_mSEC);
	
	return ret;
}

SPS30_Error_et sps30_stop_measurement()
{
	int16_t ret = sensirion_i2c_write_cmd(SPS30_I2C_ADDRESS, SPS_CMD_STOP_MEASUREMENT);
	Sleep(SPS_CMD_DELAY_mSEC);

	return ret;
}

SPS30_Error_et sps30_read_data_ready(uint16_t *data_ready)
{
	return sensirion_i2c_read_cmd(SPS30_I2C_ADDRESS, SPS_CMD_GET_DATA_READY,
								  data_ready, SENSIRION_NUM_WORDS(*data_ready));
}

SPS30_Error_et sps30_read_measurement(SPS30_MeasureTypeDef_st *measurement)
{
	int16_t ret;
	uint16_t idx;
	union
	{
		uint16_t u16_value[2];
		uint32_t u32_value;
		float32_t f32_value;
	} val, data[10];
	
	ret = sensirion_i2c_read_cmd(SPS30_I2C_ADDRESS, SPS_CMD_READ_MEASUREMENT,
								data->u16_value, SENSIRION_NUM_WORDS(data));
	if (ret != SPS30_OK)
		return ret;
	
	SENSIRION_WORDS_TO_BYTES(data->u16_value, SENSIRION_NUM_WORDS(data));
	
	idx = 0;
	val.u32_value = be32_to_cpu(data[idx].u32_value);
	measurement->mc_1p0 = val.f32_value;
	++idx;
	val.u32_value = be32_to_cpu(data[idx].u32_value);
	measurement->mc_2p5 = val.f32_value;
	++idx;
	val.u32_value = be32_to_cpu(data[idx].u32_value);
	measurement->mc_4p0 = val.f32_value;
	++idx;
	val.u32_value = be32_to_cpu(data[idx].u32_value);
	measurement->mc_10p0 = val.f32_value;
	++idx;
	val.u32_value = be32_to_cpu(data[idx].u32_value);
	measurement->nc_0p5 = val.f32_value;
	++idx;
	val.u32_value = be32_to_cpu(data[idx].u32_value);
	measurement->nc_1p0 = val.f32_value;
	++idx;
	val.u32_value = be32_to_cpu(data[idx].u32_value);
	measurement->nc_2p5 = val.f32_value;
	++idx;
	val.u32_value = be32_to_cpu(data[idx].u32_value);
	measurement->nc_4p0 = val.f32_value;
	++idx;
	val.u32_value = be32_to_cpu(data[idx].u32_value);
	measurement->nc_10p0 = val.f32_value;
	++idx;
	val.u32_value = be32_to_cpu(data[idx].u32_value);
	measurement->typical_particle_size = val.f32_value;
	
	return 0;
}

SPS30_Error_et sps30_get_fan_auto_cleaning_interval(uint32_t *interval_seconds)
{
	union 
	{
		uint16_t u16_value[2];
		uint32_t u32_value;
	} data;
	int16_t ret = sensirion_i2c_delayed_read_cmd(
		SPS30_I2C_ADDRESS, SPS_CMD_AUTOCLEAN_INTERVAL, SPS_CMD_DELAY_mSEC,
		data.u16_value, SENSIRION_NUM_WORDS(data.u16_value));
	if (ret != SPS30_OK)
		return ret;
	
	SENSIRION_WORDS_TO_BYTES(data.u16_value, SENSIRION_NUM_WORDS(data.u16_value));
	
	*interval_seconds = be32_to_cpu(data.u32_value);
	
	return 0;
}

SPS30_Error_et sps30_set_fan_auto_cleaning_interval(uint32_t interval_seconds)
{
	int16_t ret;
	const uint16_t data[] = {(uint16_t)((interval_seconds & 0xFFFF0000) >> 16),
							(uint16_t)(interval_seconds & 0x0000FFFF)};
	
	ret = sensirion_i2c_write_cmd_with_args(SPS30_I2C_ADDRESS,
											SPS_CMD_AUTOCLEAN_INTERVAL, data,
											SENSIRION_NUM_WORDS(data));
	Sleep(SPS_WRITE_DELAY_mSEC);

	return ret;
}

SPS30_Error_et sps30_get_fan_auto_cleaning_interval_days(uint8_t *interval_days)
{
	int16_t ret;
	uint32_t interval_seconds;
	
	ret = sps30_get_fan_auto_cleaning_interval(&interval_seconds);
	if (ret < 0)
		return ret;
	
	*interval_days = interval_seconds / (24 * 60 * 60);
	
	return ret;
}

SPS30_Error_et sps30_set_fan_auto_cleaning_interval_days(uint8_t interval_days)
{
	return sps30_set_fan_auto_cleaning_interval((uint32_t)interval_days * 24 * 60 * 60);
}

SPS30_Error_et sps30_start_manual_fan_cleaning()
{
	int16_t ret;
	
	ret = sensirion_i2c_write_cmd(SPS30_I2C_ADDRESS,
								SPS_CMD_START_MANUAL_FAN_CLEANING);
	if (ret)
		return ret;
	
	Sleep(SPS_CMD_DELAY_mSEC);
	
	return 0;
}

SPS30_Error_et sps30_reset()
{
	return sensirion_i2c_write_cmd(SPS30_I2C_ADDRESS, SPS_CMD_RESET);
}

SPS30_Error_et SPS30_Get_Measurement(SPS30_MeasureTypeDef_st *Measurement_Values)
{
	uint16_t DataReady;

	if (sps30_read_data_ready(&DataReady))
		return SPS30_ERROR;
	if (sps30_read_measurement(Measurement_Values))
		return SPS30_ERROR;

	return SPS30_OK;
}

SPS30_Error_et MX_SPS30_Init()
{
    char Serial[SPS30_MAX_SERIAL_LEN];
    char Product_Type[SPS30_MAX_PRODUCT_TYPE_LEN];
    uint8_t Major_Rel, Minor_Rel;

	if (sps30_get_serial(Serial))
		return SPS30_ERROR;
	if (sps30_start_measurement())
		return SPS30_ERROR;
	if (sps30_read_firmware_version(&Major_Rel, &Minor_Rel))
		return SPS30_ERROR;
	if (sps30_read_product_type(Product_Type))
		return SPS30_ERROR;
	if (sps30_set_fan_auto_cleaning_interval_days(7))
		return SPS30_ERROR;
	if (sps30_stop_measurement())
		return SPS30_ERROR;

	return SPS30_OK;
}

