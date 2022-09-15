/*  MCP23017 library for STM32CubeMX
    Tommaso Sabatini 2017 <algol6555@gmail.com>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/ 

// User include starts here
#include "platform/mcp23017.h"
// User include ends here

/*! ------------------------------------------------------------------------------------------------------------------
 *  \struct     MCP23017
 *  \brief      Used to define a MCP23017 device on the I2C bus
 *  \details    MCP23017 type is used to hold the device address and the initialization data to 
 *              be written to the device
 */
uint8_t master_1[22] = {
							MASTER1_IODIRA,	 
							MASTER1_IODIRB, 	
							MASTER1_IPOLA, 	
							MASTER1_IPOLB, 	
							MASTER1_GPINTENA,
							MASTER1_GPINTENB,
							MASTER1_DEFVALA, 
							MASTER1_DEFVALB, 
							MASTER1_INTCONA, 
							MASTER1_INTCONB, 
							MASTER1_IOCONA,
							MASTER1_IOCONB,
							MASTER1_GPPUA, 	
							MASTER1_GPPUB, 	
							MASTER1_INTFA, 	
							MASTER1_INTFB, 	
							MASTER1_INTCAPA, 
							MASTER1_INTCAPB, 
							MASTER1_GPIOA, 	
							MASTER1_GPIOB, 	
							MASTER1_OLATA, 	
							MASTER1_OLATB 	
						};
uint8_t master_2[22] = {
							MASTER2_IODIRA,	 
							MASTER2_IODIRB, 	
							MASTER2_IPOLA, 	
							MASTER2_IPOLB, 	
							MASTER2_GPINTENA,
							MASTER2_GPINTENB,
							MASTER2_DEFVALA, 
							MASTER2_DEFVALB, 
							MASTER2_INTCONA, 
							MASTER2_INTCONB, 
							MASTER2_IOCONA,
							MASTER2_IOCONB,
							MASTER2_GPPUA, 	
							MASTER2_GPPUB, 	
							MASTER2_INTFA, 	
							MASTER2_INTFB, 	
							MASTER2_INTCAPA, 
							MASTER2_INTCAPB, 
							MASTER2_GPIOA, 	
							MASTER2_GPIOB, 	
							MASTER2_OLATA, 	
							MASTER2_OLATB 	
						};

/*! ----------------------------------------------------------------------------
 *  \fn			uint8_t BCD2DEC(uint8_t data)			
 *  \brief      Convert Binary Coded Decimal (BCD) to Decimal 
 *  
 */
uint8_t BCD2DEC(uint8_t data)
{
	return ((data >> 4)*10 + (data & 0x0F));
}

/*! ----------------------------------------------------------------------------
 *  \fn			uint8_t DEC2BCD(uint8_t data)			
 *  \brief      Convert Decimal to Binary Coded Decimal (BCD)
 *  
 */
uint8_t DEC2BCD(uint8_t data)
{
	 return ((data/10) << 4 | (data %10)); 
} 

/*! ----------------------------------------------------------------------------
 *  \fn         void mcp23017_init(uint8_t *RegValues, uint16_t address)
 *  \brief      Initialize the MCP23017 device.
 *  \details    Setup the device to use on the I2C bus.
 *  \param      address The uint16_t address should just be the hardware address
 *              component setup at the circuit level, i.e. at A2-A0 (e.g.
 *              0x4000 if all are GND.)
 */
MCP23017_Error_et mcp23017_init(uint8_t *RegValues, uint8_t base_address)
{
	if (HAL_I2C_Mem_Write_DMA(&hi2c1,(uint16_t)base_address,0,I2C_MEMADD_SIZE_8BIT,(uint8_t*)RegValues,22))
		return MCP23017_ERROR;
	return MCP23017_OK;
}

/*******************************************************************************
 * @brief  Read a register of the device through BUS
 * @param  B_Addr Device address on BUS
 * @param  Reg The target register address to read
 * @param  pBuffer The data to be read
 * @param  Size Number of bytes to be read
 * @retval 0 in case of success
 * @retval 1 in case of failure
 *******************************************************************************/
void mcp23017_read_register(uint8_t base_address, uint8_t reg, uint8_t* pBuffer, uint8_t size)
{
	uint32_t i = 0;
//	static uint32_t delay = 0x9DD97B; //27ns x cycle -> about 0.3s
//	static uint32_t delay = 0x13BB2F7; //27ns x cycle -> about 0.6s
	static uint32_t delay = 0xE20E5F; //27ns x cycle -> about 0.4s
	bool TimeOut = false;

	HAL_I2C_Mem_Read(&hi2c1,(uint16_t)base_address,reg,I2C_MEMADD_SIZE_8BIT,&pBuffer[0],size,10);
	while ((HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) && (!TimeOut))
	{
    	if (++i > delay)
    	{
    		TimeOut = true;
    	}
	}
	if (TimeOut)
	{
		I2CResetBus(&hi2c1);
		TimeOut = false;
	}
}

/*! -----------------------------------------------------------------------------------------
 *  \fn         	mcp23017_read_register(uint16_t base_address, uint8_t reg)
 *  \brief      	Reads "size" number of registers starting from base_address | reg
 *  \param      	base_address: the base address to the MCP23017 structure.
 *  \param      	reg: The register to be read as uint8_t.  
 *  \receive_data   The method returns the contents of the specified register in receive_data
 *  				as uint8_t
 */
void mcp23017_read_registers(uint8_t base_address, uint8_t reg, uint8_t size)
{
	uint32_t i = 0;
//	static uint32_t delay = 0x9DD97B; //27ns x cycle -> about 0.3s
//	static uint32_t delay = 0x13BB2F7; //27ns x cycle -> about 0.6s
	static uint32_t delay = 0xE20E5F; //27ns x cycle -> about 0.4s
	bool TimeOut = false;

	ioext_a = false;
	I2C_done = false;
	if (base_address == MCP23017_MASTER1_BADDR)
	{
		ioext_a = true;
	}
	HAL_I2C_Mem_Read_DMA(&hi2c1,(uint16_t)base_address,reg,I2C_MEMADD_SIZE_8BIT,(uint8_t*)receive_data,size);
	while ((HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) && (!TimeOut))
	{
    	if (++i > delay)
    	{
    		TimeOut = true;
    	}
	}
	if (TimeOut)
	{
		I2CResetBus(&hi2c1);
		TimeOut = false;
	}
}

/*! -----------------------------------------------------------------------------------------
 *  \fn         	mcp23017_write_register(uint16_t base_address, uint8_t reg, uint8_t size)
 *  \brief      	Writes "size" number of registers starting from base_address | reg
 *  \param      	base_address: the base address to the MCP23017 structure.
 *  \param      	reg: The register to be write as uint8_t.  
 *  \send_data   	The method write the contents of send_data the specified register in
 *  				receive_data as uint8_t
 */
void mcp23017_write_registers(uint8_t base_address, uint8_t reg, uint8_t* t_data, uint8_t size)
{
	uint32_t i = 0;
//	static uint32_t delay = 0x9DD97B; //27ns x cycle -> about 0.3s
//	static uint32_t delay = 0x13BB2F7; //27ns x cycle -> about 0.6s
	static uint32_t delay = 0x88CFFF; //27ns x cycle -> about 0.4s
	bool TimeOut = false;

//	ioext_a = false;
	HAL_I2C_Mem_Write_DMA(&hi2c1,(uint16_t)base_address,reg,I2C_MEMADD_SIZE_8BIT,(uint8_t*)t_data,size);
//	HAL_Delay(i2c_delay);
	while ((HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) && (!TimeOut))
	{
    	if (++i > delay)
    	{
    		TimeOut = true;
    	}
	}
	if (TimeOut)
	{
		I2CResetBus(&hi2c1);
		TimeOut = false;
	}
/*	if (base_address == MCP23017_MASTER1_BADDR)
	{
		ioext_a = true;
	}*/
}

/*! -----------------------------------------------------------------------------------------
 *  \fn			HAL_I2C_MemRxCpltCallback(I2C_HandleTypedef *hi2c)			
 *  \brief      I2C Rx Interrupt Service Routine
 *  
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c->Instance == hi2c1.Instance)
	{
		if (ioext_a)
		{
			intcap1_a = receive_data[0x10];
			intcap1_b = receive_data[0x11];
			gpio1_a = receive_data[0x12];
			gpio1_b = receive_data[0x13];
			ioext_a = false;
		}
		else
		{
			intcap2_a = receive_data[0x10];
			intcap2_b = receive_data[0x11];
			gpio2_a = receive_data[0x12];
			gpio2_b = receive_data[0x13];
		}
		while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
		{
		}
		I2C_done = true;
	}
}

/*! -----------------------------------------------------------------------------------------
 *  \fn			MX_MCP23017_Init()			
 *  \brief      Initialize the MCP23017 IO Expanders
 *  
 */
MCP23017_Error_et MX_MCP23017_Init()
{
	static uint16_t i = 0;

	if (mcp23017_init(master_1, MCP23017_MASTER1_BADDR))
		return MCP23017_ERROR;
	HAL_Delay(i2c_delay);
	if (mcp23017_init(master_2, MCP23017_MASTER2_BADDR))
		return MCP23017_ERROR;
	HAL_Delay(i2c_delay);

	mcp23017_read_registers(MCP23017_MASTER1_BADDR, 0, 22);	//Read all MC23017_1 Registers in init to avoid Int pin hang
    while (!I2C_done)
	{
    	Sleep(1);
    	if (++i > 100)
    	{
    		I2C_done = true;
    		i= 0;
    	}
	}
 	mcp23017_read_registers(MCP23017_MASTER2_BADDR, 0, 22);	//Read all MC23017_2 Registers in init to avoid Int pin hang
    while (!I2C_done)
	{
    	Sleep(1);
    	if (++i > 100)
    	{
    		I2C_done = true;
    		i= 0;
    	}
	}
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);

	return MCP23017_OK;
}

/*
 * @fn      write_port(uint8_t port_number, uint8_t value)
 * @brief   Activate/Deactivate the selected port
**/
void write_port(uint16_t port_number, uint8_t value)
{
	switch (port_number)
	{
		case 0x3131:						//Bank A Port 1 (Out1)
			if (value == 0x31)
			{
				BIT_SET(send_data1,0);		//Enable uP_OUT_1 -> HS_OUT_1 = VC_BATT
			}
			else
			{
				BIT_CLEAR(send_data1,0);	//Disable uP_OUT_1 -> HS_OUT_1 = 0V
			}
			mcp23017_write_registers(MCP23017_MASTER1_BADDR, MCP23017_OLATA, &send_data1, 1);
			break;
		case 0x3132:						//Bank A Port 2 (Out2)
			if (value == 0x31)
			{
				BIT_SET(send_data1,1);		//Enable uP_OUT_2 -> HS_OUT_2 = VC_BATT
			}
			else
			{
				BIT_CLEAR(send_data1,1);	//Disable uP_OUT_2 -> HS_OUT_2 = 0V
			}
			mcp23017_write_registers(MCP23017_MASTER1_BADDR, MCP23017_OLATA, &send_data1, 1);
			break;
		case 0x3133:						//Bank A Port 3 (Out3)
			if (value == 0x31)
			{
				BIT_SET(send_data1,2);		//Enable uP_OUT_3 -> HS_OUT_3 = VC_BATT
			}
			else
			{
				BIT_CLEAR(send_data1,2);	//Disable uP_OUT_3 -> HS_OUT_3 = 0V
			}
			mcp23017_write_registers(MCP23017_MASTER1_BADDR, MCP23017_OLATA, &send_data1, 1);
			break;
		case 0x3134:						//Bank A Port 4 (Out4)
			if (value == 0x31)
			{
				BIT_SET(send_data1,3);		//Enable uP_OUT_4 -> HS_OUT_4 = VC_BATT
			}
			else
			{
				BIT_CLEAR(send_data1,3);	//Disable uP_OUT_4 -> HS_OUT_4 = 0V
			}
			mcp23017_write_registers(MCP23017_MASTER1_BADDR, MCP23017_OLATA, &send_data1, 1);
			break;
		case 0x3135:						//Bank A Port 5 (Out5)
			if (value == 0x31)
			{
				BIT_SET(send_data1,4);		//Enable uP_OUT_5 -> HS_OUT_5 = VC_BATT
			}
			else
			{
				BIT_CLEAR(send_data1,4);	//Disable uP_OUT_5 -> HS_OUT_5 = 0V
			}
			mcp23017_write_registers(MCP23017_MASTER1_BADDR, MCP23017_OLATA, &send_data1, 1);
			break;
		case 0x3231:						//Bank B Port 1 (Out6)
			if (value == 0x31)
			{
				BIT_SET(send_data1,5);		//Enable uP_OUT_6 -> HS_OUT_6 = VC_BATT
			}
			else
			{
				BIT_CLEAR(send_data1,5);	//Disable uP_OUT_6 -> HS_OUT_6 = 0V
			}
			mcp23017_write_registers(MCP23017_MASTER1_BADDR, MCP23017_OLATA, &send_data1, 1);
			break;
		case 0x3232:						//Bank B Port 2 (Out7)
			if (value == 0x31)
			{
				BIT_SET(send_data1,6);		//Enable uP_OUT_7 -> HS_OUT_7 = VC_BATT
			}
			else
			{
				BIT_CLEAR(send_data1,6);	//Disable uP_OUT_7 -> HS_OUT_7 = 0V
			}
			mcp23017_write_registers(MCP23017_MASTER1_BADDR, MCP23017_OLATA, &send_data1, 1);
			break;
		case 0x3233:						//Bank B Port 3 (Out8)
			if (value == 0x31)
			{
				BIT_SET(send_data1,7);		//Enable uP_OUT_8 -> HS_OUT_8 = VC_BATT
			}
			else
			{
				BIT_CLEAR(send_data1,7);	//Disable uP_OUT_8 -> HS_OUT_8 = 0V
			}
			mcp23017_write_registers(MCP23017_MASTER1_BADDR, MCP23017_OLATA, &send_data1, 1);
			break;
		case 0x3234:						//Bank B Port 4 (Out9)
			if (value == 0x31)
			{
				BIT_SET(send_data2,0);		//Enable uP_OUT_9 -> HS_OUT_9 = VC_BATT
			}
			else
			{
				BIT_CLEAR(send_data2,0);	//Disable uP_OUT_9 -> HS_OUT_9 = 0V
			}
			mcp23017_write_registers(MCP23017_MASTER2_BADDR, MCP23017_OLATA, &send_data2, 1);
			break;
		case 0x3235:						//Bank B Port 5 (Out10)
			if (value == 0x31)
			{
				BIT_SET(send_data2,1);		//Enable uP_OUT_10 -> HS_OUT_10 = VC_BATT
			}
			else
			{
				BIT_CLEAR(send_data2,1);	//Disable uP_OUT_10 -> HS_OUT_10 = 0V
			}
			mcp23017_write_registers(MCP23017_MASTER2_BADDR, MCP23017_OLATA, &send_data2, 1);
			break;
		case 0x3331:						//Bank C Port 1 (Out11)
			if (value == 0x31)
			{
				BIT_SET(send_data2,2);		//Enable uP_OUT_11 -> HS_OUT_11 = VC_BATT
			}
			else
			{
				BIT_CLEAR(send_data2,2);	//Disable uP_OUT_11 -> HS_OUT_11 = 0V
			}
			mcp23017_write_registers(MCP23017_MASTER2_BADDR, MCP23017_OLATA, &send_data2, 1);
			break;
		case 0x3332:						//Bank C Port 2 (Out12)
			if (value == 0x31)
			{
				BIT_SET(send_data2,3);		//Enable uP_OUT_12 -> HS_OUT_12 = VC_BATT
			}
			else
			{
				BIT_CLEAR(send_data2,3);	//Disable uP_OUT_12 -> HS_OUT_12 = 0V
			}
			mcp23017_write_registers(MCP23017_MASTER2_BADDR, MCP23017_OLATA, &send_data2, 1);
			break;
		case 0x3333:						//Bank C Port 3 (Out13)
			if (value == 0x31)
			{
				BIT_SET(send_data2,4);		//Enable uP_OUT_13 -> HS_OUT_13 = VC_BATT
			}
			else
			{
				BIT_CLEAR(send_data2,4);	//Disable uP_OUT_13 -> HS_OUT_13 = 0V
			}
			mcp23017_write_registers(MCP23017_MASTER2_BADDR, MCP23017_OLATA, &send_data2, 1);
			break;
		case 0x3334:						//Bank C Port 4 (Out14)
			if (value == 0x31)
			{
				BIT_SET(send_data2,5);		//Enable uP_OUT_14 -> HS_OUT_14 = VC_BATT
			}
			else
			{
				BIT_CLEAR(send_data2,5);	//Disable uP_OUT_14 -> HS_OUT_14 = 0V
			}
			mcp23017_write_registers(MCP23017_MASTER2_BADDR, MCP23017_OLATA, &send_data2, 1);
			break;
		case 0x3335:						//Bank C Port 5 (Out15)
			if (value == 0x31)
			{
				BIT_SET(send_data2,6);		//Enable uP_OUT_15 -> HS_OUT_15 = VC_BATT
			}
			else
			{
				BIT_CLEAR(send_data2,6);	//Disable uP_OUT_15 -> HS_OUT_15 = 0V
			}
			mcp23017_write_registers(MCP23017_MASTER2_BADDR, MCP23017_OLATA, &send_data2, 1);
			break;
		case 0x3431:						//Bank D Port 1 (Out16)
			if (value == 0x31)
			{
				BIT_SET(send_data2,7);		//Enable uP_OUT_16 -> HS_OUT_16 = VC_BATT
			}
			else
			{
				BIT_CLEAR(send_data2,7);	//Disable uP_OUT_16 -> HS_OUT_16 = 0V
			}
			mcp23017_write_registers(MCP23017_MASTER2_BADDR, MCP23017_OLATA, &send_data2, 1);
			break;
		default:
			break;
	}
}

/* @fn 		DinCheck()
 * @brief 	Check if there was an event from a digital input and manage it
 * @return	null
 * */
void DinCheck(uint16_t b)
{
	if (b & 0x0001)
	{
	}
	if (b & 0x0002)
	{
	}
	if (b & 0x0004)
	{
	}
	if (b & 0x0008)
	{
	}
	if (b & 0x0010)
	{
	}
	if (b & 0x0020)
	{
	}
	if (b & 0x0040)
	{
	}
	if (b & 0x0080)
	{
	}
	if (b & 0x0100)
	{
	}
	if (b & 0x0200)
	{
	}
	if (b & 0x0400)
	{
	}
	if (b & 0x0800)
	{
	}
	if (b & 0x1000)
	{
	}
	if (b & 0x2000)
	{
	}
	if (b & 0x0400)
	{
	}
	if (b & 0x8000)
	{
		button_manage();
	}
}

/*
 * @fn      read_ports()
 * @brief   Read all the input port status
 * @return	null
**/
void read_ports()
{
	static uint16_t io_exp_gpio1 = 0;
	static uint8_t send_data = 0xFF;

	if (ServiceTimer1.Expired)
	{
		ServiceTimer1.Expired = false;
		//Clears pending interrupts on port B
		mcp23017_read_registers(MCP23017_MASTER2_BADDR, MCP23017_INTCAPB, 1);
		//Re-enables the interrupt of the button IO_EXP input
		//It was be disabled in the HAL_GPIO_EXTI_Callback() function
		mcp23017_write_registers(MCP23017_MASTER2_BADDR, MCP23017_GPINTENB, &send_data, 1);
	}
	gpio1_b = (gpio1_b | internal_notifies_0);
	gpio2_b = (gpio2_b | internal_notifies_1);
 	io_exp_gpio1 = ((gpio2_b << 8) | (gpio1_b));
	if (input_changed | refresh)	//Check only if a digital input is changed
	{
		DinCheck(io_exp_gpio1);
		input_changed = false;
		refresh = false;
	}
}
