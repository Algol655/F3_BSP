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
 
#ifndef _MCP23017_H_
#define _MCP23017_H_

// User include starts here
#include "i2c.h"
#include "port.h"
// User include ends here

/************************************************************************/
/* MCP23017 REGISTER DEFINITIONS                                        */
/************************************************************************/

//MCP23017 Register Addresses   					bit 7 -----> bit 0    | bit functions |
#define MCP23017_IODIRA		((uint8_t)0x00)			// IO7 - IO6 - IO5 - IO4 - IO3 - IO2 - IO1 - IO0
#define MCP23017_IODIRB 	((uint8_t)0x01)			// IO7 - IO6 - IO5 - IO4 - IO3 - IO2 - IO1 - IO0
#define MCP23017_IPOLA 		((uint8_t)0x02)			// IP7 - IP6 - IP5 - IP4 - IP3 - IP2 - IP1 - IP0
#define MCP23017_IPOLB 		((uint8_t)0x03)			// IP7 - IP6 - IP5 - IP4 - IP3 - IP2 - IP1 - IP0
#define MCP23017_GPINTENA 	((uint8_t)0x04) 		// GPINT7 - GPINT6 - GPINT5 - GPINT4 - GPINT3 - GPINT2 - GPINT1 - GPINT0
#define MCP23017_GPINTENB 	((uint8_t)0x05) 		// GPINT7 - GPINT6 - GPINT5 - GPINT4 - GPINT3 - GPINT2 - GPINT1 - GPINT0
#define MCP23017_DEFVALA 	((uint8_t)0x06)			// DEF7 - DEF6 - DEF5 - DEF4 - DEF3 - DEF2 - DEF1 - DEF0 
#define MCP23017_DEFVALB 	((uint8_t)0x07)			// DEF7 - DEF6 - DEF5 - DEF4 - DEF3 - DEF2 - DEF1 - DEF0 
#define MCP23017_INTCONA 	((uint8_t)0x08)  		// IOC7 - IOC6 - IOC5 - IOC4 - IOC3 - IOC2 - IOC1 - IOC0
#define MCP23017_INTCONB 	((uint8_t)0x09)  		// IOC7 - IOC6 - IOC5 - IOC4 - IOC3 - IOC2 - IOC1 - IOC0
#define MCP23017_IOCONA 	((uint8_t)0x0A)			// BANK - MIRROR - SEQOP - DISSLW - DAEN - ODR - INTPOL - (null)
#define MCP23017_IOCONB 	((uint8_t)0x0B)			// BANK - MIRROR - SEQOP - DISSLW - DAEN - ODR - INTPOL - (null) // IOCON is a device wide register and doesn't vary between banks
#define MCP23017_GPPUA 		((uint8_t)0x0C)			// PU7 - PU6 - PU5 - PU4 - PU3 - PU2 - PU1 - PU0
#define MCP23017_GPPUB 		((uint8_t)0x0D)			// PU7 - PU6 - PU5 - PU4 - PU3 - PU2 - PU1 - PU0
#define MCP23017_INTFA 		((uint8_t)0x0E)			// INT7 - INT6 - INT5 - INT4 - INT3 - INT2 - INT1 - INT0
#define MCP23017_INTFB 		((uint8_t)0x0F)			// INT7 - INT6 - INT5 - INT4 - INT3 - INT2 - INT1 - INT0
#define MCP23017_INTCAPA 	((uint8_t)0x10)			// ICP7 - ICP6 - ICP5 - ICP4 - ICP3 - ICP2 - ICP1 - ICP0
#define MCP23017_INTCAPB 	((uint8_t)0x11)			// ICP7 - ICP6 - ICP5 - ICP4 - ICP3 - ICP2 - ICP1 - ICP0
#define MCP23017_GPIOA 		((uint8_t)0x12)			// GP7 - GP6 - GP5 - GP4 - GP3 - GP2 - GP1 - GP0 
#define MCP23017_GPIOB 		((uint8_t)0x13)			// GP7 - GP6 - GP5 - GP4 - GP3 - GP2 - GP1 - GP0
#define MCP23017_OLATA 		((uint8_t)0x14)			// OL7 - OL6 - OL5 - OL4 - OL3 - OL2 - OL1 - OL0
#define MCP23017_OLATB 		((uint8_t)0x15) 		// OL7 - OL6 - OL5 - OL4 - OL3 - OL2 - OL1 - OL0

//MCP23017_1 Register Addresses Values				bit 7 -----> bit 0    | bit functions |
#define MASTER1_IODIRA		((uint8_t)0x00)			// IO7 - IO6 - IO5 - IO4 - IO3 - IO2 - IO1 - IO0
#define MASTER1_IODIRB 		((uint8_t)0xFF)			// IO7 - IO6 - IO5 - IO4 - IO3 - IO2 - IO1 - IO0
#define MASTER1_IPOLA 		((uint8_t)0x00)			// IP7 - IP6 - IP5 - IP4 - IP3 - IP2 - IP1 - IP0
#define MASTER1_IPOLB 		((uint8_t)0xFF)			// IP7 - IP6 - IP5 - IP4 - IP3 - IP2 - IP1 - IP0
#define MASTER1_GPINTENA 	((uint8_t)0x00) 		// GPINT7 - GPINT6 - GPINT5 - GPINT4 - GPINT3 - GPINT2 - GPINT1 - GPINT0
#define MASTER1_GPINTENB 	((uint8_t)0xFF) 		// GPINT7 - GPINT6 - GPINT5 - GPINT4 - GPINT3 - GPINT2 - GPINT1 - GPINT0
#define MASTER1_DEFVALA 	((uint8_t)0xFF)			// DEF7 - DEF6 - DEF5 - DEF4 - DEF3 - DEF2 - DEF1 - DEF0
#define MASTER1_DEFVALB 	((uint8_t)0xFF)			// DEF7 - DEF6 - DEF5 - DEF4 - DEF3 - DEF2 - DEF1 - DEF0
#define MASTER1_INTCONA 	((uint8_t)0x00)  		// IOC7 - IOC6 - IOC5 - IOC4 - IOC3 - IOC2 - IOC1 - IOC0
#define MASTER1_INTCONB 	((uint8_t)0x00)  		// IOC7 - IOC6 - IOC5 - IOC4 - IOC3 - IOC2 - IOC1 - IOC0
#define MASTER1_IOCONA 		((uint8_t)0b01011100)	// BANK - MIRROR - SEQOP - DISSLW - DAEN - ODR - INTPOL - (null)
#define MASTER1_IOCONB 		((uint8_t)0b01011100)	// BANK - MIRROR - SEQOP - DISSLW - DAEN - ODR - INTPOL - (null) // IOCO IOCON is a device wide register and doesn't vary between banks
#define MASTER1_GPPUA 		((uint8_t)0x00)			// PU7 - PU6 - PU5 - PU4 - PU3 - PU2 - PU1 - PU0
#define MASTER1_GPPUB 		((uint8_t)0xFF)			// PU7 - PU6 - PU5 - PU4 - PU3 - PU2 - PU1 - PU0
#define MASTER1_INTFA 		((uint8_t)0xFF)			// INT7 - INT6 - INT5 - INT4 - INT3 - INT2 - INT1 - INT0
#define MASTER1_INTFB 		((uint8_t)0xFF)			// INT7 - INT6 - INT5 - INT4 - INT3 - INT2 - INT1 - INT0
#define MASTER1_INTCAPA 	((uint8_t)0x00)			// ICP7 - ICP6 - ICP5 - ICP4 - ICP3 - ICP2 - ICP1 - ICP0
#define MASTER1_INTCAPB 	((uint8_t)0x00)			// ICP7 - ICP6 - ICP5 - ICP4 - ICP3 - ICP2 - ICP1 - ICP0
#define MASTER1_GPIOA 		((uint8_t)0x00)			// GP7 - GP6 - GP5 - GP4 - GP3 - GP2 - GP1 - GP0
#define MASTER1_GPIOB 		((uint8_t)0x00)			// GP7 - GP6 - GP5 - GP4 - GP3 - GP2 - GP1 - GP0
#define MASTER1_OLATA 		((uint8_t)0x00)			// OL7 - OL6 - OL5 - OL4 - OL3 - OL2 - OL1 - OL0
#define MASTER1_OLATB 		((uint8_t)0x00) 		// OL7 - OL6 - OL5 - OL4 - OL3 - OL2 - OL1 - OL0

//MCP23017_2 Register Addresses Values				bit 7 -----> bit 0    | bit functions |
#define MASTER2_IODIRA		((uint8_t)0x00)			// IO7 - IO6 - IO5 - IO4 - IO3 - IO2 - IO1 - IO0
#define MASTER2_IODIRB 		((uint8_t)0xFF)			// IO7 - IO6 - IO5 - IO4 - IO3 - IO2 - IO1 - IO0
#define MASTER2_IPOLA 		((uint8_t)0x00)			// IP7 - IP6 - IP5 - IP4 - IP3 - IP2 - IP1 - IP0
#define MASTER2_IPOLB 		((uint8_t)0xFF)			// IP7 - IP6 - IP5 - IP4 - IP3 - IP2 - IP1 - IP0
#define MASTER2_GPINTENA 	((uint8_t)0x00) 		// GPINT7 - GPINT6 - GPINT5 - GPINT4 - GPINT3 - GPINT2 - GPINT1 - GPINT0
#define MASTER2_GPINTENB 	((uint8_t)0xFF) 		// GPINT7 - GPINT6 - GPINT5 - GPINT4 - GPINT3 - GPINT2 - GPINT1 - GPINT0
#define MASTER2_DEFVALA 	((uint8_t)0xFF)			// DEF7 - DEF6 - DEF5 - DEF4 - DEF3 - DEF2 - DEF1 - DEF0
#define MASTER2_DEFVALB 	((uint8_t)0x7F)			// DEF7 - DEF6 - DEF5 - DEF4 - DEF3 - DEF2 - DEF1 - DEF0
#define MASTER2_INTCONA 	((uint8_t)0x00)  		// IOC7 - IOC6 - IOC5 - IOC4 - IOC3 - IOC2 - IOC1 - IOC0
#define MASTER2_INTCONB 	((uint8_t)0x80)  		// IOC7 - IOC6 - IOC5 - IOC4 - IOC3 - IOC2 - IOC1 - IOC0
#define MASTER2_IOCONA 		((uint8_t)0b01011100)	// BANK - MIRROR - SEQOP - DISSLW - DAEN - ODR - INTPOL - (null)
#define MASTER2_IOCONB 		((uint8_t)0b01011100)	// BANK - MIRROR - SEQOP - DISSLW - DAEN - ODR - INTPOL - (null) // IOCO IOCON is a device wide register and doesn't vary between banks
#define MASTER2_GPPUA 		((uint8_t)0x00)			// PU7 - PU6 - PU5 - PU4 - PU3 - PU2 - PU1 - PU0
#define MASTER2_GPPUB 		((uint8_t)0xFF)			// PU7 - PU6 - PU5 - PU4 - PU3 - PU2 - PU1 - PU0
#define MASTER2_INTFA 		((uint8_t)0xFF)			// INT7 - INT6 - INT5 - INT4 - INT3 - INT2 - INT1 - INT0
#define MASTER2_INTFB 		((uint8_t)0xFF)			// INT7 - INT6 - INT5 - INT4 - INT3 - INT2 - INT1 - INT0
#define MASTER2_INTCAPA 	((uint8_t)0x00)			// ICP7 - ICP6 - ICP5 - ICP4 - ICP3 - ICP2 - ICP1 - ICP0
#define MASTER2_INTCAPB 	((uint8_t)0x00)			// ICP7 - ICP6 - ICP5 - ICP4 - ICP3 - ICP2 - ICP1 - ICP0
#define MASTER2_GPIOA 		((uint8_t)0x80)			// GP7 - GP6 - GP5 - GP4 - GP3 - GP2 - GP1 - GP0
#define MASTER2_GPIOB 		((uint8_t)0x00)			// GP7 - GP6 - GP5 - GP4 - GP3 - GP2 - GP1 - GP0
#define MASTER2_OLATA 		((uint8_t)0x80)			// OL7 - OL6 - OL5 - OL4 - OL3 - OL2 - OL1 - OL0
#define MASTER2_OLATB 		((uint8_t)0x00) 		// OL7 - OL6 - OL5 - OL4 - OL3 - OL2 - OL1 - OL0

#define MCP23017_MASTER1_BADDR ((uint8_t)0x40)
#define MCP23017_MASTER2_BADDR ((uint8_t)0x42)

typedef enum
{
	MCP23017_OK	=	 0x00,
	MCP23017_ERROR = 0x01
} MCP23017_Error_et;

uint8_t intcap1_a, intcap1_b, gpio1_a, gpio1_b;
uint8_t intcap2_a, intcap2_b, gpio2_a, gpio2_b;
uint8_t BADDR;
uint8_t receive_data[22];
uint8_t send_data1, send_data2;
bool ioext_a;

extern uint8_t L20_menu_items_row7[64];
extern const uint8_t L20_menu_items_row4[64];
/*! ------------------------------------------------------------------------------------------------------------------
 *  \fn			uint8_t BCD2DEC(uint8_t data)
 *  \brief      Convert Binary Coded Decimal (BCD) to Decimal 
 *  
 */ 
uint8_t BCD2DEC(uint8_t data); 

/*! ------------------------------------------------------------------------------------------------------------------
 *  \fn			uint8_t DEC2BCD(uint8_t data)
 *  \brief      Convert Decimal to Binary Coded Decimal (BCD) 
 * 
 */ 
uint8_t DEC2BCD(uint8_t data);

/*! ------------------------------------------------------------------------------------------------------------------
 *  \fn         void mcp23017_init(uint8_t *RegValues, uint16_t address)
 *  \brief      Initialize the MCP23017 device.
 *  \details    Setup the device to use on the I2C bus.
 *  \param      address The uint16_t address should just be the hardware address
 *              component setup at the circuit level, i.e. at A2-A0 (e.g.
 *              0x4000 if all are GND.)
 */
MCP23017_Error_et mcp23017_init(uint8_t *RegValues, uint8_t base_address);
 
void mcp23017_read_register(uint8_t base_address, uint8_t reg, uint8_t* pBuffer, uint8_t size);

 /*! ------------------------------------------------------------------------------------------------------------------
 *  \fn         	void mcp23017_read_registers(uint16_t base_address, uint8_t reg, uint8_t size)
 *  \brief      	Reads "size" number of registers starting from base_address | reg
 *  \param      	base_address: the base address to the MCP23017 structure.
 *  \param      	reg: The register to be read as uint8_t.  
 *  \receive_data   The method returns the contents of the specified register in receive_data as uint8_t
 */
 void mcp23017_read_registers(uint8_t base_address, uint8_t reg, uint8_t size);

/*! ------------------------------------------------------------------------------------------------------------------
 *  \fn         	mcp23017_write_register(uint16_t base_address, uint8_t reg)
 *  \brief      	Writes "size" number of registers starting from base_address | reg
 *  \param      	base_address: the base address to the MCP23017 structure.
 *  \param      	reg: The register to be write as uint8_t.  
 *  \send_data   	The method write the contents of send_data the specified register in receive_data as uint8_t
 */
void mcp23017_write_registers(uint8_t base_address, uint8_t reg, uint8_t *t_data, uint8_t size);

/*! ------------------------------------------------------------------------------------------------------------------
 *  \fn			HAL_I2C_MemRxCpltCallback(I2C_HandleTypedef *hi2c)			
 *  \brief      I2C Rx Interrupt Service Routine
 *  
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);

/*! ------------------------------------------------------------------------------------------------------------------
 *  \fn			MX_MCP23017_Init()			
 *  \brief      Initialize the MCP23017 IO Expanders
 *  
 */
MCP23017_Error_et MX_MCP23017_Init();

/*
 * @fn      write_port(uint8_t port_number, uint8_t value)
 * @brief   Activate/Deactivate the selected port
**/
void write_port(uint16_t port_number, uint8_t value);
void read_ports(void);

#endif /* MCP23017_H_ */

