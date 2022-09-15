/*! ----------------------------------------------------------------------------
 * @file	deca_usb.h
 * @brief	Middleware for USB communications
 *
 * @author	Tommaso Sabatini, 2018
 */

/* Includes */
#include "platform/port.h"

//local data
#define SOFTWARE_VER_STRINGUSB "EVB1000 USB2SPI 2.0"
typedef enum applicationModes
{
	STAND_ALONE,
	USB_TO_SPI,
	USB_PRINT_ONLY,
	NUM_APP_MODES
} APP_MODE;

//extern uint16_t local_buff_length;
//extern uint8_t  local_buff[];


