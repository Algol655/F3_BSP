/*
 * OpModes.h
 *
 *  Created on: 27 nov 2021
 *      Author: tommaso
 */

#ifndef BLE_OPMODES_H_
#define BLE_OPMODES_H_

#include "main.h"
//BLE Operating Mode defines section
#define BEACON_APP		((1) & BLE_SUPPORT)
#define CENTRAL_APP		((0) & BLE_SUPPORT)
#define SERIAL_PORT_APP	((0) & BLE_SUPPORT)
#define SENSOR_APP		((0) & BLE_SUPPORT)

#endif /* BLE_OPMODES_H_ */
