/*
 * sensors_address.h
 *
 *  Created on: Jun 21, 2025
 *      Author: magaa
 */

#ifndef INC_SENSORS_ADDRESS_H_
#define INC_SENSORS_ADDRESS_H_


//7-bit I2C address WITHOUT shifted for write/read

#define SENSOR_ADDRESS_TEMP_HUM_HTS221 0XBE>>1
#define SENSOR_ADDRESS_PRESSURE_LPS22HH 0XBA>>1
#define SENSOR_ADDRESS_ACC_ISM330DHCX 0XD6>>1
#define SENSOR_ADDRESS_PPG_SEN0344 0x57

#endif /* INC_SENSORS_ADDRESS_H_ */
