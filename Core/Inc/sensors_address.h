/*
 * sensors_address.h
 *
 *  Created on: Jun 21, 2025
 *      Author: magaa
 */

#ifndef INC_SENSORS_ADDRESS_H_
#define INC_SENSORS_ADDRESS_H_


//7-bit I2C address shifted for write/read

#define SENSOR_ADDRESS_TEMP_HUM_HTS221 0XBE
#define SENSOR_ADDRESS_PRESSURE_LPS22HH 0XBA
#define SENSOR_ADDRESS_ACC_ISM330DHCX 0XD6
#define SENSOR_ADDRESS_PPG_SEN0344 0x57<<1   //Conversion to 8bit address

#endif /* INC_SENSORS_ADDRESS_H_ */
