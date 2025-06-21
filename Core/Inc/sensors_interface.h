/*
 * sensors_interface.h
 *
 *  Created on: Jun 21, 2025
 *      Author: magaa
 */

#ifndef INC_SENSORS_INTERFACE_H_
#define INC_SENSORS_INTERFACE_H_
#include <stdint.h>

// Initializes all sensor modules (I2C setup, sensor init, etc.)
void sensor_interface_init(void);

// Releases sensor resources
void sensor_interface_deinit(void);

// Called periodically to trigger sensor readings (e.g., from a timer)
void sensor_interface_schedule_readings(void);

/**
 * @brief  Handles incoming UART command requests directed to the sensor layer.
 *         This is typically called from the data handler or UART handler layer.
 *
 * @param  cmd_id   The command identifier (see BLE/UART protocol definitions)
 * @param  payload  Optional payload received with the command
 * @param  len      Length of the payload
 */
void sensor_interface_handle_cmd(uint8_t cmd_id, uint8_t* payload, uint8_t len, uint8_t* payload_out, uint8_t* len_out);


#endif /* INC_SENSORS_INTERFACE_H_ */
