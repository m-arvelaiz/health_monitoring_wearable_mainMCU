/*
 * sensors_interface.c
 *
 *  Created on: Jun 21, 2025
 *      Author: magaa
 */


#include "sensors_interface.h"
#include "temp_sensor.h"
#include "i2c_handler.h"
#include "data_handler.h"
#include <string.h>

// Internal buffer for I2C tx/rx
static uint8_t i2c_tx_buf[4];
static uint8_t i2c_rx_buf[8];

void sensor_interface_init(void) {
    temp_sensor_init(TEMP_SENSOR_ADDRESS);
    // TODO: Add other sensors
}

void sensor_interface_deinit(void) {
    temp_sensor_deinit();
    // TODO: Add other sensor deinit
}

void sensor_interface_schedule_readings(void) {
    Temp_Sensor_t* temp = temp_sensor_get();
    if (!temp) return;

    // Prepare I2C payload (depends on sensor)
    temp->prepare_i2c_request(i2c_tx_buf);

    if (i2c_handler_write(temp->i2c_address, i2c_tx_buf, 1)) {
        // Delay or wait for sensor to be ready (optional)
        // Read back result
        if (i2c_handler_read(temp->i2c_address, i2c_rx_buf, 4)) {
            temp->decode_i2c_response(i2c_rx_buf, 4);
        }
    }
}

void sensor_interface_handle_cmd(uint8_t cmd_id, uint8_t* payload, uint8_t len, uint8_t* payload_out, uint8_t* len_out) {


    switch (cmd_id) {
        case CMD_REQ_TEMP_DATA:
		if (payload[0] == 0x02) {
			Temp_Sensor_t* env_temp = temp_sensor_get();
			if (env_temp && env_temp->get_last_n_data_serial_format) {
				payload_out[0]=0x02;
				(*len_out)=env_temp->get_last_n_data_serial_format(1,payload_out+1);
				(*len_out)= (*len_out)+1;
			}

        }

            break;

        case CMD_REQ_ALL_DATA:
            // TODO: Add support for composing full sensor packet with all sensor readings
            break;

        default:
            // Unknown command
            break;
    }
}
