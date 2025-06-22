/*
 * temp_sensor.c
 *
 *  Created on: Jun 21, 2025
 *      Author: magaa
 */


#include "temp_sensor.h"
#include "i2c_handler.h"
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

static Temp_Sensor_t* temp_sensor = NULL;
static Temp_Data_t* temp_data_ptr_buffer[TEMP_HUMIDITY_SENSOR_HISTORY_SIZE];

// === Local Function Prototypes ===
static void temp_format_uart_response(uint8_t* payload_out);
static Temp_Data_t* temp_sensor_get_last_temp_data();
static Temp_Data_t** temp_sensor_get_last_n_data(uint8_t n);
static uint8_t temp_sensor_get_last_n_temp_data_serial_format(uint8_t n, uint8_t* out);
static void temp_prepare_i2c_request(uint8_t* payload_out, uint8_t* len);
static void temp_decode_i2c_response(uint8_t* data, uint8_t len);
bool temp_sensor_trigger_data_collection();



// === Public API ===


void temp_sensor_init(uint8_t address) {
    if (temp_sensor == NULL) {
        temp_sensor = (Temp_Sensor_t*) malloc(sizeof(Temp_Sensor_t));
    }


    memset(temp_sensor, 0, sizeof(Temp_Sensor_t));

	temp_sensor->i2c_address = address;
	temp_sensor->format_uart_response = temp_format_uart_response;
//	temp_sensor->prepare_i2c_request = temp_prepare_i2c_request;
	temp_sensor->decode_i2c_response = temp_decode_i2c_response;
	temp_sensor->get_last_data = temp_sensor_get_last_temp_data;
	temp_sensor->get_last_n_data = temp_sensor_get_last_n_data;
	temp_sensor->get_last_n_data_serial_format = temp_sensor_get_last_n_temp_data_serial_format;
	temp_sensor->trigger_data_collection=temp_sensor_trigger_data_collection;
	memset(temp_data_ptr_buffer, 0, sizeof(temp_data_ptr_buffer));

	// TODO: Perform initial I2C check or sensor config if needed

	I2C_Handler_t* i2c_handle=i2c_handler_get();
	uint8_t reg = TEMP_SENSOR_HTS221_WHO_AM_I_REG;

	// Step 1: Check WHO_AM_I
	if (!i2c_handle->read_reg(temp_sensor->i2c_address, &reg, 1, 1)){

	}
	HAL_Delay(10); // Optional: wait for completion

	if(i2c_handle->state==I2C_STATE_MSG_WAITING_FOR_PROCESSING){
		i2c_handle->state=I2C_STATE_IDLE;
		if (i2c_handle->Response_buffer[0] = TEMP_SENSOR_HTS221_WHO_AM_I_VAL){
			//OKK

		}
	}

	// Step 2: Optional - Reboot memory content
	uint8_t reboot_cmd[2] = { TEMP_SENSOR_HTS221_CTRL_REG2, 0x80 };  // BOOT bit = 1
	if (!i2c_handler_write_reg(temp_sensor->i2c_address, reboot_cmd, 2)){

	}
	HAL_Delay(15);  // Wait for reboot

	if (i2c_handle->state == I2C_STATE_IDLE) {
		//OKK
	}

	// Step 3: Enable sensor in continuous mode at 1 Hz
	uint8_t init_cmd[2] = { TEMP_SENSOR_HTS221_CTRL_REG1, 0x81 };  // PD=1, BDU=0, ODR=1Hz
	if (!i2c_handler_write_reg(temp_sensor->i2c_address, init_cmd, 2)){

	}

	HAL_Delay(15);  // Wait for reboot
	if (i2c_handle->state == I2C_STATE_IDLE) {
			//OKK
	}
}

void temp_sensor_deinit(void) {
    if (temp_sensor != NULL) {
        free(temp_sensor);
        temp_sensor = NULL;
    }
}


Temp_Sensor_t* temp_sensor_get(void) {
    return temp_sensor;
}


// === Local Functions ===

static void temp_format_uart_response(uint8_t* payload_out) {
    Temp_Data_t* last = &temp_sensor->history[
        (temp_sensor->head_index - 1 + TEMP_HUMIDITY_SENSOR_HISTORY_SIZE) % TEMP_HUMIDITY_SENSOR_HISTORY_SIZE];

    payload_out[0] = (last->temperature >> 8) & 0xFF;
    payload_out[1] = (last->temperature >> 0) & 0xFF;
    payload_out[2] = 0x00;
    payload_out[3] = 0x00;
    payload_out[4] = (last->timestamp >> 24) & 0xFF;
    payload_out[5] = (last->timestamp >> 16) & 0xFF;
    payload_out[6] = (last->timestamp >> 8) & 0xFF;
    payload_out[7] = (last->timestamp >> 0) & 0xFF;


}

/**
 * @brief Retrieve last Temp_Data_t pointer from history.
 *
 */

Temp_Data_t* temp_sensor_get_last_temp_data(){
	 Temp_Data_t* last = &temp_sensor->history[
	        (temp_sensor->head_index - 1 + TEMP_HUMIDITY_SENSOR_HISTORY_SIZE) % TEMP_HUMIDITY_SENSOR_HISTORY_SIZE];
	return last;
}

/**
 * @brief Retrieve last n Temp_Data_t pointers from history.
 * Result is stored in a static buffer and ordered from newest to oldest.
 */

static Temp_Data_t** temp_sensor_get_last_n_data(uint8_t n) {
    if (!temp_sensor || n == 0 || n > temp_sensor->count) {
        return NULL; // Invalid request
    }

    for (uint8_t i = 0; i < n; ++i) {
        int index = (temp_sensor->head_index - 1 - i + TEMP_HUMIDITY_SENSOR_HISTORY_SIZE) % TEMP_HUMIDITY_SENSOR_HISTORY_SIZE;
        temp_data_ptr_buffer[i] = &temp_sensor->history[index];
    }

    return temp_data_ptr_buffer;
}


/**
 * @brief Retrieve last n readings from the circular buffer.
 * Each entry in out_buffer consists of:
 * - Bytes 0-1: temperature * 100 (2 bytes, MSB first)
 * - Bytes 2-5: timestamp (4 bytes, MSB first)
 *
 * Return the lenght of the output data
 */

uint8_t temp_sensor_get_last_n_temp_data_serial_format(uint8_t n, uint8_t* out_buffer) {
    if (!temp_sensor || !out_buffer || n == 0 || n > temp_sensor->count) {
        return 0x00;
    }

    uint8_t i;
    for (i = 0; i <= n; ++i) {
        int index = (temp_sensor->head_index - 1 - i + TEMP_HUMIDITY_SENSOR_HISTORY_SIZE) % TEMP_HUMIDITY_SENSOR_HISTORY_SIZE;
        Temp_Data_t* data = &temp_sensor->history[index];

		// Pack: [TEMP x100 MSB, LSB], [TIMESTAMP MSB to LSB]
		out_buffer[i * 8 + 0] = ((data->temperature * 100) >> 8) & 0xFF;
		out_buffer[i * 8 + 1] = ((data->temperature * 100) >> 0) & 0xFF;
		out_buffer[i * 8 + 2] = 0x00;
		out_buffer[i * 8 + 3] = 0x00;
		out_buffer[i * 8 + 4] = (data->timestamp >> 24) & 0xFF;
		out_buffer[i * 8 + 5] = (data->timestamp >> 16) & 0xFF;
		out_buffer[i * 8 + 6] = (data->timestamp >> 8)  & 0xFF;
        out_buffer[i * 8 + 7] = (data->timestamp >> 0)  & 0xFF;
    }

    return (i*8);
}


//static void temp_prepare_i2c_request(uint8_t* payload_out, uint8_t* len) {
//    // TODO: Set up sensor-specific I2C command to trigger a measurement
//    // Example: write register address to start measurement
//    payload_out[0] = 0x00; // Replace with actual command if needed
//
//    if (i2c_handler_write(temp->i2c_address, i2c_tx_buf, 1)) {
//            // Delay or wait for sensor to be ready (optional)
//            // Read back result
//            if (i2c_handler_read(temp->i2c_address, i2c_rx_buf, 4)) {
//                temp->decode_i2c_response(i2c_rx_buf, 4);
//            }
//        }
//}

static void temp_decode_i2c_response(uint8_t* data, uint8_t len) {
    if (len < 4) {

    } // Expecting at least temp + hum

    Temp_Data_t sample;
    sample.temperature = (data[0] << 8) | data[1];
    sample.humidity    = (data[2] << 8) | data[3];

    // TODO: Replace with real-time acquisition
    sample.timestamp = 0x60D4A000; // Fixed dummy Unix timestamp

    temp_sensor->history[temp_sensor->head_index] = sample;
    temp_sensor->head_index = (temp_sensor->head_index + 1) % TEMP_HUMIDITY_SENSOR_HISTORY_SIZE;
    if (temp_sensor->count < TEMP_HUMIDITY_SENSOR_HISTORY_SIZE)
        temp_sensor->count++;

}

bool temp_sensor_trigger_data_collection(){
	I2C_Handler_t* i2c_handle=i2c_handler_get();

	uint8_t reg = TEMP_SENSOR_HTS221_HUMIDITY_OUT_L | 0x80; // Auto-increment
	i2c_handle->read_reg(temp_sensor->i2c_address, &reg, 1, 4);


}
