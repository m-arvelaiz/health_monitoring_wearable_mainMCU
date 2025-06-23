/*
 * pressure_sensor.c
 *
 *  Created on: Jun 21, 2025
 *      Author: magaa
 */

#include "pressure_sensor.h"
#include "i2c_handler.h"
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>


static Pressure_Sensor_t* pressure_sensor = NULL;
static Pressure_Data_t* pressure_data_ptr_buffer[PRESSURE_SENSOR_HISTORY_SIZE];

// === Local Prototypes ===
static void pressure_format_uart_response(uint8_t* payload_out);
static Pressure_Data_t* pressure_get_last_data();
static Pressure_Data_t** pressure_get_last_n_data(uint8_t n);
static uint8_t pressure_get_last_n_serial_format(uint8_t n, uint8_t* out);
static void pressure_decode_i2c_response(uint8_t* data, uint8_t len);
static bool pressure_trigger_measurement();
static bool pressure_trigger_data_collection();





void pressure_sensor_init(uint8_t address) {
    pressure_sensor = (Pressure_Sensor_t*) malloc(sizeof(Pressure_Sensor_t));
    memset(pressure_sensor, 0, sizeof(Pressure_Sensor_t));
    memset(pressure_data_ptr_buffer, 0, sizeof(pressure_data_ptr_buffer));

    pressure_sensor->i2c_address = address;
    pressure_sensor->format_uart_response = pressure_format_uart_response;
    pressure_sensor->get_last_data = pressure_get_last_data;
    pressure_sensor->get_last_n_data = pressure_get_last_n_data;
    pressure_sensor->get_last_n_data_serial_format = pressure_get_last_n_serial_format;
    pressure_sensor->decode_i2c_response = pressure_decode_i2c_response;
    pressure_sensor->trigger_data_collection = pressure_trigger_data_collection;
    pressure_sensor->trigger_measurement = pressure_trigger_measurement;

    I2C_Handler_t* i2c = i2c_handler_get();
    uint8_t reg = PRESSURE_SENSOR_LPS22HH_WHO_AM_I_REG;

    if (!i2c->read_reg(address, &reg, 1, 1)) return;
    HAL_Delay(5);
    if (i2c->state == I2C_STATE_MSG_WAITING_FOR_PROCESSING && i2c->Response_buffer[0] == PRESSURE_SENSOR_LPS22HH_WHO_AM_I_VAL) {
        // WHO_AM_I OK
    	i2c->state=I2C_STATE_IDLE;
    }


    // Enable Continuous Mode: ODR=10Hz (0b010), BDU=0, LPFP=0
    // CTRL_REG1 = 0b01000000 = 0x40
    uint8_t ctrl1[2] = { PRESSURE_SENSOR_LPS22HH_CTRL_REG1, 0x40 };

//    // Power on: CTRL_REG1 = BDU=1, ODR=0 → 0x02
//    uint8_t ctrl1[2] = { PRESSURE_SENSOR_LPS22HH_CTRL_REG1, 0x02 };
    i2c->write_reg(address, ctrl1, 2);
	HAL_Delay(5);

	reg = PRESSURE_SENSOR_LPS22HH_CTRL_REG1;
	if (!i2c->read_reg(address, &reg, 1, 1))
		return;
	HAL_Delay(5);
	if (i2c->state == I2C_STATE_MSG_WAITING_FOR_PROCESSING) {
		// WHO_AM_I OK
		i2c->state = I2C_STATE_IDLE;
	}



}








void pressure_sensor_deinit(void) {
    if (pressure_sensor) {
        free(pressure_sensor);
        pressure_sensor = NULL;
    }
}

Pressure_Sensor_t* pressure_sensor_get(void) {
    return pressure_sensor;
}

// === Local Functions ===

static void pressure_format_uart_response(uint8_t* payload_out) {
    Pressure_Data_t* last = &pressure_sensor->history[
        (pressure_sensor->head_index - 1 + PRESSURE_SENSOR_HISTORY_SIZE) % PRESSURE_SENSOR_HISTORY_SIZE];

    payload_out[0] = (last->pressure >> 16) & 0xFF;
    payload_out[1] = (last->pressure >> 8) & 0xFF;
    payload_out[2] = (last->pressure >> 0) & 0xFF;
    payload_out[3] = 0x00;  // padding

    payload_out[4] = (last->timestamp >> 24) & 0xFF;
    payload_out[5] = (last->timestamp >> 16) & 0xFF;
    payload_out[6] = (last->timestamp >> 8) & 0xFF;
    payload_out[7] = (last->timestamp >> 0) & 0xFF;
}

static Pressure_Data_t* pressure_get_last_data() {
    return &pressure_sensor->history[
        (pressure_sensor->head_index - 1 + PRESSURE_SENSOR_HISTORY_SIZE) % PRESSURE_SENSOR_HISTORY_SIZE];
}



static Pressure_Data_t** pressure_get_last_n_data(uint8_t n) {
    if (!pressure_sensor || n == 0 || n > pressure_sensor->count) return NULL;

    for (uint8_t i = 0; i < n; ++i) {
        int index = (pressure_sensor->head_index - 1 - i + PRESSURE_SENSOR_HISTORY_SIZE) % PRESSURE_SENSOR_HISTORY_SIZE;
        pressure_data_ptr_buffer[i] = &pressure_sensor->history[index];
    }
    return pressure_data_ptr_buffer;
}


static uint8_t pressure_get_last_n_serial_format(uint8_t n, uint8_t* out) {
    if (!pressure_sensor || !out || n == 0 || n > pressure_sensor->count) return 0;

    for (uint8_t i = 0; i < n; ++i) {
        int index = (pressure_sensor->head_index - 1 - i + PRESSURE_SENSOR_HISTORY_SIZE) % PRESSURE_SENSOR_HISTORY_SIZE;
        Pressure_Data_t* data = &pressure_sensor->history[index];

        out[i * 8 + 0] = (data->pressure >> 16) & 0xFF;
        out[i * 8 + 1] = (data->pressure >> 8) & 0xFF;
        out[i * 8 + 2] = (data->pressure >> 0) & 0xFF;
        out[i * 8 + 3] = 0x00;
        out[i * 8 + 4] = (data->timestamp >> 24) & 0xFF;
        out[i * 8 + 5] = (data->timestamp >> 16) & 0xFF;
        out[i * 8 + 6] = (data->timestamp >> 8) & 0xFF;
        out[i * 8 + 7] = (data->timestamp >> 0) & 0xFF;
    }

    return (n * 8);
}


uint8_t pressure_get_temp_last_n_serial_format(uint8_t n, uint8_t* out) {
    if (!pressure_sensor || !out || n == 0 || n > pressure_sensor->count) return 0;

    for (uint8_t i = 0; i < n; ++i) {
        int index = (pressure_sensor->head_index - 1 - i + PRESSURE_SENSOR_HISTORY_SIZE) % PRESSURE_SENSOR_HISTORY_SIZE;
        Pressure_Data_t* data = &pressure_sensor->history[index];

        out[i * 8 + 0] = (data->temperature >> 8) & 0xFF;
        out[i * 8 + 1] = (data->temperature >> 0) & 0xFF;
        out[i * 8 + 2] = 0x00;
        out[i * 8 + 3] = 0x00;
        out[i * 8 + 4] = (data->timestamp >> 24) & 0xFF;
        out[i * 8 + 5] = (data->timestamp >> 16) & 0xFF;
        out[i * 8 + 6] = (data->timestamp >> 8) & 0xFF;
        out[i * 8 + 7] = (data->timestamp >> 0) & 0xFF;
    }

    return (n * 8);
}



static bool pressure_trigger_measurement() {

	//USED if one shot mode
    I2C_Handler_t* i2c = i2c_handler_get();
    uint8_t cmd[2] = { PRESSURE_SENSOR_LPS22HH_CTRL_REG2, 0x01 };  // ONE_SHOT
    return i2c->write_reg(pressure_sensor->i2c_address, cmd, 2);
}



static bool pressure_trigger_data_collection() {
    I2C_Handler_t* i2c = i2c_handler_get();

    // Step 1: Check data ready
    uint8_t reg = PRESSURE_SENSOR_LPS22HH_STATUS_REG;
    if (!i2c->read_reg(pressure_sensor->i2c_address, &reg, 1, 1)){
    	return false;
    }
//    HAL_Delay(5);

    if (!(i2c->Response_buffer[0] == 0x33)) {
        return false;  // Pressure data not ready
    }

    // Step 2: Read 3-byte pressure (0x28–0x2A), auto-increment bit
    reg = PRESSURE_SENSOR_LPS22HH_PRESS_OUT_XL;// auto-increment
    if (!i2c->read_reg(pressure_sensor->i2c_address, &reg, 1, 5)) return false;

    pressure_sensor->decode_i2c_response(i2c->Response_buffer, 3);


    return false;
}

static void pressure_decode_i2c_response(uint8_t* data, uint8_t len) {
    if (len < 5){

	}
	uint32_t raw_pressure = ((uint32_t) data[2] << 16) | ((uint32_t) data[1] << 8)
			| (uint32_t) data[0];
	float pressure = raw_pressure / 4096.0f; //hpA

	uint16_t raw_temp = (int16_t)((data[4] << 8) | data[3]);
	float temperature = raw_temp / 100.0f;

	Pressure_Data_t reading;
	reading.pressure = pressure*10;
	reading.temperature=temperature*100;
    reading.timestamp = 0x60D4A000;  // Dummy timestamp; replace with RTC if available

    pressure_sensor->history[pressure_sensor->head_index] = reading;
    pressure_sensor->head_index = (pressure_sensor->head_index + 1) % PRESSURE_SENSOR_HISTORY_SIZE;
    if (pressure_sensor->count < PRESSURE_SENSOR_HISTORY_SIZE)
        pressure_sensor->count++;
}
