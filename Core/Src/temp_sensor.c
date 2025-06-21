/*
 * temp_sensor.c
 *
 *  Created on: Jun 21, 2025
 *      Author: magaa
 */


#include "temp_sensor.h"
#include "i2c_handler.h"
#include <stdlib.h>
#include <string.h>

static Temp_Sensor_t* temp_sensor = NULL;

// === Local Function Prototypes ===
static void temp_format_uart_response(uint8_t* payload_out);
static void temp_prepare_i2c_request(uint8_t* payload_out);
static void temp_decode_i2c_response(uint8_t* data, uint8_t len);


// === Public API ===


void temp_sensor_init(uint8_t address) {
    if (temp_sensor == NULL) {
        temp_sensor = (Temp_Sensor_t*) malloc(sizeof(Temp_Sensor_t));
    }
    memset(temp_sensor, 0, sizeof(Temp_Sensor_t));

    temp_sensor->i2c_address = address;
    temp_sensor->format_uart_response = temp_format_uart_response;
    temp_sensor->prepare_i2c_request = temp_prepare_i2c_request;
    temp_sensor->decode_i2c_response = temp_decode_i2c_response;

    // TODO: Perform initial I2C check or sensor config if needed
}



void temp_sensor_deinit(void) {
    if (temp_sensor != NULL) {
        free(temp_sensor);
        temp_sensor = NULL;
    }
    // TODO: Additional cleanup if needed
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



static void temp_prepare_i2c_request(uint8_t* payload_out) {
    // TODO: Set up sensor-specific I2C command to trigger a measurement
    // Example: write register address to start measurement
    payload_out[0] = 0x00; // Replace with actual command if needed
}

static void temp_decode_i2c_response(uint8_t* data, uint8_t len) {
    if (len < 4) return; // Expecting at least temp + hum

    Temp_Data_t sample;
    sample.temperature = (data[0] << 8) | data[1];
    sample.humidity    = (data[2] << 8) | data[3];

    // TODO: Replace with real-time acquisition
    sample.timestamp = 0x60D4A000; // Fixed dummy Unix timestamp

    temp_sensor->history[temp_sensor->head_index] = sample;
    temp_sensor->head_index = (temp_sensor->head_index + 1) % TEMP_HUMIDITY_SENSOR_HISTORY_SIZE;
    if (temp_sensor->count < TEMP_HUMIDITY_SENSOR_HISTORY_SIZE)
        temp_sensor->count++;

    // TODO: Notify main system or trigger response transmission
}
