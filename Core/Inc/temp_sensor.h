/*
 * temp_sensor.h
 *
 *  Created on: Jun 21, 2025
 *      Author: magaa
 */

#ifndef INC_TEMP_SENSOR_H_
#define INC_TEMP_SENSOR_H_

#include <stdint.h>
#include <stdbool.h>


#define TEMP_SENSOR_ADDRESS 0
#define TEMP_HUMIDITY_SENSOR_HISTORY_SIZE 10

// Sensor-specific data type
typedef struct {
    uint32_t timestamp;  // Unix time (seconds)
    uint16_t temperature; // in tenths of °C (e.g., 365 = 36.5°C)
    uint16_t humidity;    // in tenths of %RH (e.g., 455 = 45.5%) (optional/expandable)
} Temp_Data_t;

// Sensor instance structure
typedef struct Temp_Sensor {
    uint8_t i2c_address;
    Temp_Data_t history[TEMP_HUMIDITY_SENSOR_HISTORY_SIZE];
    uint8_t head_index;
    uint8_t count;

    // Function pointers for custom handling
    void (*format_uart_response)(uint8_t* payload_out); // Local function in .c
    Temp_Data_t* (*get_last_data)();
    Temp_Data_t** (*get_last_n_data)(uint8_t n);
    uint8_t (*get_last_n_data_serial_format)(uint8_t n, uint8_t* out_buffer);
    void (*prepare_i2c_request)(uint8_t* payload_out);  // Prepares I2C payload
    void (*decode_i2c_response)(uint8_t* data, uint8_t len); // Parses I2C rx data
} Temp_Sensor_t;

// Initialize and release lifecycle
void temp_sensor_init(uint8_t address);
void temp_sensor_deinit(void);

// Accessor
Temp_Sensor_t* temp_sensor_get(void);

#endif /* INC_TEMP_SENSOR_H_ */
