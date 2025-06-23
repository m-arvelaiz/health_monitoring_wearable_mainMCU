/*
 * pressure_sensor.h
 *
 *  Created on: Jun 21, 2025
 *      Author: magaa
 */

#ifndef INC_PRESSURE_SENSOR_H_
#define INC_PRESSURE_SENSOR_H_

#include <stdint.h>
#include <stdbool.h>

/* Register Definitions for LPS22HH */
#define PRESSURE_SENSOR_LPS22HH_WHO_AM_I_REG   0x0F
#define PRESSURE_SENSOR_LPS22HH_WHO_AM_I_VAL   0xB3
#define PRESSURE_SENSOR_LPS22HH_CTRL_REG1      0x10
#define PRESSURE_SENSOR_LPS22HH_CTRL_REG2      0x11
#define PRESSURE_SENSOR_LPS22HH_STATUS_REG     0x27
#define PRESSURE_SENSOR_LPS22HH_PRESS_OUT_XL   0x28  // XL = least significant byte

#define PRESSURE_SENSOR_HISTORY_SIZE 10



// Sensor-specific data type
typedef struct {
    uint32_t timestamp;  // Unix time (seconds)
    uint32_t pressure;   // in Pa (e.g., 101325 = 1013.25 hPa)
    uint16_t temperature;
} Pressure_Data_t;

// Sensor instance structure
typedef struct Pressure_Sensor {
    uint8_t i2c_address;
    Pressure_Data_t history[PRESSURE_SENSOR_HISTORY_SIZE];
    uint8_t head_index;
    uint8_t count;

    // Function pointers for custom handling
    void (*format_uart_response)(uint8_t* payload_out); // Local function in .c
    Pressure_Data_t* (*get_last_data)();
    Pressure_Data_t** (*get_last_n_data)(uint8_t n);
    uint8_t (*get_last_n_data_serial_format)(uint8_t n, uint8_t* out_buffer);
    void (*decode_i2c_response)(uint8_t* data, uint8_t len); // Parses I2C rx data
    bool (*trigger_data_collection)();
    bool (*trigger_measurement)();
} Pressure_Sensor_t;


// Initialize and release lifecycle
void pressure_sensor_init(uint8_t address);
void pressure_sensor_deinit(void);

// Accessor
Pressure_Sensor_t* pressure_sensor_get(void);



uint8_t pressure_get_temp_last_n_serial_format(uint8_t n, uint8_t* out);

#endif /* INC_PRESSURE_SENSOR_H_ */
