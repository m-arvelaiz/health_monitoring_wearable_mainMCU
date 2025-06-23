/*
 * hr&spo2_sensor.h
 *
 *  Created on: Jun 21, 2025
 *      Author: magaa
 */

#ifndef INC_PPG_SENSOR_H_
#define INC_PPG_SENSOR_H_

#include <stdint.h>
#include <stdbool.h>

#define PPG_SENSOR_SEN0344_START_STOP 0x20
#define PPG_SENSOR_SEN0344_HR_SPO2 0x0C
#define PPG_SENSOR_SEN0344_TEMP 0x14


#define PPG_SENSOR_HISTORY_SIZE 10

// Sensor-specific data type
typedef struct {
	uint32_t timestamp;  // Unix time (seconds)
	uint8_t spo2;        // Oxigen saturation
	uint32_t hr;         // heart rate
	uint16_t temp;       //patient temperature
//	uint16_t red;        // Red light reading
//	uint32_t ir;         // Infrared light reading
// Optionally: add green or status if needed
} PPG_Data_t;

// Sensor instance structure
typedef struct PPG_Sensor {
	uint8_t i2c_address;
	PPG_Data_t history[PPG_SENSOR_HISTORY_SIZE];
	uint8_t head_index;
	uint8_t count;

	// Function pointers for custom handling
	void (*format_uart_response)(uint8_t *payload_out); // Local function in .c
	PPG_Data_t* (*get_last_data)();
    PPG_Data_t** (*get_last_n_data)(uint8_t n);
    uint8_t (*get_last_n_data_serial_format)(uint8_t n, uint8_t* out_buffer);
    void (*decode_i2c_response)(uint8_t* data, uint8_t len); // Parses I2C rx data
    bool (*trigger_data_collection)();
    bool (*trigger_measurement)();
} PPG_Sensor_t;

// Initialize and release lifecycle
void ppg_sensor_init(uint8_t address);
void ppg_sensor_deinit(void);

// Accessor
PPG_Sensor_t* ppg_sensor_get(void);


#endif /* INC_PPG_SENSOR_H_ */
