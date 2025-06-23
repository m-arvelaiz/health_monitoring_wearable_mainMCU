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



#define TEMP_SENSOR_HTS221_WHO_AM_I_REG   0x0F
#define TEMP_SENSOR_HTS221_WHO_AM_I_VAL   0xBC
#define TEMP_SENSOR_HTS221_CTRL_REG1      0x20
#define TEMP_SENSOR_HTS221_CTRL_REG2      0x21
#define TEMP_SENSOR_HTS221_STATUS_REG     0x27
#define TEMP_SENSOR_HTS221_HUMIDITY_OUT_L   0x28
#define TEMP_SENSOR_HTS221_TEMP_OUT_L       0x2A



typedef struct {
    float T0_degC;
    float T1_degC;
    int16_t T0_OUT;
    int16_t T1_OUT;

    float H0_rH;
    float H1_rH;
    int16_t H0_T0_OUT;
    int16_t H1_T0_OUT;
} HTS221_Calib_t;

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


    //Custom from the sensor
    HTS221_Calib_t calibration_regs;

    // Function pointers for custom handling
    void (*format_uart_response)(uint8_t* payload_out); // Local function in .c
    Temp_Data_t* (*get_last_data)();
    Temp_Data_t** (*get_last_n_data)(uint8_t n);
    uint8_t (*get_last_n_data_serial_format)(uint8_t n, uint8_t* out_buffer);
//    void (*prepare_i2c_request)(uint8_t* payload_out, uint8_t* lenght);  // Prepares I2C payload
    void (*decode_i2c_response)(uint8_t* data, uint8_t len); // Parses I2C rx data
    bool (*trigger_data_collection)();
    bool (*trigger_measurement)();
} Temp_Sensor_t;

// Initialize and release lifecycle
void temp_sensor_init(uint8_t address);
void temp_sensor_deinit(void);

// Accessor
Temp_Sensor_t* temp_sensor_get(void);

#endif /* INC_TEMP_SENSOR_H_ */
