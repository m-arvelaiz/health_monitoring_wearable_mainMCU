/*
 * ppg_sensor.c
 *
 *  Created on: Jun 21, 2025
 *      Author: magaa
 */

#include "ppg_sensor.h"
#include "i2c_handler.h"
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

static PPG_Sensor_t* ppg_sensor = NULL;
static PPG_Data_t* ppg_data_ptr_buffer[PPG_SENSOR_HISTORY_SIZE];


// === Local Prototypes ===
static void ppg_format_uart_response(uint8_t* payload_out);
static PPG_Data_t* ppg_get_last_data();
static PPG_Data_t** ppg_get_last_n_data(uint8_t n);
static uint8_t ppg_get_last_n_serial_format(uint8_t n, uint8_t* out);
static void ppg_decode_i2c_response(uint8_t* data, uint8_t len);
static bool ppg_trigger_measurement();
static bool ppg_trigger_data_collection();



bool ppg_custom_read_command(uint8_t device_addr, uint8_t cmd, uint8_t* out, uint8_t len) {
	I2C_Handler_t* i2c_handler = i2c_handler_get();
	// 1) send the “get HR/SpO2” command
    if (HAL_I2C_Master_Transmit(i2c_handler->hi2c, device_addr << 1, &cmd, 1, I2C_DELAY) != HAL_OK) {
        return false;
    }
    // 2) then read back len bytes
    if (HAL_I2C_Master_Receive(i2c_handler->hi2c, device_addr << 1, out, len, I2C_DELAY) != HAL_OK) {
        return false;
    }
    return true;
}


void ppg_sensor_init(uint8_t address) {
    ppg_sensor = (PPG_Sensor_t*) malloc(sizeof(PPG_Sensor_t));
    memset(ppg_sensor, 0, sizeof(PPG_Sensor_t));
    memset(ppg_data_ptr_buffer, 0, sizeof(ppg_data_ptr_buffer));

    ppg_sensor->i2c_address = address;
    ppg_sensor->format_uart_response = ppg_format_uart_response;
    ppg_sensor->get_last_data = ppg_get_last_data;
    ppg_sensor->get_last_n_data = ppg_get_last_n_data;
    ppg_sensor->get_last_n_data_serial_format = ppg_get_last_n_serial_format;
    ppg_sensor->decode_i2c_response = ppg_decode_i2c_response;
    ppg_sensor->trigger_data_collection = ppg_trigger_data_collection;
    ppg_sensor->trigger_measurement = ppg_trigger_measurement;

    // TODO: Initialize SEN0344 / MAX30102 sensor configuration:
    // - Reset, mode configuration
    // - Set SpO2 mode, sample rate, LED current
    // - Enable interrupts if needed
    // - Power on sequence
    // - WHO_AM_I equivalent if applicable

    // Example (placeholder):
    // uint8_t init_cmd[2] = { SENSOR_REG_MODE_CONFIG, 0x03 };
    // i2c_handler_write_reg(address, init_cmd, 2);
    I2C_Handler_t* i2c = i2c_handler_get();
    uint8_t ctrl1[3] = { PPG_SENSOR_SEN0344_START_STOP, 0x00,0x01 };
//	if (!i2c->write_reg(ppg_sensor->i2c_address, ctrl1, 3)) {
//		return;
//	}

    if (HAL_I2C_Master_Transmit(i2c->hi2c, address << 1, ctrl1, 3, I2C_DELAY) != HAL_OK) {
            return false;
        }
    HAL_Delay(5);
}

void ppg_sensor_deinit(void) {
    if (ppg_sensor) {
        free(ppg_sensor);
        ppg_sensor = NULL;
    }
}

PPG_Sensor_t* ppg_sensor_get(void) {
    return ppg_sensor;
}



// === Local Functions ===

static void ppg_format_uart_response(uint8_t* payload_out) {
    PPG_Data_t* last = &ppg_sensor->history[
        (ppg_sensor->head_index - 1 + PPG_SENSOR_HISTORY_SIZE) % PPG_SENSOR_HISTORY_SIZE];


	payload_out[0] = (last->hr >> 8) & 0xFF;
	payload_out[1] = (last->hr >> 0) & 0xFF;
	payload_out[2] = (last->spo2 >> 8) & 0xFF;
	payload_out[3] = (last->spo2 >> 0) & 0xFF;
	payload_out[4] = 0x00;
	payload_out[5] = 0x00;
	payload_out[6] = (last->timestamp >> 24) & 0xFF;
	payload_out[7] = (last->timestamp >> 16) & 0xFF;
    payload_out[8] = (last->timestamp >> 8)  & 0xFF;
    payload_out[9] = (last->timestamp >> 0)  & 0xFF;
}

static PPG_Data_t* ppg_get_last_data() {
    return &ppg_sensor->history[
        (ppg_sensor->head_index - 1 + PPG_SENSOR_HISTORY_SIZE) % PPG_SENSOR_HISTORY_SIZE];
}

static PPG_Data_t** ppg_get_last_n_data(uint8_t n) {
    if (!ppg_sensor || n == 0 || n > ppg_sensor->count) return NULL;

    for (uint8_t i = 0; i < n; ++i) {
        int index = (ppg_sensor->head_index - 1 - i + PPG_SENSOR_HISTORY_SIZE) % PPG_SENSOR_HISTORY_SIZE;
        ppg_data_ptr_buffer[i] = &ppg_sensor->history[index];
    }
    return ppg_data_ptr_buffer;
}

static uint8_t ppg_get_last_n_serial_format(uint8_t n, uint8_t* out) {
    if (!ppg_sensor || !out || n == 0 || n > ppg_sensor->count) return 0;

    for (uint8_t i = 0; i < n; ++i) {
        int index = (ppg_sensor->head_index - 1 - i + PPG_SENSOR_HISTORY_SIZE) % PPG_SENSOR_HISTORY_SIZE;
        PPG_Data_t* data = &ppg_sensor->history[index];

        out[i * 8 + 0] = (data->hr >> 8) & 0xFF;
        out[i * 8 + 1] = (data->hr >> 0)  & 0xFF;
        out[i * 8 + 2] = (data->spo2 >> 8)  & 0xFF;
        out[i * 8 + 3] = (data->spo2 >> 0)  & 0xFF;

        out[i * 8 + 4] = (data->timestamp >> 24) & 0xFF;
        out[i * 8 + 5] = (data->timestamp >> 16) & 0xFF;
        out[i * 8 + 6] = (data->timestamp >> 8)  & 0xFF;
        out[i * 8 + 7] = (data->timestamp >> 0)  & 0xFF;
    }

    return (n * 8);
}

uint8_t ppg_get_last_n_bodytemp_serial_format(uint8_t n, uint8_t* out) {
    if (!ppg_sensor || !out || n == 0 || n > ppg_sensor->count) return 0;

	uint8_t i;
	for (i = 0; i < n; ++i) {
		int index = (ppg_sensor->head_index - 1 - i
				+ PPG_SENSOR_HISTORY_SIZE)
				% PPG_SENSOR_HISTORY_SIZE;
		PPG_Data_t *data = &ppg_sensor->history[index];


		// Pack: [TEMP x100 MSB, LSB], [TIMESTAMP MSB to LSB]
		out[i * 8 + 0] = (data->temp >> 8) & 0xFF;
		out[i * 8 + 1] = (data->temp >> 0) & 0xFF;
		out[i * 8 + 2] = 0x00;
		out[i * 8 + 3] = 0x00;
		out[i * 8 + 4] = (data->timestamp >> 24) & 0xFF;
		out[i * 8 + 5] = (data->timestamp >> 16) & 0xFF;
		out[i * 8 + 6] = (data->timestamp >> 8) & 0xFF;
		out[i * 8 + 7] = (data->timestamp >> 0) & 0xFF;
	}

	return (i * 8);
}

static bool ppg_trigger_measurement() {
    // NOT needed here
    return true;
}

static bool ppg_trigger_data_collection() {
	I2C_Handler_t *i2c = i2c_handler_get();

	uint8_t reg = PPG_SENSOR_SEN0344_HR_SPO2;
//	if (!i2c->read_reg(ppg_sensor->i2c_address, &reg, 1, 8)) {
//		i2c->state=I2C_STATE_IDLE;
//		return false;
//	}
	if (!ppg_custom_read_command(ppg_sensor->i2c_address, reg, i2c->Response_buffer, 10))
	        return false;

    // TODO: Read red & IR from FIFO or sensor registers
    // Example (pseudo):
    // uint8_t reg = FIFO_DATA_REGISTER;
    // i2c->read_reg(ppg_sensor->i2c_address, &reg, 1, 6);  // 3 bytes red + 3 bytes IR




    ppg_sensor->decode_i2c_response(i2c->Response_buffer, 10);

    return true;
}

static void ppg_decode_i2c_response(uint8_t* data, uint8_t len) {
    // TODO: Parse raw bytes into red/IR values if using direct reads from FIFO
	// sample.red = ((data[0] << 16) | (data[1] << 8) | data[2]);
	// sample.ir  = ((data[3] << 16) | (data[4] << 8) | data[5]);

	PPG_Data_t sample;
	sample.spo2 = ((uint8_t)data[0]);;
	sample.hr = ((uint32_t)data[2] << 24) | ((uint32_t)data[3] << 16) |
		       ((uint32_t)data[4] << 8) | ((uint32_t)data[5]);

	float Temperature = data[8] * 1.0 + data[9] / 100.0;
	sample.temp=(uint16_t)(Temperature*100);

	sample.timestamp = 0x60D4A000; // Replace with RTC time

	ppg_sensor->history[ppg_sensor->head_index] = sample;
	ppg_sensor->head_index = (ppg_sensor->head_index + 1)
			% PPG_SENSOR_HISTORY_SIZE;
	if (ppg_sensor->count < PPG_SENSOR_HISTORY_SIZE)
		ppg_sensor->count++;
}
