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

bool hts221_wait_data_ready(uint32_t timeout_ms) {
    I2C_Handler_t *i2c = i2c_handler_get();
    uint8_t reg = 0x27;  // STATUS_REG
    uint8_t status = 0;

    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < timeout_ms) {
        if (!i2c->read_reg(temp_sensor->i2c_address, &reg, 1, 1)) {
            return false;
        }
        HAL_Delay(2); // Wait a bit for the sensor

        if (i2c->state == I2C_STATE_MSG_WAITING_FOR_PROCESSING) {
        	i2c->state=I2C_STATE_IDLE;
            status = i2c->Response_buffer[0];
            if ((status & 0x03) == 0x03) {  // Both humidity and temp ready
                return true;
            }
        }
    }

    return false; // Timeout
}

bool hts221_trigger_one_shot_measurement() {
    I2C_Handler_t *i2c_handle = i2c_handler_get();

    // Step 1: Write to CTRL_REG2 to trigger one-shot measurement
    uint8_t trigger_cmd[2] = { TEMP_SENSOR_HTS221_CTRL_REG2, 0x01 };  // ONE_SHOT = 1
    if (!i2c_handle->write_reg(temp_sensor->i2c_address, trigger_cmd, 2)) {
        return false;  // Failed to trigger
    }

    return true;
}

bool hts221_read_calibration(HTS221_Calib_t* calibration_regs) {

	I2C_Handler_t *i2c_handle = i2c_handler_get();
	uint8_t reg;
	uint8_t *buf = i2c_handle->Response_buffer;

	// 1. Read H0_rH and H1_rH (0x30 and 0x31)
	reg = 0x30| 0x80;
	if (!i2c_handle->read_reg(temp_sensor->i2c_address, &reg, 1, 2))
		return false;
	HAL_Delay(2); // Wait for interrupt-driven reception

	if (i2c_handle->state != I2C_STATE_MSG_WAITING_FOR_PROCESSING) {

	}
	calibration_regs->H0_rH = buf[0] / 2.0f;
	calibration_regs->H1_rH = buf[1] / 2.0f;

	// 2. Read H0_T0_OUT (0x36, 0x37)
	reg = 0x36| 0x80;
	if (!i2c_handle->read_reg(temp_sensor->i2c_address, &reg, 1, 2))
		return false;
	HAL_Delay(2);
	if (i2c_handle->state != I2C_STATE_MSG_WAITING_FOR_PROCESSING)
		return false;
	calibration_regs->H0_T0_OUT = (int16_t) ((buf[1] << 8) | buf[0]);

	// 3. Read H1_T0_OUT (0x3A, 0x3B)
	reg = 0x3A| 0x80;
	if (!i2c_handle->read_reg(temp_sensor->i2c_address, &reg, 1, 2))
		return false;
	HAL_Delay(2);
	if (i2c_handle->state != I2C_STATE_MSG_WAITING_FOR_PROCESSING)
		return false;
	calibration_regs->H1_T0_OUT = (int16_t) ((buf[1] << 8) | buf[0]);

	// 4. Read T0_degC_x8 and T1_degC_x8 (0x32, 0x33)
	reg = 0x32| 0x80;
	if (!i2c_handle->read_reg(temp_sensor->i2c_address, &reg, 1, 2))
		return false;
	HAL_Delay(2);
	if (i2c_handle->state != I2C_STATE_MSG_WAITING_FOR_PROCESSING)
		return false;
	uint16_t T0_x8 = buf[0];
	uint16_t T1_x8 = buf[1];

	// 5. Read T1_T0_MSB (0x35)
	reg = 0x35;
	if (!i2c_handle->read_reg(temp_sensor->i2c_address, &reg, 1, 1))
		return false;
	HAL_Delay(2);
	if (i2c_handle->state != I2C_STATE_MSG_WAITING_FOR_PROCESSING)
		return false;
	uint8_t msb = buf[0];
	uint16_t t0msb=(msb & 0x03) << 8;
	uint16_t t1msb=(msb & 0x0C) << 6;

	T0_x8 |= (msb & 0x03) << 8;
	T1_x8 |= (msb & 0x0C) << 6;
	calibration_regs->T0_degC = T0_x8 / 8.0f;
	calibration_regs->T1_degC = T1_x8 / 8.0f;

	// 6. Read T0_OUT (0x3C, 0x3D)
	reg = 0x3C| 0x80;
	if (!i2c_handle->read_reg(temp_sensor->i2c_address, &reg, 1, 2))
		return false;
	HAL_Delay(2);
	if (i2c_handle->state != I2C_STATE_MSG_WAITING_FOR_PROCESSING)
		return false;
	calibration_regs->T0_OUT = (int16_t) ((buf[1] << 8) | buf[0]);

	// 7. Read T1_OUT (0x3E, 0x3F)
	reg = 0x3E| 0x80;
	if (!i2c_handle->read_reg(temp_sensor->i2c_address, &reg, 1, 2))
		return false;
	HAL_Delay(2);
	if (i2c_handle->state != I2C_STATE_MSG_WAITING_FOR_PROCESSING)
		return false;
	calibration_regs->T1_OUT = (int16_t) ((buf[1] << 8) | buf[0]);

	i2c_handle->state=I2C_STATE_IDLE;
	return true;

}


static float interpolate(int16_t x0, float y0, int16_t x1, float y1, int16_t x) {
    if (x1 == x0) return y0; // Avoid divide by zero
    return y0 + ((float)(x - x0) * (y1 - y0)) / (x1 - x0);
}

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
	temp_sensor->get_last_n_data_serial_format =
			temp_sensor_get_last_n_temp_data_serial_format;
	temp_sensor->trigger_data_collection = temp_sensor_trigger_data_collection;
	temp_sensor->trigger_measurement=hts221_trigger_one_shot_measurement;
	memset(temp_data_ptr_buffer, 0, sizeof(temp_data_ptr_buffer));

	// TODO: Perform initial I2C check or sensor config if needed

	I2C_Handler_t *i2c_handle = i2c_handler_get();
	uint8_t reg = TEMP_SENSOR_HTS221_WHO_AM_I_REG;

	// Step 1: Check WHO_AM_I
	if (!i2c_handle->read_reg(temp_sensor->i2c_address, &reg, 1, 1)) {

	}
	HAL_Delay(10); // Optional: wait for completion

	if (i2c_handle->state == I2C_STATE_MSG_WAITING_FOR_PROCESSING) {
		i2c_handle->state = I2C_STATE_IDLE;
		if (i2c_handle->Response_buffer[0] = TEMP_SENSOR_HTS221_WHO_AM_I_VAL) {
			//OKK

		}
	}

	// Step 2: Optional - Reboot memory content
//	uint8_t reboot_cmd[2] = { TEMP_SENSOR_HTS221_CTRL_REG2, 0x80 }; // BOOT bit = 1
//	if (!i2c_handler_write_reg(temp_sensor->i2c_address, reboot_cmd, 2)) {
//
//	}
//	HAL_Delay(15);  // Wait for reboot
//
//	if (i2c_handle->state == I2C_STATE_IDLE) {
//		//OKK
//	}


	//Step 4: Read calibration registers
//	hts221_read_calibration(&(temp_sensor->calibration_regs));

	// Step 3: Enable sensor in continuous mode at 1 Hz

	uint8_t init_cmd[2] = { TEMP_SENSOR_HTS221_CTRL_REG1, 0x81 }; // PD=1, BDU=0, ODR=1Hz
	if (!i2c_handler_write_reg(temp_sensor->i2c_address, init_cmd, 2)) {

	}

	HAL_Delay(15);  // Wait for reboot
	if (i2c_handle->state == I2C_STATE_IDLE) {
		//OKK
	}

	//Step 4: Read calibration registers
	hts221_read_calibration(&(temp_sensor->calibration_regs));



	HAL_Delay(15);  // Wait for reboot
	if (i2c_handle->state == I2C_STATE_IDLE) {
		//OKK
	}

//	temp_sensor->trigger_data_collection();
//		HAL_Delay(20);  // Wait for reboot

//	temp_sensor->trigger_measurement();

//	HAL_Delay(10000);  // Wait for reboot
	temp_sensor->trigger_data_collection();
	HAL_Delay(1000);  // Wait for reboot

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

    // HTS221 returns temperature and humidity LSB first (little-endian)
	int16_t H_out = (int16_t) ((data[1] << 8) | data[0]);  // Humidity raw
	int16_t T_out = (int16_t) ((data[3] << 8) | data[2]);  // Temperature raw

	HTS221_Calib_t* calib=&(temp_sensor->calibration_regs);

	float humidity = interpolate(calib->H0_T0_OUT, calib->H0_rH,
			calib->H1_T0_OUT, calib->H1_rH, H_out);

	float temperature = interpolate(calib->T0_OUT, calib->T0_degC,
			calib->T1_OUT, calib->T1_degC, T_out);

    sample.temperature = temperature*10;
    sample.humidity    = humidity*10;

    // TODO: Replace with real-time acquisition
    sample.timestamp = 0x60D4A000; // Fixed dummy Unix timestamp

    temp_sensor->history[temp_sensor->head_index] = sample;
    temp_sensor->head_index = (temp_sensor->head_index + 1) % TEMP_HUMIDITY_SENSOR_HISTORY_SIZE;
    if (temp_sensor->count < TEMP_HUMIDITY_SENSOR_HISTORY_SIZE)
        temp_sensor->count++;

}

bool temp_sensor_trigger_data_collection(){
	I2C_Handler_t* i2c_handle=i2c_handler_get();


	// Wait until the sensor sets both H_DA and T_DA
	    if (!hts221_wait_data_ready(20000)) {
	        return false; // Timeout or error
	    }

	uint8_t reg = TEMP_SENSOR_HTS221_TEMP_OUT_L | 0x80; // Auto-increment
	i2c_handle->read_reg(temp_sensor->i2c_address, &reg, 1, 2);

	HAL_Delay(5);
	if (i2c_handle->state != I2C_STATE_MSG_WAITING_FOR_PROCESSING) {
		return false;
	}

return true;
}
