/*
 * i2c_handler.h
 *
 *  Created on: Jun 21, 2025
 *      Author: magaa
 */

#ifndef INC_I2C_HANDLER_H_
#define INC_I2C_HANDLER_H_

typedef enum {
    I2C_STATE_IDLE = 0,
    I2C_STATE_BUSY,
    I2C_STATE_ERROR
} I2C_State_t;

#include <stdint.h>
#include <stdbool.h>
#include "stm32u5xx_hal.h"  // Adjust as needed for your MCU

#define I2C_MAX_BUFFER_SIZE 32

// Simplified I2C Handler

typedef struct I2C_Handler {
    I2C_HandleTypeDef* hi2c;
} I2C_Handler_t;

// Initialization
void i2c_handler_init(I2C_HandleTypeDef* hi2c);
I2C_Handler_t* i2c_handler_get(void);
void i2c_handler_deinit(I2C_HandleTypeDef* hi2c);

// Basic operations
bool i2c_handler_write(uint8_t device_addr, uint8_t* data, uint8_t len);
bool i2c_handler_read(uint8_t device_addr, uint8_t* data_out, uint8_t len);

#endif /* INC_I2C_HANDLER_H_ */
