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
    uint8_t last_address;
    uint8_t Write_buffer[I2C_MAX_BUFFER_SIZE];
    uint8_t write_buffer_lenght;
    uint8_t Response_buffer[I2C_MAX_BUFFER_SIZE];
    uint8_t response_buffer_lenght;
    bool (*write_reg)(uint8_t device_addr, uint8_t* data, uint8_t len);
    bool (*read_reg)(uint8_t device_addr, uint8_t* data_out, uint8_t len);
} I2C_Handler_t;

// Initialization
void i2c_handler_init(I2C_HandleTypeDef* hi2c);
I2C_Handler_t* i2c_handler_get(void);
void i2c_handler_deinit();



#endif /* INC_I2C_HANDLER_H_ */
