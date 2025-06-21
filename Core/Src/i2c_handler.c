/*
 * i2c_handler.c
 *
 *  Created on: Jun 21, 2025
 *      Author: magaa
 */

#include "i2c_handler.h"
#include <stdlib.h>
#include <string.h>

static I2C_Handler_t* i2c_handler = NULL;


bool i2c_handler_write(uint8_t device_addr, uint8_t* data, uint8_t len) {
    if (!i2c_handler || !i2c_handler->hi2c || !data || len == 0) return false;

    if (HAL_I2C_Master_Transmit(i2c_handler->hi2c, device_addr << 1, data, len, HAL_MAX_DELAY) != HAL_OK) {
        return false;
    }
    return true;
}

bool i2c_handler_read(uint8_t device_addr, uint8_t* data_out, uint8_t len) {
    if (!i2c_handler || !i2c_handler->hi2c || !data_out || len == 0) return false;

    if (HAL_I2C_Master_Receive(i2c_handler->hi2c, device_addr << 1, data_out, len, HAL_MAX_DELAY) != HAL_OK) {
        return false;
    }
    return true;
}


void i2c_handler_init(I2C_HandleTypeDef* hi2c) {
    if (i2c_handler == NULL) {
        i2c_handler = (I2C_Handler_t*) malloc(sizeof(I2C_Handler_t));
    }
    i2c_handler->hi2c = hi2c;
}

I2C_Handler_t* i2c_handler_get(void) {
    return i2c_handler;
}

void i2c_handler_deinit(void) {
    if (i2c_handler != NULL) {
        free(i2c_handler);
        i2c_handler = NULL;
    }
}

