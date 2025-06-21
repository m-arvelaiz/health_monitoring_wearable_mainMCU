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

// Basic operations
bool i2c_handler_write_reg(uint8_t device_addr, uint8_t* data, uint8_t len);
bool i2c_handler_read_reg(uint8_t device_addr, uint8_t* data_out, uint8_t len);


bool i2c_handler_write_reg(uint8_t device_addr) {
	i2c_handler->last_address=device_addr;
    if (!i2c_handler || !i2c_handler->hi2c || !i2c_handler->Write_buffer || i2c_handler->write_buffer_lenght == 0) return false;

    if (HAL_I2C_Master_Transmit(i2c_handler->hi2c, device_addr << 1, i2c_handler->Write_buffer, i2c_handler->write_buffer_lenght, HAL_MAX_DELAY) != HAL_OK) {
        return false;
    }
    return true;
}

bool i2c_handler_read_reg(uint8_t device_addr) {

	i2c_handler->last_address = device_addr;
	if (!i2c_handler || !i2c_handler->hi2c || !i2c_handler->Write_buffer
			|| i2c_handler->write_buffer_lenght == 0){
		return false;
	}
	if (HAL_I2C_Master_Transmit(i2c_handler->hi2c, device_addr << 1,
			i2c_handler->Write_buffer, i2c_handler->write_buffer_lenght,
			HAL_MAX_DELAY) != HAL_OK) {
		return false;
	}

	if (!i2c_handler || !i2c_handler->hi2c || !i2c_handler->Response_buffer){
		return false;}

	//TODO: Check if Interruption needed

	if (HAL_I2C_Master_Receive(i2c_handler->hi2c, device_addr << 1,
			i2c_handler->Response_buffer, i2c_handler->response_buffer_lenght,
			HAL_MAX_DELAY) != HAL_OK) {
		return false;
	}
	return true;
}


void i2c_handler_init(I2C_HandleTypeDef* hi2c) {
    if (i2c_handler == NULL) {
        i2c_handler = (I2C_Handler_t*) malloc(sizeof(I2C_Handler_t));
    }
    i2c_handler->hi2c = hi2c;
    memset(i2c_handler->Write_buffer, 0, I2C_MAX_BUFFER_SIZE);
    memset(i2c_handler->Response_buffer, 0, I2C_MAX_BUFFER_SIZE);
    i2c_handler->read_reg=i2c_handler_read_reg;
    i2c_handler->write_reg=i2c_handler_write_reg;
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

