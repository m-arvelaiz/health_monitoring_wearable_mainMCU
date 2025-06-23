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
#define I2C_DELAY 100

// Basic operations
bool i2c_handler_write_reg(uint8_t device_addr, uint8_t* buffer, uint8_t len);
bool i2c_handler_read_reg(uint8_t device_addr, uint8_t* buffer, uint8_t len, uint8_t len_expected);

void HAL_I2C_MasterTxCpltCallback (I2C_HandleTypeDef * hi2c)
{
	if(i2c_handler->hi2c==hi2c){
		if(i2c_handler->operation_type==I2C_OP_TYPE_WRITE_REG){
			i2c_handler->state= I2C_STATE_IDLE;
		}else{
			i2c_handler->state= I2C_STATE_BUSY;

			if (HAL_I2C_Master_Receive_IT(i2c_handler->hi2c, i2c_handler->last_address << 1,
						i2c_handler->Response_buffer, i2c_handler->response_buffer_lenght) != HAL_OK) {

			}
		}


	}
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){

}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c){

}

void HAL_I2C_MasterRxCpltCallback (I2C_HandleTypeDef * hi2c)
 {
	if (i2c_handler->hi2c == hi2c) {
		i2c_handler->state= I2C_STATE_MSG_WAITING_FOR_PROCESSING;
	}
}


bool i2c_handler_write_reg(uint8_t device_addr, uint8_t* buffer, uint8_t len) {
	i2c_handler->last_address = device_addr;

	if (!i2c_handler || !i2c_handler->hi2c)
		return false;

	i2c_handler->state = I2C_STATE_BUSY;
	i2c_handler->operation_type = I2C_OP_TYPE_WRITE_REG;

    if (HAL_I2C_Master_Transmit_IT(i2c_handler->hi2c, device_addr << 1, buffer, len) != HAL_OK) {
        return false;
    }
//	if (HAL_I2C_Master_Transmit(i2c_handler->hi2c, device_addr << 1, buffer,
//			len, I2C_DELAY) != HAL_OK) {
//		return false;
//	}

	i2c_handler->state = I2C_STATE_IDLE;
	return true;
}

bool i2c_handler_read_reg(uint8_t device_addr, uint8_t* buffer, uint8_t len, uint8_t len_expected) {

	i2c_handler->last_address = device_addr;
	i2c_handler->response_buffer_lenght=len_expected;
	if (!i2c_handler || !i2c_handler->hi2c ){
		return false;
	}
	i2c_handler->state= I2C_STATE_BUSY;
	i2c_handler->operation_type= I2C_OP_TYPE_READ_REG;


//VERSION1
//	if (HAL_I2C_Master_Transmit_IT(i2c_handler->hi2c, device_addr << 1,
//			buffer, len) != HAL_OK) {
//		return false;
//	}
// 	i2c_handler->state= I2C_STATE_BUSY;

//VERSION2

//	// Transmit register address
//	if (HAL_I2C_Master_Transmit(i2c_handler->hi2c, device_addr << 1, buffer,
//			len, I2C_DELAY) != HAL_OK) {
//		return false;
//	}
////
//	// Immediately follow with read
//	if (HAL_I2C_Master_Receive(i2c_handler->hi2c, device_addr << 1,
//			i2c_handler->Response_buffer, len_expected, I2C_DELAY) != HAL_OK) {
//		return false;
//	}


	if (HAL_I2C_Mem_Read(i2c_handler->hi2c,
		                     device_addr << 1,
		                     *buffer,
		                     I2C_MEMADD_SIZE_8BIT,
		                     i2c_handler->Response_buffer,
		                     len_expected,
		                     I2C_DELAY) != HAL_OK) {
			return false;
		}
	i2c_handler->state= I2C_STATE_IDLE;


	return true;
}


void i2c_handler_init(I2C_HandleTypeDef* hi2c) {
    if (i2c_handler == NULL) {
        i2c_handler = (I2C_Handler_t*) malloc(sizeof(I2C_Handler_t));
    }
    i2c_handler->hi2c = hi2c;
    i2c_handler->state= I2C_STATE_IDLE;
    i2c_handler->operation_type=I2C_OP_TYPE_NONE;
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

