/*
 * uart_handler.h
 *
 *  Created on: Jun 18, 2025
 *      Author: magaa
 */

#ifndef INC_UART_HANDLER_H_
#define INC_UART_HANDLER_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32u5xx_hal.h"

#define UART_RX_BUFFER_SIZE  64
#define UART_TX_BUFFER_SIZE  64
#define UART_TIMESTAMP_SIZE  4

typedef enum {
    UART_STATE_IDLE = 0,
    UART_STATE_WAITING_RESPONSE,
    UART_STATE_PROCESSING,
    UART_STATE_ERROR
} UART_State_t;

typedef enum {
    CMD_TYPE_MEASURE = 0x01,
    CMD_TYPE_CONFIG  = 0x02,
    CMD_TYPE_STATUS  = 0x03,
    CMD_TYPE_UNKNOWN = 0xFF
} UART_CommandType_t;

typedef struct {
    uint8_t  header;
    uint8_t  cmd_type;
    uint8_t  payload_len;
    uint8_t  payload[UART_RX_BUFFER_SIZE - 6];
    uint8_t  crc;
} UART_CommandPacket_t;

typedef struct {
    uint8_t  header;
    uint8_t  status;
    uint8_t  payload_len;
    uint8_t  payload[UART_TX_BUFFER_SIZE - 10];
    uint8_t  timestamp[UART_TIMESTAMP_SIZE];
    uint8_t  crc;
} UART_ResponsePacket_t;

typedef struct UART_Handler {
    UART_State_t state;
    uint8_t max_payload_len;
    UART_CommandPacket_t *cmd_packet;
    UART_ResponsePacket_t *resp_packet;
    UART_HandleTypeDef *huart;
    void (*process_recived_pck)(uint8_t* pck, uint16_t size);
    void (*send_response)(uint8_t cmd, uint8_t* payload, uint8_t payload_len);
    uint8_t* tx_buffer;
    uint8_t* rx_buffer;
} UART_Handler_t;

// Public interface
void uart_handler_Init(UART_HandleTypeDef *huart);
void uart_handler_DeInit(void);
void uart_handler_Reset_State(void);
UART_Handler_t* uart_handler_get(void);


#endif /* INC_UART_HANDLER_H_ */
