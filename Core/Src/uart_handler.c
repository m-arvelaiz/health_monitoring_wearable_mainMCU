/*
 * uart_handler.c
 *
 *  Created on: Jun 18, 2025
 *      Author: magaa
 */


/*
 * uart_handler_main.c (for MAIN MCU)
 * Adjusted UART handler to receive BLE MCU requests
 */

#include "uart_handler.h"
#include "data_handler.h"
#include <stdlib.h>
#include <string.h>
#include <main.h>

static UART_Handler_t *uart_handler = NULL;

static uint8_t tx_buffer[UART_TX_BUFFER_SIZE];
static uint8_t rx_buffer[UART_RX_BUFFER_SIZE];

// Forward declarations
static void uart_handler_Process_Received_pck(uint8_t* pck, uint16_t size);
static void uart_handler_Send_Response(uint8_t cmd, uint8_t* payload, uint8_t payload_len);

static uint8_t Calculate_CRC(const uint8_t *data, uint8_t length) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < length; ++i) {
        crc ^= data[i];
    }
    return crc;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (Size >= 7 && rx_buffer[0] == 0xAA) {
        uart_handler->process_recived_pck(rx_buffer, Size);
    }
    HAL_UARTEx_ReceiveToIdle_IT(uart_handler->huart, rx_buffer, UART_RX_BUFFER_SIZE);
}

static void uart_handler_Process_Received_pck(uint8_t* pck, uint16_t size) {
    if (size < 7) return;

    uint8_t received_crc = pck[size - 1];
    uint8_t calculated_crc = Calculate_CRC(&pck[1], size - 2); // exclude start byte and CRC
    if (received_crc != calculated_crc) {
        uart_handler->state = UART_STATE_ERROR;
        return;
    }

    uart_handler->cmd_packet->header = pck[0];
    uart_handler->cmd_packet->cmd_type = pck[1];
    uart_handler->cmd_packet->payload_len = pck[2];
    memcpy(uart_handler->cmd_packet->payload, &pck[3], 4);
    uart_handler->cmd_packet->crc = received_crc;

    uart_handler->state = UART_STATE_PROCESSING;

    // Here you should now call your command dispatcher logic:
    data_handler_dispatcher(uart_handler->cmd_packet);
}

static void uart_handler_Send_Response(uint8_t cmd, uint8_t* payload, uint8_t payload_len) {
    uint8_t* buf = tx_buffer;
    uint8_t idx = 0;

    buf[idx++] = 0xAA;                     // Start byte
    buf[idx++] = cmd;                  // Response CMD/status
    buf[idx++] = payload_len;             // Payload length

    memcpy(&buf[idx], payload, payload_len);
    idx += payload_len;


    buf[idx] = Calculate_CRC(&buf[1], idx - 1); // CRC over CMD + LEN + payload + timestamp
    idx++;

    HAL_UART_Transmit(uart_handler->huart, buf, idx, HAL_MAX_DELAY);
    uart_handler->state = UART_STATE_IDLE;
}

void uart_handler_Init(UART_HandleTypeDef *huart) {
    uart_handler = (UART_Handler_t *)malloc(sizeof(UART_Handler_t));
    uart_handler->cmd_packet = (UART_CommandPacket_t *)malloc(sizeof(UART_CommandPacket_t));
    uart_handler->resp_packet = (UART_ResponsePacket_t *)malloc(sizeof(UART_ResponsePacket_t));

    uart_handler->huart = huart;
    uart_handler->state = UART_STATE_IDLE;
    uart_handler->max_payload_len = UART_RX_BUFFER_SIZE - 6;
    uart_handler->rx_buffer = rx_buffer;
    uart_handler->tx_buffer = tx_buffer;

    uart_handler->process_recived_pck = uart_handler_Process_Received_pck;
    uart_handler->send_response = uart_handler_Send_Response;

    HAL_UARTEx_ReceiveToIdle_IT(uart_handler->huart, rx_buffer, UART_RX_BUFFER_SIZE);
}

void uart_handler_DeInit(void) {
    if (uart_handler) {
        free(uart_handler->cmd_packet);
        free(uart_handler->resp_packet);
        free(uart_handler);
        uart_handler = NULL;
    }
}

void uart_handler_Reset_State(void) {
    if (uart_handler) {
        uart_handler->state = UART_STATE_IDLE;
        memset(uart_handler->cmd_packet, 0, sizeof(UART_CommandPacket_t));
        memset(uart_handler->resp_packet, 0, sizeof(UART_ResponsePacket_t));
    }
}

UART_Handler_t* uart_handler_get(void) {
    return uart_handler;
}
