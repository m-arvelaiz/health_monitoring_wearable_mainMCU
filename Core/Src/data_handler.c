/*
 * data_handler.c
 *
 *  Created on: Jun 18, 2025
 *      Author: magaa
 */


#include "data_handler.h"
#include "sensors_interface.h"

//TODO: #include "sensor_interface.h"  // hypothetical file to connect to sensors
#include <stdlib.h>
#include <string.h>

static Data_Handler_t* data_handler = NULL;
static uint8_t data_handler_buffer[DATA_HANDLER_PAYLOAD_LENGHT];
static uint8_t data_handler_buffer_out[DATA_HANDLER_PAYLOAD_LENGHT];

// Internal prototypes (req functions)
static void data_handler_req_hr_spo2(void);
static void data_handler_req_temp(uint8_t temp_type);
static void data_handler_req_pressure(void);
static void data_handler_req_all_data(void);
static void data_handler_req_historical_data(uint32_t start_time);
static void data_handler_req_set_unix_time(uint32_t unix_time);
static void data_handler_req_set_sensor_config(uint8_t sensor_type, uint8_t config_val);
static void data_handler_req_start_stream(uint8_t stream_mask);
static void data_handler_req_stop_stream(void);


//// Notify (response) functions
void data_handler_notify_hr_spo2(void) {
//    UART_Handler_t* uart = uart_handler_get();
//    //TODO:uint32_t ts = sensor_get_timestamp();
//    uart->send_cmd(CMD_REQ_HR_SPO2_DATA, data_handler->payload, 4, ts); // optional pre-send if needed
//
}

void data_handler_notify_temp(void) {
//    UART_Handler_t* uart = uart_handler_get();
//    uint32_t ts = sensor_get_timestamp();
//    uart_handler_Send_Response(CMD_REQ_TEMP_DATA, data_handler->payload, 5, ts);
}
//
void data_handler_notify_pressure(void) {
//    UART_Handler_t* uart = uart_handler_get();
//    uint32_t ts = sensor_get_timestamp();
//    uart_handler_Send_Response(CMD_REQ_PRESSURE_DATA, data_handler->payload, 2, ts);
}
//
void data_handler_notify_data_stream(void) {
//    UART_Handler_t* uart = uart_handler_get();
//    uint32_t ts = sensor_get_timestamp();
//    uart_handler_Send_Response(CMD_REQ_ALL_DATA, data_handler->payload, 8, ts);
}
//
void data_handler_notify_unix_time(void) {
//    UART_Handler_t* uart = uart_handler_get();
//    uart_handler_Send_Response(CMD_SET_UNIX_TIME, data_handler->payload, 4, 0);
}
//
void data_handler_notify_set_sensor_config(void) {
//    UART_Handler_t* uart = uart_handler_get();
//    uart_handler_Send_Response(CMD_SET_SENSOR_CONFIG, data_handler->payload, 2, 0);
}
//
void data_handler_notify_start_stream(void) {
//    UART_Handler_t* uart = uart_handler_get();
//    uart_handler_Send_Response(CMD_START_STREAM, data_handler->payload, 1, 0);
}
//
void data_handler_notify_stop_stream(void) {
//    UART_Handler_t* uart = uart_handler_get();
//    uart_handler_Send_Response(CMD_STOP_STREAM, data_handler->payload, 1, 0);
}

void data_handler_dispatcher(UART_CommandPacket_t* cmd) {
    data_handler->data_cmd = (Data_CmdID_t)cmd->cmd_type;
    memcpy(data_handler->payload, cmd->payload, 5);
    data_handler->lenght=cmd->payload_len;

    //TODO: filter if the message is to configure something or to request a cmd

    sensor_interface_handle_cmd(data_handler->data_cmd, data_handler->payload, data_handler->lenght, data_handler->payload_out, &(data_handler->lenght_out));
    UART_Handler_t* uart = uart_handler_get();
    uart->send_response(data_handler->data_cmd, data_handler->payload_out, data_handler->lenght_out );


//    switch (data_handler->data_cmd) {
//        case CMD_REQ_HR_SPO2_DATA:
//            data_handler_req_hr_spo2();
//            break;
//        case CMD_REQ_TEMP_DATA:
//            data_handler_req_temp(cmd->payload[0]);
//            break;
//        case CMD_REQ_PRESSURE_DATA:
//            data_handler_req_pressure();
//            break;
//        case CMD_REQ_ALL_DATA:
//            data_handler_req_all_data();
//            break;
//        case CMD_REQ_HISTORICAL_DATA: {
//            uint32_t ts = (cmd->payload[0] << 24) | (cmd->payload[1] << 16) |
//                          (cmd->payload[2] << 8) | cmd->payload[3];
//            data_handler_req_historical_data(ts);
//            break;
//        }
//        case CMD_SET_UNIX_TIME: {
//            uint32_t unix_time = (cmd->payload[0] << 24) | (cmd->payload[1] << 16) |
//                                 (cmd->payload[2] << 8) | cmd->payload[3];
//            data_handler_req_set_unix_time(unix_time);
//            break;
//        }
//        case CMD_SET_SENSOR_CONFIG:
//            data_handler_req_set_sensor_config(cmd->payload[0], cmd->payload[1]);
//            break;
//        case CMD_START_STREAM:
//            data_handler_req_start_stream(cmd->payload[0]);
//            break;
//        case CMD_STOP_STREAM:
//            data_handler_req_stop_stream();
//            break;
//        default:
//            // Unknown command handling
//            break;
//    }
}

//static void data_handler_req_hr_spo2(void) {
//    // Collect HR + SpO2 from sensor, fill payload and call notify
//	//TODO: sensor_collect_hr_spo2(data_handler->payload);  // Expected to fill 4 bytes
//    data_handler_notify_hr_spo2();
//}
//
//static void data_handler_req_temp(uint8_t temp_type) {
//	//TODO: sensor_collect_temp(temp_type, data_handler->payload);
//    data_handler_notify_temp();
//}
//
//static void data_handler_req_pressure(void) {
//	//TODO: sensor_collect_pressure(data_handler->payload);
//    data_handler_notify_pressure();
//}
//
//static void data_handler_req_all_data(void) {
//    //TODO: sensor_collect_all(data_handler->payload);
//    data_handler_notify_data_stream();
//}
//
//static void data_handler_req_historical_data(uint32_t start_time) {
//    // TODO: implement logic or buffering to retrieve historical logs
//}
//
//static void data_handler_req_set_unix_time(uint32_t unix_time) {
//	//TODO:sensor_set_unix_time(unix_time);
//    data_handler_notify_unix_time();
//}
//
//static void data_handler_req_set_sensor_config(uint8_t sensor_type, uint8_t config_val) {
//	//TODO:sensor_configure(sensor_type, config_val);
//    data_handler_notify_set_sensor_config();
//}
//
//static void data_handler_req_start_stream(uint8_t stream_mask) {
//	//TODO:sensor_stream_start(stream_mask);
//    data_handler_notify_start_stream();
//}
//
//static void data_handler_req_stop_stream(void) {
//	//TODO:sensor_stream_stop();
//    data_handler_notify_stop_stream();
//}

void data_handler_Init(void) {
    data_handler = (Data_Handler_t *)malloc(sizeof(Data_Handler_t));
    data_handler->payload = data_handler_buffer;
    data_handler->payload_out = data_handler_buffer_out;
    memset(data_handler->payload, 0, DATA_HANDLER_PAYLOAD_LENGHT);
    memset(data_handler->payload_out, 0, DATA_HANDLER_PAYLOAD_LENGHT);
}

void data_handler_DeInit(void) {
    if (data_handler != NULL) {
        free(data_handler);
        data_handler = NULL;
    }
}

Data_Handler_t* data_handler_get(void) {
    return data_handler;
}
