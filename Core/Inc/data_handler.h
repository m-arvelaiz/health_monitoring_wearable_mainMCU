/*
 * data_handler.h
 *
 *  Created on: Jun 18, 2025
 *      Author: magaa
 */

#ifndef INC_DATA_HANDLER_H_
#define INC_DATA_HANDLER_H_
#include <stdint.h>
#include "uart_handler.h"


#define DATA_HANDLER_PAYLOAD_LENGHT 10
typedef enum {
    CMD_REQ_HR_SPO2_DATA      = 0x01, // HR & SpO2 + timestamp
    CMD_REQ_TEMP_DATA         = 0x02, // Temperature + timestamp
    CMD_REQ_PRESSURE_DATA     = 0x03, // Pressure + timestamp
    CMD_REQ_ALL_DATA          = 0x04, // HR, SpO2, Temp, Pressure + timestamp
    CMD_REQ_HISTORICAL_DATA   = 0x10, // Request historical data from timestamp
    CMD_SET_UNIX_TIME         = 0x20, // Set device time
    CMD_SET_SENSOR_CONFIG     = 0x30, // Configure sampling or future features
    CMD_START_STREAM          = 0x40, // Begin live sensor stream
    CMD_STOP_STREAM           = 0x41, // Stop live stream
    CMD_UNKNOWN               = 0xFF  // Unknown or unsupported command
} Data_CmdID_t;


typedef struct Data_Handler {
    Data_CmdID_t data_cmd;
    uint8_t* payload;
    uint8_t* payload_out;
    uint8_t lenght;
    uint8_t lenght_out;
} Data_Handler_t;

// Lifecycle
void data_handler_Init(void);
void data_handler_DeInit(void);
Data_Handler_t* data_handler_get(void);

// Dispatcher
void data_handler_dispatcher(UART_CommandPacket_t* cmd);

// Response/Notify functions
void data_handler_notify_hr_spo2(void);
void data_handler_notify_temp(void);
void data_handler_notify_pressure(void);
void data_handler_notify_data_stream(void);
void data_handler_notify_unix_time(void);
void data_handler_notify_set_sensor_config(void);
void data_handler_notify_start_stream(void);
void data_handler_notify_stop_stream(void);

#endif /* INC_DATA_HANDLER_H_ */
