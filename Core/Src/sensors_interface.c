/*
 * sensors_interface.c
 *
 *  Created on: Jun 21, 2025
 *      Author: magaa
 */


#include "sensors_interface.h"
#include "sensors_address.h"
#include "temp_sensor.h"
#include "ppg_sensor.h"
#include "pressure_sensor.h"
#include "i2c_handler.h"
#include "data_handler.h"
#include <string.h>


//sensor_interface_trigger_schedule_t sensor_interface_trigger_schedule_state;
sensor_interface_trigger_schedule_t sensor_interface_schedule;

// Internal buffer for I2C tx/rx

void sensor_interface_init(void) {
	sensor_interface_schedule=TRIGGER_ENV_TEMP;
//    temp_sensor_init(SENSOR_ADDRESS_TEMP_HUM_HTS221);
    pressure_sensor_init(SENSOR_ADDRESS_PRESSURE_LPS22HH);
    ppg_sensor_init(SENSOR_ADDRESS_PPG_SEN0344);
    // TODO: Add other sensors
}

void sensor_interface_deinit(void) {
//    temp_sensor_deinit();
    // TODO: Add other sensor deinit
}

void sensor_interface_schedule_readings(void) {

	//STEP1:
	//Check if a data processing is queue
	I2C_Handler_t* i2c_handle=i2c_handler_get();
	Temp_Sensor_t* temp = temp_sensor_get();
	Pressure_Sensor_t* pres=pressure_sensor_get();
	PPG_Sensor_t* ppg=ppg_sensor_get();
//	//Version1
//	if(i2c_handle->state==I2C_STATE_MSG_WAITING_FOR_PROCESSING){
//		//current = (current + 1) % TRIGGER_COUNT;
//		i2c_handle->state=I2C_STATE_IDLE;
//		//ENV_TEMP
//		if(sensor_interface_schedule==TRIGGER_ENV_TEMP){
//			temp->decode_i2c_response(i2c_handle->Response_buffer, i2c_handle->response_buffer_lenght);
//		}
//
//
//		sensor_interface_schedule=(sensor_interface_schedule+1)%TRIGGER_COUNT;// schedule next reading
//	}


	//STEP2:
	//Schedule new reading
    if(i2c_handle->state==I2C_STATE_IDLE){

    	sensor_interface_schedule=(sensor_interface_schedule+1)%TRIGGER_COUNT;// schedule next reading

//    	temp->trigger_data_collection();
    	pres->trigger_data_collection();
    	ppg->trigger_data_collection();




    }



}

void sensor_interface_handle_cmd(uint8_t cmd_id, uint8_t* payload, uint8_t len, uint8_t* payload_out, uint8_t* len_out) {

	switch (cmd_id) {
	case CMD_REQ_TEMP_DATA:
		if (payload[0] == 0x02) {
			//TODO: UNcomment this when temp HT221 sensor is fixed for now using temperature from pressure sensor
//			Temp_Sensor_t* env_temp = temp_sensor_get();
//			if (env_temp && env_temp->get_last_n_data_serial_format) {
//				payload_out[0]=0x02;
//				(*len_out)=env_temp->get_last_n_data_serial_format(1,payload_out+1);
//				(*len_out)= (*len_out)+1;
//			}
			payload_out[0] = 0x02;

			(*len_out) = pressure_get_temp_last_n_serial_format(1,
					payload_out + 1);
			(*len_out) = (*len_out) + 1;

		}

		break;

	case CMD_REQ_PRESSURE_DATA:
		Pressure_Sensor_t *press = pressure_sensor_get();
		if (press && press->get_last_n_data_serial_format) {
			payload_out[0] = 0x00; //Reserved bit in this case not used
			(*len_out) = press->get_last_n_data_serial_format(1, payload_out + 1);
			(*len_out) = (*len_out) + 1;
		}
		break;

	case CMD_REQ_HR_SPO2_DATA:
//		Pressure_Sensor_t *press = pressure_sensor_get();
//		if (env_temp && env_temp->get_last_n_data_serial_format) {
//			payload_out[0] = 0x00; //Reserved bit in this case not used
//			(*len_out) = press->get_last_n_data_serial_format(1,
//					payload_out + 1);
//			(*len_out) = (*len_out) + 1;
//		}
		//TODO: After finishig sensor declaration
		break;

	case CMD_REQ_ALL_DATA:
		// TODO: Add support for composing full sensor packet with all sensor readings
		break;

        default:
            // Unknown command
            break;
    }
}
