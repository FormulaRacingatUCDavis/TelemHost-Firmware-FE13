/*
 * xsens.c
 *
 *  Created on: Mar 4, 2024
 *      Author: leoja
 */


#include "serial.h"
#include "xsens/xsens_mti.h"      // Main xsens library
#include "xsens/xsens_utility.h"  // Needed for quaternion conversion function
#include "fatfs.h"
#include "can_manager.h"
#include "sd_card.h"
#include "serial_print.h"

#include <stdbool.h>

extern CAN_HandleTypeDef hcan1;

#define BUFLEN 200

#define HI8(x) ((x>>8)&0xFF)
#define LO8(x) (x&0xFF);

// PRIVATE FUNCTION PROTOTYPES
void imu_callback(XsensEventFlag_t event, XsensEventData_t *mtdata);


// PUBLIC FUNCTION DEFINITIONS

void Xsens_Update(UART_HandleTypeDef* h_uart){
	static bool first_run = true;
	static Serial_t serial;
	static uint8_t rx_buf[BUFLEN];
	static xsens_interface_t imu_interface = XSENS_INTERFACE_RX(&imu_callback);

	if(first_run){
		Serial_Init(&serial, h_uart, rx_buf, BUFLEN);
		Serial_StartListening(&serial);

		// in case it doesn't start reading at the right place (doesn't work)
//		static const uint8_t goto_measurement[] =
//		{ 0xFA, 0xFF, 0x10, 0x00, 0x11 };   // Xbus: Preamble, BusID, MID=0x10, LEN=0, CS
//		Serial_SendBytes(&serial, goto_measurement, sizeof(goto_measurement), 10);

		first_run = false;

		// for some reason it doesn't seem to receive anything until I've sent a byte?
		// kinda sus somebody should look into this one
//		uint8_t bytes[1] = {1};
//		Serial_SendBytes(&serial, bytes, 1, 10);
	}

	uint32_t b = Serial_BytesAvailable(&serial);
	if (b) { // flash faster if reading
		HAL_GPIO_TogglePin(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin);
	}

	for(uint32_t i = 0; i < b; i++){
//		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);

		 xsens_mti_parse(&imu_interface, Serial_GetByte(&serial));
		 print(rx_buf);

	}
}



// PRIVATE FUNCTION DEFINITIONS


// Called when the library decoded an inbound packet
//   - If the packet was an MData2 frame (which contains packed motion data)
//   - the callback is called once for each sub-field
void imu_callback(XsensEventFlag_t event, XsensEventData_t *mtdata)
{
    // The library provides a pointer to the a union containing decoded data
    // Use XsensEventFlag_t to determine what kind of packet arrived,
    // then copy data from the union as needed.

    // union
    // {
    //     uint8_t u1;
    //     uint16_t u2;
    //     uint32_t u4;
    //     float    f4;
    //     float    f4x2[2];
    //     float    f4x3[3];
    //     float    f4x4[4];
    //     float    f4x9[9];
    // } data;

	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);

    switch( event )
    {
        case XSENS_EVT_DELTA_V:
          if( mtdata->type == XSENS_EVT_TYPE_FLOAT3 )
            {

            }
            break;

        case XSENS_EVT_EULER:
          if( mtdata->type == XSENS_EVT_TYPE_FLOAT3 )
            {
        	  //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
			int16_t ang_x = (int16_t)mtdata->data.f4x3[0];
			int16_t ang_y = (int16_t)mtdata->data.f4x3[1];
			int16_t ang_z = (int16_t)mtdata->data.f4x3[2];

			uint8_t data[8] = {0};
			data[0] = HI8(ang_x);
			data[1] = LO8(ang_x);
			data[2] = HI8(ang_y);
			data[3] = LO8(ang_y);
			data[4] = HI8(ang_z);
			data[5] = LO8(ang_z);


			//CAN_Send(&hcan1, 0x100, data, 6);
			sd_card_write_data(0xE100, data);
            }
            break;

        case XSENS_EVT_FREE_ACCELERATION:
          if(mtdata->type == XSENS_EVT_TYPE_FLOAT3)
            {
        	  //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
			int16_t acc_x = (int16_t)(mtdata->data.f4x3[0] * 100);
			int16_t acc_y = (int16_t)(mtdata->data.f4x3[1] * 100);
			int16_t acc_z = (int16_t)(mtdata->data.f4x3[2] * 100);

			uint8_t data[8] = {0};
			data[0] = HI8(acc_x);
			data[1] = LO8(acc_x);
			data[2] = HI8(acc_y);
			data[3] = LO8(acc_y);
			data[4] = HI8(acc_z);
			data[5] = LO8(acc_z);

			//CAN_Send(&hcan1, 0x101, data, 6);
			sd_card_write_data(0xA100, data);
            }
            break;

        case XSENS_EVT_LAT_LON:
        	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
        	break;

        default:
        	// IGNORE
        	break;
    }
}
