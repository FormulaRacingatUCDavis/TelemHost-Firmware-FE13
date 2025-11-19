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

    switch (event) {

    	case XSENS_EVT_TEMPERATURE: //temperature in celcius
    		if (mtdata->type == XSENS_EVT_TYPE_FLOAT) {
    			//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
    			int16_t temp = (int16_t)(mtdata->data.f4 * 100);

    			uint8_t data[8] = {0};
    			data[0] = HI8(temp);
    			data[1] = LO8(temp);

    			//CAN_Send(&hcan1, 0x100, data, 6);
    			sd_card_write_data(0x100, data);
    		}
    		break;

    	//INCOMPLETE - Time is given in a type_none

    	case XSENS_EVT_UTC_TIME:
        	if (mtdata->type == XSENS_EVT_TYPE_NONE) {
        			//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
        			int16_t ang_x = (int16_t)mtdata->data.f4x3[0];
        			int16_t ang_y = (int16_t)mtdata->data.f4x3[1];
        			int16_t ang_z = (int16_t)mtdata->data.f4x3[2];

        			//CAN_Send(&hcan1, 0x100, data, 6);
        			sd_card_write_data(0x101, data);
        		}
        	break;

    	case XSENS_EVT_PACKET_COUNT: // Number of packets, incremented per MTData2 message
        	if (mtdata->type == XSENS_EVT_TYPE_U16) {
        			//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
        			int16_t pckt_cnt = mtdata->u2;

        			uint8_t data[8] = {0};
        			data[0] = HI8(pckt_cnt);
        			data[1] = LO8(pckt_cnt);

        			//CAN_Send(&hcan1, 0x100, data, 6);
        			sd_card_write_data(0x102, data);
        		}
        	break;

    	case XSENS_EVT_TIME_FINE: // Sample time in # of 10 kHz clock ticks
    		if (mtdata->type == XSENS_EVT_TYPE_U32) {
    			//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
    			int16_t time_fine_high = mtdata->data.u4 >> 16;
    			int16_t time_fine_low = mtdata->data.u4 & 0xFF;

    			uint8_t data[8] = {0};

    			data[0] = HI8(time_fine_high);
    			data[1] = LO8(time_fine_high);
    			data[2] = HI8(time_fine_low);
    			data[3] = LO8(time_fine_low);

    			//CAN_Send(&hcan1, 0x101, data, 6);
    			sd_card_write_data(0x103, data);
    		}
    		break;

    	case XSENS_EVT_TIME_COARSE: // Sample time in # of seconds
    		if (mtdata->type == XSENS_EVT_TYPE_U3s2) {
    			//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
    			int16_t time_coarse_high = mtdata->data.u4 >> 16;
    			int16_t time_corase_low = mtdata->data.u4 & 0xFF;

    			uint8_t data[8] = {0};

    			data[0] = HI8(time_corase_high);
    			data[1] = LO8(time_coarse_high);
    			data[2] = HI8(time_coarse_low);
    			data[3] = LO8(time_coarse_low);

    			//CAN_Send(&hcan1, 0x101, data, 6);
    			sd_card_write_data(0x104, data);
    		}
    		break;

        case XSENS_EVT_QUARTERNION: // Quarternion form of orientation
          if(mtdata->type == XSENS_EVT_TYPE_FLOAT4)
            {
        	  //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
			int16_t x = (int16_t)(mtdata->data.f4x4[0] * 100);
			int16_t y = (int16_t)(mtdata->data.f4x4[1] * 100);
			int16_t z = (int16_t)(mtdata->data.f4x4[2] * 100);
			int16_t w = (int16_t)(mtdata->data.f4x4[3] * 100);

			uint8_t data[8] = {0};

			data[0] = HI8(x);
			data[1] = LO8(x);
			data[2] = HI8(y);
			data[3] = LO8(y);
			data[4] = HI8(z);
			data[5] = LO8(z);
			data[6] = HI8(w);
			data[7] = LO8(w);

			//CAN_Send(&hcan1, 0x100, data, 6);
			sd_card_write_data(0x105, data);
            }
            break;

        case XSENS_EVT_EULER:
          if(mtdata->type == XSENS_EVT_TYPE_FLOAT3)
            {
        	  //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
			int16_t ang_x = (int16_t)(mtdata->data.f4x3[0] * 100);
			int16_t ang_y = (int16_t)(mtdata->data.f4x3[1] * 100);
			int16_t ang_z = (int16_t)(mtdata->data.f4x3[2] * 100);

			uint8_t data[8] = {0};
			data[0] = HI8(ang_x);
			data[1] = LO8(ang_x);
			data[2] = HI8(ang_y);
			data[3] = LO8(ang_y);
			data[4] = HI8(ang_z);
			data[5] = LO8(ang_z);


			//CAN_Send(&hcan1, 0x100, data, 6);
			sd_card_write_data(0x106, data);
            }
            break;

        case XSENS_EVT_ROT_MATRIX: // Rotation matrix for orientation
          if(mtdata->type == XSENS_EVT_TYPE_FLOAT9)
            {
        	  //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
			int16_t a = (int16_t)(mtdata->data.f4x9[0] * 100);
			int16_t b = (int16_t)(mtdata->data.f4x9[1] * 100);
			int16_t c = (int16_t)(mtdata->data.f4x9[2] * 100);
			int16_t d = (int16_t)(mtdata->data.f4x9[3] * 100);
			int16_t e = (int16_t)(mtdata->data.f4x9[4] * 100);
			int16_t f = (int16_t)(mtdata->data.f4x9[5] * 100);
			int16_t g = (int16_t)(mtdata->data.f4x9[6] * 100);
			int16_t h = (int16_t)(mtdata->data.f4x9[7] * 100);
			int16_t i = (int16_t)(mtdata->data.f4x9[8] * 100);

			uint8_t data1[8] = {0};
			uint8_t data2[8] = {0};
			uint8_t data3[8] = {0};

			data1[0] = HI8(a);
			data1[1] = LO8(a);
			data1[2] = HI8(b);
			data1[3] = LO8(b);
			data1[4] = HI8(c);
			data1[5] = LO8(c);
			data1[6] = HI8(d);
			data1[7] = LO8(d);
			data2[0] = HI8(e);
			data2[1] = LO8(e);
			data2[2] = HI8(f);
			data2[3] = LO8(f);
			data2[4] = HI8(g);
			data2[5] = LO8(g);
			data2[6] = HI8(h);
			data2[7] = LO8(h);
			data3[0] = HI8(i);
			data3[1] = LO8(i);


			//CAN_Send(&hcan1, 0x100, data, 6);
			sd_card_write_data(0x107, data1);
			sd_card_write_data(0x108, data2);
			sd_card_write_data(0x109, data3);
            }
            break;

        case XSENS_EVT_PRESSURE: // Pressure measured by internal barometer in Pascals
          if(mtdata->type == XSENS_EVT_TYPE_U32)
            {
        	  //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
  			int16_t pressure_high = mtdata->data.u4 >> 16;
  			int16_t pressure_low = mtdata->data.u4 & 0xFF;

			uint8_t data[8] = {0};

			data[0] = HI8(pressure_high);
			data[1] = LO8(pressure_high);
			data[2] = HI8(pressure_low);
			data[3] = LO8(pressure_low);

			//CAN_Send(&hcan1, 0x100, data, 6);
			sd_card_write_data(0x110, data);
            }
            break;

        case XSENS_EVT_DELTA_V:
          if(mtdata->type == XSENS_EVT_TYPE_FLOAT3)
            {
        	  //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
			int16_t delta_v_x = (int16_t)(mtdata->data.f4x3[0] * 10000);
			int16_t delta_v_y = (int16_t)(mtdata->data.f4x3[1] * 10000);
			int16_t delta_v_z = (int16_t)(mtdata->data.f4x3[2] * 10000);

			uint8_t data[8] = {0};
			data[0] = HI8(delta_v_x);
			data[1] = LO8(delta_v_x);
			data[2] = HI8(delta_v_y);
			data[3] = LO8(delta_v_y);
			data[4] = HI8(delta_v_z);
			data[5] = LO8(delta_v_z);

			//CAN_Send(&hcan1, 0x101, data, 6);
			sd_card_write_data(0x111, data);
            }
            break;

        case XSENS_EVT_LAT_LON:
        	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);

        	int16_t lat = (int16_t)(mtdata->data.f4x3[0] * 100);
        	int16_t lon = (int16_t)(mtdata->data.f4x3[1] * 100);

        	uint8_t data[8] = {0};
        	data[0] = HI8(lat);
        	data[1] = LO8(lat);
        	data[2] = HI8(lat);
        	data[3] = LO8(lat);

        	break;

        default:
        	// IGNORE
        	break;
    }
}
