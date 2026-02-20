/*
 * xsens.c
 *
 *  Created on: Mar 4, 2024
 *      Author: leoja
 */

#include <string.h>
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
	static uint8_t rx_buf[BUFLEN] __attribute__((section(".XsensRxBufSection")));
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
		case XSENS_EVT_UTC_TIME:
			if ( mtdata->type == XSENS_EVT_TYPE_NONE )
			{
				uint32_t nanoseconds = mtdata->data.utc_time.nanoseconds;
				uint16_t year = mtdata->data.utc_time.year;
				uint8_t month = mtdata->data.utc_time.month;
				uint8_t day = mtdata->data.utc_time.day;
				uint8_t hour = mtdata->data.utc_time.hour;
				uint8_t minute = mtdata->data.utc_time.minute;
				uint8_t second = mtdata->data.utc_time.second;
				uint8_t flags = mtdata->data.utc_time.flags;
			}
			break;

		case XSENS_EVT_PACKET_COUNT:
			if ( mtdata->type == XSENS_EVT_TYPE_U16 )
			{
				uint16_t packet_count = mtdata->data.u2;
			}
			break;

		case XSENS_EVT_EULER:
			if ( mtdata->type == XSENS_EVT_TYPE_FLOAT3 )
			{
				int16_t ang_x = (int16_t)(mtdata->data.f4x3[0] * 100);
				int16_t ang_y = (int16_t)(mtdata->data.f4x3[1] * 100);
				int16_t ang_z = (int16_t)(mtdata->data.f4x3[2] * 100);

				uint8_t data[8];
				data[0] = HI8(ang_x);
				data[1] = LO8(ang_x);
				data[2] = HI8(ang_y);
				data[3] = LO8(ang_y);
				data[4] = HI8(ang_z);
				data[5] = LO8(ang_z);
			}
			break;

		case XSENS_EVT_PRESSURE:
			if (mtdata->type == XSENS_EVT_TYPE_U32 )
			{
				uint32_t pressure = mtdata->data.u4;
			}
			break;

    	case XSENS_EVT_DELTA_Q:
			if( mtdata->type == XSENS_EVT_TYPE_FLOAT4 )
			{
			  int16_t delta1 = (int16_t)(mtdata->data.f4x4[0] * 100);
			  int16_t delta2 = (int16_t)(mtdata->data.f4x4[1] * 100);
			  int16_t delta3 = (int16_t)(mtdata->data.f4x4[2] * 100);
			  int16_t delta4 = (int16_t)(mtdata->data.f4x4[3] * 100);

			  uint8_t data[8] = {0};
			  data[0] = HI8(delta1);
			  data[1] = LO8(delta1);
			  data[2] = HI8(delta2);
			  data[3] = LO8(delta2);
			  data[4] = HI8(delta3);
			  data[5] = LO8(delta3);
			  data[6] = HI8(delta4);
			  data[7] = LO8(delta4);

			  //CAN_Send(&hcan1, 0x100, data, 6);
			  sd_card_write_data(0x112, data);
			}
			break;

        case XSENS_EVT_LAT_LON:
            if ( mtdata->type == XSENS_EVT_TYPE_FLOAT2 )
            {
                int32_t lat = (int32_t)(mtdata->data.f4x2[0] * 10000);
                int32_t lon = (int32_t)(mtdata->data.f4x2[1] * 10000);
                uint8_t data[8] = {0};

                data[0] = (lat >> 24) & 0xFF;
                data[1] = (lat >> 16) & 0xFF;
                data[2] = (lat >> 8)  & 0xFF;
                data[3] = (lat >> 0)  & 0xFF;

                data[4] = (lon >> 24) & 0xFF;
                data[5] = (lon >> 16) & 0xFF;
                data[6] = (lon >> 8)  & 0xFF;
                data[7] = (lon >> 0)  & 0xFF;
                sd_card_write_data(0x125, data);
            }
            break;

        case XSENS_EVT_ACCELERATION:
			if( mtdata->type == XSENS_EVT_TYPE_FLOAT3 )
			{
				int16_t accX = (int16_t)(mtdata->data.f4x3[0] * 100);
				int16_t accY = (int16_t)(mtdata->data.f4x3[1] * 100);
				int16_t accZ = (int16_t)(mtdata->data.f4x3[2] * 100);

				uint8_t data[6] = {0};
				data[0] = HI8(accX);
				data[1] = LO8(accX);
				data[2] = HI8(accY);
				data[3] = LO8(accY);
				data[4] = HI8(accZ);
				data[5] = LO8(accZ);

				//CAN_Send(&hcan1, 0x100, data, 6);
				sd_card_write_data(0x113, data);
			}
			break;

        case XSENS_EVT_FREE_ACCELERATION:
			if( mtdata->type == XSENS_EVT_TYPE_FLOAT3 )
			{
				int16_t acc_x = (int16_t)(mtdata->data.f4x3[0] * 100);
				int16_t acc_y = (int16_t)(mtdata->data.f4x3[1] * 100);
				int16_t acc_z = (int16_t)(mtdata->data.f4x3[2] * 100);

				uint8_t data[8];
				data[0] = HI8(acc_x);
				data[1] = LO8(acc_x);
				data[2] = HI8(acc_y);
				data[3] = LO8(acc_y);
				data[4] = HI8(acc_z);
				data[5] = LO8(acc_z);
			}
			break;

        case XSENS_EVT_DELTA_V:
			if( mtdata->type == XSENS_EVT_TYPE_FLOAT3 )
			{
				//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
				int16_t freeAccX = (int16_t)(mtdata->data.f4x3[0] * 100);
				int16_t freeAccY = (int16_t)(mtdata->data.f4x3[1] * 100);
				int16_t freeAccZ = (int16_t)(mtdata->data.f4x3[2] * 100);

				uint8_t data[8] = {0};
				data[0] = HI8(freeAccX);
				data[1] = LO8(freeAccX);
				data[2] = HI8(freeAccY);
				data[3] = LO8(freeAccY);
				data[4] = HI8(freeAccZ);
				data[5] = LO8(freeAccZ);

				//CAN_Send(&hcan1, 0x101, data, 6);
				sd_card_write_data(0x114, data);
			}
			break;

        case XSENS_EVT_STATUS_BYTE:
			if ( mtdata->type == XSENS_EVT_TYPE_U8 )
			{
				uint8_t status_byte = mtdata->data.u1;
				uint8_t data[8] = {0};

				data[0] = status_byte;

				sd_card_write_data(0x120, data);
			}
			break;

		case XSENS_EVT_STATUS_WORD:
			if ( mtdata->type == XSENS_EVT_TYPE_U32 )
			{
				uint32_t status_word = mtdata->data.u4;
				uint8_t data[8] = {0};

				data[0] = (status_word >> 24) & 0xFF;
				data[1] = (status_word>> 16) & 0xFF;
				data[2] = (status_word >> 8)  & 0xFF;
				data[3] = (status_word >> 0)  & 0xFF;

				sd_card_write_data(0x121, data);
			}
			break;

		case XSENS_EVT_DEVICE_ID:
			if ( mtdata->type == XSENS_EVT_TYPE_U32 )
			{
				uint32_t dev_id = mtdata->data.u4;
				uint8_t data[8] = {0};

				data[0] = (dev_id >> 24) & 0xFF;
				data[1] = (dev_id >> 16) & 0xFF;
				data[2] = (dev_id >> 8)  & 0xFF;
				data[3] = (dev_id >> 0)  & 0xFF;

				sd_card_write_data(0x122, data);
			}
			break;

		case XSENS_EVT_LOCATION_ID:
			if ( mtdata->type == XSENS_EVT_TYPE_U16 )
			{
				uint16_t location = mtdata->data.u2;
				uint8_t data[8] = {0};

				data[0] = HI8(location);
				data[1] = LO8(location);

				sd_card_write_data(0x123, data);
			}
			break;

		case XSENS_EVT_POSITION_ECEF:
			if( mtdata->type == XSENS_EVT_TYPE_FLOAT3 )
			{
				uint8_t data[12] = {0};
				memcpy(data, mtdata->data.f4x3, 12);

				sd_card_write_data(0x124, data);
			}
			int16_t ecef_x = (int16_t)(mtdata->data.f4x3[0] * 100);
			int16_t ecef_y = (int16_t)(mtdata->data.f4x3[1] * 100);
			int16_t ecef_z = (int16_t)(mtdata->data.f4x3[2] * 100);

			uint8_t data[8] = {0};
			data[0] = HI8(ecef_x);
			data[1] = LO8(ecef_x);
			data[2] = HI8(ecef_y);
			data[3] = LO8(ecef_y);
			data[4] = HI8(ecef_z);
			data[5] = LO8(ecef_z);


			sd_card_write_data(0xA100, data);
			break;

		case XSENS_EVT_ACCELERATION_HR:
			if( mtdata->type == XSENS_EVT_TYPE_FLOAT3 )
			{

				int16_t accX = (int16_t)(mtdata->data.f4x3[0] * 100);
				int16_t accY = (int16_t)(mtdata->data.f4x3[1] * 100);
				int16_t accZ = (int16_t)(mtdata->data.f4x3[2] * 100);

				uint8_t data[8] = {0};
				data[0] = HI8(accX);
				data[1] = LO8(accX);
				data[2] = HI8(accY);
				data[3] = LO8(accY);
				data[4] = HI8(accZ);
				data[5] = LO8(accZ);

				//CAN_Send(&hcan1, 0x101, data, 6);
				sd_card_write_data(0x115, data);
			}
			break;

		case XSENS_EVT_RATE_OF_TURN:
			if( mtdata->type == XSENS_EVT_TYPE_FLOAT3 )
			{
				//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
				int16_t gyrX = (int16_t)(mtdata->data.f4x3[0] * 100);
				int16_t gyrY = (int16_t)(mtdata->data.f4x3[1] * 100);
				int16_t gyrZ = (int16_t)(mtdata->data.f4x3[2] * 100);

				uint8_t data[8] = {0};
				data[0] = HI8(gyrX);
				data[1] = LO8(gyrX);
				data[2] = HI8(gyrY);
				data[3] = LO8(gyrY);
				data[4] = HI8(gyrZ);
				data[5] = LO8(gyrZ);

				//CAN_Send(&hcan1, 0x101, data, 6);
				sd_card_write_data(0x116, data);
			}
			break;

		case XSENS_EVT_RATE_OF_TURN_HR:
			if( mtdata->type == XSENS_EVT_TYPE_FLOAT3 )
			{
				//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
				int16_t gyrX = (int16_t)(mtdata->data.f4x3[0] * 100);
				int16_t gyrY = (int16_t)(mtdata->data.f4x3[1] * 100);
				int16_t gyrZ = (int16_t)(mtdata->data.f4x3[2] * 100);

				uint8_t data[8] = {0};
				data[0] = HI8(gyrX);
				data[1] = LO8(gyrX);
				data[2] = HI8(gyrY);
				data[3] = LO8(gyrY);
				data[4] = HI8(gyrZ);
				data[5] = LO8(gyrZ);

				//CAN_Send(&hcan1, 0x101, data, 6);
				sd_card_write_data(0x117, data);
			}
			break;

		case XSENS_EVT_GNSS_PVT_PULSE:
			if( mtdata->type == XSENS_EVT_TYPE_U32 )
			{
				//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
				int32_t pulse = mtdata->data.u4;

				uint8_t data[4] = {0};
				data[0] = pulse & 0xFF;
				data[1] = (pulse >> 8) & 0xFF;
				data[2] = (pulse >> 16) & 0xFF;
				data[3] = (pulse >> 24) & 0xFF;

				//CAN_Send(&hcan1, 0x101, data, 6);
				sd_card_write_data(0x118, data);
			}
			break;

		case XSENS_EVT_MAGNETIC:
			if( mtdata->type == XSENS_EVT_TYPE_FLOAT3 )
			{
				//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
				int16_t magX = (int16_t)(mtdata->data.f4x3[0] * 100);
				int16_t magY = (int16_t)(mtdata->data.f4x3[1] * 100);
				int16_t magZ = (int16_t)(mtdata->data.f4x3[2] * 100);

				uint8_t data[8] = {0};
				data[0] = HI8(magX);
				data[1] = LO8(magX);
				data[2] = HI8(magY);
				data[3] = LO8(magY);
				data[4] = HI8(magZ);
				data[5] = LO8(magZ);

				//CAN_Send(&hcan1, 0x101, data, 6);
				sd_card_write_data(0x119, data);
			}
			break;

		case XSENS_EVT_ALTITUDE_ELLIPSOID:
			if ( mtdata->type == XSENS_EVT_TYPE_FLOAT )
			{
				int16_t alt = (int16_t)(mtdata->data.f4 * 100);
				uint8_t data[8] = {0};

				data[0] = HI8(alt);
				data[1] = LO8(alt);

				sd_card_write_data(0x126, data);
			}
			break;

		case XSENS_EVT_VELOCITY_XYZ:
			if ( mtdata->type == XSENS_EVT_TYPE_FLOAT3 )
			{
				int16_t vel_x = (int16_t)(mtdata->data.f4x3[0] * 100);
				int16_t vel_y = (int16_t)(mtdata->data.f4x3[1] * 100);
				int16_t vel_z = (int16_t)(mtdata->data.f4x3[2] * 100);
				uint8_t data[8] = {0};

				data[0] = HI8(vel_x);
				data[1] = LO8(vel_x);
				data[2] = HI8(vel_y);
				data[3] = LO8(vel_y);
				data[4] = HI8(vel_z);
				data[5] = LO8(vel_z);

				sd_card_write_data(0x127, data);
			}
			break;

		default:
			// IGNORE
			break;
    }
}



