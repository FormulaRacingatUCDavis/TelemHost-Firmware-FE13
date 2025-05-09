#include <stdint.h>
#include <stdio.h>
#include "telem.h"
#include "can_manager.h"
#include "wheel_speed.h"

const uint8_t packet_validation[2] = {0x00, 0xff};
uint32_t send_time;
uint32_t prev_time = 0;

extern UART_HandleTypeDef huart7;

extern uint16_t sg_adc;

void telem_send(void) {
	send_time = HAL_GetTick();
	if (send_time - prev_time > TELEM_DELAY) {
		Packet p = {{packet_validation[0], packet_validation[1]},
					telem_id, {0, 0, 0, 0}, send_time};
		switch(telem_id) {
			case 0:
				p.data[0] = front_right_wheel_speed;
				p.data[1] = front_left_wheel_speed;
				p.data[2] = rear_right_wheel_speed;
				p.data[3] = rear_left_wheel_speed;
				break;
			case 1:
				p.data[1] = outlet_temp;
				p.data[2] = inlet_pres;
				p.data[3] = outlet_pres;
				break;
//			case 2:
//				p.data[0] = sg_adc >> 8;
//				p.data[1] = sg_adc & 0xff;
//				p.data[2] = 0;
//				p.data[3] = 0;
			default:
				return;
		}
		// TODO remove, temp testing
		p.data[0]= 99;

		HAL_StatusTypeDef v = HAL_UART_Transmit(&huart7, (uint8_t*)&p, PACKET_LENGTH, 1000);
		telem_id = !telem_id;
		prev_time = send_time;
	}
}

