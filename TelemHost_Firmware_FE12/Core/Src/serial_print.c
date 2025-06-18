#include "serial_print.h"
#include <string.h>
#include "stm32f7xx_hal.h"
#include "serial.h"

extern UART_HandleTypeDef huart3;

void print(const char *str) {
//	static uint8_t first_run = 1;
//	static Serial_t serial;
//
//	if (first_run) {
//		Serial_Init(&serial, &huart3, 0, 0);
//		first_run = 0;
//	}
//
//	Serial_SendBytes(&serial, str, strlen(str), 20);

	HAL_UART_Transmit(&huart3, str, strlen(str), 10);
}

void dump_can_data() {
	print("insert CAN data here\n");
}
