/*
 * wheel_speed.h
 *
 *  Created on: Apr 17, 2024
 *      Author: leoja
 */

#ifndef INC_WHEEL_SPEED_H_
#define INC_WHEEL_SPEED_H_

#include "stm32f7xx_hal.h"
#include "stdint.h"

//extern volatile uint32_t front_right_wheel_speed;
//extern volatile uint32_t front_left_wheel_speed;

typedef struct
{
	TIM_HandleTypeDef* h_tim;
	uint32_t last_tick;
	uint32_t last_count;
} WheelSpeed_t;

typedef struct
{
	TIM_HandleTypeDef* h_tim;
	uint32_t tim_channel;
	uint32_t buf[3];
	uint32_t hclk;
} WheelSpeedPW_t;

extern volatile uint32_t front_right_wheel_speed;
extern volatile uint32_t front_left_wheel_speed;

void WheelSpeed_Init(WheelSpeed_t* ws, TIM_HandleTypeDef* h_tim);
uint32_t WheelSpeed_GetCPS(WheelSpeed_t* ws);

void WheelSpeedPW_Init(WheelSpeedPW_t* ws, TIM_HandleTypeDef* h_tim, uint32_t tim_channel);
uint32_t WheelSpeedPW_GetCPS(WheelSpeedPW_t* ws);

#endif /* INC_WHEEL_SPEED_H_ */
