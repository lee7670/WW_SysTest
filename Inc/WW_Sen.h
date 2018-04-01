/*
 * WW_Sen.h
 *
 *  Created on: Apr 1, 2018
 *      Author: lee7670 and rishabhk7
 */

#ifndef WW_SEN_H_
#define WW_SEN_H_
#include <stdbool.h>
#include "stm32l1xx_hal.h"
#include "stm32l1xx_hal_tim.h"
#include "WW_Defs.h"
struct ultrasonic{
	uint32_t echo_rising_count;
	uint32_t echo_falling_count;
	uint32_t echo_pulse_width_count;
	bool edge_detect;
	GPIO_TypeDef* GPIO_PingBank;
	uint16_t GPIO_PingPin;
	TIM_HandleTypeDef* IC;
};
void Ping_Ultrasonic(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
double Get_Ultrasonic_Reading(struct ultrasonic* ult);
void initUltrasonics(TIM_HandleTypeDef* htim);

#endif /* WW_SEN_H_ */
