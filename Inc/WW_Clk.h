/*
 * WW_Clk.h
 *
 *  Created on: Mar 25, 2018
 *      Author: lee7670
 */

#ifndef WW_CLK_H_
#define WW_CLK_H_
#include "stm32l1xx_hal.h"
#include "stm32l1xx_hal_tim.h"
uint64_t timeElapsed;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
uint64_t Get_Time();


#endif /* WW_CLK_H_ */
