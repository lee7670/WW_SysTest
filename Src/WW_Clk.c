/*
 * WW_Clk.c
 *
 *  Created on: Mar 25, 2018
 *      Author: lee7670
 */
#include "WW_Clk.h"
//internal clock counter
uint64_t timeElapsed = 0;
//general timer callback, maybe not a great idea if we need to have other period elapsed callbacks
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance==TIM6) //check if the interrupt comes from TIM3
	{
		timeElapsed++;
	}
}
//returns time in milliseconds since beginning of timer interrupt
uint64_t millis(){
	return timeElapsed;
}
