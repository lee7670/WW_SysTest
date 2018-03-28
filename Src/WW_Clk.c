/*
 * WW_Clk.c
 *
 *  Created on: Mar 25, 2018
 *      Author: lee7670
 */
#include "WW_Clk.h"

uint64_t timeElapsed = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance==TIM6) //check if the interrupt comes from TIM3
	{
		timeElapsed++;
	}
}
uint64_t Get_Time(){
	return timeElapsed;
}
