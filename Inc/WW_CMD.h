/*
 * WW_CMD.h
 *
 *  Created on: Mar 25, 2018
 *      Author: lee7670
 */

#ifndef WW_CMD_H_
#define WW_CMD_H_

#include <string.h>
#include <stdlib.h>
#include "stm32l1xx_hal.h"
#include "stm32l1xx_hal_tim.h"
#include "stm32l1xx_hal_uart.h"
#include "WW_Loc.h"
#include "WW_Defs.h"
void Parse_CMD(TIM_HandleTypeDef* Fan_TIM,UART_HandleTypeDef* huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void UART_ReadStart(UART_HandleTypeDef *huart);
void EXE_CMD(char*command, TIM_HandleTypeDef* Fan_TIM, UART_HandleTypeDef* huart);

#endif /* WW_CMD_H_ */
