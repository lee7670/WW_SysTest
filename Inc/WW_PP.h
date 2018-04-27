/*
 * WW_PP.h
 *
 *  Created on: Apr 11, 2018
 *      Author: Rishabh K.
 */

#ifndef WW_PP_H_
#define WW_PP_H_

#include "stm32l1xx_hal.h"
#include "stm32l1xx_hal_tim.h"
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "WW_Defs.h"
#include "WW_CMD.h"
#include "WW_Sen.h"
#include "main.h"
#include "tim.h"
#include "usart.h"

#define UltrasonicYPLACEMENT 12.7

#define WINDOWBOTTOMMARGIN 96.2

#define ROTATIONANGLE 43.6

#define WINDOWANGLEDDISTANCE (744.2-230)

#define WINDOWLENGTH 1473.2

#define BARRIERTHRESHOLD 150
void startPP();
void stopPP();
bool isPP_Running();
void togglePP();
void RunMotionPlanning(float End_of_Window_Threshold, UART_HandleTypeDef* huart);

#endif /* WW_PP_H_ */
