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
#include <string.h>
#include "WW_Defs.h"
#include "WW_CMD.h"
#include "WW_Sen.h"
#include "tim.h"
#define UltrasonicYPLACEMENT 0.0
#define WINDOWBOTTOMMARGIN 88.9
#define ROTATIONANGLE 43.6
#define WINDOWANGLEDDISTANCE 744.2
#define WINDOWLENGTH 1473.2

void RunMotionPlanning(float End_of_Window_Threshold);

#endif /* WW_PP_H_ */
