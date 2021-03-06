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

#define WINDOWBOTTOMMARGIN 77.0

#define ROTATIONANGLE 20.0

#define WINDOWLENGTH 965.2

void startPP();
void stopPP();
bool isPP_Running();
bool Barrier_Crossed();
void togglePP();
void RunMotionPlanning();
void Linear_Move_Down();
void Rotation_to_Angle();
void Linear_Move_Up_at_Angle();
void Rotation_to_Straight();
void Barrier_Crossing();

#endif /* WW_PP_H_ */
