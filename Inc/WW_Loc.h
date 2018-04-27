/*
 * WW_Loc.h
 *
 *  Created on: Mar 25, 2018
 *      Author: lee7670
 */

#ifndef WW_LOC_H_
#define WW_LOC_H_
#include "stm32l1xx_hal.h"
#include "stm32l1xx_hal_tim.h"
#include "stm32l1xx_hal_uart.h"
#include <stdbool.h>
#include <stdlib.h>
#include "pid_controller.h"
#include "WW_Defs.h"
#include "WW_CMD.h"
#include "tim.h"
#include "usart.h"
#define WHEELRAD 34.925
#define CENTERDIS 222.25
#define KP 0.9
#define KI 3.3
#define KD 0.07
#define KP_Pos 20.0
#define KI_Pos 0.0
#define KD_Pos 3.0
#define PID_PERIOD 25
#define TOLERANCE 10
#define ENCODERCOUNTSPERREV 893.76f
struct motor{
	float setRPM;
	bool dir;
	float distance_traveled;
	float setDis;
	uint16_t prevcount;
	TIM_HandleTypeDef* encoder;
	TIM_HandleTypeDef* pwm;
	PIDControl PID;
	PIDControl PosPID;
};

void togglePosPID();
void startPosPID();
void stopPosPID();
void setLin(float dis, float spd);
void setArc(float r, float w, float phi);
void initMot(TIM_HandleTypeDef* TIM_RightEnc, TIM_HandleTypeDef* TIM_LeftEnc,
		TIM_HandleTypeDef* TIM_RightMot, TIM_HandleTypeDef* TIM_LeftMot);
void Run_PID(UART_HandleTypeDef* huart);
uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);
void Set_MotorDir(void);
uint16_t Get_EncoderPos(struct motor* Mot);
void Set_PIDOut(float rpm1, float rpm2, float distance_traveled1, float distance_traveled2, UART_HandleTypeDef* huart);
void Run_MotorPWM(int16_t pwm);
uint16_t Get_RightEncoderPos();
uint16_t Get_LeftEncoderPos();
void Stop_Motors();
#endif /* WW_LOC_H_ */
