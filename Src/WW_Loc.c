/*
 * WW_Loc.c
 *
 *  Created on: Mar 25, 2018
 *      Author: lee7670
 */

#include "WW_Loc.h"
struct motor left;
struct motor right;
uint64_t prevTime = 0;
void initMot(TIM_HandleTypeDef* TIM_RightEnc, TIM_HandleTypeDef* TIM_LeftEnc,
		TIM_HandleTypeDef* TIM_RightMot, TIM_HandleTypeDef* TIM_LeftMot){
	right.prevcount = __HAL_TIM_GET_COUNTER(TIM_RightEnc);
	left.prevcount = __HAL_TIM_GET_COUNTER(TIM_LeftEnc);
	right.encoder = TIM_RightEnc;
	left.encoder = TIM_LeftEnc;
	right.pwm = TIM_RightMot;
	left.pwm = TIM_LeftMot;
	right.dir = false;
	left.dir = false;
	right.setRPM = 0.0;
	right.setDis = 0.0;
	right.distance_traveled = 0.0f;
	right.prevpos = 0.0;
	left.setRPM = 0.0;
	left.setDis = 0.0;
	left.distance_traveled = 0.0f;
	left.prevpos = 0.0;
	PIDInit(&right.PID, KP, KI, KD, .1, ((float)PID_PERIOD)/1000.0, 255, AUTOMATIC, DIRECT);
	PIDInit(&left.PID, KP, KI, KD, .1, ((float)PID_PERIOD)/1000.0, 255, AUTOMATIC, DIRECT);

}
void setArc(float R/*mm*/, float w/*degrees/s*/, float phi/*degrees*/){
  float scalingfactor = 60.0/360.0;
  right.setRPM = (w*(R+CENTERDIS))*scalingfactor*(1/WHEELRAD);
  left.setRPM = (w*(R-CENTERDIS))*scalingfactor*(1/WHEELRAD);

  if (right.setRPM < 0){
    right.setRPM = -1*right.setRPM;
    right.dir = true;
  }
  else
  {
    right.dir = false;
  }

  if (left.setRPM < 0){
    left.setRPM = -1*left.setRPM;
    left.dir = true;
  }
  else
  {
    left.dir = false;
  }
  right.setDis = abs((R+CENTERDIS)*phi*M_PI/180);
  left.setDis = abs((R-CENTERDIS)*phi*M_PI/180);
  if (phi < 0){
	  left.setRPM = -1*left.setRPM;
	  left.dir = !left.dir;
	  right.setRPM = -1*right.setRPM;
	  right.dir = !right.dir;
  }
  //Re-initialize targeting to a rotation/distance
  right.distance_traveled = 0.0;
  left.distance_traveled = 0.0;
}
void setLin(float dis, float spd){
  right.setRPM = (spd/(2.0*M_PI*WHEELRAD))*60.0;
  left.setRPM = right.setRPM;

  if (right.setRPM < 0){
    right.setRPM = -1.0*right.setRPM;
    left.setRPM = -1.0*left.setRPM;
    right.dir = true;
    left.dir = true;
  }
  else
  {
    right.dir = false;
    left.dir = false;
  }

  right.setDis = dis;
  left.setDis = dis;
  //Re-initialize targeting to a rotation/distance
  right.distance_traveled = 0.0;
  left.distance_traveled = 0.0;

  return;
}
void Run_PID(UART_HandleTypeDef* huart){
	static uint64_t prevtim = 0;

	volatile uint64_t tim = Get_Time();
	volatile uint32_t deltat = (uint32_t)(tim - prevtim);
	if(deltat < PID_PERIOD){
		return;
	}
	Set_MotorDir();
	uint16_t newposition1 = Get_EncoderPos(&right);
	uint16_t newposition2 = Get_EncoderPos(&left);
	char buffer[25];
	uint8_t len = sprintf(buffer, "newposition:%i\r\n", newposition1);
	HAL_UART_Transmit(huart, buffer, len, 1000);
	prevtim = tim;
	float vel1 = ((float)newposition1) * 10; //encoder pulses per second
	float rpm1 = ((vel1 * 60)/3000); //Measured motor RPM
	float vel2 = ((float)newposition2) * 10; //encoder pulses per second
	float rpm2 = ((vel2 * 60)/3000);
	float realspeed1 = (rpm1 * 2.0 * M_PI * WHEELRAD)/60; //Linear speed in mm/s
	float realspeed2 = (rpm2 * 2.0 * M_PI * WHEELRAD)/60; //Linear speed in mm/s
	if (right.dir)
	{
		rpm1 = -1*rpm1;
	}
	if (left.dir)
	{
		rpm2 = -1*rpm2;
	}
	Set_PIDOut(rpm1, rpm2, huart);
	right.distance_traveled = right.distance_traveled + (realspeed1*deltat*1e-3); //integrate linear velocity to obtain distance
	left.distance_traveled = left.distance_traveled + (realspeed2*deltat*1e-3); //integrate linear velocity to obtain distance
	if ((abs(right.distance_traveled-right.setDis) <= 5)||(abs(left.distance_traveled-left.setDis) <= 5) ){
		__HAL_TIM_SetCompare(right.pwm, TIM_CHANNEL_1, 0);
		__HAL_TIM_SetCompare(left.pwm, TIM_CHANNEL_1, 0);
		right.setRPM = 0.0; //Brake
		right.dir = false;
		left.setRPM = 0.0; //Brake
		left.dir = false;
		right.setDis = 0.0;
		left.setDis = 0.0;
		Set_MotorDir();
	}
	right.prevpos = newposition1;
	left.prevpos = newposition2;
	return;
}
uint16_t Get_EncoderPos(struct motor* Mot){
	uint16_t newposition;
	if(__HAL_TIM_IS_TIM_COUNTING_DOWN(Mot->encoder)){
		if(Mot->prevcount < __HAL_TIM_GET_COUNTER(Mot->encoder)){
			newposition = 65535 - __HAL_TIM_GET_COUNTER(Mot->encoder) + Mot->prevcount;
		}else{
			newposition = abs(__HAL_TIM_GET_COUNTER(Mot->encoder) - Mot->prevcount);
		}
	}else{
		if(Mot->prevcount > __HAL_TIM_GET_COUNTER(Mot->encoder)){
			newposition = __HAL_TIM_GET_COUNTER(Mot->encoder) + 65535 - Mot->prevcount;
		}else{
			newposition = abs(__HAL_TIM_GET_COUNTER(Mot->encoder) - Mot->prevcount);
		}
	}
	Mot->prevcount = __HAL_TIM_GET_COUNTER(Mot->encoder);
	return newposition;
}
uint16_t Get_LeftEncoderPos(){
	return Get_EncoderPos(&left);
}
uint16_t Get_RightEncoderPos(){
	return Get_EncoderPos(&right);
}
void Set_MotorDir(){

	if(right.dir == true) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
	} else if (right.setRPM == 0.0) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
	}
	if(left.dir == true) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
	} else if (left.setRPM == 0.0) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
	}
}
void Set_PIDOut(float rpm1, float rpm2, UART_HandleTypeDef* huart){
	PIDSetpointSet(&right.PID,right.setRPM);
	PIDInputSet(&right.PID,rpm1);
	PIDCompute(&right.PID);
	PIDSetpointSet(&left.PID,left.setRPM);
	PIDInputSet(&left.PID,rpm2);
	PIDCompute(&left.PID);
	uint16_t speed1 = (uint16_t)PIDOutputGet(&right.PID);
	uint16_t speed2 = (uint16_t)PIDOutputGet(&left.PID);
	speed1 = map(speed1, 0, 255, 0, 2000);
	speed2 = map(speed2, 0, 255, 0, 2000);
//	char buffer[25];
//	uint8_t len = sprintf(buffer, "PWM:%i\r\n", speed1);
//	HAL_UART_Transmit(huart, buffer, len, 1000);
	__HAL_TIM_SetCompare(right.pwm, TIM_CHANNEL_1, speed1);
	__HAL_TIM_SetCompare(left.pwm, TIM_CHANNEL_1, speed2);
	return;
}
void Run_MotorPWM(int16_t pwm){
	__HAL_TIM_SetCompare(right.pwm, TIM_CHANNEL_1, abs(pwm));
	__HAL_TIM_SetCompare(left.pwm, TIM_CHANNEL_1, abs(pwm));
	if(pwm >= 0){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
	} else{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
	}

}
uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
