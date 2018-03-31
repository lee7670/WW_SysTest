/*
 * WW_Loc.c
 *
 *  Created on: Mar 25, 2018
 *      Author: lee7670
 */

#include "WW_Loc.h"
struct motor left;
struct motor right;
//initiate right and left motor data structures. Also initializes PID controllers.
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
	left.setRPM = 0.0;
	left.setDis = 0.0;
	left.distance_traveled = 0.0f;
	PIDInit(&right.PID, KP, KI, KD, .1, ((float)PID_PERIOD)/1000.0, 255, AUTOMATIC, DIRECT);
	PIDInit(&left.PID, KP, KI, KD, .1, ((float)PID_PERIOD)/1000.0, 255, AUTOMATIC, DIRECT);

}
//sets targets for arc move
void setArc(float R/*mm*/, float w/*degrees/s*/, float phi/*degrees*/){
	float scalingfactor = 60.0/360.0;//(seconds/minute)/(degrees/revolution)
	right.setRPM = (w*(R+CENTERDIS))*scalingfactor*(1/WHEELRAD);
	left.setRPM = (w*(R-CENTERDIS))*scalingfactor*(1/WHEELRAD);
	//set appropriate values for reverse operation
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
	//get arc length for distance target
	right.setDis = abs((R+CENTERDIS)*phi*M_PI/180);
	left.setDis = abs((R-CENTERDIS)*phi*M_PI/180);
	//adjust again for negative phi
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
//sets targets for linear move
void setLin(float dis/*mm*/, float spd/*mm/s*/){
	//calculate rpm
	right.setRPM = (spd/(2.0*M_PI*WHEELRAD))*60.0;
	left.setRPM = right.setRPM;
	//reverse if rpm negative
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
	//set distance
	right.setDis = dis;
	left.setDis = dis;
	//Re-initialize targeting to a rotation/distance
	right.distance_traveled = 0.0;
	left.distance_traveled = 0.0;

	return;
}
//Main PID task function
void Run_PID(UART_HandleTypeDef* huart){
	//previous timestam the full task was run
	static uint64_t prevtim = 0;
	//get current time
	volatile uint64_t tim = millis();
	volatile uint32_t deltat = (uint32_t)(tim - prevtim);
	//if PID_PERIOD time has passed, run routine
	if(deltat < PID_PERIOD){
		return;
	}
	//setup GPIOS for appropriate motor directions
	Set_MotorDir();
	//get absolute value of encoder change accounting for over/underflow
	uint16_t newposition1 = Get_RightEncoderPos();
	uint16_t newposition2 = Get_LeftEncoderPos();
	//set previous time to current time
	prevtim = tim;
	//calculate current RPM for PID velocity control and Real linear speed for stop condition
	float vel1 = ((float)newposition1) * 10; //encoder pulses per second
	float rpm1 = ((vel1 * 60)/3000); //Measured motor RPM
	float vel2 = ((float)newposition2) * 10; //encoder pulses per second
	float rpm2 = ((vel2 * 60)/3000);
	float realspeed1 = (rpm1 * 2.0 * M_PI * WHEELRAD)/60; //Linear speed in mm/s
	float realspeed2 = (rpm2 * 2.0 * M_PI * WHEELRAD)/60; //Linear speed in mm/s
	//if negative, reverse
	if (right.dir)
	{
		rpm1 = -1*rpm1;
	}
	if (left.dir)
	{
		rpm2 = -1*rpm2;
	}
	//run PID calculations and set outputs
	Set_PIDOut(rpm1, rpm2, huart);
	//check for stop condition
	right.distance_traveled = right.distance_traveled + (realspeed1*deltat*1e-3); //integrate linear velocity to obtain distance
	left.distance_traveled = left.distance_traveled + (realspeed2*deltat*1e-3); //integrate linear velocity to obtain distance
	if ((abs(right.distance_traveled-right.setDis) <= TOLERANCE)||(abs(left.distance_traveled-left.setDis) <= TOLERANCE) ){
		//Stop condition, reset all values
		__HAL_TIM_SetCompare(right.pwm, TIM_CHANNEL_1, 0);
		__HAL_TIM_SetCompare(left.pwm, TIM_CHANNEL_1, 0);
		right.setRPM = 0.0; //Brake
		right.dir = false;
		left.setRPM = 0.0; //Brake
		left.dir = false;
		right.setDis = 0.0;
		left.setDis = 0.0;
		Set_MotorDir();
		Get_EncoderPos(&right);
		Get_EncoderPos(&left);
	}
	return;
}
//get Absolute difference in count of requested encoder since last call of Get_EncoderPos(), accounting for over/underflow
uint16_t Get_EncoderPos(struct motor* Mot){
	uint16_t newposition;
	//establish one master value of count
	uint16_t currentcount = __HAL_TIM_GET_COUNTER(Mot->encoder);
	if(__HAL_TIM_IS_TIM_COUNTING_DOWN(Mot->encoder)){
		if(Mot->prevcount < currentcount){
			//underflow condition
			newposition = 65535 - currentcount + Mot->prevcount;
		}else{
			newposition = abs(Mot->prevcount-currentcount);
		}
	}else{
		if(Mot->prevcount > currentcount){
			//overflow condition
			newposition = currentcount + 65535 - Mot->prevcount;
		}else{
			newposition = abs(currentcount - Mot->prevcount);
		}
	}
	Mot->prevcount = currentcount;
	return newposition;
}
//wrapper
uint16_t Get_LeftEncoderPos(){
	return Get_EncoderPos(&left);
}
//wrapper
uint16_t Get_RightEncoderPos(){
	return Get_EncoderPos(&right);
}
//sets GPIOS for requested motor direction/break for L298 driver based on motor's data structure values
void Set_MotorDir(){
	if (right.setRPM == 0.0) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
	} else if(right.dir == true) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
	}
	if (left.setRPM == 0.0) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
	}else if(left.dir == true) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
	}
}
//updates left and right PID setpoints and Inputs, computes new output and updates PWM
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
	__HAL_TIM_SetCompare(right.pwm, TIM_CHANNEL_1, speed1);
	__HAL_TIM_SetCompare(left.pwm, TIM_CHANNEL_1, speed2);
	return;
}
//run motor open loop, debug only, does not work when PID is enabled
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
//hard stop motors and reset targets
void Stop_Motors(){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
	__HAL_TIM_SetCompare(right.pwm, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(left.pwm, TIM_CHANNEL_1, 0);
	right.setRPM = 0.0; //Brake
	right.dir = false;
	left.setRPM = 0.0; //Brake
	left.dir = false;
	right.setDis = 0.0;
	left.setDis = 0.0;
	Get_EncoderPos(&right);
	Get_EncoderPos(&left);
	return;
}
//Arduino map function
uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
