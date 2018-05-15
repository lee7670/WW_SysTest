/*
 * WW_PP.c
 *
 *  Created on: Apr 11, 2018
 *      Author: Rishabh K.
 */
#include <WW_PP.h>
bool BarrierCrossed;
int len, lenr;
char *dis, *rot;
float lin_dis;
float theta = ROTATIONANGLE;
double d_y;
bool running = false;
bool BC_CMD;
int i = 0;
float x_dis;
char buffer[15];
char temp[5];
char* out;
void (*state)();
void startPP(){
	running = true;
	BarrierCrossed = false;
	BC_CMD = true;
	x_dis = 0.0;
	stopPosPID();
	state = Linear_Move_Down;
	return;
}
void stopPP(){
	running = false;
	BarrierCrossed = false;
	BC_CMD = true;
	x_dis = 0.0;
	int n = GetCurrentSize();
	for (int i=0;i<n;i++){
		deq();
	}
	return;
}
bool isPP_Running(){
	return running;
}
bool Barrier_Crossed(){
	return BarrierCrossed;
}
void togglePP(){
	if (running==false){
		startPP();
	}
	else
	{
		stopPP();
	}
}
void RunMotionPlanning(){
	//only operate if path planning is running
	if(!running){
		return;
	}
	//if queue has any items
	if (isFull()){
	    return;
	}
	state();
}
void Linear_Move_Down(){
	lin_dis = Get_PP_LinDis() - WINDOWBOTTOMMARGIN - UltrasonicYPLACEMENT;
	memset(buffer, 0, 15);
	if (x_dis >= 2*WINDOWLENGTH){
		stopPP();
	}
	else {
		if (x_dis >= WINDOWLENGTH && !BarrierCrossed) {
		  out = gcvt((Get_PP_LinDis()+76.2+330.2),5, temp);
		  sprintf(buffer,"l %s 400\n",out);
		  enq(buffer);
		  __HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_2,60);
		  state = Barrier_Crossing;
		}
		else {
		  out = gcvt(lin_dis,5, temp);
		  sprintf(buffer,"l %s 250\n",out);
		  enq(buffer);
		  state = Rotation_to_Angle;
		}
	}
}
void Rotation_to_Angle(){
	memset(buffer, 0, 15);
	out = gcvt((theta*(11/9)),5, temp);
	if (!BarrierCrossed) {
		sprintf(buffer,"r -12 %s\n",out);
	}
	else {
		sprintf(buffer,"r 12 %s\n",out);
	}

	enq(buffer);
	state = Linear_Move_Up_at_Angle;
}
void Linear_Move_Up_at_Angle(){
	memset(buffer, 0, 15);
	float d = -1*((Get_PP_LinDis() - WINDOWBOTTOMMARGIN - UltrasonicYPLACEMENT)/(cos(ROTATIONANGLE*(M_PI/180)))) - 130.0;
	x_dis = x_dis - d*sin(ROTATIONANGLE*(M_PI/180));
	out = gcvt(d,5, temp);
	sprintf(buffer,"l %s -400\n",out);
	enq(buffer);
	state = Rotation_to_Straight;
}
void Rotation_to_Straight(){
	memset(buffer, 0, 15);
	out = gcvt((theta*(11/9)),5, temp);
	if (!BarrierCrossed) {
		sprintf(buffer,"r 12 %s\n",out);
	}
	else {
		sprintf(buffer,"r -12 %s\n",out);
	}

	enq(buffer);
	state = Linear_Move_Down;
}
void Barrier_Crossing(){
	memset(buffer, 0, 15);
	if (BC_CMD) {
		out = gcvt(25.4,5, temp);
		sprintf(buffer,"l %s 250\n",out);
		enq(buffer);
		BC_CMD = false;
		state = Barrier_Crossing;
	}
	else {
		__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_2,540);
		HAL_Delay(1000);
		BC_CMD = true;
		BarrierCrossed = true;
		state = Linear_Move_Down;
	}
}
