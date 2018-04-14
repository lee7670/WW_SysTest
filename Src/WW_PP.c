/*
 * WW_PP.c
 *
 *  Created on: Apr 11, 2018
 *      Author: Rishabh K.
 */
#include <WW_PP.h>
bool BarrierCrossed = false;
int len, lenr;
char *dis, *rot;
float lin_dis;
float theta = ROTATIONANGLE;
double d_y;
bool running = false;
void startPP(){
	running = !running;
	return;
}

void RunMotionPlanning(float End_of_Window_Threshold){
	//only operate if path planning is running
	if(!running){
		return;
	}
	//if queue is full
	if (GetCurrentSize()+QUEUESIZE>QUEUESIZE){
	    return;
	}
	//double d_x = Get_Ultrasonic_Reading(&x);
	//get ultrasonic y distance
	d_y = 10.0*GetUltrasonicY();
	char buffer[25];
	uint8_t len=sprintf(buffer,"distance:%i\r\n", (int)(d_y)); //sprintf will return the length of 'buffer'
	HAL_UART_Transmit(&huart1, (unsigned char*)buffer, len, 1000);
	//if closer than 38mm to barrier, stop and back up 1 inch
	if (d_y <= BARRIERTHRESHOLD){
		Stop_Motors();
		enq("l 25.4 -200");
	}
	//if finished wwith one side of window and barrier not crossed, cross barrier
	if (/*(d_x >= End_of_Window_Threshold) && (!BarrierCrossed)*/ false){
		enq("b 180");
		enq("l 991 500");
		BarrierCrossed = true;
	}
	//otherwise
	else {
		lin_dis = d_y - WINDOWBOTTOMMARGIN - UltrasonicYPLACEMENT;
		char buffer[15];
		char temp[5];
		char* out;
		out = gcvt(lin_dis,5, temp);
		sprintf(buffer,"l %s 200\n",out);
		enq(buffer);
		//if barrier not crossed traverse one way, switch directions once crossed
		if (!BarrierCrossed){
			memset(buffer, 0, 15);
			out = gcvt(theta,5, temp);
			sprintf(buffer,"r 15 %s\n",out);
			enq(buffer);
		}
		else {
			memset(buffer, 0, 15);
			out = gcvt(theta,5, temp);
			sprintf(buffer,"r -15 %s\n",out);
			enq(buffer);
		}

		lin_dis = WINDOWANGLEDDISTANCE;
		memset(buffer, 0, 15);
		out = gcvt(lin_dis,5, temp);
		sprintf(buffer,"l %s -200\n",out);
		enq(buffer);

		if (!BarrierCrossed){
			memset(buffer, 0, 15);
			out = gcvt(theta,5, temp);
			sprintf(buffer,"r -15 %s\n",out);
			enq(buffer);
		}
		else {
			memset(buffer, 0, 100);
			out = gcvt(theta,5, temp);
			sprintf(buffer,"r 15 %s\n",out);
			enq(buffer);
		}

	}
	//if finished, return
	if ((BarrierCrossed)/*&&(d_x<=WINDOWBOTTOMMARGIN)*/&&(d_y<=WINDOWBOTTOMMARGIN)){
		return;
	}
}

