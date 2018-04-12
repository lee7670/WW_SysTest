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
	if(!running){
		return;
	}
	if (GetCurrentSize()+QUEUESIZE>QUEUESIZE){
	    return;
	}
	//double d_x = Get_Ultrasonic_Reading(&x);
	d_y = 10.0*GetUltrasonicY();
	if (d_y <= 38){
		Stop_Motors();
		enq("l 25.4 -200");
	}

	if (/*(d_x >= End_of_Window_Threshold) && (!BarrierCrossed)*/ false){
		enq("b 180");
		enq("l 991 500");
		BarrierCrossed = true;
	}

	else {
		lin_dis = d_y - WINDOWBOTTOMMARGIN - UltrasonicYPLACEMENT;
		char buffer[15];
		char temp[5];
		char* out;
		out = gcvt(lin_dis,5, temp);
		sprintf(buffer,"l %s 200\n",out);
		enq(buffer);
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

	if ((BarrierCrossed)/*&&(d_x<=WINDOWBOTTOMMARGIN)*/&&(d_y<=WINDOWBOTTOMMARGIN)){
		return;
	}
}

