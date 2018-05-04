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
int i = 0;
float x_dis = 0.0;
void startPP(){
	running = true;
	stopPosPID();
	return;
}
void stopPP(){
	running = false;
	int n = GetCurrentSize();
	for (int i=0;i<n;i++){
		deq();
	}
	return;
}
bool isPP_Running(){
	return running;
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
void RunMotionPlanning(float End_of_Window_Threshold){
	//only operate if path planning is running
	if(!running){
		return;
	}
	//if queue has any items
	if (GetCurrentSize()+QUEUESIZE>QUEUESIZE){
	    return;
	}
//	if (i >= 6){
//		running = false;
//		startPosPID();
//	}
	//double d_x = Get_Ultrasonic_Reading(&x);
	//get ultrasonic y distance
//	d_y = 10.0*GetUltrasonicY();
//	char buffer[25];
//	uint8_t len=sprintf(buffer,"distance:%i\r\n", (int)(d_y)); //sprintf will return the length of 'buffer'
//	HAL_UART_Transmit(&huart1, (unsigned char*)buffer, len, 1000);
//	//if closer than 38mm to barrier, stop and back up 1 inch
//	if (d_y <= BARRIERTHRESHOLD){
//		Stop_Motors();
//		enq("l 25.4 -200");
//	}
	//if finished wwith one side of window and barrier not crossed, cross barrier
	if (/*(x_dis >= End_of_Window_Threshold) && (!BarrierCrossed)*/i==8){
		char buffer[15];
		char temp[5];
		char* out;
		out = gcvt(Get_PP_LinDis()+76.2,5,temp);
		sprintf(buffer,"l %s 150\n",out);
		enq(buffer);
		enq("l 419.1 500");
		BarrierCrossed = true;
		i++;
		return;
	}
	//otherwise
	else {
		lin_dis = Get_PP_LinDis() - WINDOWBOTTOMMARGIN - UltrasonicYPLACEMENT;
		char buffer[15];
		char temp[5];
		char* out;
		out = gcvt(lin_dis,5, temp);
		sprintf(buffer,"l %s 150\n",out);
		enq(buffer);
		//if barrier not crossed traverse one way, switch directions once crossed
		if (!BarrierCrossed){
			memset(buffer, 0, 15);
			out = gcvt((theta*(11/9)),5, temp);
			sprintf(buffer,"r -6 %s\n",out);
			enq(buffer);
		}
		else {
			memset(buffer, 0, 15);
			out = gcvt((theta*(11/9)),5, temp);
			sprintf(buffer,"r 6 %s\n",out);
			enq(buffer);
		}

		lin_dis = -1*(lin_dis/(cos(ROTATIONANGLE*(M_PI/180)))) - 180.0;
		x_dis = x_dis + lin_dis*sin(ROTATIONANGLE*(M_PI/180));
		memset(buffer, 0, 15);
		out = gcvt(lin_dis,5, temp);
		sprintf(buffer,"l %s -300\n",out);
		enq(buffer);

		if (!BarrierCrossed){
			memset(buffer, 0, 15);
			out = gcvt((theta*(11/9)),5, temp);
			sprintf(buffer,"r 6 %s\n",out);
			enq(buffer);
		}
		else {
			memset(buffer, 0, 100);
			out = gcvt((theta*(11/9)),5, temp);
			sprintf(buffer,"r -6 %s\n",out);
			enq(buffer);
		}
	}
	//if finished, return
	if(i>=16) {
		stopPP();
		return;
	}
	i++;
}

