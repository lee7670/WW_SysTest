/*
 * WW_PP.c
 *
 *  Created on: Apr 11, 2018
 *      Author: Rishabh K.
 */
#include <WW_PP.h>
bool BarrierCrossed = false;
struct ultrasonic x;
struct ultrasonic y;
int len, lenr;
char *dis, *rot;
float lin_dis;
float theta = ROTATIONANGLE;

void RunMotionPlanning(float End_of_Window_Threshold){
	double d_x = Get_Ultrasonic_Reading(&x);
	double d_y = Get_Ultrasonic_Reading(&y);
	if ((d_x >= End_of_Window_Threshold) && (!BarrierCrossed)){
		enq("b 180");
		enq("l 991 500");
	}

	else {
		lin_dis = d_y - WINDOWBOTTOMMARGIN - UltrasonicYPLACEMENT;
		len = snprintf("", 0, "%f", lin_dis);
		dis = (char *)malloc(len + 1);
		snprintf(dis, len + 1, "%f", lin_dis);
		char buffer[25];
		strcpy(buffer, "l ");
		strcat(buffer, dis);
		strcat(buffer, " 200");
		if (d_y <= WINDOWBOTTOMMARGIN + UltrasonicYPLACEMENT){
			enq("sm");
		}
		else {
			enq(buffer);
		}
		free(dis);
		if (!BarrierCrossed){
			lenr = snprintf("", 0, "%f",theta);
			rot = (char *)malloc(lenr + 1);
			snprintf(rot, lenr + 1, "%f", theta);
			memset(buffer, 0, 25);
			strcpy(buffer,"r 15 ");
			strcat(buffer, rot);
			enq(buffer);
			free(rot);
		}
		else {
			lenr = snprintf("", 0, "%f",theta);
			rot = (char *)malloc(lenr + 1);
			snprintf(rot, lenr + 1, "%f", theta);
			memset(buffer, 0, 25);
			strcpy(buffer,"r -15 ");
			strcat(buffer, rot);
			enq(buffer);
			free(rot);
		}

		 lin_dis = WINDOWANGLEDDISTANCE;
	     len = snprintf("", 0, "%f", lin_dis);
		 dis = (char *)malloc(len + 1);
		 snprintf(dis, len + 1, "%f", lin_dis);
		 memset(buffer, 0, 25);
		 strcpy(buffer, "l ");
		 strcat(buffer, dis);
		 strcat(buffer, " -100");
		 enq(buffer);

		if (!BarrierCrossed){
			lenr = snprintf("", 0, "%f",theta);
			rot = (char *)malloc(lenr + 1);
			snprintf(rot, lenr + 1, "%f", theta);
			memset(buffer, 0, 25);
			strcpy(buffer,"r -15 ");
			strcat(buffer, rot);
			enq(buffer);
			free(rot);
		}
		else {
			lenr = snprintf("", 0, "%f",theta);
			rot = (char *)malloc(lenr + 1);
			snprintf(rot, lenr + 1, "%f", theta);
			memset(buffer, 0, 25);
			strcpy(buffer,"r 15 ");
			strcat(buffer, rot);
			enq(buffer);
			free(rot);
		}

	}

	if ((BarrierCrossed)&&(d_x<=WINDOWBOTTOMMARGIN)&&(d_y<=WINDOWBOTTOMMARGIN)){
		return;
	}
}

