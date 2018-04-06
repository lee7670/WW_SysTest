/*
 * WW_Defs.c
 *
 *  Created on: Apr 3, 2018
 *      Author: lee7670
 */
#include "WW_Defs.h"
struct cmdqueue COM;
//char** queue;
char buildqueue[QUEUESIZE][100];
char deqtemp[100];
void initCOM(){
	COM.back = QUEUESIZE-1;
	COM.front = 0;
	COM.size = 0;
	COM.capacity = (uint8_t)QUEUESIZE;
	for(int i = 0; i<QUEUESIZE; i++){
		for(int j=0; j<100; j++){
			COM.cmds[i][j] = 0;
		}
	}
	return;
}
int isFull(){
	if(COM.size == COM.capacity) return true;
	return false;

}
int isEmpty(){
	if (COM.size == 0) return true;
	return false;
}
char* deq(){
	if(isEmpty()){
		return "";
	}

	strcpy(deqtemp, COM.cmds[COM.front]);
	char*item = deqtemp;
	memset(COM.cmds[COM.front],0, sizeof COM.cmds[COM.front]);
	COM.front = (COM.front + 1) % COM.capacity;
	COM.size -=1;
	return item;
}
int enq(char* str){
	if(isFull()){
		return false;
	}
	char temp[100];
	memset(temp, 0, sizeof temp);
	strcpy(temp, str);

	COM.back = (COM.back + 1) % COM.capacity;
	strcpy(COM.cmds[COM.back], temp);
	COM.size += 1;
	return true;
}
