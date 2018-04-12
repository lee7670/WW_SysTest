/*
 * WW_Defs.h
 *
 *  Created on: Mar 30, 2018
 *      Author: lee7670
 */

#ifndef WW_DEFS_H_
#define WW_DEFS_H_
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#define DEBUG
#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif
#define SYSCLK 32000000.0F
#define QUEUESIZE 5
struct cmdqueue{
	uint8_t front;
	uint8_t back;
	uint8_t size;
	uint8_t capacity;
	char cmds[QUEUESIZE][100];
};
void initCOM();
int isFull();
int isEmpty();
char* deq();
int enq(char* str);
int GetCurrentSize();
#endif /* WW_DEFS_H_ */
