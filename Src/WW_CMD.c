/*
 * WW_CMD.c
 *
 *  Created on: Mar 25, 2018
 *      Author: Theodore Lee
 */
#include "WW_CMD.h"
unsigned char Rx_indx, Rx_data[2], Transfer_cplt;
char Rx_Buffer[100];
void Parse_CMD(TIM_HandleTypeDef* Fan_TIM, UART_HandleTypeDef* huart){
	if(Transfer_cplt >= 1){
		char cmd[100];
		memset(cmd,0,sizeof cmd);
		strcpy(cmd, Rx_Buffer);
		cmd[Transfer_cplt]='\0';
		char* tkpnt;
		tkpnt = strtok(cmd, " ");
		if(strncmp(tkpnt, "f",1)==0){
			tkpnt = strtok(NULL, " ");
			int pwm = atoi(tkpnt);
			pwm = pwm + 255;
			__HAL_TIM_SetCompare(Fan_TIM, TIM_CHANNEL_1, pwm);
		}else if(strncmp(tkpnt, "l",1)==0){
			tkpnt = strtok(NULL, " ");
			float lindis = atof(tkpnt);
			tkpnt = strtok(NULL, " ");
			float linspd = atof(tkpnt);
			setLin(lindis, linspd);
		}else if(strncmp(tkpnt, "t",1)==0){
			char buffer[100];
			uint8_t len=sprintf(buffer,"%i\r\n", Get_Time()); //sprintf will return the length of 'buffer'
			HAL_UART_Transmit(huart, buffer, len, 1000);
		}else if(strncmp(tkpnt, "d",1)==0){
			tkpnt = strtok(NULL, " ");
			int16_t pwm = atoi(tkpnt);
			Run_MotorPWM(pwm);
		}else if(strncmp(tkpnt, "e",1)==0){
			char buffer[25];
			uint8_t len=sprintf(buffer,"Right Encoder Delta :%i\r\n", Get_RightEncoderPos()); //sprintf will return the length of 'buffer'
			HAL_UART_Transmit(huart, buffer, len, 1000);
			len=sprintf(buffer,"Right Encoder:%i\r\n", TIM3->CNT); //sprintf will return the length of 'buffer'
			HAL_UART_Transmit(huart, buffer, len, 1000);
			len = len=sprintf(buffer,"Left Encoder Delta:%i\r\n", Get_LeftEncoderPos());
			HAL_UART_Transmit(huart, buffer, len, 1000);
			len=sprintf(buffer,"Left Encoder:%i\r\n", TIM2->CNT); //sprintf will return the length of 'buffer'
			HAL_UART_Transmit(huart, buffer, len, 1000);
		}

		Transfer_cplt = 0;
	}
}
//Interrupt callback routine
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t i;
    if (huart->Instance == USART1)  //current UART
	{
		if (Rx_indx==0) {for (i=0;i<100;i++) Rx_Buffer[i]=0;}   //clear Rx_Buffer before receiving new data

		if (Rx_data[0]!=13) //if received data different from ascii 13 (enter)
		{
			Rx_Buffer[Rx_indx++]=Rx_data[0];    //add data to Rx_Buffer
		}
		else            //if received data = 13
		{
			Transfer_cplt=Rx_indx;//transfer complete, data is ready to read
			Rx_indx=0;
		}

		HAL_UART_Receive_IT(huart, Rx_data, 1);   //activate UART receive interrupt every time
	}

}
void UART_ReadStart(UART_HandleTypeDef *huart){
	HAL_UART_Receive_IT(huart, Rx_data, 1);
}

