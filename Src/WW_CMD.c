/*
 * WW_CMD.c
 *
 *  Created on: Mar 25, 2018
 *      Author: Theodore Lee
 */
#include "WW_CMD.h"
unsigned char Rx_indx, Rx_data[2], Transfer_cplt;
char Rx_Buffer[100];
/*
 * Parses commands from the serial port
 * If the HAL_UART_RxCpltCallback has set the transfer completed value, a new command is evaluated
 * Breaks command into tokens based on space delimiter
 */
void Parse_CMD(TIM_HandleTypeDef* Fan_TIM, UART_HandleTypeDef* huart){
	UART_ReadStart(huart);
	if(Transfer_cplt >= 1){
		char cmd[100];
		//clear memory in command buffer
		memset(cmd,0,sizeof cmd);
		//copy command into buffer
		strcpy(cmd, Rx_Buffer);
		//add c string null terminator
		cmd[Transfer_cplt]='\0';
		char* tkpnt;
		//get first token
		tkpnt = strtok(cmd, " ");
		if(strncmp(tkpnt, "s",1)==0){
			/*
			 * Stop command, immediately stops all PWM, Brakes, and zeros all targets
			 */
			__HAL_TIM_SetCompare(Fan_TIM, TIM_CHANNEL_1, 0);
			Stop_Motors();
		}else if(strncmp(tkpnt, "sm",2)==0){
			/*
			 * Stop motor command, immediately stops motor PWM, Brakes, and zeros all targets
			 */
			Stop_Motors();
		}else if(strncmp(tkpnt, "sf",2)==0){
			/*
			 * Stop fan command, immediately stops fan pwm
			 */
			__HAL_TIM_SetCompare(Fan_TIM, TIM_CHANNEL_1, 0);
		}else if(strncmp(tkpnt, "f",1)==0){
			/*
			 * Fan command
			 * Activates fan pin PWM
			 * Accepts values from 0 to 255 which is mapped from 1ms to 2ms pulse width
			 * To control fans, with fan power initially off, make sure PWM is off by sending "f -255"
			 * After fan power is applied, wait for 3 beeps, increasing in pitch
			 * Then apply 0 throttle "f 0"
			 * Wait for arming beeps, there should only be two beeps, a low and a high,
			 * if not, turn off power, wait for caps to discharge and reset pwm to zero "f -255".
			 * Any fan command from 0 to 255 will then start the fans, be careful
			 */
			tkpnt = strtok(NULL, " ");
			int pwm = atoi(tkpnt);
			pwm = pwm + 255;
			__HAL_TIM_SetCompare(Fan_TIM, TIM_CHANNEL_1, pwm);
		}else if(strncmp(tkpnt, "l",1)==0){
			/*
			 * Start Linear Move
			 * Sets targets for a linear move
			 * Format: "l @length @speed"
			 * Ex. "l 10000 500"
			 * Speed can be negative
			 * @length is length of move in mm
			 * @speed is speed of move in mm/s
			 */
			tkpnt = strtok(NULL, " ");
			float lindis = atof(tkpnt);
			tkpnt = strtok(NULL, " ");
			float linspd = atof(tkpnt);
			setLin(lindis, linspd);
		}else if(strncmp(tkpnt, "t",1)==0){
			/*
			 * Prints system time in milliseconds to UART
			 */
			char buffer[100];
			uint8_t len=sprintf(buffer,"%i\r\n", millis()); //sprintf will return the length of 'buffer'
			HAL_UART_Transmit(huart, buffer, len, 1000);
		}else if(strncmp(tkpnt, "d",1)==0){
			/*
			 * Drive motors open loop
			 * Format: "d @pwm"
			 * @pwm is between -2000 and 2000
			 */
			tkpnt = strtok(NULL, " ");
			int16_t pwm = atoi(tkpnt);
			Run_MotorPWM(pwm);
		#ifdef DEBUG
		}else if(strncmp(tkpnt, "e",1)==0){
			/*
			 * Prints encoder timer count and count since last call of Get_Left/RightEncoderPos()
			 * !Debug only
			 * Resets internal variable in Encoder delta position function
			 */
			char buffer[25];
			uint8_t len = sprintf(buffer,"Right Encoder Delta :%i\r\n", Get_RightEncoderPos()); //sprintf will return the length of 'buffer'
			HAL_UART_Transmit(huart, buffer, len, 1000);
			len = sprintf(buffer,"Right Encoder:%i\r\n", TIM3->CNT); //sprintf will return the length of 'buffer'
			HAL_UART_Transmit(huart, buffer, len, 1000);
			len = sprintf(buffer,"Left Encoder Delta:%i\r\n", Get_LeftEncoderPos());
			HAL_UART_Transmit(huart, buffer, len, 1000);
			len = sprintf(buffer,"Left Encoder:%i\r\n", TIM2->CNT); //sprintf will return the length of 'buffer'
			HAL_UART_Transmit(huart, buffer, len, 1000);
		#endif
		}else{
			char buffer[25];
			uint8_t len = sprintf(buffer,"Invalid Command\r\n"); //sprintf will return the length of 'buffer'
			HAL_UART_Transmit(huart, buffer, len, 1000);
		}
		//signal ready for new command
		Transfer_cplt = 0;
	}
}
//Interrupt callback routine
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//when character received
    uint8_t i;
    if (huart->Instance == USART1)  //current UART
	{
		if (Rx_indx == 0) {
			//clear Rx_Buffer before receiving new data
			for (i=0;i<100;i++) Rx_Buffer[i]=0;
		}

		if (Rx_data[0] != 13) //if received data different from ASCII 13 (carriage return \r)
		{
			Rx_Buffer[Rx_indx++] = Rx_data[0];    //add data to Rx_Buffer
		}
		else            //if received data = 13
		{
			Transfer_cplt = Rx_indx;//transfer complete, data is ready to read
			Rx_indx=0;
		}

		HAL_UART_Receive_IT(huart, Rx_data, 1);   //activate UART receive interrupt every time
	}

}
//Wrapper function to force recieve interrupt start
void UART_ReadStart(UART_HandleTypeDef *huart){
	HAL_UART_Receive_IT(huart, Rx_data, 1);
}

