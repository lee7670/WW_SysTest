/*
 * WW_Sen.c
 *
 *  Created on: Apr 1, 2018
 *      Author: lee7670 and rishabhk7
 */
#include <WW_Sen.h>
struct ultrasonic x;
struct ultrasonic y;
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance==TIM5)
	{
		if ((htim->Channel)==HAL_TIM_ACTIVE_CHANNEL_3){
			y.edge_detect = !(y.edge_detect);
			//__HAL_TIM_SET_CAPTUREPOLARITY()
			if (y.edge_detect){
				y.echo_rising_count = __HAL_TIM_GET_COMPARE(y.IC, TIM_CHANNEL_3);    //read TIM2 channel 1 capture value
				//__HAL_TIM_SET_CAPTUREPOLARITY(&htim5,TIM_CHANNEL_3,TIM_INPUTCHANNELPOLARITY_FALLING);
			}
			else {
				y.echo_falling_count = __HAL_TIM_GET_COMPARE(y.IC, TIM_CHANNEL_3);
				if(y.echo_falling_count>y.echo_rising_count){
					__HAL_TIM_SetCounter(y.IC, 0);    //reset counter after input capture interrupt occurs
					y.echo_pulse_width_count = y.echo_falling_count - y.echo_rising_count;
				}
				else {
					y.echo_falling_count += 1048575;
					__HAL_TIM_SetCounter(y.IC, 0);
					y.echo_pulse_width_count = y.echo_falling_count - y.echo_rising_count;
				}
			}
		} else if((htim->Channel)==HAL_TIM_ACTIVE_CHANNEL_4){
			x.edge_detect = !(x.edge_detect);
			//__HAL_TIM_SET_CAPTUREPOLARITY()
			if (x.edge_detect){
				x.echo_rising_count = __HAL_TIM_GET_COMPARE(x.IC, TIM_CHANNEL_4);    //read TIM2 channel 1 capture value
				//__HAL_TIM_SET_CAPTUREPOLARITY(x.IC,TIM_CHANNEL_3,TIM_INPUTCHANNELPOLARITY_FALLING);
			}
			else {
				x.echo_falling_count = __HAL_TIM_GET_COMPARE(x.IC, TIM_CHANNEL_4);
				if(x.echo_falling_count>x.echo_rising_count){
					__HAL_TIM_SetCounter(x.IC, 0);    //reset counter after input capture interrupt occurs
					x.echo_pulse_width_count = x.echo_falling_count - x.echo_rising_count;
				}
				else {
					x.echo_falling_count += 1048575;
					__HAL_TIM_SetCounter(x.IC, 0);
					x.echo_pulse_width_count = x.echo_falling_count - x.echo_rising_count;
				}
			}
		}
	}
}
void Ping_Ultrasonic(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin){
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
	return;
}
double Get_Ultrasonic_Reading(struct ultrasonic* ult){
	double d;
	d = (((ult->echo_pulse_width_count)*(1/SYSCLK))/(1e-6))*(0.034/2);
	d = (0.997889*d)-0.26247;
	return d;
}
void initUltrasonics(TIM_HandleTypeDef* htim){
	x.GPIO_PingBank = GPIOA;
	x.GPIO_PingPin = 9;
	x.echo_falling_count = 0;
	x.echo_pulse_width_count = 0;
	x.echo_rising_count = 0;
	x.edge_detect = false;
	x.IC = htim;

	y.GPIO_PingBank = GPIOA;
	y.GPIO_PingPin = 8;
	y.echo_falling_count = 0;
	y.echo_pulse_width_count = 0;
	y.echo_rising_count = 0;
	y.edge_detect = false;
	y.IC = htim;
	return;
}
