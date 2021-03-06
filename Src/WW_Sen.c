/*
 * WW_Sen.c
 *
 *  Created on: Apr 1, 2018
 *      Author: lee7670 and rishabhk7
 */
#include <WW_Sen.h>
struct imu accel;
struct ultrasonic x;
struct ultrasonic y;
bool encBval_R;
bool encBval_L;

void Ping_Ultrasonic(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin){
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
	HAL_Delay(5);
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
	x.GPIO_PingPin = GPIO_PIN_9;
	x.echo_falling_count = 0;
	x.echo_pulse_width_count = 0;
	x.echo_rising_count = 0;
	x.edge_detect = false;
	x.IC = htim;
	HAL_GPIO_WritePin(x.GPIO_PingBank,x.GPIO_PingPin,GPIO_PIN_RESET);
	HAL_Delay(500);

	y.GPIO_PingBank = GPIOA;
	y.GPIO_PingPin = GPIO_PIN_8;
	y.echo_falling_count = 0;
	y.echo_pulse_width_count = 0;
	y.echo_rising_count = 0;
	y.edge_detect = false;
	y.IC = htim;
	HAL_GPIO_WritePin(y.GPIO_PingBank, y.GPIO_PingPin, GPIO_PIN_RESET);
	HAL_Delay(500);
	return;
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance == TIM5)
	{
		if ((htim->Channel) == HAL_TIM_ACTIVE_CHANNEL_3){
			y.edge_detect = !(y.edge_detect);
			//__HAL_TIM_SET_CAPTUREPOLARITY()
			if (y.edge_detect){
				y.echo_rising_count = __HAL_TIM_GET_COMPARE(y.IC, TIM_CHANNEL_3);    //read TIM2 channel 1 capture value
				//__HAL_TIM_SET_CAPTUREPOLARITY(&htim5,TIM_CHANNEL_3,TIM_INPUTCHANNELPOLARITY_FALLING);
			}
			else {
				y.echo_falling_count = __HAL_TIM_GET_COMPARE(y.IC, TIM_CHANNEL_3);
				if(y.echo_falling_count > y.echo_rising_count){
					__HAL_TIM_SetCounter(y.IC, 0);    //reset counter after input capture interrupt occurs
					y.echo_pulse_width_count = y.echo_falling_count - y.echo_rising_count;
				}
				else {
					y.echo_falling_count = y.echo_falling_count+1048575;
					__HAL_TIM_SetCounter(y.IC, 0);
					y.echo_pulse_width_count = y.echo_falling_count - y.echo_rising_count;
				}
			}
		} else if((htim->Channel) == HAL_TIM_ACTIVE_CHANNEL_4){
			x.edge_detect = !(x.edge_detect);
			//__HAL_TIM_SET_CAPTUREPOLARITY()
			if (x.edge_detect){
				x.echo_rising_count = __HAL_TIM_GET_COMPARE(x.IC, TIM_CHANNEL_4);    //read TIM2 channel 1 capture value
				//__HAL_TIM_SET_CAPTUREPOLARITY(x.IC,TIM_CHANNEL_3,TIM_INPUTCHANNELPOLARITY_FALLING);
			}
			else {
				x.echo_falling_count = __HAL_TIM_GET_COMPARE(x.IC, TIM_CHANNEL_4);
				if(x.echo_falling_count > x.echo_rising_count){
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

	if (htim->Instance == TIM2)
	{
		if ((htim->Channel) == HAL_TIM_ACTIVE_CHANNEL_1){
			encBval_L = (GPIOA->IDR & GPIO_PIN_1);
		}
	}

	if (htim->Instance == TIM3)
	{
		if ((htim->Channel) == HAL_TIM_ACTIVE_CHANNEL_1){
			encBval_R = (GPIOC->IDR & GPIO_PIN_7);
		}
	}
}
bool initIMU(I2C_HandleTypeDef* hi2c){
	accel.I2C = hi2c;
	accel.address = DEV_ADD;
	uint8_t id = checkIMUID();
	if(id != 0xA0){
		HAL_Delay(500);
		id = checkIMUID();
		if(id != 0xA0){
			return false;
		}
	}
	HAL_Delay(30);
	setMode(OPERATION_MODE_CONFIG);
	HAL_Delay(10);

	/* Reset */
	write8(BNO055_SYS_TRIGGER_ADDR, 0x20);
	while (checkIMUID() != BNO055_ID)
	{
		HAL_Delay(10);
	}
	HAL_Delay(50);

	/* Set to normal power mode */
	write8(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
	HAL_Delay(10);

	write8(BNO055_PAGE_ID_ADDR, 0x00);
	write8(BNO055_SYS_TRIGGER_ADDR, 0x00);
	HAL_Delay(10);
	/* Set the requested operating mode (see section 3.3) */
	setMode(OPERATION_MODE_IMUPLUS);
	HAL_Delay(20);
	return true;
}
uint8_t read8(uint8_t regid){
	uint8_t result = 0;
	HAL_I2C_Master_Transmit(accel.I2C, DEV_ADD, &regid, 1, 100);
	HAL_I2C_Master_Receive(accel.I2C, DEV_ADD, &result, 1, 100);
	return result;
}
void readLen(uint8_t regid, uint8_t* buffer, uint8_t len){
	HAL_I2C_Master_Transmit(accel.I2C, DEV_ADD, &regid, 1, 100);
	HAL_I2C_Master_Receive(accel.I2C, DEV_ADD, buffer, len, 100);
	return;
}
void write8(uint8_t regid, uint8_t val){
	HAL_I2C_Master_Transmit(accel.I2C, DEV_ADD, &regid, 1, 100);
	HAL_I2C_Master_Transmit(accel.I2C, DEV_ADD, &val, 1, 100);
	return;
}
uint8_t checkIMUID(){
	return read8(REG_CHIP_ID);
}
void getEuler(float* result){
	uint8_t recieved[6];
	int16_t buffer[3];
	readLen(BNO055_EULER_H_LSB_ADDR, recieved, 6);
	buffer[0] = ((int16_t)recieved[0]) | (((int16_t)recieved[1]) << 8);
	buffer[1] = ((int16_t)recieved[2]) | (((int16_t)recieved[3]) << 8);
	buffer[2] = ((int16_t)recieved[4]) | (((int16_t)recieved[5]) << 8);
	result[0] = (float)(buffer[0])/16.0;
	result[1] = (float)(buffer[1])/16.0;
	result[2] = (float)(buffer[2])/16.0;
	return;
}
double GetUltrasonicY(){
//	double dist_sample;
//	double min = 0.0;
//	for (int i=0;i<20;i++){
//		Ping_Ultrasonic(y.GPIO_PingBank,y.GPIO_PingPin);
//		dist_sample = Get_Ultrasonic_Reading(&y) + 0.728;
//		if (i==0){
//			min = dist_sample;
//		}
//		else {
//			if (dist_sample<min){
//				min = dist_sample;
//			}
//		}
//	}
//	HAL_Delay(10);
//	Ping_Ultrasonic(y.GPIO_PingBank,y.GPIO_PingPin);
//	return min;
	Ping_Ultrasonic(y.GPIO_PingBank,y.GPIO_PingPin);
	return (Get_Ultrasonic_Reading(&y)+0.728);
}
double GetUltrasonicX(){
	Ping_Ultrasonic(x.GPIO_PingBank,x.GPIO_PingPin);
	return Get_Ultrasonic_Reading(&x);
}
void setMode(uint8_t modeid){
	write8(OPR_MODE_ADD, modeid);
	HAL_Delay(30);
	return;
}
bool getEncB_Left_Val(){
	return encBval_L;
}
bool getEncB_Right_Val(){
	return encBval_R;
}
