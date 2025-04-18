/*
 * nesController.h
 *
 *  Created on: Mar 26, 2025
 *      Author: mgdoan
 */

#ifndef SRC_THRUSTERS_H_
#define SRC_THRUSTERS_H_



#endif

#include "stdio.h"

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim1;

int MIN_PWM_VAL = 80;
int MAX_PWM_VAL = 160;
int PWM_VAL_SET = 100; // start this at a low speed

// 0 is left thruster, 1 is right thruster
void flash_thruster(int thruster) {
	uint32_t channel;
	if(thruster){
		channel = TIM_CHANNEL_1; //right thruster
	}
	else{
		channel = TIM_CHANNEL_2; //left thruster
	}

	__HAL_TIM_SET_COMPARE(&htim2, channel, 0);
	HAL_Delay(1000);

	// set duty cycle to 100 percent
	__HAL_TIM_SET_COMPARE(&htim2, channel, MAX_PWM_VAL);
	HAL_Delay(2000);

	// set duty cycle to 0 percent
	__HAL_TIM_SET_COMPARE(&htim2, channel, MIN_PWM_VAL);
	HAL_Delay(5000);
	printf("thruster initialized \r\n");
}

void pwm_val_increase(){
	if(PWM_VAL_SET < MAX_PWM_VAL){
		PWM_VAL_SET++;
	}
	printf("PWM val increased to %d \r\n", PWM_VAL_SET);
}

void pwm_val_decrease(){
	if(PWM_VAL_SET > MIN_PWM_VAL){
		PWM_VAL_SET--;
	}
	printf("PWM val decreased to %d \r\n", PWM_VAL_SET);
}


void init_thrusters() {
//	uint32_t channel;
//	int PWM_Val;
//	if(thruster){
//		channel = TIM_CHANNEL_1; //right thruster
//	}
//	else{
//		channel = TIM_CHANNEL_2; //left thruster
//	}
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, MIN_PWM_VAL);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, MIN_PWM_VAL);
	HAL_Delay(5000);
}

void init_thruster(int thruster) {
	//uint32_t channel;
	//int PWM_Val;
	if(thruster){
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, MIN_PWM_VAL);
		//channel = TIM_CHANNEL_3; //right thruster
	}
	else{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, MIN_PWM_VAL);
		//channel = TIM_CHANNEL_2; //left thruster
	}
	HAL_Delay(5000);
}

void set_thruster(int thruster) {
	if(thruster){
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_VAL_SET);
	}
	else{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM_VAL_SET);
	}
	return;
}


void set_thruster_reverse(int thruster) {
	uint32_t channel;
		if(thruster){
			channel = TIM_CHANNEL_1; //right thruster
		}
		else{
			channel = TIM_CHANNEL_2; //left thruster
		}
	__HAL_TIM_SET_COMPARE(&htim2, channel, PWM_VAL_SET);//normal now
	return;
}

void thruster_step_test(int thruster){
	uint32_t channel;
		if(thruster){
			channel = TIM_CHANNEL_1; //right thruster
		}
		else{
			channel = TIM_CHANNEL_2; //left thruster
		}
	for(int pwm_val = MIN_PWM_VAL; pwm_val <= MAX_PWM_VAL; pwm_val++){
		__HAL_TIM_SET_COMPARE(&htim2, channel, pwm_val);
		printf("PWM val: %d \r\n", pwm_val);
		HAL_Delay(1000);
	}
}


void stop_thruster(int thruster) {
	if(thruster){
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, MIN_PWM_VAL);
	}
	else{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, MIN_PWM_VAL);
	}
	return;
}
