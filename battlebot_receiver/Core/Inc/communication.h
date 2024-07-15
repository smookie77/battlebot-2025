/*
 * communication.h
 *
 *  Created on: Jul 8, 2024
 *      Author: smookie
 */

#ifndef INC_COMMUNICATION_H_
#define INC_COMMUNICATION_H_

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <stm32f1xx_hal.h>

//Function prototypes
void adjustCTRL();
void InitCom();
bool adjustFromMSG(char MSG[32]);

//LIB variables
uint8_t contBat = 0;
uint16_t leftSpeed = 0;
uint16_t rightSpeed = 0;
uint16_t midSpeed = 0;

#define PWM_CTRL
//#define DAC_CTRL   //If PWM doesn't work

#ifdef PWM_CTRL

extern TIM_HandleTypeDef htim4;
#define LEFT_MOTOR_TIM &htim4
#define LEFT_MOTOR_CHN TIM_CHANNEL_1
#define RIGHT_MOTOR_TIM &htim4
#define RIGHT_MOTOR_CHN TIM_CHANNEL_2

void InitCom(){
	if(HAL_TIM_PWM_Start(LEFT_MOTOR_TIM, LEFT_MOTOR_CHN) != HAL_OK){
		Error_Handler();
	}
	if(HAL_TIM_PWM_Start(RIGHT_MOTOR_TIM, RIGHT_MOTOR_CHN) != HAL_OK){
		Error_Handler();
	}
}

void adjustCTRL(){
	__HAL_TIM_SET_COMPARE(LEFT_MOTOR_TIM, LEFT_MOTOR_CHN, leftSpeed);
	__HAL_TIM_SET_COMPARE(RIGHT_MOTOR_TIM, LEFT_MOTOR_CHN, rightSpeed);
}
#else

#define LEFT_MOTOR_DAC hadc
#define LEFT_MOTOR_CHN DAC_CHANNEL_1
#define RIGHT_MOTOR_DAC hadc
#define RIGHT_MOTOR_CHN DAC_CHANNEL_2
void adjustCTRL(){
	HAL_DAC_SetValue(&LEFT_MOTOR_DAC, LEFT_MOTOR_CHN, DAC_ALIGN_12B_R, leftSpeed);
	HAL_DAC_SetValue(&RIGHT_MOTOR_DAC, RIGHT_MOTOR_CHN, DAC_ALIGN_12B_R, rightSpeed);
}
#endif
#endif /* INC_COMMUNICATION_H_ */
