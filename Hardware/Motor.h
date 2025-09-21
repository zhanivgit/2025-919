#include "stm32f10x.h" 
#ifndef __MOTOR_H
#define __MOTOR_H
//14 15右后轮   12 13 左后轮       
#define MOTOR_AIN1_PIN    GPIO_Pin_13   //后轮
#define MOTOR_AIN2_PIN    GPIO_Pin_12
#define MOTOR_PWMA_PIN    GPIO_Pin_1
#define MOTOR_BIN1_PIN    GPIO_Pin_14
#define MOTOR_BIN2_PIN    GPIO_Pin_15                                                                                      
#define MOTOR_PWMB_PIN    GPIO_Pin_0
//8  9  左前轮   5  10  右前轮
#define MOTOR_CIN1_PIN    GPIO_Pin_8    //前轮
#define MOTOR_CIN2_PIN    GPIO_Pin_9
#define MOTOR_PWMC_PIN    GPIO_Pin_6
#define MOTOR_DIN1_PIN    GPIO_Pin_5
#define MOTOR_DIN2_PIN    GPIO_Pin_10 
#define MOTOR_PWMD_PIN    GPIO_Pin_7

#define MOTOR_GPIO_PORT   GPIOB

void Motor_Init(void);
void MotorA_SetSpeed(int16_t speed);
void MotorB_SetSpeed(int16_t speed);
void MotorC_SetSpeed(int16_t speed);
void MotorD_SetSpeed(int16_t speed);
void Motor_Stop(void);
void Move(int16_t speed);
void Motor_TranslateLeft(int16_t speed);
void Motor_TranslateRight(int16_t speed);

#endif
