#include "stm32f10x.h" 
#ifndef __MOTOR_H
#define __MOTOR_H
// --- 后轮电机引脚定义 ---
// 左后轮 (Motor A)
#define MOTOR_A_PORT_DIR    GPIOB
#define MOTOR_AIN1_PIN      GPIO_Pin_13
#define MOTOR_AIN2_PIN      GPIO_Pin_12
#define MOTOR_A_PORT_PWM    GPIOB
#define MOTOR_PWMA_PIN      GPIO_Pin_1

// 右后轮 (Motor B)
#define MOTOR_B_PORT_DIR    GPIOB
#define MOTOR_BIN1_PIN      GPIO_Pin_14
#define MOTOR_BIN2_PIN      GPIO_Pin_15
#define MOTOR_B_PORT_PWM    GPIOB
#define MOTOR_PWMB_PIN      GPIO_Pin_0

// --- 前轮电机引脚定义 ---
// 左前轮 (Motor C)
#define MOTOR_C_PORT_DIR    GPIOB
#define MOTOR_CIN1_PIN      GPIO_Pin_8
#define MOTOR_CIN2_PIN      GPIO_Pin_9
#define MOTOR_C_PORT_PWM    GPIOA
#define MOTOR_PWMC_PIN      GPIO_Pin_6

// 右前轮 (Motor D)
#define MOTOR_D_PORT_DIR    GPIOB
#define MOTOR_DIN1_PIN      GPIO_Pin_5
#define MOTOR_DIN2_PIN      GPIO_Pin_10
#define MOTOR_D_PORT_PWM    GPIOA
#define MOTOR_PWMD_PIN      GPIO_Pin_7

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
