#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"  		//OLED
#include "LED.h"		//LED
#include "Key.h"       	//按键
#include "Buzzer.h"     //蜂鸣器
#include "Motor.h"     	//电机
#include "ENCODER.h"   	//编码器
#include "Serial.h"    	//串口
#include "Control.h"   //PID



// 参数宏定义
#define WHEEL_DIAMETER_MM   96.0f
#define WHEEL_BASE_MM       260.0f
#define ENCODER_PPR         1040 
#define PI                  3.1415926535f
// 全局变量，供Control.c使用
int left_current_pulses;
int right_current_pulses;
float rho_err;
int Base_Speed = 200; // 基础速度，可调


int main(void)
{
	OLED_Init();
	LED_Init();
	Key_Init();
	Buzzer_Init();
	Motor_Init();
	Encoder_Init();
	Serial_Init();
	Delay_ms(1000);
	// turn_degrees(90, 300); //转向测试
	// Delay_ms(1000);
	// turn_degrees(-90, 300); //转向测试
	// PID_Init(3, 0.0, 0.1);//视觉pid初始化
	// move_straight(100, 300); // 编码器前进测试
	// OLED_ShowString(1, 1, "Motor Test..."); //显示测试信息

	// // 正转2秒
	// OLED_ShowString(2, 1, "Forward ");
	// MotorA_SetSpeed(200);         //左后轮
	// MotorB_SetSpeed(200);         //右后轮
	// MotorC_SetSpeed(200);		//左前轮
	// MotorD_SetSpeed(200);		//右前轮	
	// Delay_ms(2000);
	// Move(-200);
	// Delay_ms(2000);
	// Move(0);
	while (1)
	{
		 int rear_left_encoder = Read_Rear_Left_Encoder();        //左后轮
		 int rear_right_encoder = Read_Rear_Right_Encoder();    //右后轮
		 int front_left_encoder = Read_Front_Left_Encoder();         //右前轮
		 int front_right_encoder = Read_Front_Right_Encoder();      //左前轮

		 OLED_ShowString(1, 1, "BL:");
		 OLED_ShowSignedNum(1, 4, rear_left_encoder, 5);
		 OLED_ShowString(2, 1, "BR:");
		 OLED_ShowSignedNum(2, 4, rear_right_encoder, 5);
		 OLED_ShowString(3, 1, "FL:");
		 OLED_ShowSignedNum(3, 4, front_left_encoder, 5);
		 OLED_ShowString(4, 1, "FR:");
		 OLED_ShowSignedNum(4, 4, front_right_encoder, 5);
	}
}
