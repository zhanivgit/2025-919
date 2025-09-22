#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"  		//OLED
#include "LED.h"		//LED
#include "Key.h"       	//按键
#include "Buzzer.h"     //蜂鸣器
#include "Motor.h"     	//电机
#include "ENCODER.h"   	//编码器
#include "Serial.h"    	//串口
#include "Control.h"   //控制逻辑
#include "GY25.h"      //GY-25陀螺仪
#include "stm32f10x_usart.h" // 显式包含USART头文件

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
	GY25_Init();		//GY-25陀螺仪初始化
	Delay_ms(1000);
	GY25_SendQuery();
	OLED_ShowString(2, 1, "YAW:");
	Delay_ms(1000);
	Move_Distance_Mecanum(100.0f, 200, MOVE_TRANSLATE_LEFT);
	Delay_ms(1000);
	Move_Distance_Mecanum(20.0f, 200, MOVE_FORWARD);
	Move_Distance_Mecanum(100.0f, 200, MOVE_TRANSLATE_RIGHT);
	Delay_ms(1000);


	

	while (1)
	{
		GY25_ProcessData();
	}
	
}
