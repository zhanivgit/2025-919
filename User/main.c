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
	
	// 显示初始信息
	OLED_ShowString(1, 1, "Encoder Test");
	OLED_ShowString(2, 1, "Basic Directions");
	Delay_ms(2000);
	
	// 测试向前移动
	OLED_ShowString(1, 1, "Forward 5cm");
	OLED_ShowString(4, 1, "Running...");
	Clear_All_Encoder_Count();
	Delay_ms(500);
	Move_Distance_Mecanum(5.0f, 200, MOVE_FORWARD);
	OLED_ShowString(4, 1, "Complete!");
	Delay_ms(1000);
	
	// 测试向后移动
	OLED_ShowString(1, 1, "Backward 5cm");
	OLED_ShowString(4, 1, "Running...");
	Clear_All_Encoder_Count();
	Delay_ms(500);
	Move_Distance_Mecanum(5.0f, 200, MOVE_BACKWARD);
	OLED_ShowString(4, 1, "Complete!");
	Delay_ms(1000);
	
	// 测试向左移动
	OLED_ShowString(1, 1, "Left 5cm");
	OLED_ShowString(4, 1, "Running...");
	Clear_All_Encoder_Count();
	Delay_ms(500);
	Move_Distance_Mecanum(5.0f, 200, MOVE_TRANSLATE_LEFT);
	OLED_ShowString(4, 1, "Complete!");
	Delay_ms(1000);
	
	// 测试向右移动
	OLED_ShowString(1, 1, "Right 5cm");
	OLED_ShowString(4, 1, "Running...");
	Clear_All_Encoder_Count();
	Delay_ms(500);
	Move_Distance_Mecanum(5.0f, 200, MOVE_TRANSLATE_RIGHT);
	Delay_ms(1000);
	
	// 显示测试完成
	OLED_ShowString(1, 1, "Basic Tests");
	OLED_ShowString(2, 1, "Complete!");
	OLED_ShowString(3, 1, "");
	OLED_ShowString(4, 1, "");
	
	while (1)
	{
		// 主循环可以添加其他测试代码
	}
}