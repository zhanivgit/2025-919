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
#include "HCSR04.h"    //HC-SR04超声波传感器
#include "stm32f10x_usart.h" // 显式包含USART头文件

int Base_Speed = 170; // 基础速度，可调


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
	HC_SR04_Init();     //HC-SR04超声波传感器初始化
	Delay_ms(2000);
	GY25_SendQuery();
	// OLED_ShowString(2, 1, "YAW:");
	// Move_Distance_Mecanum(50, Base_Speed, MOVE_FORWARD);
	// Move_Distance_Mecanum(80, Base_Speed, MOVE_TRANSLATE_LEFT);
	// Move_Distance_Mecanum(25, Base_Speed, MOVE_FORWARD);
	// Move_Distance_Mecanum(90, Base_Speed, MOVE_TRANSLATE_LEFT);
	// Move_Distance_Mecanum(25, Base_Speed, MOVE_BACKWARD);
	// Move_Distance_Mecanum(30, Base_Speed, MOVE_TRANSLATE_LEFT);
	

	while (1)
	{
		GY25_ProcessData();
		
		// 读取超声波传感器距离值并显示在OLED上
		int16_t distance = sonar_mm();
		OLED_ShowString(1, 1, "Dist:");
		OLED_ShowNum(1, 6, distance, 4);
		OLED_ShowString(1, 10, "mm");
		
		Delay_ms(100); // 延时100ms，避免刷新过快
	}
	
}
