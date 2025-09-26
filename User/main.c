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

#include "Sensor.h"    //光电传感器
#include "stm32f10x_usart.h" // 显式包含USART头文件

int Base_Speed = 170; // 基础速度，可调

/**
  * @brief  测试Move_Distance_With_Speed函数
  * @note   该函数测试以恒定速度移动指定距离的功能
  * @param  无
  * @retval 无
  */
void test_move_distance_with_speed(void)
{
	// 测试前进50cm，速度为200
	Move_Distance_With_Speed(50, 200, MOVE_FORWARD);
	Delay_ms(1000);
	
	// 测试左平移30cm，速度为150
	Move_Distance_With_Speed(100, 400, MOVE_TRANSLATE_LEFT);
	Delay_ms(1000);
	
	// 测试后退20cm，速度为180
	Move_Distance_With_Speed(20, 180, MOVE_BACKWARD);
	Delay_ms(1000);
	
	// 测试右平移25cm，速度为160
	Move_Distance_With_Speed(100, 400, MOVE_TRANSLATE_RIGHT);
}

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
	Sensor_Init();      //光电传感器初始化
	Delay_ms(2000);
	GY25_SendQuery();
	
	// 测试Move_Distance_With_Speed函数
	test_move_distance_with_speed();

	
	// 下面的代码不会被执行，保留用于调试参考
	while (1)
	{
		 GY25_ProcessData();
		

	}
}