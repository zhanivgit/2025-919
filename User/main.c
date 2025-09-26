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
void step12(void)
{
	Move_Distance_With_Speed(60, 300, MOVE_FORWARD, NULL);
	
	Move_Distance_With_Speed(60, 300, MOVE_TRANSLATE_LEFT, NULL);

	Move_Distance_With_Speed(25, 300, MOVE_FORWARD, NULL);


	Move_Distance_With_Speed(90, 300, MOVE_TRANSLATE_LEFT, NULL);

    Move_Distance_With_Speed(20,300, MOVE_BACKWARD, NULL);
    Move_Distance_With_Speed(50,300, MOVE_TRANSLATE_LEFT, NULL);
}
void step3(void)
{
	// 向左平移直到左侧传感器检测到0(0为悬崖)
	Move_Distance_With_Speed(70, 250, MOVE_TRANSLATE_LEFT, Sensor_Left_Get);
	
	// 开始后退，并在过程中检测中间传感器
	while(1) {
		// 检查中间传感器是否检测到1(1表示障碍)
		if(Sensor_Middle_Get() == 1) {
			// 检测到1，开始向右平移
			// 持续向右平移直到右侧传感器检测到0(0为悬崖)
			Move_Distance_With_Speed(50, 400, MOVE_TRANSLATE_RIGHT, Sensor_Right_Get);
			// 右侧检测到0后停止平移（Move_Distance_With_Speed内部已处理）
			// 停止后，外层循环会继续执行后退
		}
		
		Move_Distance_With_Speed(20, 200, MOVE_BACKWARD, NULL);  // 持续后退一小段
	}
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
	step12();
	step3();
	
	// 下面的代码不会被执行，保留用于调试参考
	while (1)
	{
		 GY25_ProcessData();
		

	}
}
