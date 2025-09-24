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
#include "Sensor.h"    //光电传感器
#include "stm32f10x_usart.h" // 显式包含USART头文件

int Base_Speed = 170; // 基础速度，可调

/**
  * @brief  执行特定路径的移动序列，包括前进和平移操作
  * @note   该函数按照预定义路径控制麦克纳姆轮小车移动，依次执行前进、左平移等动作
  * @param  无
  * @retval 无
  */
void step_12(void)
{
	Move_Distance_Mecanum(50, Base_Speed, MOVE_FORWARD);
	Move_Distance_Mecanum(80, Base_Speed, MOVE_TRANSLATE_LEFT);
	Move_Distance_Mecanum(25, Base_Speed, MOVE_FORWARD);
	Move_Distance_Mecanum(90, Base_Speed, MOVE_TRANSLATE_LEFT);
	Move_Distance_Mecanum(25, Base_Speed, MOVE_BACKWARD);
	Move_Distance_Mecanum(30, Base_Speed, MOVE_TRANSLATE_LEFT);
}

/**
  * @brief  执行第三阶段的任务，控制麦克纳姆轮小车进行左右平移并检测传感器和距离
  * @note   该函数实现一个循环过程，包括左平移到检测点、后退至指定距离、右平移到检测点、再后退至指定距离
  *         主要用于在特定路径上通过传感器检测目标位置，并使用超声波测量距离进行定位
  * @param  无
  * @retval 无
  */
void step_3(void) {
    int16_t distance;
    uint8_t leftSensor, rightSensor;
    
    while (1) {
        // 执行左平移，并在过程中检测传感器
        Clear_All_Encoder_Count(); // 清零编码器计数
        leftSensor = Sensor_Left_Get();
        while (leftSensor != 0) { // 当未检测到信号时继续平移
		OLED_ShowString(4, 1, "L:");
		OLED_ShowNum(4, 3, leftSensor, 1);
		OLED_ShowString(4, 6, "R:");
		OLED_ShowNum(4, 7, rightSensor, 1);
        Move_Distance_Mecanum(0.1, Base_Speed, MOVE_TRANSLATE_LEFT); // 小步移动
        leftSensor = Sensor_Left_Get();
        Delay_ms(10);
        }
        
        // 左传感器检测到信号，开始后退，直到超声波检测到距离小于10cm
        Move(-Base_Speed); // 持续后退
        do {
            distance = sonar_mm();

            OLED_ShowNum(4, 9, distance, 4);
            Delay_ms(10);
        } while (distance >= 100 || distance <= 0); // 持续后退直到距离小于10cm或无效
        Motor_Stop(); // 停止后退
        
        // 距离小于10cm，执行右平移，并在过程中检测传感器
        Clear_All_Encoder_Count(); // 清零编码器计数
        rightSensor = Sensor_Right_Get();
        while (rightSensor != 0) { // 当未检测到信号时继续平移
            Move_Distance_Mecanum(0.1, Base_Speed, MOVE_TRANSLATE_RIGHT); // 小步移动
            rightSensor = Sensor_Right_Get();
            Delay_ms(10);
        }
        
        // 右传感器检测到信号，开始后退，直到超声波检测到距离小于10cm
        Move(-Base_Speed); // 持续后退
        do {
            distance = sonar_mm();
            Delay_ms(10);
        } while (distance >= 100 || distance <= 0); // 持续后退直到距离小于10cm或无效
        Motor_Stop(); // 停止后退
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
	HC_SR04_Init();     //HC-SR04超声波传感器初始化
	Sensor_Init();      //光电传感器初始化
	Delay_ms(2000);
	GY25_SendQuery();
// step_12();
 step_3();

	
	// 下面的代码不会被执行，保留用于调试参考
	while (1)
	{
		 GY25_ProcessData();
		
		// 读取超声波传感器距离值并显示在OLED上
		int16_t distance = sonar_mm();
		OLED_ShowString(1, 1, "Dist:");
		OLED_ShowNum(1, 6, distance, 4);
		OLED_ShowString(1, 10, "mm");
		
		// 读取光电传感器状态并显示在OLED上
		uint8_t leftSensor = Sensor_Left_Get();
		uint8_t rightSensor = Sensor_Right_Get();
		OLED_ShowString(2, 1, "L:");
		OLED_ShowNum(2, 3, leftSensor, 1);
		OLED_ShowString(3, 1, "R:");
		OLED_ShowNum(3, 3, rightSensor, 1);
		
		Delay_ms(100); // 延时100ms，避免刷新过快
	}
}