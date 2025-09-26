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

/**
  * @brief  【高精度版】持续后退并精确地贴着左侧边缘移动，直到检测到障碍物
  * @note   此函数为实现高精度贴边的最优方案。它通过极小距离的“停止-平移”来精确修正路线。
  * 为了达到最佳效果，请根据实际情况微调下面的参数。
  * @param  target_speed: 基础的后退速度
  * @retval 无
  */
void Move_Backward_Along_Edge_Precise(int16_t target_speed) {
    // --- 可调参数 ---
    float translate_distance = 3.0;  // 【关键】每次平移修正的距离(cm)。减小此值可大幅提高精度。可以尝试0.5, 1.0, 1.5等。
    int16_t translate_speed = 150;   // 平移时的速度，可以适当调高以减少调整时间。
    uint32_t step_delay = 250;        // 【关键】每次后退步伐之间的延时(ms)。减小此值可提高反应速度。可以尝试10, 20, 30。
    // --- 参数结束 ---

    uint8_t last_left_sensor_state = Sensor_Left_Get(); // 记录初始左侧传感器状态

    // 持续后退并沿着边缘移动
    while (Sensor_Middle_Get() == 0) {  // 当未检测到障碍物时继续
        // 1. 检查左侧传感器状态
        uint8_t current_left_sensor_state = Sensor_Left_Get();

        // 2. 如果传感器状态发生变化，则执行精确的平移调整
        if (current_left_sensor_state != last_left_sensor_state) {
            Motor_Stop(); // 立即停止后退，准备精确调整

            if (current_left_sensor_state == 0) { // 检测到悬崖，需要向右调整
                Move_Distance_With_Speed(translate_distance, translate_speed, MOVE_TRANSLATE_RIGHT, NULL);
            } else { // 回到平台，需要向左调整
                Move_Distance_With_Speed(translate_distance, translate_speed, MOVE_TRANSLATE_LEFT, NULL);
            }
            last_left_sensor_state = current_left_sensor_state; // 更新状态，等待下一次变化
        }
        
        // 3. 持续后退一小步
        Motor_SetSpeed(-target_speed, -target_speed, -target_speed, -target_speed);
        Delay_ms(step_delay); 
    }
    Motor_Stop(); // 检测到障碍物时停止
}

/**
  * @brief  测试用的任务步骤1和2
  */
void step12(void)
{
	Move_Distance_With_Speed(56, 300, MOVE_FORWARD, NULL);
	Move_Distance_With_Speed(60, 300, MOVE_TRANSLATE_LEFT, NULL);
	Move_Distance_With_Speed(25, 300, MOVE_FORWARD, NULL);
	Move_Distance_With_Speed(90, 300, MOVE_TRANSLATE_LEFT, NULL);
    Move_Distance_With_Speed(15, 300, MOVE_BACKWARD, NULL);
    Move_Distance_With_Speed(50, 300, MOVE_TRANSLATE_LEFT, NULL);
}

/**
  * @brief  第三步任务流程（高精度最终版）:
  */
void step3(void)
{	
	// 1. 向左平移直到左侧传感器检测到边缘（0为悬崖）
	Move_Distance_With_Speed(70, 250, MOVE_TRANSLATE_LEFT, Sensor_Left_Get);
	
	// 2. 【关键】调用高精度贴边函数
	Move_Backward_Along_Edge_Precise(200); 
	
	// 3. 检查中间传感器是否确实检测到了障碍物
	if(Sensor_Middle_Get() == 1) {
		// 如果检测到障碍物，执行一次右平移来躲避
		Move_Distance_With_Speed(50, 300, MOVE_TRANSLATE_RIGHT, Sensor_Right_Get);
	}
	Move_Distance_With_Speed(100, 300, MOVE_BACKWARD, NULL);
	
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
	
	// 依次执行任务步骤
	step12();
	step3();
	
	// 主循环
	while (1)
	{
		 GY25_ProcessData();
         Delay_ms(100);
	}
}