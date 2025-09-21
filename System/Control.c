#include "Control.h"
#include "math.h"
#include "stdlib.h"
#include "Motor.h"
#include "ENCODER.h"
#include "OLED.h"
#include "Delay.h"
#include "Buzzer.h"
#include "LED.h"


// 参数宏定义
#define WHEEL_DIAMETER_MM   48.0f
#define WHEEL_BASE_MM       260.0f
#define ENCODER_PPR         1040
#define PI                  3.1415926535f

/**
  * @brief  麦克纳姆轮小车按指定方向行驶固定距离
  * @param  distance_cm 距离（厘米）
  * @param  speed 速度（0 到 1000）
  * @param  movement_type 移动方向
  * @retval 无
  */
void Move_Distance_Mecanum(float distance_cm, int16_t speed, MecanumMovementType movement_type)
{
    float distance_mm = distance_cm * 10.0f;
    float turns = distance_mm / (PI * WHEEL_DIAMETER_MM);
    int required_pulses = (int)(turns * ENCODER_PPR);

    Clear_All_Encoder_Count(); // 清零所有编码器计数

    int current_average_pulses = 0;
    
    while (1)
    {
        // 读取所有四个编码器的绝对值并计算平均值
        int pulse_A = abs(Read_Rear_Left_Encoder());
        int pulse_B = abs(Read_Rear_Right_Encoder());
        int pulse_C = abs(Read_Front_Left_Encoder());
        int pulse_D = abs(Read_Front_Right_Encoder());
        
        current_average_pulses = (pulse_A + pulse_B + pulse_C + pulse_D) / 4;

        // 判断是否到达目标距离
        if (current_average_pulses >= required_pulses) {
            Motor_Stop();
            break;
        }

        // 根据移动类型控制电机
        switch (movement_type)
        {
            case MOVE_FORWARD:
                Move(speed);
                break;
            case MOVE_BACKWARD:
                Move(-speed);
                break;
            case MOVE_TRANSLATE_LEFT:
                Motor_TranslateLeft(speed);
                break;
            case MOVE_TRANSLATE_RIGHT:
                Motor_TranslateRight(speed);
                break;
            default:
                Motor_Stop();
                break;
        }
        

    }
    Motor_Stop(); // 确保最终停止
}
