#include "Control.h"
#include "math.h"
#include "stdlib.h"
#include "Motor.h"
#include "ENCODER.h"
#include "OLED.h"
#include "Delay.h"
#include "Buzzer.h"
#include "LED.h"
#include "GY25.h"   // 添加GY25头文件
// #include "Serial.h" // 移除Serial头文件

// 参数宏定义
#define WHEEL_DIAMETER_MM   48.0f
#define WHEEL_BASE_MM       260.0f
#define ENCODER_PPR         1040
#define PI                  3.1415926535f

// PID参数定义 (需要根据实际情况调整)
#define YAW_PID_KP          2.5f    // 增加Kp提高响应速度
#define YAW_PID_KI          0.0f    // 保持Ki为0避免积分累积问题
#define YAW_PID_KD          0.5f    // 增加Kd提高系统稳定性
#define YAW_PID_INTEGRAL_LIMIT 100.0f
#define YAW_PID_OUTPUT_LIMIT   200.0f

// 全局PID控制器实例
PID_TypeDef Yaw_PID;

/**
  * @brief  PID控制器初始化
  * @param  pid: PID结构体指针
  * @param  Kp, Ki, Kd: PID参数
  * @param  integral_limit: 积分限幅
  * @param  output_limit: 输出限幅
  * @retval 无
  */
void PID_Init(PID_TypeDef* pid, float Kp, float Ki, float Kd, float integral_limit, float output_limit)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->Target = 0.0f;
    pid->Error = 0.0f;
    pid->LastError = 0.0f;
    pid->PreError = 0.0f;
    pid->Integral = 0.0f;
    pid->Output = 0.0f;
    pid->IntegralLimit = integral_limit;
    pid->OutputLimit = output_limit;
}

/**
  * @brief  PID计算函数
  * @param  pid: PID结构体指针
  * @param  actual_value: 当前实际值
  * @retval PID输出
  */
float PID_Calculate(PID_TypeDef* pid, float actual_value)
{
    // 计算角度误差，处理角度周期性
    float angle_diff = pid->Target - actual_value; // 修正误差计算方向
    // 将角度差限制在-180到180度之间
    if (angle_diff > 180.0f) {
        angle_diff -= 360.0f;
    } else if (angle_diff < -180.0f) {
        angle_diff += 360.0f;
    }
    pid->Error = angle_diff;
    
    pid->Integral += pid->Error;

    // 积分限幅
    if (pid->Integral > pid->IntegralLimit)
    {
        pid->Integral = pid->IntegralLimit;
    }
    else if (pid->Integral < -pid->IntegralLimit)
    {
        pid->Integral = -pid->IntegralLimit;
    }

    pid->Output = pid->Kp * pid->Error +
                  pid->Ki * pid->Integral +
                  pid->Kd * (pid->Error - pid->LastError);

    pid->PreError = pid->LastError;
    pid->LastError = pid->Error;

    // 输出限幅
    if (pid->Output > pid->OutputLimit)
    {
        pid->Output = pid->OutputLimit;
    }
    else if (pid->Output < -pid->OutputLimit)
    {
        pid->Output = -pid->OutputLimit;
    }

    return pid->Output;
}


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
    float initial_yaw = 0.0f; // 记录平移开始时的YAW角
    float yaw_correction = 0.0f;
    int16_t motor_speed_FL, motor_speed_FR, motor_speed_RL, motor_speed_RR;

    // 初始化YAW角PID控制器
    PID_Init(&Yaw_PID, YAW_PID_KP, YAW_PID_KI, YAW_PID_KD, YAW_PID_INTEGRAL_LIMIT, YAW_PID_OUTPUT_LIMIT);
    
    // 获取初始YAW角作为目标值
    if (GY25_IsDataValid()) {
        initial_yaw = (float)GY25_Data.YAW / 100.0f; // GY25数据是实际值的100倍
        Yaw_PID.Target = initial_yaw;
    } else {
        // 如果陀螺仪数据无效，可以考虑报错或使用默认值
        Yaw_PID.Target = 0.0f; 
    }

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
            case MOVE_TRANSLATE_RIGHT:
                // 读取当前YAW角并进行PID计算
                if (GY25_IsDataValid()) {
                    yaw_correction = PID_Calculate(&Yaw_PID, (float)GY25_Data.YAW / 100.0f);
                } else {
                    yaw_correction = 0; // 如果陀螺仪数据无效，不进行校正
                }

                // 根据平移方向和YAW角校正调整电机速度
                if (movement_type == MOVE_TRANSLATE_LEFT) {
                    // 左平移的基础速度分配
                    motor_speed_FL = -speed;
                    motor_speed_FR = speed;
                    motor_speed_RL = speed;
                    motor_speed_RR = -speed;

                    // 应用YAW角校正
                    motor_speed_FL -= (int16_t)yaw_correction;  // 统一使用减法进行校正
                    motor_speed_FR += (int16_t)yaw_correction;
                    motor_speed_RL -= (int16_t)yaw_correction;
                    motor_speed_RR += (int16_t)yaw_correction;
                } else { // MOVE_TRANSLATE_RIGHT
                    // 右平移的基础速度分配
                    motor_speed_FL = speed;
                    motor_speed_FR = -speed;
                    motor_speed_RL = -speed;
                    motor_speed_RR = speed;

                    // 应用YAW角校正
                    motor_speed_FL -= (int16_t)yaw_correction;  // 统一使用减法进行校正
                    motor_speed_FR += (int16_t)yaw_correction;
                    motor_speed_RL -= (int16_t)yaw_correction;
                    motor_speed_RR += (int16_t)yaw_correction;
                }
                
                // 对计算出的电机速度进行限幅，确保在-1000到1000之间
                motor_speed_FL = fmaxf(-1000, fminf(1000, motor_speed_FL));
                motor_speed_FR = fmaxf(-1000, fminf(1000, motor_speed_FR));
                motor_speed_RL = fmaxf(-1000, fminf(1000, motor_speed_RL));
                motor_speed_RR = fmaxf(-1000, fminf(1000, motor_speed_RR));

                Motor_SetSpeed(motor_speed_FL, motor_speed_FR, motor_speed_RL, motor_speed_RR);
                break;
            default:
                Motor_Stop();
                break;
        }
        

        // 添加OLED调试输出
        OLED_ShowString(1, 1, "YAW:");
        OLED_ShowSignedNum(1, 5, (int16_t)((float)GY25_Data.YAW / 100.0f), 4);
        OLED_ShowString(2, 1, "Tar:");
        OLED_ShowSignedNum(2, 5, (int16_t)Yaw_PID.Target, 4);
        OLED_ShowString(3, 1, "Cor:");
        OLED_ShowSignedNum(3, 5, (int16_t)Yaw_PID.Output, 4);

        Delay_ms(10); // 短暂延时以稳定控制
    }
    Motor_Stop(); // 确保最终停止
}
