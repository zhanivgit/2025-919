#include "Control.h"
#include "math.h"
#include "stdlib.h"
#include "Motor.h"
#include "ENCODER.h"
#include "OLED.h"
#include "Delay.h"
#include "GY25.h"

// 参数宏定义
#define WHEEL_DIAMETER_MM   48.0f
#define ENCODER_PPR         1040
#define PI                  3.1415926535f

// PID参数定义 - 进一步优化参数以提高运动平滑度
#define YAW_PID_KP          1.5f    // 进一步降低Kp以减少过度校正
#define YAW_PID_KI          0.0f    // 保持Ki为0避免积分累积问题
#define YAW_PID_KD          0.1f    // 进一步降低Kd提高系统稳定性
#define YAW_PID_INTEGRAL_LIMIT 30.0f
#define YAW_PID_OUTPUT_LIMIT   50.0f

// 轮子速度控制PID参数 - 优化参数以提高运动平滑度
#define WHEEL_PID_KP        1.5f   // 进一步降低Kp减少抖动
#define WHEEL_PID_KI        0.01f   // 保持Ki为0
#define WHEEL_PID_KD        0.2f   // 进一步降低Kd提高稳定性
#define WHEEL_PID_INTEGRAL_LIMIT 30.0f
#define WHEEL_PID_OUTPUT_LIMIT   100.0f

// 全局PID控制器实例
PID_TypeDef Yaw_PID;
PID_TypeDef Wheel_FL_PID;
PID_TypeDef Wheel_FR_PID;
PID_TypeDef Wheel_RL_PID;
PID_TypeDef Wheel_RR_PID;

// 全局变量用于记录上一次编码器读数
int last_encoder_FL = 0;
int last_encoder_FR = 0;
int last_encoder_RL = 0;
int last_encoder_RR = 0;

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
    float angle_diff = pid->Target - actual_value;
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
  * @brief  四轮速度PID计算函数
  * @param  pid: PID结构体指针
  * @param  actual_value: 当前实际速度值
  * @retval PID输出
  */
float PID_Calculate_Speed(PID_TypeDef* pid, float actual_value)
{
    // 计算速度误差
    pid->Error = pid->Target - actual_value;
    
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
  * @brief  计算轮子实际速度（脉冲/秒）
  * @param  current_encoder: 当前编码器读数
  * @param  last_encoder: 上次编码器读数
  * @param  delta_time_ms: 时间间隔（毫秒）
  * @retval 实际速度（脉冲/秒）
  */
int calculate_wheel_speed(int current_encoder, int last_encoder, uint32_t delta_time_ms) {
    // 计算编码器变化量（处理溢出情况）
    int delta_encoder = current_encoder - last_encoder;
    
    // 计算每秒脉冲数（转换为脉冲/秒）
    int speed_pps = (delta_encoder * 1000) / delta_time_ms;
    
    return speed_pps;
}

/**
  * @brief  麦克纳姆轮小车按指定速度行驶固定距离
  * @param  distance_cm 距离（厘米）
  * @param  target_speed 速度（0 到 1000）
  * @param  movement_type 移动方向
  * @retval 无
  */
void Move_Distance_With_Speed(float distance_cm, int16_t target_speed, MecanumMovementType movement_type)
{
    float distance_mm = distance_cm * 10.0f;
    float turns = distance_mm / (PI * WHEEL_DIAMETER_MM);
    int required_pulses = (int)(turns * ENCODER_PPR);
    
    Clear_All_Encoder_Count();
    
    int current_average_pulses = 0;
    float initial_yaw = 0.0f;
    float yaw_correction = 0.0f;
    int16_t motor_speed_FL, motor_speed_FR, motor_speed_RL, motor_speed_RR;
    
    // 读取当前编码器值
    int current_encoder_FL, current_encoder_FR, current_encoder_RL, current_encoder_RR;
    
    // 计算各轮实际速度（5ms间隔）
    int actual_speed_FL, actual_speed_FR, actual_speed_RL, actual_speed_RR;
    
    // PID计算（速度控制）
    float speed_pid_output_FL, speed_pid_output_FR, speed_pid_output_RL, speed_pid_output_RR;
    
    // 初始化YAW角PID控制器
    PID_Init(&Yaw_PID, YAW_PID_KP, YAW_PID_KI, YAW_PID_KD, YAW_PID_INTEGRAL_LIMIT, YAW_PID_OUTPUT_LIMIT);
    
    // 获取初始YAW角作为目标值
    if (GY25_IsDataValid()) {
        initial_yaw = (float)GY25_Data.YAW / 100.0f;
        Yaw_PID.Target = initial_yaw;
    } else {
        Yaw_PID.Target = 0.0f; 
    }
    
    // 初始化轮子速度PID控制器（用于平移时的速度控制）
    if (movement_type == MOVE_TRANSLATE_LEFT || movement_type == MOVE_TRANSLATE_RIGHT) {
        PID_Init(&Wheel_FL_PID, WHEEL_PID_KP, WHEEL_PID_KI, WHEEL_PID_KD, 
                 WHEEL_PID_INTEGRAL_LIMIT, WHEEL_PID_OUTPUT_LIMIT);
        PID_Init(&Wheel_FR_PID, WHEEL_PID_KP, WHEEL_PID_KI, WHEEL_PID_KD, 
                 WHEEL_PID_INTEGRAL_LIMIT, WHEEL_PID_OUTPUT_LIMIT);
        PID_Init(&Wheel_RL_PID, WHEEL_PID_KP, WHEEL_PID_KI, WHEEL_PID_KD, 
                 WHEEL_PID_INTEGRAL_LIMIT, WHEEL_PID_OUTPUT_LIMIT);
        PID_Init(&Wheel_RR_PID, WHEEL_PID_KP, WHEEL_PID_KI, WHEEL_PID_KD, 
                 WHEEL_PID_INTEGRAL_LIMIT, WHEEL_PID_OUTPUT_LIMIT);
        
        // 设置PID目标值
        Wheel_FL_PID.Target = abs(target_speed);
        Wheel_FR_PID.Target = abs(target_speed);
        Wheel_RL_PID.Target = abs(target_speed);
        Wheel_RR_PID.Target = abs(target_speed);
        
        // 初始化编码器记录
        last_encoder_FL = Read_Front_Left_Encoder();
        last_encoder_FR = Read_Front_Right_Encoder();
        last_encoder_RL = Read_Rear_Left_Encoder();
        last_encoder_RR = Read_Rear_Right_Encoder();
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
                Move(target_speed);
                break;
            case MOVE_BACKWARD:
                Move(-target_speed);
                break;
            case MOVE_TRANSLATE_LEFT:
            case MOVE_TRANSLATE_RIGHT:
                // 读取当前YAW角并进行PID计算
                if (GY25_IsDataValid()) {
                    yaw_correction = PID_Calculate(&Yaw_PID, (float)GY25_Data.YAW / 100.0f);
                } else {
                    yaw_correction = 0;
                }
                
                current_encoder_FL = Read_Front_Left_Encoder();
                current_encoder_FR = Read_Front_Right_Encoder();
                current_encoder_RL = Read_Rear_Left_Encoder();
                current_encoder_RR = Read_Rear_Right_Encoder();
                
                actual_speed_FL = calculate_wheel_speed(current_encoder_FL, last_encoder_FL, 5);
                actual_speed_FR = calculate_wheel_speed(current_encoder_FR, last_encoder_FR, 5);
                actual_speed_RL = calculate_wheel_speed(current_encoder_RL, last_encoder_RL, 5);
                actual_speed_RR = calculate_wheel_speed(current_encoder_RR, last_encoder_RR, 5);
                
                // 更新编码器记录
                last_encoder_FL = current_encoder_FL;
                last_encoder_FR = current_encoder_FR;
                last_encoder_RL = current_encoder_RL;
                last_encoder_RR = current_encoder_RR;
                
                speed_pid_output_FL = PID_Calculate_Speed(&Wheel_FL_PID, abs(actual_speed_FL));
                speed_pid_output_FR = PID_Calculate_Speed(&Wheel_FR_PID, abs(actual_speed_FR));
                speed_pid_output_RL = PID_Calculate_Speed(&Wheel_RL_PID, abs(actual_speed_RL));
                speed_pid_output_RR = PID_Calculate_Speed(&Wheel_RR_PID, abs(actual_speed_RR));
                
                // 根据平移方向和YAW角校正调整电机速度
                if (movement_type == MOVE_TRANSLATE_LEFT) {
                    // 左平移的基础速度分配
                    motor_speed_FL = -target_speed;
                    motor_speed_FR = target_speed;
                    motor_speed_RL = target_speed;
                    motor_speed_RR = -target_speed;
                } else {
                    // 右平移的基础速度分配
                    motor_speed_FL = target_speed;
                    motor_speed_FR = -target_speed;
                    motor_speed_RL = -target_speed;
                    motor_speed_RR = target_speed;
                }
                
                // 应用速度PID校正
                motor_speed_FL = motor_speed_FL > 0 ? (int16_t)(motor_speed_FL + speed_pid_output_FL) : (int16_t)(motor_speed_FL - speed_pid_output_FL);
                motor_speed_FR = motor_speed_FR > 0 ? (int16_t)(motor_speed_FR + speed_pid_output_FR) : (int16_t)(motor_speed_FR - speed_pid_output_FR);
                motor_speed_RL = motor_speed_RL > 0 ? (int16_t)(motor_speed_RL + speed_pid_output_RL) : (int16_t)(motor_speed_RL - speed_pid_output_RL);
                motor_speed_RR = motor_speed_RR > 0 ? (int16_t)(motor_speed_RR + speed_pid_output_RR) : (int16_t)(motor_speed_RR - speed_pid_output_RR);
                
                // 应用YAW角校正
                motor_speed_FL -= (int16_t)yaw_correction;
                motor_speed_FR += (int16_t)yaw_correction;
                motor_speed_RL -= (int16_t)yaw_correction;
                motor_speed_RR += (int16_t)yaw_correction;
                
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
        
        Delay_ms(5); // 缩短控制周期到5ms以提高响应速度
    }
    Motor_Stop();
}
