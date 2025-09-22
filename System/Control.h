#ifndef __CONTROL_H
#define __CONTROL_H
 
#include <stdint.h> // 添加此行以定义 int16_t

// PID结构体定义
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float Target;
    float Error;
    float LastError;
    float PreError;
    float Integral;
    float Output;
    float IntegralLimit; // 积分限幅
    float OutputLimit;   // 输出限幅
} PID_TypeDef;

// 定义麦克纳姆轮小车的移动方向
typedef enum {
    MOVE_FORWARD,
    MOVE_BACKWARD,
    MOVE_TRANSLATE_LEFT,
    MOVE_TRANSLATE_RIGHT,
} MecanumMovementType;


// PID函数声明
void PID_Init(PID_TypeDef* pid, float Kp, float Ki, float Kd, float integral_limit, float output_limit);
float PID_Calculate(PID_TypeDef* pid, float actual_value);

// 麦克纳姆轮小车按指定方向行驶固定距离
void Move_Distance_Mecanum(float distance_cm, int16_t speed, MecanumMovementType movement_type);

#endif
