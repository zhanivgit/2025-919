#ifndef __CONTROL_H
#define __CONTROL_H
 
#include <stdint.h>

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
    float IntegralLimit;
    float OutputLimit;
} PID_TypeDef;

// 定义麦克纳姆轮小车的移动方向
typedef enum {
    MOVE_FORWARD,
    MOVE_BACKWARD,
    MOVE_TRANSLATE_LEFT,
    MOVE_TRANSLATE_RIGHT,
} MecanumMovementType;

void PID_Init(PID_TypeDef* pid, float Kp, float Ki, float Kd, float integral_limit, float output_limit);
float PID_Calculate(PID_TypeDef* pid, float actual_value);
void Move_Distance_With_Speed(float distance_cm, int16_t target_speed, MecanumMovementType movement_type);

#endif
