#ifndef __GY25_H
#define __GY25_H

#include "stm32f10x.h"

// GY-25陀螺仪数据结构
typedef struct {
    int16_t YAW;    // 偏航角
    int16_t PITCH;  // 俯仰角
    int16_t ROLL;   // 横滚角
    uint8_t flag;   // 数据更新标志
} GY25_Data_t;

// 函数声明
void GY25_Init(void);
void GY25_SendQuery(void);
void GY25_ProcessData(void);
void GY25_DisplayOnOLED(void);
void GY25_GetData(GY25_Data_t* data);
void GY25_Reset(void);
uint8_t GY25_IsDataValid(void);

// 外部变量声明
extern GY25_Data_t GY25_Data;

#endif 
