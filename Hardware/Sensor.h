#ifndef __SENSOR_H
#define __SENSOR_H

#include "stm32f10x.h"

/**
  * @brief  光电传感器初始化
  * @param  无
  * @retval 无
  */
void Sensor_Init(void);

/**
  * @brief  获取左侧光电传感器状态
  * @param  无
  * @retval 传感器状态 0-有信号 1-无信号
  */
uint8_t Sensor_Left_Get(void);

/**
  * @brief  获取右侧光电传感器状态
  * @param  无
  * @retval 传感器状态 0-有信号 1-无信号
  */
uint8_t Sensor_Right_Get(void);

/**
  * @brief  获取中间光电传感器状态
  * @param  无
  * @retval 传感器状态 0-有信号 1-无信号
  */
uint8_t Sensor_Middle_Get(void);

#endif