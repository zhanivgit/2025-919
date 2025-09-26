#include "stm32f10x.h"                  // Device header
#include "Sensor.h"

/**
  * @brief  光电传感器初始化
  * @param  无
  * @retval 无
  */
void Sensor_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}

/**
  * @brief  获取左侧光电传感器状态
  * @param  无
  * @retval 传感器状态 0-有信号 1-无信号
  */
uint8_t Sensor_Left_Get(void)
{
	return GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2);
}

/**
  * @brief  获取右侧光电传感器状态
  * @param  无
  * @retval 传感器状态 0-有信号 1-无信号
  */
uint8_t Sensor_Right_Get(void)
{
	return GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_3);
}

/**
  * @brief  获取中间光电传感器状态
  * @param  无
  * @retval 传感器状态 0-有信号 1-无信号
  */
uint8_t Sensor_Middle_Get(void)
{
	return GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_4);
}