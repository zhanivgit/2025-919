#include "stm32f10x.h"
#include "Delay.h"
#include "HCSR04.h"

#define Echo GPIO_Pin_1    // HC-SR04 Echo引脚改为GPIOD_Pin_1
#define Trig GPIO_Pin_0    // HC-SR04 Trig引脚改为GPIOD_Pin_0

uint64_t time = 0;        // 计时变量，计数
uint64_t time_end = 0;    // 计时变量，计数结束

/**
  * @brief  初始化HC-SR04超声波传感器使用的定时器
  * @param  无
  * @retval 无
  * @note   配置TIM4定时器用于测量超声波回波时间
  */
void HC_SR04_Timer_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);        // 使能TIM4时钟
    
    TIM_InternalClockConfig(TIM4);                               // 设置TIM4内部时钟
    
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;           // 定义结构体，初始化定时器
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; // 时钟分频1(无分频)
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; // 设置向上计数模式
    TIM_TimeBaseInitStructure.TIM_Period = 10 - 1;              // 设置自动重装载值,每10微秒中断一次
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;           // 设置预分频值,每1微秒+1
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;        // 设置重复计数
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);         // 初始化TIM4参数
    
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);                       // 清除TIM4更新标志
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);                  // 使能更新中断
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);             // 设置NVIC中断分组2
    
    NVIC_InitTypeDef NVIC_InitStructure;                        // 定义结构体，初始化NVIC
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;            // 设置中断通道
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            // 使能中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  // 设置抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;         // 设置响应优先级
    NVIC_Init(&NVIC_InitStructure);
    
    TIM_Cmd(TIM4, ENABLE);                                      // 使能定时器
}

/**
  * @brief  初始化HC-SR04超声波传感器
  * @param  无
  * @retval 无
  * @note   配置Trig引脚为输出模式，Echo引脚为输入模式
  */
void HC_SR04_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);         // 使能GPIOD时钟    
    GPIO_InitTypeDef GPIO_InitStructure;                      
    
    // 配置Trig引脚
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;          // 设置GPIO输出推挽模式
    GPIO_InitStructure.GPIO_Pin = Trig;                        // 指定GPIO引脚
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;          // 设置GPIO输出速度50Mhz
    GPIO_Init(GPIOD, &GPIO_InitStructure);                    // 初始化GPIOD
    
    // 配置Echo引脚
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;          // 设置GPIO下拉输入模式
    GPIO_InitStructure.GPIO_Pin = Echo;                      // 指定GPIO引脚
    GPIO_Init(GPIOD, &GPIO_InitStructure);                   // 初始化GPIOD
    
    GPIO_WriteBit(GPIOD, GPIO_Pin_0, Bit_RESET);                  // Trig引脚初始为低电平
    Delay_us(15);                                             // 延时15微秒
    
    HC_SR04_Timer_Init();                                   // 初始化定时器
}

/**
  * @brief  获取超声波传感器测量的距离值（毫米）
  * @param  无
  * @retval 距离值（单位：毫米）
  * @note   通过超声波飞行时间计算距离，最大测量距离38mm
  */
int16_t sonar_mm(void)								
{
	uint32_t Distance,Distance_mm = 0;
	GPIO_WriteBit(GPIOD,Trig,Bit_SET);						// Trig引脚输出高电平
	Delay_us(15);										// 延时15微秒
	GPIO_WriteBit(GPIOD,Trig,Bit_RESET);						// Trig引脚输出低电平
	while(GPIO_ReadInputDataBit(GPIOD,Echo)==0);		// 等待Echo引脚变高电平
	time=0;												// 计时清零
	while(GPIO_ReadInputDataBit(GPIOD,Echo)==1);		// 等待Echo引脚变低电平
	time_end=time;										// 记录计时结束值
	if(time_end/100<38)									// 判断距离是否小于38毫米,超过38毫米则返回0
	{
		Distance=(time_end*346)/2;						// 计算距离,25°C时声音速度为346m/s
		Distance_mm=Distance/100;						// 距离换算，time_end单位为10微秒,需要换算成秒，实际数据需要除以100
	}
	return Distance_mm;									// 返回距离值
}

/**
  * @brief  获取超声波传感器测量的距离值（米）
  * @param  无
  * @retval 距离值（单位：米）
  * @note   通过超声波飞行时间计算距离，最大测量距离38mm
  */
float sonar(void)										
{
	uint32_t Distance,Distance_mm = 0;
	float Distance_m=0;
	GPIO_WriteBit(GPIOD,Trig,Bit_SET);					// Trig引脚输出高电平
	Delay_us(15);
	GPIO_WriteBit(GPIOD,Trig,Bit_RESET);					// Trig引脚输出低电平
	while(GPIO_ReadInputDataBit(GPIOD,Echo)==0);
	time=0;
	while(GPIO_ReadInputDataBit(GPIOD,Echo)==1);
	time_end=time;
	if(time_end/100<38)
	{
		Distance=(time_end*346)/2;
		Distance_mm=Distance/100;
		Distance_m=Distance_mm/1000;
	}
	return Distance_m;
}

/**
  * @brief  TIM4定时器中断服务函数
  * @param  无
  * @retval 无
  * @note   每10微秒增加一次time计数值
  */
void TIM4_IRQHandler(void)			
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)		// 判断TIM4更新中断是否发生
	{
		time++;
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);			// 清除TIM4更新中断标志
	}
}