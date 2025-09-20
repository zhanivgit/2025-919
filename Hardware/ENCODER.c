#include "ENCODER.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"

void Encoder_Init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    // 1. 开启GPIO和定时器时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM5, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM8 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

	// 禁用JTAG，保留SWD，释放PA15, PB3, PB4
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	
    // 2. 配置GPIO引脚为上拉输入
    // TIM1_CH1 (PA.08), TIM1_CH2 (PA.09) - 右后轮编码器
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // TIM2_CH1 (PA.15), TIM2_CH2 (PB.03) - 左后轮编码器 (重映射)
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // TIM5_CH1 (PA.00), TIM5_CH2 (PA.01) - 左前轮编码器
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // TIM8_CH1 (PC.06), TIM8_CH2 (PC.07) - 右前轮编码器
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
    // 3. 配置定时器为编码器模式
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 65535;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

    // 配置输入捕获
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 6;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;

    // TIM1 (右后轮)
    TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_ICInit(TIM1, &TIM_ICInitStructure);

    // TIM2 (左后轮)
    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_ICInit(TIM2, &TIM_ICInitStructure);

    // TIM5 (左前轮)
    TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_ICInit(TIM5, &TIM_ICInitStructure);

    // TIM8 (右前轮)
    TIM_EncoderInterfaceConfig(TIM8, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_ICInit(TIM8, &TIM_ICInitStructure);

    // 4. 使能定时器
    TIM_Cmd(TIM1, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
    TIM_Cmd(TIM5, ENABLE);
    TIM_Cmd(TIM8, ENABLE);
}

// 读取左后轮编码器脉冲数 (TIM2)
int Read_Rear_Left_Encoder(void) {
    return (short)TIM_GetCounter(TIM2);
}

// 读取右后轮编码器脉冲数 (TIM1)
int Read_Rear_Right_Encoder(void) {
    return (short)TIM_GetCounter(TIM1);
}

// 读取右前轮编码器脉冲数 (TIM5)
int Read_Front_Right_Encoder(void) {
    return (short)TIM_GetCounter(TIM5);
}

// 读取左前轮编码器脉冲数 (TIM8)
int Read_Front_Left_Encoder(void) {
    return (short)TIM_GetCounter(TIM8);
}

// 清零所有编码器计数
void Clear_All_Encoder_Count(void) {
    TIM_SetCounter(TIM1, 0);
    TIM_SetCounter(TIM2, 0);
    TIM_SetCounter(TIM5, 0);
    TIM_SetCounter(TIM8, 0);
}
