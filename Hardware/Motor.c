#include "stm32f10x.h"                  // Device header
#include "Motor.h"
#include "stm32f10x_gpio.h" // 添加GPIO头文件
#include "stm32f10x_tim.h"  // 添加TIM头文件
#include "stm32f10x_rcc.h"  // 添加RCC头文件
/**
  * 函    数：直流电机初始化
  * 参    数：无
  * 返 回 值：无
 **/
void Motor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    
    /* 1. 开启相关时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);    // 开启GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);    // 开启GPIOB时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);     // 开启TIM3时钟
    
    /* 2. 配置方向控制GPIO */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
    // Motor A & B & C & D 方向引脚 (都在GPIOB)
    GPIO_InitStructure.GPIO_Pin = MOTOR_AIN1_PIN | MOTOR_AIN2_PIN | 
                                 MOTOR_BIN1_PIN | MOTOR_BIN2_PIN |
                                 MOTOR_CIN1_PIN | MOTOR_CIN2_PIN |
                                 MOTOR_DIN1_PIN | MOTOR_DIN2_PIN;
    GPIO_Init(MOTOR_A_PORT_DIR, &GPIO_InitStructure); // 使用任意一个宏的端口即可，因为它们都在GPIOB

    /* 3. 配置PWM输出GPIO */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 复用推挽输出
    
    // Motor A & B PWM引脚 (GPIOB)
    GPIO_InitStructure.GPIO_Pin = MOTOR_PWMA_PIN | MOTOR_PWMB_PIN;
    GPIO_Init(MOTOR_A_PORT_PWM, &GPIO_InitStructure);
    
    // Motor C & D PWM引脚 (GPIOA)
    GPIO_InitStructure.GPIO_Pin = MOTOR_PWMC_PIN | MOTOR_PWMD_PIN;
    GPIO_Init(MOTOR_C_PORT_PWM, &GPIO_InitStructure);
    
    /* 4. 配置TIM3 PWM (用于所有四个电机) */
    // 时基配置
    TIM_TimeBaseStructure.TIM_Period = 1000-1;               // 自动重装载值, 产生1KHz的PWM
    TIM_TimeBaseStructure.TIM_Prescaler = 72-1;              // 72MHz/72 = 1MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    
    // PWM模式配置
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = 0;                       // 初始占空比0%
    
    // 配置所有四个通道
    TIM_OC1Init(TIM3, &TIM_OCInitStructure); // CH1 -> PA6 (Motor C)
    TIM_OC2Init(TIM3, &TIM_OCInitStructure); // CH2 -> PA7 (Motor D)
    TIM_OC3Init(TIM3, &TIM_OCInitStructure); // CH3 -> PB0 (Motor B)
    TIM_OC4Init(TIM3, &TIM_OCInitStructure); // CH4 -> PB1 (Motor A)
    
    // 使能所有通道的预装载寄存器
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
    
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);

    /* 5. 初始状态设置 */
    GPIO_ResetBits(MOTOR_A_PORT_DIR, MOTOR_AIN1_PIN | MOTOR_AIN2_PIN |
                                   MOTOR_BIN1_PIN | MOTOR_BIN2_PIN |
                                   MOTOR_CIN1_PIN | MOTOR_CIN2_PIN |
                                   MOTOR_DIN1_PIN | MOTOR_DIN2_PIN);
}

/**
  * 函    数：直流电机设置速度
  * 参    数：Speed 要设置的速度，范围：-100~100
  * 返 回 值：无
  */
void MotorA_SetSpeed(int16_t speed)           //左后轮
{
    // 设置方向
    if(speed > 0) {
        // 正转：AIN1=1, AIN2=0
        GPIO_SetBits(MOTOR_A_PORT_DIR, MOTOR_AIN1_PIN);
        GPIO_ResetBits(MOTOR_A_PORT_DIR, MOTOR_AIN2_PIN);
    } 
    else if(speed < 0) {
        // 反转：AIN1=0, AIN2=1
        GPIO_ResetBits(MOTOR_A_PORT_DIR, MOTOR_AIN1_PIN);
        GPIO_SetBits(MOTOR_A_PORT_DIR, MOTOR_AIN2_PIN);
    }
    else {
        // 刹车：AIN1=1, AIN2=1
        GPIO_SetBits(MOTOR_A_PORT_DIR, MOTOR_AIN1_PIN);
        GPIO_SetBits(MOTOR_A_PORT_DIR, MOTOR_AIN2_PIN);
    }
    
    // 设置PWM占空比（取绝对值）
    uint16_t pwm = (speed < 0) ? -speed : speed;
    if(pwm > 1000) pwm = 1000;  // 限制最大值
    TIM_SetCompare4(TIM3, pwm);  // 通道4对应PB1（MOTOR_PWMA）
}

// 设置电机B速度（-1000到+1000）
void MotorB_SetSpeed(int16_t speed)           //右后轮
{
    // 设置方向
    if(speed > 0) {
        // 正转：BIN1=1, BIN2=0
        GPIO_SetBits(MOTOR_B_PORT_DIR, MOTOR_BIN1_PIN);
        GPIO_ResetBits(MOTOR_B_PORT_DIR, MOTOR_BIN2_PIN);
    } 
    else if(speed < 0) {
        // 反转：BIN1=0, BIN2=1
        GPIO_ResetBits(MOTOR_B_PORT_DIR, MOTOR_BIN1_PIN);
        GPIO_SetBits(MOTOR_B_PORT_DIR, MOTOR_BIN2_PIN);
    }
    else {
        // 刹车：BIN1=1, BIN2=1
        GPIO_SetBits(MOTOR_B_PORT_DIR, MOTOR_BIN1_PIN);
        GPIO_SetBits(MOTOR_B_PORT_DIR, MOTOR_BIN2_PIN);
    }
    
    // 设置PWM占空比
    uint16_t pwm = (speed < 0) ? -speed : speed;
    if(pwm > 1000) pwm = 1000;
    TIM_SetCompare3(TIM3, pwm);  // 通道3对应PB0（MOTOR_PWMB）
}

// 设置电机C速度（-1000到+1000）
void MotorC_SetSpeed(int16_t speed)         //左前轮
{
    // 设置方向
    if(speed > 0) {
        // 正转：CIN1=1, CIN2=0
        GPIO_SetBits(MOTOR_C_PORT_DIR, MOTOR_CIN1_PIN);
        GPIO_ResetBits(MOTOR_C_PORT_DIR, MOTOR_CIN2_PIN);
    } 
    else if(speed < 0) {
        // 反转：CIN1=0, CIN2=1
        GPIO_ResetBits(MOTOR_C_PORT_DIR, MOTOR_CIN1_PIN);
        GPIO_SetBits(MOTOR_C_PORT_DIR, MOTOR_CIN2_PIN);
    }
    else {
        // 刹车：CIN1=1, CIN2=1
        GPIO_SetBits(MOTOR_C_PORT_DIR, MOTOR_CIN1_PIN);
        GPIO_SetBits(MOTOR_C_PORT_DIR, MOTOR_CIN2_PIN);
    }
    
    // 设置PWM占空比
    uint16_t pwm = (speed < 0) ? -speed : speed;
    if(pwm > 1000) pwm = 1000;
    TIM_SetCompare1(TIM3, pwm);  // 通道1对应PA6（MOTOR_PWMC）
}

// 设置电机D速度（-1000到+1000）
void MotorD_SetSpeed(int16_t speed)          //右前轮
{
    // 设置方向
    if(speed > 0) {
        // 正转：DIN1=1, DIN2=0
        GPIO_SetBits(MOTOR_D_PORT_DIR, MOTOR_DIN1_PIN);
        GPIO_ResetBits(MOTOR_D_PORT_DIR, MOTOR_DIN2_PIN);
    } 
    else if(speed < 0) {
        // 反转：DIN1=0, DIN2=1
        GPIO_ResetBits(MOTOR_D_PORT_DIR, MOTOR_DIN1_PIN);
        GPIO_SetBits(MOTOR_D_PORT_DIR, MOTOR_DIN2_PIN);
    }
    else {
        // 刹车：DIN1=1, DIN2=1
        GPIO_SetBits(MOTOR_D_PORT_DIR, MOTOR_DIN1_PIN);
        GPIO_SetBits(MOTOR_D_PORT_DIR, MOTOR_DIN2_PIN);
    }
    
    // 设置PWM占空比
    uint16_t pwm = (speed < 0) ? -speed : speed;
    if(pwm > 1000) pwm = 1000;
    TIM_SetCompare2(TIM3, pwm);  // 通道2对应PA7（MOTOR_PWMD）
}

/**
  * 函    数：正向行驶
  * 参    数：Speed 要设置的速度，范围：-100~100
  * 返 回 值：无
  */
void Move(int16_t speed)
{
	MotorA_SetSpeed(speed);
	MotorB_SetSpeed(speed);
	MotorC_SetSpeed(speed);
	MotorD_SetSpeed(speed);
}



// 停止函数
void Motor_Stop(void)
{
    // 刹车停止
    GPIO_SetBits(MOTOR_A_PORT_DIR, MOTOR_AIN1_PIN | MOTOR_AIN2_PIN |
                                   MOTOR_BIN1_PIN | MOTOR_BIN2_PIN |
                                   MOTOR_CIN1_PIN | MOTOR_CIN2_PIN |
                                   MOTOR_DIN1_PIN | MOTOR_DIN2_PIN);
    TIM_SetCompare1(TIM3, 0);
    TIM_SetCompare2(TIM3, 0);
    TIM_SetCompare3(TIM3, 0);
    TIM_SetCompare4(TIM3, 0);
}

/**
  * @brief  控制小车向左平移
  * @param  speed 速度（-1000 到 1000）
  * @retval 无
  */
void Motor_TranslateLeft(int16_t speed)
{
    MotorC_SetSpeed(-speed); // 左前轮
    MotorD_SetSpeed(speed);  // 右前轮
    MotorA_SetSpeed(speed);  // 左后轮
    MotorB_SetSpeed(-speed); // 右后轮
}

/**
  * @brief  控制小车向右平移
  * @param  speed 速度（-1000 到 1000）
  * @retval 无
  */
void Motor_TranslateRight(int16_t speed)
{
    MotorC_SetSpeed(speed);   // 左前轮
    MotorD_SetSpeed(-speed);  // 右前轮
    MotorA_SetSpeed(-speed);  // 左后轮
    MotorB_SetSpeed(speed);   // 右后轮
}

/**
  * @brief  设置所有四个麦克纳姆轮的独立速度
  * @param  speed_FL: 左前轮速度
  * @param  speed_FR: 右前轮速度
  * @param  speed_RL: 左后轮速度
  * @param  speed_RR: 右后轮速度
  * @retval 无
  */
void Motor_SetSpeed(int16_t speed_FL, int16_t speed_FR, int16_t speed_RL, int16_t speed_RR)
{
    MotorC_SetSpeed(speed_FL); // 左前轮
    MotorD_SetSpeed(speed_FR); // 右前轮
    MotorA_SetSpeed(speed_RL); // 左后轮
    MotorB_SetSpeed(speed_RR); // 右后轮
}

/**
  * @brief  控制小车原地左转90度
  * @param  speed 速度（0 到 1000）
  * @retval 无
  */
 void Motor_TurnLeft90(int16_t speed)
{
    // 左转时，左轮向后，右轮向前
    MotorC_SetSpeed(-speed); // 左前轮向后
    MotorD_SetSpeed(speed);  // 右前轮向前
    MotorA_SetSpeed(-speed); // 左后轮向后
    MotorB_SetSpeed(speed);  // 右后轮向前
}

/**
  * @brief  控制小车原地右转90度
  * @param  speed 速度（0 到 1000）
  * @retval 无
  */
void Motor_TurnRight90(int16_t speed)
{
    // 右转时，左轮向前，右轮向后
    MotorC_SetSpeed(speed);   // 左前轮向前
    MotorD_SetSpeed(-speed);  // 右前轮向后
    MotorA_SetSpeed(speed);   // 左后轮向前
    MotorB_SetSpeed(-speed);  // 右后轮向后
}
