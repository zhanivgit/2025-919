#include "GY25.h"
#include "OLED.h"
#include "Delay.h"

// 全局变量定义
GY25_Data_t GY25_Data = {0, 0, 0, 0};

// 接收缓冲区
static uint8_t GY25_RxBuffer[8] = {0};
static uint8_t GY25_RxIndex = 0;
static uint32_t GY25_LastUpdateTime = 0;
static uint8_t GY25_DataValid = 0;

/**
 * @brief  GY-25陀螺仪初始化
 * @param  无
 * @retval 无
 */
void GY25_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // 使能USART1和GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    
    // 配置USART1_TX (PA9)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 配置USART1_RX (PA10)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 配置NVIC
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    // 配置USART1
    USART_DeInit(USART1);
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    
    // 使能接收中断
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    
    // 使能USART1
    USART_Cmd(USART1, ENABLE);
}

/**
 * @brief  发送查询指令
 * @param  无
 * @retval 无
 */
void GY25_SendQuery(void)
{
    USART_SendData(USART1, 0xA5);
    USART_SendData(USART1, 0x51);  // 单一查询指令
}

/**
 * @brief  处理接收到的数据
 * @param  无
 * @retval 无
 */
void GY25_ProcessData(void)
{
    if (GY25_Data.flag)
    {
        GY25_Data.flag = 0;
        GY25_DisplayOnOLED();
    }
}

/**
 * @brief  在OLED上显示GY-25数据
 * @param  无
 * @retval 无
 */
void GY25_DisplayOnOLED(void)
{
    int16_t yaw_int, pitch_int, roll_int;
    uint8_t yaw_dec, pitch_dec, roll_dec;
    
    // 计算整数和小数部分
    yaw_int = GY25_Data.YAW / 100;
    yaw_dec = (GY25_Data.YAW % 100) / 10;
    
    pitch_int = GY25_Data.PITCH / 100;
    pitch_dec = (GY25_Data.PITCH % 100) / 10;
    
    roll_int = GY25_Data.ROLL / 100;
    roll_dec = (GY25_Data.ROLL % 100) / 10;
    
    // 显示YAW角度
    if (yaw_int >= 0)
    {
        OLED_ShowChar(2, 5, '+');
        OLED_ShowNum(2, 6, yaw_int, 3);
    }
    else
    {
        OLED_ShowChar(2, 5, '-');
        OLED_ShowNum(2, 6, -yaw_int, 3);
    }
    OLED_ShowChar(2, 9, '.');
    OLED_ShowNum(2, 10, yaw_dec, 1);
    
    // 显示PITCH角度
    if (pitch_int >= 0)
    {
        OLED_ShowChar(3, 7, '+');
        OLED_ShowNum(3, 8, pitch_int, 3);
    }
    else
    {
        OLED_ShowChar(3, 7, '-');
        OLED_ShowNum(3, 8, -pitch_int, 3);
    }
    OLED_ShowChar(3, 11, '.');
    OLED_ShowNum(3, 12, pitch_dec, 1);
    
    // 显示ROLL角度
    if (roll_int >= 0)
    {
        OLED_ShowChar(4, 6, '+');
        OLED_ShowNum(4, 7, roll_int, 3);
    }
    else
    {
        OLED_ShowChar(4, 6, '-');
        OLED_ShowNum(4, 7, -roll_int, 3);
    }
    OLED_ShowChar(4, 10, '.');
    OLED_ShowNum(4, 11, roll_dec, 1);
}

/**
 * @brief  获取GY-25数据
 * @param  data: 数据指针
 * @retval 无
 */
void GY25_GetData(GY25_Data_t* data)
{
    data->YAW = GY25_Data.YAW;
    data->PITCH = GY25_Data.PITCH;
    data->ROLL = GY25_Data.ROLL;
    data->flag = GY25_Data.flag;
}

/**
 * @brief  重置GY-25模块
 * @param  无
 * @retval 无
 */
void GY25_Reset(void)
{
    GY25_RxIndex = 0;
    GY25_DataValid = 0;
    GY25_Data.flag = 0;
    for(int i = 0; i < 8; i++)
    {
        GY25_RxBuffer[i] = 0;
    }
}

/**
 * @brief  检查数据是否有效
 * @param  无
 * @retval 1: 数据有效, 0: 数据无效
 */
uint8_t GY25_IsDataValid(void)
{
    return GY25_DataValid;
}

/**
 * @brief  USART1中断服务函数
 * @param  无
 * @retval 无
 */
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        uint8_t received_data = USART_ReceiveData(USART1);
        
        // 检查帧头
        if (GY25_RxIndex == 0 && received_data != 0xAA)
        {
            return;  // 不是帧头，丢弃
        }
        
        GY25_RxBuffer[GY25_RxIndex++] = received_data;
        
        // 检查是否接收到完整数据包
        if (GY25_RxIndex >= 8)
        {
            // 检查帧尾
            if (GY25_RxBuffer[7] == 0x55)
            {
                // 解析数据 (数据为实际值的100倍)
                GY25_Data.YAW = (GY25_RxBuffer[1] << 8) | GY25_RxBuffer[2];
                GY25_Data.PITCH = (GY25_RxBuffer[3] << 8) | GY25_RxBuffer[4];
                GY25_Data.ROLL = (GY25_RxBuffer[5] << 8) | GY25_RxBuffer[6];
                GY25_Data.flag = 1;  // 设置数据更新标志
                GY25_DataValid = 1;  // 设置数据有效标志
            }
            
            // 重置接收索引
            GY25_RxIndex = 0;
        }
        
        // 防止缓冲区溢出
        if (GY25_RxIndex >= 8)
        {
            GY25_RxIndex = 0;
        }
    }
} 
