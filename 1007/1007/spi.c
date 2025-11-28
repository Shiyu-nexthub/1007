#include "spi.h"

/* SPI2 句柄 */
SPI_HandleTypeDef hspi2;

/* SPI超时时间 (ms) */
#define SPI_TIMEOUT_MS  100

/**
 * @brief SPI2 初始化
 * 
 * 配置参数 (根据项目参数手册):
 * - Mode 3: CPOL=1, CPHA=1
 * - 8-bit 数据位
 * - MSB First (高位在前)
 * - Prescaler = 256 (APB1=36MHz, SPI时钟约140.625kHz)
 * - 软件NSS控制
 */
void MX_SPI2_Init(void)
{
    /* 使能时钟 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_SPI2_CLK_ENABLE();
    
    /* GPIO配置 */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* SCK, MOSI: 复用推挽输出 */
    GPIO_InitStruct.Pin = SPI2_SCK_PIN | SPI2_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /* MISO: 浮空输入 */
    GPIO_InitStruct.Pin = SPI2_MISO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /* NSS: GPIO输出 (软件控制) */
    GPIO_InitStruct.Pin = SPI2_NSS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /* NSS默认高电平 (未选中) */
    SPI2_NSS_HIGH();
    
    /* SPI2配置 */
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;      /* CPOL = 1 */
    hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;           /* CPHA = 1 */
    hspi2.Init.NSS = SPI_NSS_SOFT;                   /* 软件NSS */
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;  /* 36MHz/256 ≈ 140.6kHz */
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;          /* 高位在前 */
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 10;
    
    HAL_SPI_Init(&hspi2);
}

/**
 * @brief SPI2 发送数据
 */
HAL_StatusTypeDef SPI2_Transmit(uint8_t *pData, uint16_t Size)
{
    return HAL_SPI_Transmit(&hspi2, pData, Size, SPI_TIMEOUT_MS);
}

/**
 * @brief SPI2 接收数据
 */
HAL_StatusTypeDef SPI2_Receive(uint8_t *pData, uint16_t Size)
{
    return HAL_SPI_Receive(&hspi2, pData, Size, SPI_TIMEOUT_MS);
}

/**
 * @brief SPI2 同时发送接收
 */
HAL_StatusTypeDef SPI2_TransmitReceive(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)
{
    return HAL_SPI_TransmitReceive(&hspi2, pTxData, pRxData, Size, SPI_TIMEOUT_MS);
}
