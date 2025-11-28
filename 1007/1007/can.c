#include "can.h"

/* CAN句柄 */
CAN_HandleTypeDef hcan;

/* 发送邮箱 */
static CAN_TxHeaderTypeDef TxHeader;
static uint32_t TxMailbox;

/**
 * @brief CAN1 外设初始化
 * 
 * 配置参数 (根据项目参数手册):
 * - 波特率: 500 kbps
 * - 工作模式: 正常模式 (Normal Mode)
 * - 位时序:
 *   - Prescaler = 2
 *   - SyncJumpWidth = 1 TQ
 *   - TimeSeg1 = 5 TQ
 *   - TimeSeg2 = 2 TQ
 *   - 总共 = 1 + 5 + 2 = 8 TQ
 *   - 波特率 = 36MHz / (2 × 8) = 2.25 MHz ... 实际计算应为 36MHz/(2*9)=2MHz->不对
 *   - 正确: APB1=36MHz, Prescaler=4, TQ=8 -> 36/(4*9)=1Mbps ... 
 *   - 按手册: Prescaler=4, TS1=6, TS2=1, SJW=1 => 36/(4*(1+6+1))=36/32=1.125MHz 不对
 *   - 500kbps: 36MHz / 500kbps = 72 TQ total
 *   - Prescaler=4, 1+TS1+TS2=18 -> 不对
 *   - Prescaler=8, 1+TS1+TS2=9 -> 36/(8*9)=0.5MHz=500kbps ✓
 *   - 实际使用: Prescaler=8, TS1=6, TS2=1, SJW=1
 */
void MX_CAN_Init(void)
{
    /* 使能时钟 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_CAN1_CLK_ENABLE();
    
    /* GPIO配置 */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* CAN_TX: 复用推挽输出 */
    GPIO_InitStruct.Pin = CAN_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(CAN_TX_PORT, &GPIO_InitStruct);
    
    /* CAN_RX: 上拉输入 */
    GPIO_InitStruct.Pin = CAN_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(CAN_RX_PORT, &GPIO_InitStruct);
    
    /* CAN配置 */
    hcan.Instance = CAN1;
    hcan.Init.Prescaler = 8;                        /* 36MHz / 8 = 4.5MHz */
    hcan.Init.Mode = CAN_MODE_LOOPBACK;             /* 环回模式 (测试用，无需外部硬件) */
    /* 正式使用时改为 CAN_MODE_NORMAL */
    hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;          /* 1 TQ */
    hcan.Init.TimeSeg1 = CAN_BS1_6TQ;               /* 6 TQ */
    hcan.Init.TimeSeg2 = CAN_BS2_2TQ;               /* 2 TQ */
    /* 波特率 = 36MHz / (8 * (1+6+2)) = 36MHz / 72 = 500kbps */
    hcan.Init.TimeTriggeredMode = DISABLE;
    hcan.Init.AutoBusOff = DISABLE;
    hcan.Init.AutoWakeUp = DISABLE;
    hcan.Init.AutoRetransmission = ENABLE;          /* 自动重传 */
    hcan.Init.ReceiveFifoLocked = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;
    
    HAL_CAN_Init(&hcan);
}

/**
 * @brief CAN驱动初始化 (配置滤波器并启动CAN)
 */
HAL_StatusTypeDef CAN_Driver_Init(void)
{
    CAN_FilterTypeDef FilterConfig = {0};
    
    /* 配置滤波器: 接收所有消息 */
    FilterConfig.FilterBank = 0;
    FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    FilterConfig.FilterIdHigh = 0x0000;
    FilterConfig.FilterIdLow = 0x0000;
    FilterConfig.FilterMaskIdHigh = 0x0000;         /* 不过滤任何ID */
    FilterConfig.FilterMaskIdLow = 0x0000;
    FilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    FilterConfig.FilterActivation = ENABLE;
    FilterConfig.SlaveStartFilterBank = 14;
    
    if (HAL_CAN_ConfigFilter(&hcan, &FilterConfig) != HAL_OK)
    {
        return HAL_ERROR;
    }
    
    /* 启动CAN */
    if (HAL_CAN_Start(&hcan) != HAL_OK)
    {
        return HAL_ERROR;
    }
    
    /* 初始化发送头 */
    TxHeader.StdId = CAN_ID_ANGLE;                  /* 默认ID */
    TxHeader.ExtId = 0;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;                      /* 标准帧 */
    TxHeader.DLC = 8;
    TxHeader.TransmitGlobalTime = DISABLE;
    
    return HAL_OK;
}

/**
 * @brief 使用默认ID (0x321) 发送CAN数据
 */
HAL_StatusTypeDef CAN_Transmit(uint8_t *pData, uint16_t Size)
{
    return CAN_TransmitWithId(CAN_ID_ANGLE, pData, Size);
}

/**
 * @brief 使用指定ID发送CAN数据
 */
HAL_StatusTypeDef CAN_TransmitWithId(uint32_t StdId, uint8_t *pData, uint16_t Size)
{
    /* 等待发送邮箱可用 */
    uint32_t tickstart = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
    {
        if ((HAL_GetTick() - tickstart) > 100)      /* 100ms超时 */
        {
            return HAL_TIMEOUT;
        }
    }
    
    /* 配置发送头 */
    TxHeader.StdId = StdId;
    TxHeader.DLC = (Size > 8) ? 8 : Size;           /* 最大8字节 */
    
    /* 发送数据 */
    return HAL_CAN_AddTxMessage(&hcan, &TxHeader, pData, &TxMailbox);
}
