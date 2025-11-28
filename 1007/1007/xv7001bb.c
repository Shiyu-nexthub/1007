#include "xv7001bb.h"
#include "spi.h"

/*============================================================================
 * 私有变量
 *============================================================================*/
static float g_temp_bias = 0.0f;    /* 温度偏置 */

/*============================================================================
 * 私有函数
 *============================================================================*/

/**
 * @brief SPI传输一个字节
 */
static uint8_t SPI_TransferByte(uint8_t tx)
{
    uint8_t rx = 0;
    HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1, 100);
    return rx;
}

/*============================================================================
 * 公共函数实现
 *============================================================================*/

/**
 * @brief 初始化XV7001BB传感器
 */
XV7_Status XV7001bb_Init(void)
{
    XV7_StatusReg status;
    uint32_t timeout = 1000;  /* 1秒超时 */
    
    /* 等待一段时间让传感器稳定 */
    HAL_Delay(100);
    
    /* 发送唤醒命令 (SLEEP_OUT) */
    XV7001bb_WriteData(XV7_REG_SLEEP_OUT, 0x00);
    HAL_Delay(100);
    
    /* 等待传感器就绪 */
    while (timeout > 0)
    {
        if (XV7001bb_ReadStatus(&status) == XV7_OK)
        {
            if (status.proc_ok && status.state == XV7_STATE_SLEEP_OUT)
            {
                return XV7_OK;
            }
        }
        HAL_Delay(10);
        timeout -= 10;
    }
    
    return XV7_ERR_TIMEOUT;
}

/**
 * @brief 写入寄存器数据
 * 写操作: [0][地址6:0] [数据7:0]
 */
XV7_Status XV7001bb_WriteData(uint8_t reg, uint8_t data)
{
    uint8_t cmd = reg & 0x7F;  /* bit7=0 表示写 */
    
    SPI2_NSS_LOW();
    SPI_TransferByte(cmd);
    SPI_TransferByte(data);
    SPI2_NSS_HIGH();
    
    return XV7_OK;
}

/**
 * @brief 读取寄存器数据
 * 读操作: [1][地址6:0] -> [数据7:0]
 */
XV7_Status XV7001bb_ReadReg(uint8_t reg, uint8_t *data)
{
    uint8_t cmd = reg | 0x80;  /* bit7=1 表示读 */
    
    if (data == NULL)
    {
        return XV7_ERR_SPI;
    }
    
    SPI2_NSS_LOW();
    SPI_TransferByte(cmd);
    *data = SPI_TransferByte(0xFF);  /* 发送dummy字节读取数据 */
    SPI2_NSS_HIGH();
    
    return XV7_OK;
}

/**
 * @brief 读取状态寄存器
 */
XV7_Status XV7001bb_ReadStatus(XV7_StatusReg *status)
{
    uint8_t data;
    XV7_Status ret;
    
    if (status == NULL)
    {
        return XV7_ERR_SPI;
    }
    
    ret = XV7001bb_ReadReg(XV7_REG_STATUS, &data);
    if (ret != XV7_OK)
    {
        return ret;
    }
    
    status->raw = data;
    status->proc_ok = (data & XV7_STATUS_PROC_OK) ? true : false;
    status->state = data & XV7_STATUS_STATE_MASK;
    
    return XV7_OK;
}

/**
 * @brief 读取温度数据
 * 12-bit模式: T = (raw / 16) - 6.0 + bias
 */
XV7_Status XV7001bb_ReadTmp(XV7_TempData *temp)
{
    uint8_t buf[2];
    XV7_Status ret;
    
    if (temp == NULL)
    {
        return XV7_ERR_SPI;
    }
    
    /* 读取2字节温度数据 */
    uint8_t cmd = XV7_REG_TEMP_READ | 0x80;
    
    SPI2_NSS_LOW();
    SPI_TransferByte(cmd);
    buf[0] = SPI_TransferByte(0xFF);
    buf[1] = SPI_TransferByte(0xFF);
    SPI2_NSS_HIGH();
    
    /* 提取12-bit值: buf[0]为高8位, buf[1]高2位为低2位 */
    temp->raw = ((uint16_t)buf[0] << 2) | ((buf[1] >> 6) & 0x03);
    
    /* 转换为摄氏度 */
    temp->celsius = ((float)temp->raw / 16.0f) - 6.0f + g_temp_bias;
    
    return XV7_OK;
}

/**
 * @brief 读取角速度数据
 * 24-bit模式: dps = raw / 71680.0
 */
XV7_Status XV7001bb_ReadAngle(XV7_GyroData *gyro)
{
    uint8_t buf[3];
    int32_t raw24;
    
    if (gyro == NULL)
    {
        return XV7_ERR_SPI;
    }
    
    /* 读取3字节角速度数据 */
    uint8_t cmd = XV7_REG_RATE_READ | 0x80;
    
    SPI2_NSS_LOW();
    SPI_TransferByte(cmd);
    buf[0] = SPI_TransferByte(0xFF);  /* 高字节 */
    buf[1] = SPI_TransferByte(0xFF);  /* 中字节 */
    buf[2] = SPI_TransferByte(0xFF);  /* 低字节 */
    SPI2_NSS_HIGH();
    
    /* 拼接24-bit原始数据 (大端序) */
    raw24 = ((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | buf[2];
    
    /* 符号扩展 (24-bit转32-bit) */
    if (raw24 & 0x800000)
    {
        raw24 |= 0xFF000000;
    }
    
    gyro->raw = raw24;
    
    /* 转换为 °/s */
    gyro->dps = (float)raw24 / XV7_GYRO_SENSITIVITY_24BIT;
    
    return XV7_OK;
}

/**
 * @brief 执行硬件零点校准
 * 注意: 调用时设备必须静止!
 */
XV7_Status XV7001bb_ZeroCalibrate(void)
{
    return XV7001bb_WriteData(XV7_REG_ZERO_CAL, 0x01);
}

/**
 * @brief 软件复位
 */
XV7_Status XV7001bb_SoftReset(void)
{
    return XV7001bb_WriteData(XV7_REG_SOFT_RST, 0x01);
}

/**
 * @brief 设置温度偏置
 */
void XV7001bb_SetTempBias(float bias)
{
    g_temp_bias = bias;
}

/**
 * @brief 获取温度偏置
 */
float XV7001bb_GetTempBias(void)
{
    return g_temp_bias;
}
