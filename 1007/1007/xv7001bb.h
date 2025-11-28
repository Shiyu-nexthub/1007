#ifndef __XV7001BB_H
#define __XV7001BB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdbool.h>

/*============================================================================
 * XV7001BB 寄存器地址定义
 *============================================================================*/
#define XV7_REG_DSP_CTL1        0x01    /* DSP控制寄存器1 */
#define XV7_REG_DSP_CTL2        0x02    /* DSP控制寄存器2 */
#define XV7_REG_DSP_CTL3        0x03    /* DSP控制寄存器3 (校准使能) */
#define XV7_REG_STATUS          0x04    /* 状态寄存器 */
#define XV7_REG_SLEEP_IN        0x05    /* 进入睡眠模式 */
#define XV7_REG_SLEEP_OUT       0x06    /* 退出睡眠模式 */
#define XV7_REG_STANDBY         0x07    /* 进入待机模式 */
#define XV7_REG_TEMP_READ       0x08    /* 温度数据读取 */
#define XV7_REG_SOFT_RST        0x09    /* 软件复位 */
#define XV7_REG_RATE_READ       0x0A    /* 角速度数据读取 */
#define XV7_REG_RATE_CTRL       0x0B    /* 角速度输出控制 */
#define XV7_REG_ZERO_CAL        0x0C    /* 零点校准触发 */
#define XV7_REG_FILTER_RST      0x0D    /* 滤波器复位 */
#define XV7_REG_TS_FORMAT       0x1C    /* 温度输出格式 */
#define XV7_REG_IF_CTRL         0x1F    /* 接口控制 */

/*============================================================================
 * 状态寄存器位定义
 *============================================================================*/
#define XV7_STATUS_PROC_OK      (1 << 3)    /* 处理器就绪标志 */
#define XV7_STATUS_STATE_MASK   0x07        /* 设备状态码掩码 */

/* 设备状态码 */
#define XV7_STATE_SLEEP         0x00        /* 睡眠模式 */
#define XV7_STATE_SLEEP_OUT     0x01        /* 唤醒模式 (正常工作) */
#define XV7_STATE_STANDBY       0x02        /* 待机模式 */
#define XV7_STATE_AFTER_POR     0x04        /* 上电复位后 */

/*============================================================================
 * 数据转换常量
 *============================================================================*/
#define XV7_GYRO_SENSITIVITY_24BIT  71680.0f    /* 24-bit: LSB/(°/s) */
#define XV7_GYRO_SENSITIVITY_16BIT  280.0f      /* 16-bit: LSB/(°/s) */

/*============================================================================
 * 返回状态枚举
 *============================================================================*/
typedef enum {
    XV7_OK = 0,
    XV7_ERR_SPI,
    XV7_ERR_TIMEOUT,
    XV7_ERR_NOT_READY
} XV7_Status;

/*============================================================================
 * 数据结构体
 *============================================================================*/

/* 状态寄存器结构体 */
typedef struct {
    uint8_t raw;            /* 原始值 */
    bool proc_ok;           /* 处理器就绪 */
    uint8_t state;          /* 设备状态码 */
} XV7_StatusReg;

/* 角速度数据结构体 */
typedef struct {
    int32_t raw;            /* 原始24-bit值 (符号扩展后) */
    float dps;              /* 角速度 (°/s) */
} XV7_GyroData;

/* 温度数据结构体 */
typedef struct {
    uint16_t raw;           /* 原始12-bit值 */
    float celsius;          /* 温度 (°C) */
} XV7_TempData;

/*============================================================================
 * 函数声明
 *============================================================================*/

/**
 * @brief 初始化XV7001BB传感器
 * @return XV7_OK=成功
 */
XV7_Status XV7001bb_Init(void);

/**
 * @brief 写入寄存器数据
 * @param reg 寄存器地址
 * @param data 要写入的数据
 * @return XV7_OK=成功
 */
XV7_Status XV7001bb_WriteData(uint8_t reg, uint8_t data);

/**
 * @brief 读取寄存器数据
 * @param reg 寄存器地址
 * @param data 读取的数据指针
 * @return XV7_OK=成功
 */
XV7_Status XV7001bb_ReadReg(uint8_t reg, uint8_t *data);

/**
 * @brief 读取温度数据
 * @param temp 温度数据结构体指针
 * @return XV7_OK=成功
 */
XV7_Status XV7001bb_ReadTmp(XV7_TempData *temp);

/**
 * @brief 读取角速度数据
 * @param gyro 角速度数据结构体指针
 * @return XV7_OK=成功
 */
XV7_Status XV7001bb_ReadAngle(XV7_GyroData *gyro);

/**
 * @brief 读取状态寄存器
 * @param status 状态结构体指针
 * @return XV7_OK=成功
 */
XV7_Status XV7001bb_ReadStatus(XV7_StatusReg *status);

/**
 * @brief 执行硬件零点校准
 * @return XV7_OK=成功
 */
XV7_Status XV7001bb_ZeroCalibrate(void);

/**
 * @brief 软件复位
 * @return XV7_OK=成功
 */
XV7_Status XV7001bb_SoftReset(void);

/**
 * @brief 设置温度偏置
 * @param bias 偏置值 (°C)
 */
void XV7001bb_SetTempBias(float bias);

/**
 * @brief 获取温度偏置
 * @return 当前偏置值 (°C)
 */
float XV7001bb_GetTempBias(void);

#ifdef __cplusplus
}
#endif

#endif /* __XV7001BB_H */
