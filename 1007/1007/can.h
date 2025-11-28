#ifndef __CAN_H
#define __CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/* CAN1 引脚定义 */
#define CAN_RX_PIN          GPIO_PIN_11
#define CAN_RX_PORT         GPIOA
#define CAN_TX_PIN          GPIO_PIN_12
#define CAN_TX_PORT         GPIOA

/* CAN ID 定义 (发送) */
#define CAN_ID_ANGLE        0x321   /* 角度数据 */
#define CAN_ID_TEMP         0x322   /* 温度数据 */
#define CAN_ID_GYRO_RATE    0x323   /* 角速度数据 */

/* CAN句柄 */
extern CAN_HandleTypeDef hcan;

/* 函数声明 */
void MX_CAN_Init(void);
HAL_StatusTypeDef CAN_Driver_Init(void);
HAL_StatusTypeDef CAN_Transmit(uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef CAN_TransmitWithId(uint32_t StdId, uint8_t *pData, uint16_t Size);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H */
