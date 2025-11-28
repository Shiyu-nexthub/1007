#ifndef __SPI_H
#define __SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/* SPI2 引脚定义 (XV7001BB陀螺仪) */
#define SPI2_SCK_PIN        GPIO_PIN_13
#define SPI2_SCK_PORT       GPIOB
#define SPI2_MISO_PIN       GPIO_PIN_14
#define SPI2_MISO_PORT      GPIOB
#define SPI2_MOSI_PIN       GPIO_PIN_15
#define SPI2_MOSI_PORT      GPIOB
#define SPI2_NSS_PIN        GPIO_PIN_12
#define SPI2_NSS_PORT       GPIOB

/* NSS 软件控制宏 */
#define SPI2_NSS_LOW()      HAL_GPIO_WritePin(SPI2_NSS_PORT, SPI2_NSS_PIN, GPIO_PIN_RESET)
#define SPI2_NSS_HIGH()     HAL_GPIO_WritePin(SPI2_NSS_PORT, SPI2_NSS_PIN, GPIO_PIN_SET)

/* SPI句柄 */
extern SPI_HandleTypeDef hspi2;

/* 函数声明 */
void MX_SPI2_Init(void);
HAL_StatusTypeDef SPI2_Transmit(uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef SPI2_Receive(uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef SPI2_TransmitReceive(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H */
