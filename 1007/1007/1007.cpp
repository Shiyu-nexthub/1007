#include <stm32f1xx_hal.h>
#include <stm32_hal_legacy.h>
#include "FreeRTOS.h"
#include "task.h"
#include "spi.h"
#include "can.h"

#ifdef __cplusplus
extern "C" {
#endif

// FreeRTOS tick hook - 为HAL_Delay提供时基
void vApplicationTickHook(void)
{
	HAL_IncTick();
}

#ifdef __cplusplus
}
#endif

// 系统时钟配置: 72MHz (HSE 8MHz × PLL 9)
static void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	// 配置HSE + PLL
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;  // 8MHz × 9 = 72MHz
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	// 配置系统时钟、AHB、APB1、APB2
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
	                            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;     // HCLK = 72MHz
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;      // APB1 = 36MHz (max)
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;      // APB2 = 72MHz
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

// LED初始化
static void LED_Init(void)
{
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

// LED闪烁任务
static void Task_LED(void *argument)
{
	(void)argument;
	const TickType_t xDelay = pdMS_TO_TICKS(500);  // 500ms
	
	for (;;)
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
		vTaskDelay(xDelay);
	}
}

int main(void)
{
	HAL_Init();
	SystemClock_Config();
	LED_Init();
	
	// 初始化SPI2 (XV7001BB陀螺仪)
	MX_SPI2_Init();
	
	// 初始化CAN1
	MX_CAN_Init();
	CAN_Driver_Init();

	// 创建LED任务
	xTaskCreate(Task_LED, "LED", 128, NULL, tskIDLE_PRIORITY + 1, NULL);
	
	// 启动FreeRTOS调度器
	vTaskStartScheduler();

	// 不应到达这里
	for (;;) {}
}
