#include <stm32f1xx_hal.h>
#include <stm32_hal_legacy.h>
#include "FreeRTOS.h"
#include "task.h"
#include "spi.h"
#include "can.h"
#include "xv7001bb.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

// FreeRTOS tick hook - 为HAL_Delay提供时基
void vApplicationTickHook(void)
{
	HAL_IncTick();
}

/*============================================================================
 * 常量定义
 *============================================================================*/
#define TASK_MAIN_PERIOD_MS         10      // 主任务周期 10ms
#define TASK_MAIN_DT                0.01f   // 积分时间步长 (秒)

// 零偏校准参数
#define GYRO_BIAS_SAMPLE_COUNT      200     // 校准采样数 (2秒@10ms)
#define GYRO_STILL_THRESHOLD_DPS    0.5f    // 静止判断阈值 (°/s)
#define GYRO_BIAS_EMA_ALPHA         0.01f   // 动态校准EMA系数

// CAN发送条件
#define ANGLE_CHANGE_THRESHOLD      0.01f   // 角度变化阈值 (°)
#define ANGLE_SEND_INTERVAL_MS      200     // 角度强制发送间隔
#define TEMP_SEND_INTERVAL_MS       1000    // 温度发送间隔
#define RATE_CHANGE_THRESHOLD       0.5f    // 角速度变化阈值 (°/s)
#define RATE_SEND_INTERVAL_MS       100     // 角速度强制发送间隔

/*============================================================================
 * 调试变量 (放在extern "C"块内，确保调试器可见)
 *============================================================================*/
volatile float debug_gyro_dps = 0.0f;       // 原始角速度
volatile float debug_corrected_dps = 0.0f;  // 校正后角速度
volatile float debug_temp_celsius = 25.0f;  // 温度
volatile float debug_angle_deg = 0.0f;      // 积分角度
volatile float debug_gyro_bias = 0.0f;      // 当前零偏
volatile uint8_t debug_status_raw = 0;
volatile bool g_sensor_ready = false;
volatile bool g_bias_ready = false;         // 零偏校准完成标志

// 角度和校准数据 (供CAN任务访问)
volatile float g_angle_deg = 0.0f;
volatile float g_gyro_dps = 0.0f;
volatile float g_temp_celsius = 25.0f;
volatile float g_gyro_bias_dps = 0.0f;

// 命令标志
volatile bool g_cmd_reset_angle = false;    // 角度清零命令
volatile bool g_cmd_calibrate = false;      // 重新校准命令

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

/*============================================================================
 * LED状态指示任务
 * 慢闪 (500ms): 传感器正常
 * 快闪 (50ms): 传感器异常/未就绪
 *============================================================================*/
static void Task_LED(void *argument)
{
	(void)argument;
	
	for (;;)
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
		
		if (g_sensor_ready)
		{
			vTaskDelay(pdMS_TO_TICKS(500));  // 慢闪
		}
		else
		{
			vTaskDelay(pdMS_TO_TICKS(50));   // 快闪
		}
	}
}

/*============================================================================
 * 零偏校准函数
 * 在静止状态下采集数据计算零偏
 *============================================================================*/
static bool CalibrateGyroBias(float *bias_out)
{
	float sum = 0.0f;
	float last_dps = 0.0f;
	int valid_count = 0;
	XV7_GyroData gyroData;
	
	for (int i = 0; i < GYRO_BIAS_SAMPLE_COUNT; i++)
	{
		if (XV7001bb_ReadAngle(&gyroData) == XV7_OK)
		{
			// 检测是否静止 (与上次读数差值小于阈值)
			if (i > 0 && fabsf(gyroData.dps - last_dps) > GYRO_STILL_THRESHOLD_DPS)
			{
				// 检测到运动，重新开始
				sum = 0.0f;
				valid_count = 0;
				i = -1;  // 重新开始
			}
			else
			{
				sum += gyroData.dps;
				valid_count++;
			}
			last_dps = gyroData.dps;
		}
		vTaskDelay(pdMS_TO_TICKS(TASK_MAIN_PERIOD_MS));
	}
	
	if (valid_count > 0)
	{
		*bias_out = sum / valid_count;
		return true;
	}
	return false;
}

/*============================================================================
 * 主任务 - 角度计算和零偏校准 (10ms周期)
 *============================================================================*/
static void Task_Main(void *argument)
{
	(void)argument;
	
	XV7_Status status;
	XV7_StatusReg statusReg;
	XV7_GyroData gyroData;
	XV7_TempData tempData;
	
	float angle = 0.0f;
	float gyro_bias = 0.0f;
	float last_dps = 0.0f;
	
	// 等待系统稳定
	vTaskDelay(pdMS_TO_TICKS(100));
	
	//--------------------------------------------------
	// 1. 初始化XV7001BB
	//--------------------------------------------------
	status = XV7001bb_Init();
	if (status != XV7_OK)
	{
		g_sensor_ready = false;
		for (;;) { vTaskDelay(pdMS_TO_TICKS(1000)); }  // 初始化失败，停止
	}
	g_sensor_ready = true;
	
	//--------------------------------------------------
	// 2. 零偏校准 (启动时静止2秒)
	//--------------------------------------------------
	g_bias_ready = false;
	if (CalibrateGyroBias(&gyro_bias))
	{
		g_gyro_bias_dps = gyro_bias;
		debug_gyro_bias = gyro_bias;
		g_bias_ready = true;
	}
	
	//--------------------------------------------------
	// 3. 主循环 - 角度积分计算 (10ms周期)
	//--------------------------------------------------
	TickType_t xLastWakeTime = xTaskGetTickCount();
	
	for (;;)
	{
		// 检查命令
		if (g_cmd_reset_angle)
		{
			angle = 0.0f;
			g_cmd_reset_angle = false;
		}
		if (g_cmd_calibrate)
		{
			g_bias_ready = false;
			if (CalibrateGyroBias(&gyro_bias))
			{
				g_gyro_bias_dps = gyro_bias;
				debug_gyro_bias = gyro_bias;
				g_bias_ready = true;
			}
			g_cmd_calibrate = false;
		}
		
		// 读取状态
		status = XV7001bb_ReadStatus(&statusReg);
		if (status == XV7_OK)
		{
			debug_status_raw = statusReg.raw;
			
			if (statusReg.proc_ok && statusReg.state == XV7_STATE_SLEEP_OUT)
			{
				g_sensor_ready = true;
				
				// 读取角速度
				if (XV7001bb_ReadAngle(&gyroData) == XV7_OK)
				{
					float raw_dps = gyroData.dps;
					debug_gyro_dps = raw_dps;
					
					// 零偏校正
					float corrected_dps = raw_dps - gyro_bias;
					debug_corrected_dps = corrected_dps;
					g_gyro_dps = corrected_dps;
					
					// 梯形积分法计算角度
					if (g_bias_ready)
					{
						float avg_dps = (corrected_dps + last_dps) * 0.5f;
						angle += avg_dps * TASK_MAIN_DT;
					}
					last_dps = corrected_dps;
					
					// 动态零偏校准 (静止时缓慢调整)
					if (g_bias_ready && fabsf(corrected_dps) < GYRO_STILL_THRESHOLD_DPS)
					{
						gyro_bias += raw_dps * GYRO_BIAS_EMA_ALPHA;
						g_gyro_bias_dps = gyro_bias;
						debug_gyro_bias = gyro_bias;
					}
				}
				
				// 读取温度
				if (XV7001bb_ReadTmp(&tempData) == XV7_OK)
				{
					debug_temp_celsius = tempData.celsius;
					g_temp_celsius = tempData.celsius;
				}
				
				// 更新全局角度
				g_angle_deg = angle;
				debug_angle_deg = angle;
			}
			else
			{
				g_sensor_ready = false;
			}
		}
		else
		{
			g_sensor_ready = false;
		}
		
		// 精确10ms周期
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TASK_MAIN_PERIOD_MS));
	}
}

/*============================================================================
 * CAN发送任务 - 发送角度、温度、角速度数据
 *============================================================================*/
static void Task_Can_Tx(void *argument)
{
	(void)argument;
	
	float last_angle = 0.0f;
	float last_rate = 0.0f;
	uint32_t last_angle_tick = 0;
	uint32_t last_temp_tick = 0;
	uint32_t last_rate_tick = 0;
	uint8_t data[8];
	
	vTaskDelay(pdMS_TO_TICKS(500));  // 等待传感器初始化
	
	for (;;)
	{
		uint32_t now = HAL_GetTick();
		
		if (g_sensor_ready && g_bias_ready)
		{
			float angle = g_angle_deg;
			float temp = g_temp_celsius;
			float rate = g_gyro_dps;
			
			// 发送角度 (变化>0.01°或超过200ms)
			if (fabsf(angle - last_angle) >= ANGLE_CHANGE_THRESHOLD ||
			    (now - last_angle_tick) >= ANGLE_SEND_INTERVAL_MS)
			{
				memcpy(data, &angle, sizeof(float));
				CAN_TransmitWithId(CAN_ID_ANGLE, data, 4);
				last_angle = angle;
				last_angle_tick = now;
			}
			
			// 发送温度 (每1000ms)
			if ((now - last_temp_tick) >= TEMP_SEND_INTERVAL_MS)
			{
				memcpy(data, &temp, sizeof(float));
				CAN_TransmitWithId(CAN_ID_TEMP, data, 4);
				last_temp_tick = now;
			}
			
			// 发送角速度 (变化>0.5°/s或超过100ms)
			if (fabsf(rate - last_rate) >= RATE_CHANGE_THRESHOLD ||
			    (now - last_rate_tick) >= RATE_SEND_INTERVAL_MS)
			{
				memcpy(data, &rate, sizeof(float));
				CAN_TransmitWithId(CAN_ID_GYRO_RATE, data, 4);
				last_rate = rate;
				last_rate_tick = now;
			}
		}
		
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

/*============================================================================
 * CAN接收任务 - 处理接收命令
 *============================================================================*/
static void Task_Can_Rx(void *argument)
{
	(void)argument;
	
	CAN_RxHeaderTypeDef rxHeader;
	uint8_t rxData[8];
	
	// 启用CAN接收中断 (FIFO0)
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	
	for (;;)
	{
		// 轮询检查是否有消息
		if (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0)
		{
			if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
			{
				// 解析命令
				if (rxHeader.DLC >= 1)
				{
					uint8_t cmd = rxData[0];
					
					switch (cmd)
					{
					case 0x01:  // 角度清零
					case 0x7B:  // 角度清零 (兼容)
						g_cmd_reset_angle = true;
						break;
						
					case 0x02:  // 硬件零点校准
						XV7001bb_ZeroCalibrate();
						break;
						
					case 0x03:  // 设置软件零偏
						if (rxHeader.DLC >= 5)
						{
							float bias;
							memcpy(&bias, &rxData[1], sizeof(float));
							g_gyro_bias_dps = bias;
							debug_gyro_bias = bias;
						}
						break;
						
					case 0x04:  // 重新校准
						g_cmd_calibrate = true;
						break;
					}
				}
			}
		}
		
		vTaskDelay(pdMS_TO_TICKS(5));
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

	// 创建LED状态指示任务
	xTaskCreate(Task_LED, "LED", 128, NULL, tskIDLE_PRIORITY + 1, NULL);
	
	// 创建主任务 (角度计算, 10ms周期)
	xTaskCreate(Task_Main, "Main", 512, NULL, tskIDLE_PRIORITY + 3, NULL);
	
	// 创建CAN发送任务
	xTaskCreate(Task_Can_Tx, "CAN_TX", 256, NULL, tskIDLE_PRIORITY + 2, NULL);
	
	// 创建CAN接收任务
	xTaskCreate(Task_Can_Rx, "CAN_RX", 256, NULL, tskIDLE_PRIORITY + 2, NULL);
	
	// 启动FreeRTOS调度器
	vTaskStartScheduler();

	// 不应到达这里
	for (;;) {}
}
