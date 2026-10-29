/**
  ******************************************************************************
  * @file    app.c
  * @brief   应用层实现：外设初始化、任务创建、调度器启动
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app.h"
#include <../CMSIS_RTOS/cmsis_os.h>
#include "bsp.h"
#include "led.h"
#include "spi.h"

/* Private function prototypes -----------------------------------------------*/
static void LED_Thread(void const *argument);

/* Private variables ---------------------------------------------------------*/
static osThreadId ledThreadHandle;

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  应用初始化：外设驱动 + FreeRTOS 任务
  * @param  None
  * @retval None
  */
void APP_Init(void)
{
	/* ---- 外设驱动初始化 ---- */
	Led_Init();
	Spi_Init();

	/* ---- 创建应用任务 ---- */

	/* LED 闪烁任务：优先级 Normal，栈大小用最小栈配置 */
	osThreadDef(LED, LED_Thread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
	ledThreadHandle = osThreadCreate(osThread(LED), NULL);
	if (ledThreadHandle == NULL)
	{
		Error_Handler();
	}

	/* 说明：SPI 收发用法（在任意任务中调用）
	 *   uint8_t rx = Spi_ReadWriteByte(0xA5U);   // 全双工收发一个字节
	 *                                            // 函数内部自动控制片选 PA4
	 */
}

/**
  * @brief  启动 FreeRTOS 调度器
  * @param  None
  * @retval None（正常情况下不返回）
  */
void APP_Start(void)
{
	osKernelStart();

	/* 只有调度器启动失败才会执行到这里 */
	Error_Handler();
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  LED 闪烁任务：每 0.5s 翻转一次（亮 0.5s、灭 0.5s）
  * @param  argument not used
  * @retval None（任务内为无限循环）
  */
static void LED_Thread(void const *argument)
{
	(void)argument;

	for (;;)
	{
		Led_Toggle();
		osDelay(APP_LED_BLINK_INTERVAL_MS);
	}
}
