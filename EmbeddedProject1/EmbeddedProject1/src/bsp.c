/**
  ******************************************************************************
  * @file    bsp.c
  * @brief   板级支持实现：HAL 初始化、系统时钟、SysTick、错误处理
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp.h"
#include <../CMSIS_RTOS/cmsis_os.h>

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  板级初始化：HAL_Init() + 系统时钟配置
  * @param  None
  * @retval None
  */
void BSP_Init(void)
{
	/* 复位所有外设并初始化 Flash 接口，配置 SysTick 产生 1 ms 中断 */
	HAL_Init();

	/* HSE 8 MHz (X1) -> PLL x9 -> SYSCLK 72 MHz */
	SystemClock_Config();
}

/**
  * @brief  致命错误处理：关闭中断并原地死循环
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
	/* Disable the interrupts and stay here */
	__disable_irq();
	while (1)
	{
	}
}

/* Interrupt handlers --------------------------------------------------------*/

/**
  * @brief  SysTick 中断（1 ms 周期）
  *         同时服务两个时基：
  *           - HAL_IncTick()     ：HAL 库的 ms 延时/超时基准
  *           - osSystickHandler()：FreeRTOS 调度节拍（configTICK_RATE_HZ = 1000）
  */
void SysTick_Handler(void)
{
	HAL_IncTick();
	osSystickHandler();
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  System Clock Configuration
  *         HSE 8 MHz crystal (X1) -> PLL x9 -> SYSCLK 72 MHz
  *           SYSCLK = 72 MHz (PLL, HSE x 9)
  *           HCLK   = 72 MHz (AHB / 1)
  *           PCLK1  = 36 MHz (APB1 / 2, 36 MHz is the maximum for APB1)
  *           PCLK2  = 72 MHz (APB2 / 1)
  *         2 wait states are required for the Flash above 48 MHz.
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/* Turn on the HSE (8 MHz crystal on OSC_IN/OSC_OUT) and the PLL */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/* Select the PLL output as system clock and set the bus prescalers */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
	                              RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	(void)file;
	(void)line;

	/* Infinite loop */
	while (1)
	{
	}
}
#endif
