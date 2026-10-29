/**
  ******************************************************************************
  * @file    spi.c
  * @brief   SPI2 驱动实现（HAL 版，全双工主机，软件片选 PB12）
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "spi.h"

/* Private define ------------------------------------------------------------*/
/* HAL 阻塞收发的超时时间（ms）
 * ★ 必须是 2 而不是 100：角速度一次读取要交换 4 个字节，每字节各带一次超时。
 *   陀螺仪没接/焊虚时，100ms × 4 = 400ms 一次，而 Task_Angle 周期是 1ms，
 *   osDelayUntil 的到期时间永远在过去 -> 立即返回 -> 该任务 100% 占住 CPU，
 *   把 Task_Bringup / Task_Motor 等低优先级任务彻底饿死（电机自然一动都不动）。
 *   2ms 足够 2.25MHz SPI 传完 1 个字节（实测 < 1us）。 */
#define SPI_TIMEOUT_MS   2U

/* Global variables ----------------------------------------------------------*/
/* SPI2 句柄（全局，供 HAL_SPI_MspInit 使用） */
SPI_HandleTypeDef hspi2;

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化 SPI2（全双工主机，8 位，MSB First，CPOL=0，CPHA=0，16 分频）
  *         引脚分配（在 HAL_SPI_MspInit 中配置）：
  *             PB12 = 片选（软件控制，普通推挽输出，初始为高）
  *             PB13 = SCK  / PB15 = MOSI（复用推挽输出）
  *             PB14 = MISO（浮空输入）
  *         时钟：SPI2 挂在 APB1（36 MHz）上，16 分频 => 2.25 MHz
  * @param  None
  * @retval None
  */
void Spi_Init(void)
{
	/* SPI2 参数配置（GPIO 已在 HAL_SPI_MspInit 中初始化） */
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;    /* CPOL = 0 */
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;        /* CPHA = 0 */
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 7;
	(void)HAL_SPI_Init(&hspi2);

	/* 开启 SPI2 外设 */
	__HAL_SPI_ENABLE(&hspi2);
}

/**
  * @brief  全双工收发一个字节（纯字节交换，不控制片选）
  *         注意：片选(PB12)的拉低/拉高由设备驱动层控制（见 Src/xv7001bb.c）。
  *         原因：一次寄存器访问 = 地址字节 + 数据字节，两个字节必须保持在
  *         同一个 CS 低电平期间；若每个字节都私自翻转 CS，一次访问会被切成两段。
  * @param  txData 要发送的字节
  * @retval 接收到的字节；收发超时返回 0
  */
uint8_t Spi_ReadWriteByte(uint8_t txData)
{
	uint8_t rxData = 0;

	(void)HAL_SPI_TransmitReceive(&hspi2, &txData, &rxData, 1U, SPI_TIMEOUT_MS);

	return rxData;
}

/* HAL MSP -------------------------------------------------------------------*/

/**
  * @brief  SPI2 底层初始化（由 HAL_SPI_Init() 内部自动回调，HAL 库中为 __weak）
  *         STM32F103 的 SPI2 引脚固定为 PB12~PB15，无需 AFIO 重映射
  * @param  hspi SPI 句柄指针
  * @retval None
  */
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	if (hspi->Instance == SPI2)
	{
		/* 1. 开启时钟：SPI2 与 GPIOB */
		__HAL_RCC_SPI2_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();

		/* 2. PB12 片选：普通推挽输出，先拉高（片选无效，不选中从机） */
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		GPIO_InitStruct.Pin = GPIO_PIN_12;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* 3. PB13(SCK)、PB15(MOSI)：复用推挽输出 */
		GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_15;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* 4. PB14(MISO)：浮空输入（F1 的 SPI 输入引脚无复用模式，配置为输入） */
		GPIO_InitStruct.Pin = GPIO_PIN_14;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}
}
