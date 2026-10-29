/**
  ******************************************************************************
  * @file    led.c
  * @brief   LED 驱动实现（PB9，低电平点亮，直接操作寄存器）
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "led.h"
#include "stm32f1xx_hal.h"

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化 LED（PB9，推挽输出，初始输出高电平 = 熄灭）
  *         1. 开启 GPIOB 时钟（RCC->APB2ENR 的 IOPBEN 位，bit3）
  *         2. 配置 PB9 为推挽输出：
  *              GPIOB->CRH 中 PB9 对应位段 bit[7:4] 写 0x3
  *              MODE9 = 11（输出模式，最大 50 MHz）
  *              CNF9  = 00（通用推挽输出）
  *         3. 初始输出高电平（LED 低电平点亮，故初始化后为灭）
  * @param  None
  * @retval None
  */
void Led_Init(void)
{
	/* 1. 开启 GPIOB 时钟：APB2ENR bit3 = IOPBEN */
	RCC->APB2ENR |= (1U << 3);

	/* 2. PB9 对应 CRH 的 bit[7:4]：先清零，再写入 0x3 */
	GPIOB->CRH &= ~(0xFU << 4);
	GPIOB->CRH |=  (0x3U << 4);

	/* 3. 初始输出高电平，LED 熄灭 */
	GPIOB->ODR |= (1U << 9);
}

/**
  * @brief  翻转 LED：对 GPIOB->ODR 的第 9 位异或 1（亮 <-> 灭 切换）
  * @param  None
  * @retval None
  */
void Led_Toggle(void)
{
	GPIOB->ODR ^= (1U << 9);
}

/**
  * @brief  点亮 LED（PB9 输出低电平）
  * @param  None
  * @retval None
  */
void Led_On(void)
{
	GPIOB->ODR &= ~(1U << 9);           /* PB9 = 0，低电平点亮 */
}

/**
  * @brief  熄灭 LED（PB9 输出高电平）
  * @param  None
  * @retval None
  */
void Led_Off(void)
{
	GPIOB->ODR |= (1U << 9);            /* PB9 = 1，高电平熄灭 */
}
