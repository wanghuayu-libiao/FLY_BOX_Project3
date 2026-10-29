/**
  ******************************************************************************
  * @file    bsp.h
  * @brief   板级支持（BSP）：HAL / 系统时钟初始化、公共错误处理
  ******************************************************************************
  */

#ifndef __BSP_H
#define __BSP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/**
  * @brief  板级初始化：HAL_Init() + 系统时钟配置
  *         （HSE 8 MHz -> PLL x9 -> SYSCLK 72 MHz）
  *         必须在任何外设初始化之前调用。
  */
void BSP_Init(void);

/**
  * @brief  致命错误处理：关闭中断并原地死循环
  *         初始化失败时调用，方便调试器在此处停下定位问题。
  */
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_H */
