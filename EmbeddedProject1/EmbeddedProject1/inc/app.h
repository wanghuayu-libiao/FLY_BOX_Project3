/**
  ******************************************************************************
  * @file    app.h
  * @brief   应用层：外设初始化、FreeRTOS 任务创建与调度器启动
  ******************************************************************************
  */

#ifndef __APP_H
#define __APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/* 应用参数 ------------------------------------------------------------------*/
/* LED 闪烁半周期（亮 0.5s、灭 0.5s）
   osDelay 的单位是 tick，configTICK_RATE_HZ = 1000，即 1 tick = 1 ms */
#define APP_LED_BLINK_INTERVAL_MS   500U

/**
  * @brief  应用初始化：初始化外设驱动（LED、SPI1）并创建应用任务
  *         必须在 BSP_Init() 之后、APP_Start() 之前调用。
  */
void APP_Init(void);

/**
  * @brief  启动 FreeRTOS 调度器
  *         正常情况下不返回；启动失败时进入 Error_Handler()。
  */
void APP_Start(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_H */
