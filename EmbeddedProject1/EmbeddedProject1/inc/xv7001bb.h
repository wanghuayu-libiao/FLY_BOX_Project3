/**
  ******************************************************************************
  * @file    xv7001bb.h
  * @brief   XV7001BB 陀螺仪驱动（4 线 SPI）
  *          接线：PB13 = SCK，PB14 = MISO，PB15 = MOSI，PB12 = 片选 CS（软件控制）
  *          SPI 模式：MSB First，CPOL = 0，CPHA = 0（由 SPI2 配置保证）
  ******************************************************************************
  */

#ifndef __XV7001BB_H
#define __XV7001BB_H

/* Includes ------------------------------------------------------------------*/
#include "spi.h"        /* 使用 SPI2 的 Spi_ReadWriteByte() */

/* Exported functions prototypes ---------------------------------------------*/
void XV7001bb_Init(void);
void XV7001bb_WriteData(uint8_t reg_addr, uint8_t data);
uint8_t XV7001bb_ReadReg(uint8_t reg_addr);
float XV7001bb_ReadTmp(void);
int32_t XV7001bb_ReadAngle(void);

#endif /* __XV7001BB_H */
