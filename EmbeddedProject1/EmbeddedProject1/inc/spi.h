/**
  ******************************************************************************
  * @file    spi.h
  * @brief   SPI2 驱动（HAL 版，全双工主机，软件片选 PB12）
  ******************************************************************************
  */

#ifndef __SPI_H
#define __SPI_H

#include "stm32f1xx_hal.h"

void Spi_Init(void);
uint8_t Spi_ReadWriteByte(uint8_t txData);

#endif /* __SPI_H */
