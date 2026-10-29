/**
  ******************************************************************************
  * @file    can.h
  * @brief   CAN1 驱动（HAL 版，STM32F103C8T6，PA11=CAN_RX，PA12=CAN_TX，1Mbps）
  ******************************************************************************
  */

#ifndef __CAN_H
#define __CAN_H

#include "stm32f1xx_hal.h"

/* 邮箱全满时最多等这么久（ms）再丢弃。
 * 1Mbps 下一帧 8 字节约 110μs，5ms 足够排空 3 个邮箱并留出总线竞争余量。 */
#ifndef CAN_TX_WAIT_MS
#define CAN_TX_WAIT_MS              5U
#endif

/* 一帧标准数据帧（接收队列的元素） */
typedef struct
{
	uint16_t id;
	uint8_t  dlc;
	uint8_t  data[8];
} CanFrame_t;

void Can_Init(void);
uint8_t Can_SendByte(uint8_t data);
uint8_t Can_SendStd(uint16_t id, const uint8_t *data, uint8_t dlc);
uint8_t Can_RxPop(CanFrame_t *frame);
void Can_SetRxCallback(void (*callback)(void));

#endif /* __CAN_H */
