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

/* ★ 诊断接口（供 flybox 的系统健康帧使用）
 *   hcan 在 can.c 里是 static，外部拿不到，所以开两个只读接口。 */

/** @brief 取 CAN 错误状态寄存器 ESR 原值
  *        bit4 = 总线关闭标志，bit16..23 = TEC，bit24..31 = REC */
uint32_t Can_GetEsr(void);

/** @brief 取「邮箱全满时丢弃旧帧」的累计次数（在涨 = 总线拥堵/无应答）*/
uint32_t Can_GetTxFlushCount(void);

#endif /* __CAN_H */
