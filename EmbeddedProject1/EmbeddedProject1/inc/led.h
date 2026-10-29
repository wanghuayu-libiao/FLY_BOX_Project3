/**
  ******************************************************************************
  * @file    led.h
  * @brief   LED 驱动（PB9，低电平点亮）
  ******************************************************************************
  */

#ifndef __LED_H
#define __LED_H

void Led_Init(void);
void Led_Toggle(void);
void Led_On(void);                      /* 点亮（PB9 输出低电平） */
void Led_Off(void);                     /* 熄灭（PB9 输出高电平） */

#endif /* __LED_H */
